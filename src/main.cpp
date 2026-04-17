#include <chrono>
#include <condition_variable>
#include <cerrno>
#include <cstring>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <utility>

#include <poll.h>
#include <pthread.h>
#include <signal.h>
#include <sys/signalfd.h>
#include <unistd.h>

#include "climbing_fsm.h"
#include "downward_sensor.h"
#include "front_distance_sensor.h"
#include "front_segment.h"
#include "imu_sensor.h"
#include "linear_actuator.h"
#include "logger.h"
#include "mcp23017_driver.h"
#include "mcp23017_limit_switch.h"
#include "middle_drive_module.h"
#include "middle_lift_module.h"
#include "motion_coordinator.h"
#include "motor_driver.h"
#include "pose_monitor.h"
#include "rear_support_module.h"
#include "robot_controller.h"
#include "safety_manager.h"
#include "front_balance_slider.h"
#include "step_detector.h"

using namespace Robot;

namespace
{
constexpr float kInitialFrontSliderExtensionM = RobotConfig::Geometry::SLIDER_HOME_OFFSET_M;
constexpr auto kSliderHomeTimeout = std::chrono::seconds(2);
constexpr auto kStartupResetTimeout = std::chrono::seconds(180);

struct WaitForImuState
{
	std::mutex mutex;
	std::condition_variable cv;
	bool pose_ready{false};
};

struct WaitForDistanceState
{
	std::mutex mutex;
	std::condition_variable cv;
	bool sample_ready{false};
};

class FixedPoseImuSensor final : public IImuSensor
{
public:
	FixedPoseImuSensor()
	{
		pose_.pitch_deg = 0.0F;
		pose_.roll_deg = 0.0F;
		pose_.yaw_deg = 0.0F;
		pose_.timestamp = SteadyClock::now();
		pose_.valid = true;
	}

	PoseData latestPose() const override
	{
		return pose_;
	}

	void setCallback(PoseCallback callback) override
	{
		callback_ = std::move(callback);
		if (callback_)
		{
			callback_(pose_);
		}
	}

private:
	PoseData pose_{};
	PoseCallback callback_;
};

bool WaitForValidImuSample(ImuSensor& imu_sensor, const std::chrono::milliseconds timeout)
{
	if (imu_sensor.latestPose().valid)
	{
		return true;
	}

	const auto wait_state = std::make_shared<WaitForImuState>();

	imu_sensor.setCallback([wait_state](const PoseData& pose) {
		if (!pose.valid)
		{
			return;
		}

		{
			std::lock_guard<std::mutex> lock(wait_state->mutex);
			wait_state->pose_ready = true;
		}

		wait_state->cv.notify_all();
	});

	std::unique_lock<std::mutex> lock(wait_state->mutex);
	(void)wait_state->cv.wait_for(lock, timeout, [wait_state]() {
		return wait_state->pose_ready;
	});
	lock.unlock();

	// Clear the callback before returning so the sensor does not hold a
	// reference to the startup wait state after bring-up has moved on.
	imu_sensor.setCallback({});
	return wait_state->pose_ready || imu_sensor.latestPose().valid;
}

bool WaitForValidFrontDistanceSample(FrontDistanceSensor& front_sensor, const std::chrono::milliseconds timeout)
{
	if (front_sensor.latest().valid)
	{
		return true;
	}

	const auto wait_state = std::make_shared<WaitForDistanceState>();

	front_sensor.setCallback([wait_state](const DistanceReading& reading) {
		if (!reading.valid)
		{
			return;
		}

		{
			std::lock_guard<std::mutex> lock(wait_state->mutex);
			wait_state->sample_ready = true;
		}

		wait_state->cv.notify_all();
	});

	std::unique_lock<std::mutex> lock(wait_state->mutex);
	(void)wait_state->cv.wait_for(lock, timeout, [wait_state]() {
		return wait_state->sample_ready;
	});
	lock.unlock();

	// Clear the callback before returning so the sensor does not hold a
	// reference to the startup wait state after bring-up has moved on.
	front_sensor.setCallback({});
	return wait_state->sample_ready || front_sensor.latest().valid;
}

bool HomeFrontSliderIfNeeded(FrontBalanceSlider& front_slider,
							 ILinearAxis& front_slider_axis,
							 ILimitSwitch& front_slider_home_switch)
{
	if (front_slider_axis.getAxisState().homed)
	{
		return true;
	}

	front_slider.home();
	if (front_slider_axis.getAxisState().homed)
	{
		return true;
	}

	if (!front_slider_home_switch.waitForTrigger(kSliderHomeTimeout))
	{
		front_slider_axis.stop();
		Logger::error(
			"Front slider homing timed out. Start the robot with the front slider retracted.");
		return false;
	}

	front_slider.home();
	if (front_slider_axis.getAxisState().homed)
	{
		return true;
	}

	Logger::error(
		"Front slider failed to home from lower limit only. Start the robot with the front slider retracted.");
	return false;
}

bool MoveAxisToStartupLimit(ILinearAxis& axis,
							ILimitSwitch& target_limit,
							const float command,
							const std::string& action_name,
							const int shutdown_fd)
{
	const auto initial_limit_state = target_limit.latestState();
	if (!initial_limit_state.valid)
	{
		Logger::error(action_name + " startup reset blocked because the target limit switch has no valid reading.");
		return false;
	}

	if (initial_limit_state.triggered)
	{
		axis.holdPosition();
		Logger::info(action_name + " already at startup limit.");
		return true;
	}

	Logger::info(action_name + " moving toward startup limit.");
	axis.moveNormalized(command);

	bool triggered = false;
	bool shutdown_seen = false;
	const auto deadline = SteadyClock::now() + kStartupResetTimeout;
	while (SteadyClock::now() < deadline)
	{
		triggered = target_limit.waitForTrigger(std::chrono::milliseconds(100)) ||
					target_limit.latestState().triggered;
		if (triggered)
		{
			break;
		}

		// Check for SIGINT/SIGTERM via the bringup signalfd (non-blocking poll).
		pollfd pfd{shutdown_fd, POLLIN, 0};
		if (poll(&pfd, 1, 0) > 0 && (pfd.revents & POLLIN) != 0)
		{
			signalfd_siginfo sig_info{};
			(void)read(shutdown_fd, &sig_info, sizeof(sig_info));
			Logger::warn(action_name + " startup reset interrupted by termination signal.");
			shutdown_seen = true;
			break;
		}
	}

	axis.holdPosition();

	if (shutdown_seen)
	{
		return false;
	}

	if (!triggered)
	{
		Logger::error(action_name + " startup reset timed out before the target limit switch triggered.");
		return false;
	}

	Logger::info(action_name + " startup limit reached.");
	return true;
}

bool ResetLiftModulesToStartupPose(ILinearAxis& first_lift_axis,
								   ILimitSwitch& first_lift_lower_limit,
								   ILinearAxis& rear_lift_axis,
								   ILimitSwitch& rear_lift_upper_limit,
								   const int shutdown_fd)
{
	Logger::info("Startup reset: moving lift modules to initial pose.");

	const bool front_lift_ready = MoveAxisToStartupLimit(
		first_lift_axis,
		first_lift_lower_limit,
		RobotConfig::Motion::BODY_LOWER_SPEED,
		"Lift-1/front lower",
		shutdown_fd);

	if (!front_lift_ready)
	{
		Logger::error("Startup reset stopped before moving Lift-2/rear because Lift-1/front did not reach its startup limit.");
		return false;
	}

	const bool rear_lift_ready = MoveAxisToStartupLimit(
		rear_lift_axis,
		rear_lift_upper_limit,
		RobotConfig::Motion::BODY_LIFT_SPEED,
		"Lift-2/rear upper",
		shutdown_fd);

	if (front_lift_ready && rear_lift_ready)
	{
		Logger::info("Startup reset complete: front lower and rear upper limits are active.");
		return true;
	}

	Logger::error("Startup reset failed. Check lift limit switch wiring and actuator direction.");
	return false;
}

int RunRobotController(IDriveSection& front_drive,
					   IDriveSection& middle_drive,
					   ILinearAxis& first_lift_axis,
					   ILinearAxis& front_slider_axis,
					   ILinearAxis& rear_slide_axis,
					   ILinearAxis& rear_lift_axis,
					   IFrontDistanceSensor& front_sensor,
					   IDownwardSensor& front_downward_sensor,
					   IDownwardSensor* middle_support_sensor,
					   IDownwardSensor* rear_support_sensor,
					   IImuSensor& imu_sensor,
					   ILimitSwitch& front_slider_home_switch)
{
	FrontBalanceSlider front_slider(front_slider_axis);
	if (RobotConfig::TestMode::BYPASS_FRONT_SLIDER)
	{
		front_slider_axis.stop();
		Logger::warn("Front slider is bypassed for full-vehicle bring-up because it is not installed.");
	}
	else
	{
		if (!HomeFrontSliderIfNeeded(front_slider, front_slider_axis, front_slider_home_switch))
		{
			return 1;
		}

		front_slider.updateExtension(kInitialFrontSliderExtensionM);
	}

	PoseMonitor pose_monitor(&imu_sensor);
	const auto initial_pose = imu_sensor.latestPose();
	if (initial_pose.valid)
	{
		pose_monitor.updatePose(initial_pose);
	}

	bool middle_drop_seen_during_transfer = false;
	const auto middle_support_confirmed = [middle_support_sensor, &middle_drop_seen_during_transfer]() {
		if (middle_support_sensor == nullptr)
		{
			return false;
		}

		const auto reading = middle_support_sensor->latest();
		if (!reading.valid)
		{
			return false;
		}

		if (reading.drop_detected)
		{
			middle_drop_seen_during_transfer = true;
		}

		return middle_drop_seen_during_transfer && reading.on_step_surface;
	};
	bool rear_drop_seen_during_transfer = false;
	const auto rear_support_confirmed = [rear_support_sensor, &rear_drop_seen_during_transfer]() {
		if (rear_support_sensor == nullptr)
		{
			return false;
		}

		const auto reading = rear_support_sensor->latest();
		if (!reading.valid)
		{
			return false;
		}

		if (reading.drop_detected)
		{
			rear_drop_seen_during_transfer = true;
		}

		return rear_drop_seen_during_transfer && reading.on_step_surface;
	};

	FrontSegment front_segment(
		front_drive,
		front_sensor,
		&first_lift_axis,
		[&front_downward_sensor]() {
			const auto reading = front_downward_sensor.latest();
			if (!reading.valid)
			{
				return false;
			}

			return reading.on_step_surface;
		});
	MiddleLiftModule middle_lift(first_lift_axis, middle_support_confirmed);
	MiddleDriveModule middle_drive_module(middle_drive, middle_support_confirmed);
	RearSupportModule rear_support_module(&rear_slide_axis, &rear_lift_axis, rear_support_confirmed);
	StepDetector step_detector(
		front_sensor,
		front_downward_sensor,
		middle_support_sensor,
		rear_support_sensor,
		[&pose_monitor]() { return pose_monitor.isSafe(); });
	SafetyManager safety_manager(&pose_monitor);
	ClimbingFsm climbing_fsm;
	MotionCoordinator motion_coordinator(
		front_segment,
		middle_lift,
		middle_drive_module,
		rear_support_module,
		&middle_drive,
		[&middle_drop_seen_during_transfer,
		 &rear_drop_seen_during_transfer](const MotionState state) {
			switch (state)
			{
			case MotionState::FrontDriveToMiddleLanding:
				middle_drop_seen_during_transfer = false;
				break;
			case MotionState::FinalDriveToRearLanding:
				rear_drop_seen_during_transfer = false;
				break;
			default:
				break;
			}
		});
	RobotController robot_controller(
		climbing_fsm,
		motion_coordinator,
		step_detector,
		pose_monitor,
		safety_manager);

	// Require N consecutive invalid readings before declaring a fault.
	// The HC-SR04 occasionally misses an echo return; a single invalid sample
	// should not abort the climb.
	int front_distance_invalid_streak = 0;
	safety_manager.addRule([&front_sensor, &front_distance_invalid_streak]() -> std::optional<SafetyStatus> {
		if (!front_sensor.latest().valid)
		{
			++front_distance_invalid_streak;
			if (front_distance_invalid_streak >=
				RobotConfig::Sensors::FRONT_DISTANCE_MAX_CONSECUTIVE_INVALID)
			{
				SafetyStatus status;
				status.level = SafetyLevel::Fault;
				status.fault = FaultCode::FrontDistanceTimeout;
				status.message =
					"Front distance sensor invalid for " +
					std::to_string(front_distance_invalid_streak) +
					" consecutive checks.";
				status.timestamp = SteadyClock::now();
				return status;
			}
			return std::nullopt;
		}

		front_distance_invalid_streak = 0;
		return std::nullopt;
	});

	safety_manager.addEmergencyStopHandler(
		[&front_segment, &middle_lift, &middle_drive_module, &rear_support_module]() {
			front_segment.stopFrontSegment();
			middle_lift.holdPosition();
			middle_drive_module.holdPosition();
			rear_support_module.stabilizeSupport();
		});

	if (!robot_controller.init())
	{
		Logger::error("Robot controller failed to initialise due to a safety fault.");
		return 1;
	}

	// Signals are already blocked process-wide (set in RunHardwareBringup).
	// Create a signalfd so the signal thread can receive them as plain file I/O
	// rather than polling a flag with sleep.
	sigset_t signal_mask;
	sigemptyset(&signal_mask);
	sigaddset(&signal_mask, SIGINT);
	sigaddset(&signal_mask, SIGTERM);
	const int signal_fd = signalfd(-1, &signal_mask, SFD_CLOEXEC);
	if (signal_fd < 0)
	{
		Logger::error("Failed to create controller signalfd: " + std::string(std::strerror(errno)));
		robot_controller.requestStop("Signal monitor unavailable.");
		return 1;
	}

	// A pipe lets the main thread wake the signal thread when the robot
	// finishes normally, so the signal thread does not block indefinitely.
	int done_pipe[2]{-1, -1};
	if (pipe(done_pipe) != 0)
	{
		Logger::error("Failed to create controller shutdown pipe: " + std::string(std::strerror(errno)));
		close(signal_fd);
		robot_controller.requestStop("Signal monitor pipe unavailable.");
		return 1;
	}

	std::thread signal_monitor([&robot_controller, signal_fd, done_pipe]() {
		pollfd pfds[2]{};
		pfds[0].fd     = signal_fd;
		pfds[0].events = POLLIN;
		pfds[1].fd     = done_pipe[0];	// read end: written when robot finishes
		pfds[1].events = POLLIN;

		// Block here with no sleep until a signal or the done pipe fires.
		if (poll(pfds, 2, -1) > 0 && (pfds[0].revents & POLLIN) != 0)
		{
			signalfd_siginfo sig_info{};
			(void)read(signal_fd, &sig_info, sizeof(sig_info));
			Logger::warn("Termination signal received. Stopping all actuators.");
			robot_controller.requestStop("Termination signal received.");
		}

		close(signal_fd);
	});

	const auto robot_state = robot_controller.waitUntilFinished();

	// Wake the signal thread so it exits cleanly.
	const char wake_byte = 0;
	(void)write(done_pipe[1], &wake_byte, 1);
	close(done_pipe[1]);

	if (signal_monitor.joinable())
	{
		signal_monitor.join();
	}
	close(done_pipe[0]);

	if (robot_state.motion_state == MotionState::Completed)
	{
		Logger::info("Climbing cycle completed successfully.");
		return 0;
	}

	Logger::error("Robot controller entered fault state: " + robot_state.safety_status.message);
	return 1;
}

int RunHardwareBringup()
{
	Logger::info("Raspberry Pi Linux hardware mode starting.");

	// Block SIGINT and SIGTERM in the main thread (and all threads it spawns)
	// before any threads are created, so the signal mask is inherited.
	sigset_t signal_mask;
	sigemptyset(&signal_mask);
	sigaddset(&signal_mask, SIGINT);
	sigaddset(&signal_mask, SIGTERM);
	const int sigmask_result = pthread_sigmask(SIG_BLOCK, &signal_mask, nullptr);
	if (sigmask_result != 0)
	{
		Logger::error("Failed to block SIGINT/SIGTERM for event-driven signal handling: " + std::string(std::strerror(sigmask_result)));
		return 1;
	}

	// Create a signalfd now so the startup reset phase can detect Ctrl+C.
	// RunRobotController() creates its own signalfd after this one is closed.
	const int bringup_signal_fd = signalfd(-1, &signal_mask, SFD_CLOEXEC);
	if (bringup_signal_fd < 0)
	{
		Logger::error("Failed to create bring-up signalfd: " + std::string(std::strerror(errno)));
		return 1;
	}

	auto mcp23017 = std::make_shared<Mcp23017Driver>();

	// failIfStartFailed: log an error and set the abort flag (used for hardware
	// whose absence would either crash the robot or silently produce wrong behaviour).
	bool bringup_ok = true;
	const auto failIfStartFailed = [&bringup_ok](const std::string& device_name,
												  const bool started,
												  const std::string& hint) {
		if (!started)
		{
			Logger::error(device_name + " failed to start — this is required hardware. " + hint);
			bringup_ok = false;
		}
	};

	failIfStartFailed(
		"MCP23017 expander",
		mcp23017->start(),
		"Check SDA/SCL/VCC/GND wiring, RESET tied high, and address 0x20.");

	MotorDriver front_drive(
		"front",
		RobotConfig::MotorGPIO::FRONT_L_IN1,
		RobotConfig::MotorGPIO::FRONT_L_IN2,
		RobotConfig::MotorGPIO::FRONT_R_IN3,
		RobotConfig::MotorGPIO::FRONT_R_IN4);

	MotorDriver middle_drive(
		"middle",
		RobotConfig::MotorGPIO::MIDDLE_L_IN1,
		RobotConfig::MotorGPIO::MIDDLE_L_IN2,
		RobotConfig::MotorGPIO::MIDDLE_R_IN3,
		RobotConfig::MotorGPIO::MIDDLE_R_IN4);

	failIfStartFailed(
		"Front drive",
		front_drive.start(),
		"Check front DRV8833 GPIO wiring: BCM17/18/27/22 -> IN1/IN2/IN3/IN4, and ensure EEP/nSLEEP is high.");
	failIfStartFailed(
		"Middle drive",
		middle_drive.start(),
		"Check middle DRV8833 GPIO wiring: BCM23/24/25/8 -> IN1/IN2/IN3/IN4, and ensure EEP/nSLEEP is high.");

	FrontDistanceSensor front_sensor;
	const bool front_sensor_started = front_sensor.start();
	failIfStartFailed("Front ultrasonic sensor", front_sensor_started, "Check TRIG/ECHO wiring and HC-SR04 level shifting on ECHO.");
	if (front_sensor_started &&
		!WaitForValidFrontDistanceSample(
			front_sensor,
			std::chrono::milliseconds(RobotConfig::Sensors::FRONT_DISTANCE_STARTUP_TIMEOUT_MS)))
	{
		Logger::warn("Front ultrasonic sensor did not produce a valid sample during startup.");
	}

	DownwardSensor front_downward_sensor(
		RobotConfig::Platform::GPIO_CHIP,
		RobotConfig::GPIO::FRONT_DOWNWARD_DO,
		RobotConfig::Sensors::FRONT_DOWNWARD_ACTIVE_ON_SURFACE);
	DownwardSensor middle_support_sensor(
		RobotConfig::Platform::GPIO_CHIP,
		RobotConfig::GPIO::MIDDLE_SUPPORT_DO,
		RobotConfig::Sensors::MIDDLE_SUPPORT_ACTIVE_ON_SURFACE);
	DownwardSensor rear_support_sensor(
		RobotConfig::Platform::GPIO_CHIP,
		RobotConfig::GPIO::REAR_SUPPORT_DO,
		RobotConfig::Sensors::REAR_SUPPORT_ACTIVE_ON_SURFACE);
	failIfStartFailed("Front downward edge sensor", front_downward_sensor.start(), "Check TCRT5000 DO wiring to BCM4 (physical pin 7).");
	failIfStartFailed("Middle support sensor", middle_support_sensor.start(), "Check TCRT5000 DO wiring to BCM5 (physical pin 29).");
	failIfStartFailed("Rear support sensor", rear_support_sensor.start(), "Check TCRT5000 DO wiring to BCM21 (physical pin 40).");

	Mcp23017LimitSwitch lift1_upper_limit(
		mcp23017,
		RobotConfig::MCP23017::LIFT_1_UPPER_LIMIT,
		LimitRole::Upper,
		RobotConfig::Limits::ACTIVE_LOW);
	Mcp23017LimitSwitch lift1_lower_limit(
		mcp23017,
		RobotConfig::MCP23017::LIFT_1_LOWER_LIMIT,
		LimitRole::Lower,
		RobotConfig::Limits::ACTIVE_LOW);
	Mcp23017LimitSwitch slide1_upper_limit(
		mcp23017,
		RobotConfig::MCP23017::SLIDE_1_UPPER_LIMIT,
		LimitRole::Upper,
		RobotConfig::Limits::ACTIVE_LOW);
	Mcp23017LimitSwitch slide1_lower_limit(
		mcp23017,
		RobotConfig::MCP23017::SLIDE_1_LOWER_LIMIT,
		LimitRole::Lower,
		RobotConfig::Limits::ACTIVE_LOW);
	Mcp23017LimitSwitch slide2_upper_limit(
		mcp23017,
		RobotConfig::MCP23017::SLIDE_2_UPPER_LIMIT,
		LimitRole::Upper,
		RobotConfig::Limits::ACTIVE_LOW);
	Mcp23017LimitSwitch slide2_lower_limit(
		mcp23017,
		RobotConfig::MCP23017::SLIDE_2_LOWER_LIMIT,
		LimitRole::Lower,
		RobotConfig::Limits::ACTIVE_LOW);
	Mcp23017LimitSwitch lift2_upper_limit(
		mcp23017,
		RobotConfig::MCP23017::LIFT_2_UPPER_LIMIT,
		LimitRole::Upper,
		RobotConfig::Limits::ACTIVE_LOW);
	Mcp23017LimitSwitch lift2_lower_limit(
		mcp23017,
		RobotConfig::MCP23017::LIFT_2_LOWER_LIMIT,
		LimitRole::Lower,
		RobotConfig::Limits::ACTIVE_LOW);

	failIfStartFailed("Lift-1 upper limit", lift1_upper_limit.start(), "Check MCP23017 GPA3 and active-low limit wiring.");
	failIfStartFailed("Lift-1 lower limit", lift1_lower_limit.start(), "Check MCP23017 GPA4 and active-low limit wiring.");
	if (RobotConfig::TestMode::BYPASS_FRONT_SLIDER)
	{
		Logger::warn("Front slider limit switches are bypassed because the front slider is not installed.");
	}
	else
	{
		failIfStartFailed("Slide-1 upper limit", slide1_upper_limit.start(), "Check MCP23017 GPA5 and active-low limit wiring.");
		failIfStartFailed("Slide-1 lower limit", slide1_lower_limit.start(), "Check MCP23017 GPA6 and active-low limit wiring.");
	}
	failIfStartFailed("Slide-2 upper limit", slide2_upper_limit.start(), "Check MCP23017 GPA7 and active-low limit wiring.");
	failIfStartFailed("Slide-2 lower limit", slide2_lower_limit.start(), "Check MCP23017 GPB0 and active-low limit wiring.");
	failIfStartFailed("Lift-2 upper limit", lift2_upper_limit.start(), "Check MCP23017 GPB1 and active-low limit wiring.");
	failIfStartFailed("Lift-2 lower limit", lift2_lower_limit.start(), "Check MCP23017 GPB2 and active-low limit wiring.");

	LinearActuator first_lift_axis(
		RobotConfig::MotorGPIO::LIFT_1_IN1_IN3,
		RobotConfig::MotorGPIO::LIFT_1_IN2_IN4,
		&lift1_upper_limit,
		&lift1_lower_limit,
		RobotConfig::Geometry::BODY_LIFT_MAX_TRAVEL_M);
	LinearActuator front_slider_axis(
		RobotConfig::MotorGPIO::SLIDE_1_IN1_IN3,
		RobotConfig::MotorGPIO::SLIDE_1_IN2_IN4,
		&slide1_upper_limit,
		&slide1_lower_limit,
		RobotConfig::Geometry::SLIDER_MAX_TRAVEL_M);
	LinearActuator rear_slide_axis(
		RobotConfig::MotorGPIO::SLIDE_2_IN1_IN3,
		RobotConfig::MotorGPIO::SLIDE_2_IN2_IN4,
		&slide2_upper_limit,
		&slide2_lower_limit,
		RobotConfig::Geometry::SLIDER_MAX_TRAVEL_M);
	LinearActuator rear_lift_axis(
		RobotConfig::MotorGPIO::LIFT_2_IN1_IN3,
		RobotConfig::MotorGPIO::LIFT_2_IN2_IN4,
		&lift2_upper_limit,
		&lift2_lower_limit,
		RobotConfig::Geometry::BODY_LIFT_MAX_TRAVEL_M);

	failIfStartFailed(
		"Lift-1 actuator",
		first_lift_axis.start(),
		"Check DRV8833 #1 direct GPIO wiring: BCM10 -> IN1+IN3 and BCM9 -> IN2+IN4.");
	if (RobotConfig::TestMode::BYPASS_FRONT_SLIDER)
	{
		Logger::warn("Slide-1 actuator GPIO outputs are bypassed because the front slider is not installed.");
	}
	else
	{
		failIfStartFailed(
			"Slide-1 actuator",
			front_slider_axis.start(),
			"Check DRV8833 #3 direct GPIO wiring: BCM5 -> IN1+IN3 and BCM6 -> IN2+IN4.");
	}
	failIfStartFailed(
		"Slide-2 actuator",
		rear_slide_axis.start(),
		"Check DRV8833 #4 direct GPIO wiring: BCM12 -> IN1+IN3 and BCM13 -> IN2+IN4.");
	failIfStartFailed(
		"Lift-2 actuator",
		rear_lift_axis.start(),
		"Check DRV8833 #2 direct GPIO wiring: BCM11 -> IN1+IN3 and BCM7 -> IN2+IN4.");

	// Abort bringup if any required hardware failed to start rather than
	// continuing into the startup reset where failures would only manifest
	// as a 30-second timeout with a confusing error message.
	if (!bringup_ok)
	{
		Logger::error("One or more required hardware components failed to start — aborting bringup.");
		close(bringup_signal_fd);
		return 1;
	}

	// Start the MCP23017 interrupt thread before any actuator motion.
	// MCP23017-backed limit switches only push state updates via this thread.
	// ResetLiftModulesToStartupPose() relies on waitForTrigger() to detect
	// limit-switch events, so if the thread is not running those calls will
	// block until the startup timeout fires.
	if (!mcp23017->startInterrupts(RobotConfig::GPIO::MCP23017_INTA, RobotConfig::GPIO::MCP23017_INTB))
	{
		Logger::error(
			"MCP23017 interrupt thread failed to start. "
			"Check INTA on BCM19 (physical pin 35) and INTB on BCM20 (physical pin 38). "
			"All MCP23017 limit switches are unavailable — aborting.");
		close(bringup_signal_fd);
		return 1;
	}

	front_drive.stop();
	middle_drive.stop();
	rear_slide_axis.stop();
	if (!ResetLiftModulesToStartupPose(
			first_lift_axis,
			lift1_lower_limit,
			rear_lift_axis,
			lift2_upper_limit,
			bringup_signal_fd))
	{
		close(bringup_signal_fd);
		return 1;
	}

	std::unique_ptr<IImuSensor> imu_sensor;
	if (RobotConfig::TestMode::BYPASS_IMU)
	{
		Logger::warn("IMU is bypassed for full-vehicle bring-up because it is not installed.");
		imu_sensor = std::make_unique<FixedPoseImuSensor>();
	}
	else
	{
		auto hardware_imu = std::make_unique<ImuSensor>();
		if (hardware_imu->start() &&
			WaitForValidImuSample(*hardware_imu, std::chrono::milliseconds(RobotConfig::Sensors::SENSOR_STALE_MS)))
		{
			Logger::info("Hardware IMU online.");
			imu_sensor = std::move(hardware_imu);
		}
		else
		{
			if (hardware_imu)
			{
				hardware_imu->stop();
			}

			Logger::warn("IMU is optional. Falling back to a fixed neutral pose sensor.");
			imu_sensor = std::make_unique<FixedPoseImuSensor>();
		}
	}

	// Close the bringup signalfd before RunRobotController() creates its own.
	// Any unread signal stays pending in the kernel queue and will be delivered
	// to the new signalfd that RunRobotController() opens.
	close(bringup_signal_fd);

	return RunRobotController(
		front_drive,
		middle_drive,
		first_lift_axis,
		front_slider_axis,
		rear_slide_axis,
		rear_lift_axis,
		front_sensor,
		front_downward_sensor,
		&middle_support_sensor,
		&rear_support_sensor,
		*imu_sensor,
		slide1_lower_limit);
}
}

int main()
{
	Logger::info("Climbing robot Linux hardware stack starting.");
	return RunHardwareBringup();
}
