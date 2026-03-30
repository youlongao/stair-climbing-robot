#include "robot_controller.h"

#include <cmath>

#include "logger.h"

namespace Robot
{
namespace
{
const char* MotionStateToString(const MotionState state)
{
	switch (state)
	{
	case MotionState::Idle:
		return "Idle";
	case MotionState::ApproachingStep:
		return "ApproachingStep";
	case MotionState::FrontClimb:
		return "FrontClimb";
	case MotionState::MiddleTransfer:
		return "MiddleTransfer";
	case MotionState::RearTransfer:
		return "RearTransfer";
	case MotionState::Completed:
		return "Completed";
	case MotionState::Fault:
	default:
		return "Fault";
	}
}
}

RobotController::RobotController(ClimbingFsm& climbing_fsm,
								 MotionCoordinator& motion_coordinator,
								 StepDetector& step_detector,
								 PoseMonitor& pose_monitor,
								 SafetyManager& safety_manager)
	: climbing_fsm_(climbing_fsm),
	  motion_coordinator_(motion_coordinator),
	  step_detector_(step_detector),
	  pose_monitor_(pose_monitor),
	  safety_manager_(safety_manager)
{
}

bool RobotController::init()
{
	motion_coordinator_.resetPhases();
	climbing_fsm_.transitionTo(MotionState::Idle);

	const auto safety_status = safety_manager_.checkAllSafetyConditions();
	{
		std::lock_guard<std::mutex> lock(mutex_);
		robot_state_.motion_state = climbing_fsm_.getCurrentState();
		robot_state_.step_assessment = step_detector_.latestAssessment();
		robot_state_.safety_status = safety_status;
		robot_state_.sensor_data.pose = pose_monitor_.currentPose();
		robot_state_.timestamp = SteadyClock::now();
	}

	if (safety_status.level == SafetyLevel::Fault)
	{
		stopAll();
		state_cv_.notify_all();
		return false;
	}

	bindEventSources();
	update();
	return true;
}

void RobotController::update()
{
	std::lock_guard<std::mutex> lock(mutex_);

	const auto current_state = climbing_fsm_.getCurrentState();
	const auto safety_status = safety_manager_.checkAllSafetyConditions();
	const auto step_assessment = step_detector_.detectStepEdge();
	const auto pose = pose_monitor_.currentPose();

	if (current_state == MotionState::Completed || current_state == MotionState::Fault)
	{
		robot_state_.motion_state = current_state;
		robot_state_.step_assessment = step_assessment;
		robot_state_.safety_status = safety_status;
		robot_state_.sensor_data.pose = pose;
		robot_state_.sensor_data.front_distance.distance_m = step_assessment.front_face_distance_m;
		robot_state_.sensor_data.front_distance.valid = !std::isnan(step_assessment.front_face_distance_m);
		robot_state_.sensor_data.front_distance.timestamp = step_assessment.timestamp;
		robot_state_.timestamp = SteadyClock::now();
		state_cv_.notify_all();
		return;
	}

	bool front_phase_complete = false;
	bool middle_transfer_phase_complete = false;
	bool rear_transfer_phase_complete = false;

	switch (current_state)
	{
	case MotionState::ApproachingStep:
	case MotionState::FrontClimb:
		front_phase_complete = motion_coordinator_.executeFrontPhase(current_state);
		break;
	case MotionState::MiddleTransfer:
		middle_transfer_phase_complete = motion_coordinator_.executeMiddleTransferPhase(current_state);
		break;
	case MotionState::RearTransfer:
		rear_transfer_phase_complete = motion_coordinator_.executeRearTransferPhase(current_state);
		break;
	case MotionState::Completed:
	case MotionState::Fault:
		stopAll();
		break;
	case MotionState::Idle:
	default:
		break;
	}

	const auto next_state = climbing_fsm_.updateState(
		step_assessment,
		safety_status,
		front_phase_complete,
		middle_transfer_phase_complete,
		rear_transfer_phase_complete);
	const bool state_changed = climbing_fsm_.transitionTo(next_state);

	if (state_changed)
	{
		Logger::info(std::string("Controller motion state = ") + MotionStateToString(next_state));
	}

	RobotState snapshot;
	snapshot.motion_state = climbing_fsm_.getCurrentState();
	snapshot.step_assessment = step_assessment;
	snapshot.safety_status = safety_status;
	snapshot.sensor_data.pose = pose;
	snapshot.sensor_data.front_distance.distance_m = step_assessment.front_face_distance_m;
	snapshot.sensor_data.front_distance.valid = !std::isnan(step_assessment.front_face_distance_m);
	snapshot.sensor_data.front_distance.timestamp = step_assessment.timestamp;
	snapshot.timestamp = SteadyClock::now();
	robot_state_ = snapshot;

	if (next_state == MotionState::Completed || next_state == MotionState::Fault)
	{
		stopAll();
		state_cv_.notify_all();
	}
}

RobotState RobotController::waitUntilFinished()
{
	std::unique_lock<std::mutex> lock(mutex_);
	state_cv_.wait(lock, [this]() {
		return robot_state_.motion_state == MotionState::Completed ||
			   robot_state_.motion_state == MotionState::Fault;
	});

	return robot_state_;
}

void RobotController::stopAll()
{
	motion_coordinator_.stopAll();
}

void RobotController::resetSystem()
{
	safety_manager_.clearFault();
	motion_coordinator_.resetPhases();
	climbing_fsm_.transitionTo(MotionState::Idle);

	std::lock_guard<std::mutex> lock(mutex_);
	robot_state_ = {};
}

RobotState RobotController::state() const
{
	std::lock_guard<std::mutex> lock(mutex_);
	return robot_state_;
}

void RobotController::bindEventSources()
{
	if (callbacks_bound_)
	{
		return;
	}

	step_detector_.setUpdateCallback([this]() {
		update();
	});
	pose_monitor_.setUpdateCallback([this](const PoseData&) {
		update();
	});
	callbacks_bound_ = true;
}
}
