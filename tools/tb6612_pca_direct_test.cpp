#include <chrono>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <string>
#include <thread>

#include "config.h"
#include "logger.h"
#include "motor_driver.h"
#include "pca9685_driver.h"

using namespace Robot;

namespace
{
enum class TestTarget
{
	Front,
	Middle,
	Both
};

TestTarget ParseTarget(const std::string& value)
{
	if (value == "front")
	{
		return TestTarget::Front;
	}

	if (value == "middle")
	{
		return TestTarget::Middle;
	}

	return TestTarget::Both;
}

void PrintUsage(const char* program_name)
{
	std::cout
		<< "Usage:\n  " << program_name
		<< " [front|middle|both] [speed] [forward_ms] [reverse_ms] [cycles]\n\n"
		<< "Default:\n"
		<< "  target     = both\n"
		<< "  speed      = 0.35\n"
		<< "  forward_ms = 1500\n"
		<< "  reverse_ms = 1500\n"
		<< "  cycles     = 3\n\n"
		<< "Hardware assumptions:\n"
		<< "  PCA9685 CH0/CH1 -> front wheel PWMA/PWMB\n"
		<< "  PCA9685 CH2/CH3 -> middle wheel PWMA/PWMB\n"
		<< "  Front TB6612: AIN1/AIN2=GPIO17/GPIO18, BIN1/BIN2=GPIO27/GPIO22\n"
		<< "  Middle TB6612: AIN1/AIN2=GPIO23/GPIO24, BIN1/BIN2=GPIO25/GPIO8\n"
		<< "  TB6612 STBY must be tied high\n"
		<< "  Motor supply, PCA9685, and Raspberry Pi must share ground\n";
}

void RunForward(MotorDriver& driver, const float speed)
{
	driver.setNormalizedSpeed(speed, speed);
}

void RunReverse(MotorDriver& driver, const float speed)
{
	driver.setNormalizedSpeed(-speed, -speed);
}

void StopDriver(MotorDriver& driver)
{
	driver.stop();
}
}

int main(int argc, char* argv[])
{
	if (argc >= 2)
	{
		const std::string first_arg = argv[1];
		if (first_arg == "-h" || first_arg == "--help")
		{
			PrintUsage(argv[0]);
			return 0;
		}
	}

	const TestTarget target = (argc >= 2) ? ParseTarget(argv[1]) : TestTarget::Both;
	const float speed = (argc >= 3) ? std::stof(argv[2]) : 0.35F;
	const int forward_ms = (argc >= 4) ? std::atoi(argv[3]) : 1500;
	const int reverse_ms = (argc >= 5) ? std::atoi(argv[4]) : 1500;
	const int cycles = (argc >= 6) ? std::atoi(argv[5]) : 3;

	if (speed <= 0.0F || speed > 1.0F || forward_ms <= 0 || reverse_ms <= 0 || cycles <= 0)
	{
		std::cerr << "Invalid arguments. speed must be in (0, 1], and all durations/cycles must be positive.\n";
		PrintUsage(argv[0]);
		return 1;
	}

	auto pwm_driver = std::make_shared<Pca9685Driver>();
	if (!pwm_driver->start())
	{
		std::cerr << "Failed to start PCA9685. Check I2C wiring, address 0x40, and power.\n";
		return 1;
	}

	MotorDriver front_drive(
		"front-test",
		pwm_driver,
		RobotConfig::PWM_Channels::FRONT_Wheel_L,
		RobotConfig::PWM_Channels::FRONT_Wheel_R,
		RobotConfig::GPIO::FRONT_L_IN1,
		RobotConfig::GPIO::FRONT_L_IN2,
		RobotConfig::GPIO::FRONT_R_IN1,
		RobotConfig::GPIO::FRONT_R_IN2);

	MotorDriver middle_drive(
		"middle-test",
		pwm_driver,
		RobotConfig::PWM_Channels::MIDDLE_Wheel_L,
		RobotConfig::PWM_Channels::MIDDLE_Wheel_R,
		RobotConfig::GPIO::MID_L_IN1,
		RobotConfig::GPIO::MID_L_IN2,
		RobotConfig::GPIO::MID_R_IN1,
		RobotConfig::GPIO::MID_R_IN2);

	if ((target == TestTarget::Front || target == TestTarget::Both) && !front_drive.start())
	{
		std::cerr << "Failed to start front TB6612 driver. Check GPIO17/18/27/22 and CH0/CH1.\n";
		return 1;
	}

	if ((target == TestTarget::Middle || target == TestTarget::Both) && !middle_drive.start())
	{
		std::cerr << "Failed to start middle TB6612 driver. Check GPIO23/24/25/8 and CH2/CH3.\n";
		return 1;
	}

	auto run_target = [&](MotorDriver& driver, const std::string& name) {
		for (int cycle = 1; cycle <= cycles; ++cycle)
		{
			std::cout << "[" << name << "] cycle " << cycle << " forward\n";
			RunForward(driver, speed);
			std::this_thread::sleep_for(std::chrono::milliseconds(forward_ms));

			std::cout << "[" << name << "] cycle " << cycle << " stop\n";
			StopDriver(driver);
			std::this_thread::sleep_for(std::chrono::milliseconds(500));

			std::cout << "[" << name << "] cycle " << cycle << " reverse\n";
			RunReverse(driver, speed);
			std::this_thread::sleep_for(std::chrono::milliseconds(reverse_ms));

			std::cout << "[" << name << "] cycle " << cycle << " stop\n";
			StopDriver(driver);
			std::this_thread::sleep_for(std::chrono::milliseconds(500));
		}
	};

	switch (target)
	{
	case TestTarget::Front:
		run_target(front_drive, "front");
		break;
	case TestTarget::Middle:
		run_target(middle_drive, "middle");
		break;
	case TestTarget::Both:
	default:
		for (int cycle = 1; cycle <= cycles; ++cycle)
		{
			std::cout << "[both] cycle " << cycle << " forward\n";
			RunForward(front_drive, speed);
			RunForward(middle_drive, speed);
			std::this_thread::sleep_for(std::chrono::milliseconds(forward_ms));

			std::cout << "[both] cycle " << cycle << " stop\n";
			StopDriver(front_drive);
			StopDriver(middle_drive);
			std::this_thread::sleep_for(std::chrono::milliseconds(500));

			std::cout << "[both] cycle " << cycle << " reverse\n";
			RunReverse(front_drive, speed);
			RunReverse(middle_drive, speed);
			std::this_thread::sleep_for(std::chrono::milliseconds(reverse_ms));

			std::cout << "[both] cycle " << cycle << " stop\n";
			StopDriver(front_drive);
			StopDriver(middle_drive);
			std::this_thread::sleep_for(std::chrono::milliseconds(500));
		}
		break;
	}

	front_drive.stop();
	middle_drive.stop();
	std::cout << "TB6612 + PCA9685 wheel test complete.\n";
	return 0;
}
