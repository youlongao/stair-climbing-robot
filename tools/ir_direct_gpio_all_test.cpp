#include <chrono>
#include <cstdlib>
#include <iostream>
#include <string>
#include <thread>

#include "config.h"
#include "downward_sensor.h"

namespace
{
void PrintUsage(const char* program_name)
{
	std::cout
		<< "Usage:\n  " << program_name << " [samples] [poll_ms]\n\n"
		<< "Default:\n"
		<< "  samples = 200\n"
		<< "  poll_ms = 100\n\n"
		<< "Direct Raspberry Pi GPIO mapping used by the main robot:\n"
		<< "  front  DO -> BCM" << RobotConfig::GPIO::FRONT_DOWNWARD_DO << " physical pin 7\n"
		<< "  middle DO -> BCM" << RobotConfig::GPIO::MIDDLE_SUPPORT_DO << " physical pin 29\n"
		<< "  rear   DO -> BCM" << RobotConfig::GPIO::REAR_SUPPORT_DO << " physical pin 40\n\n"
		<< "Expected meaning:\n"
		<< "  surface=1 means surface detected.\n"
		<< "  drop=1 means no surface / edge detected.\n";
}

void PrintReading(const char* name, const Robot::DownwardReading& reading)
{
	std::cout << name
			  << "{valid=" << (reading.valid ? 1 : 0)
			  << ", surface=" << (reading.on_step_surface ? 1 : 0)
			  << ", drop=" << (reading.drop_detected ? 1 : 0)
			  << ", edge=" << (reading.edge_detected ? 1 : 0)
			  << "} ";
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

	const int samples = (argc >= 2) ? std::atoi(argv[1]) : 200;
	const int poll_ms = (argc >= 3) ? std::atoi(argv[2]) : 100;
	if (samples <= 0 || poll_ms <= 0)
	{
		std::cerr << "samples and poll_ms must be positive integers.\n";
		PrintUsage(argv[0]);
		return 1;
	}

	Robot::DownwardSensor front_sensor(
		RobotConfig::Platform::GPIO_CHIP,
		RobotConfig::GPIO::FRONT_DOWNWARD_DO,
		RobotConfig::Sensors::FRONT_DOWNWARD_ACTIVE_ON_SURFACE);
	Robot::DownwardSensor middle_sensor(
		RobotConfig::Platform::GPIO_CHIP,
		RobotConfig::GPIO::MIDDLE_SUPPORT_DO,
		RobotConfig::Sensors::MIDDLE_SUPPORT_ACTIVE_ON_SURFACE);
	Robot::DownwardSensor rear_sensor(
		RobotConfig::Platform::GPIO_CHIP,
		RobotConfig::GPIO::REAR_SUPPORT_DO,
		RobotConfig::Sensors::REAR_SUPPORT_ACTIVE_ON_SURFACE);

	if (!front_sensor.start() || !middle_sensor.start() || !rear_sensor.start())
	{
		std::cerr << "Failed to start one or more direct GPIO downward sensors.\n";
		std::cerr << "Check TCRT5000 DO wiring, 3.3V logic level, and common ground.\n";
		return 1;
	}

	PrintUsage(argv[0]);
	std::cout << "\nStarting direct GPIO TCRT5000 all-sensor test.\n";

	for (int sample = 1; sample <= samples; ++sample)
	{
		std::cout << "[sample " << sample << "] ";
		PrintReading("front", front_sensor.latest());
		PrintReading("middle", middle_sensor.latest());
		PrintReading("rear", rear_sensor.latest());
		std::cout << '\n';
		std::this_thread::sleep_for(std::chrono::milliseconds(poll_ms));
	}

	std::cout << "Direct GPIO TCRT5000 all-sensor test complete.\n";
	return 0;
}
