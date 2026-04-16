#include <chrono>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <string>
#include <thread>

#include "config.h"
#include "mcp23017_downward_sensor.h"
#include "mcp23017_driver.h"

using namespace Robot;

namespace
{
struct SensorRig
{
	const char* name;
	std::uint8_t pin;
	bool active_on_surface;
};

void PrintUsage(const char* program_name)
{
	std::cout
		<< "Usage:\n  " << program_name
		<< " [duration_s] [poll_ms]\n\n"
		<< "Default:\n"
		<< "  duration_s = 30\n"
		<< "  poll_ms    = 250\n\n"
		<< "Default MCP23017 mapping:\n"
		<< "  front  = GPA0 / pin " << static_cast<int>(RobotConfig::MCP23017::FRONT_DOWNWARD_DO) << "\n"
		<< "  middle = GPA1 / pin " << static_cast<int>(RobotConfig::MCP23017::MIDDLE_SUPPORT_DO) << "\n"
		<< "  rear   = GPA2 / pin " << static_cast<int>(RobotConfig::MCP23017::REAR_SUPPORT_DO) << "\n\n"
		<< "This test reads all three TCRT5000 DO signals through MCP23017.\n";
}

void PrintReading(const char* label, const DownwardReading& reading)
{
	std::cout
		<< label
		<< "{valid=" << (reading.valid ? 1 : 0)
		<< ",surface=" << (reading.on_step_surface ? 1 : 0)
		<< ",drop=" << (reading.drop_detected ? 1 : 0)
		<< ",edge=" << (reading.edge_detected ? 1 : 0)
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

	const int duration_s = (argc >= 2) ? std::atoi(argv[1]) : 30;
	const int poll_ms = (argc >= 3) ? std::atoi(argv[2]) : 250;

	if (duration_s <= 0 || poll_ms <= 0)
	{
		std::cerr << "duration_s and poll_ms must be positive integers.\n";
		PrintUsage(argv[0]);
		return 1;
	}

	auto mcp23017 = std::make_shared<Mcp23017Driver>();
	if (!mcp23017->start())
	{
		std::cerr << "Failed to start MCP23017. Check SDA/SCL/VDD/VSS wiring, RESET high, and address 0x20.\n";
		return 1;
	}

	const SensorRig front_rig{
		"front",
		RobotConfig::MCP23017::FRONT_DOWNWARD_DO,
		RobotConfig::Sensors::FRONT_DOWNWARD_ACTIVE_ON_SURFACE};
	const SensorRig middle_rig{
		"middle",
		RobotConfig::MCP23017::MIDDLE_SUPPORT_DO,
		RobotConfig::Sensors::MIDDLE_SUPPORT_ACTIVE_ON_SURFACE};
	const SensorRig rear_rig{
		"rear",
		RobotConfig::MCP23017::REAR_SUPPORT_DO,
		RobotConfig::Sensors::REAR_SUPPORT_ACTIVE_ON_SURFACE};

	Mcp23017DownwardSensor front_sensor(mcp23017, front_rig.pin, front_rig.active_on_surface);
	Mcp23017DownwardSensor middle_sensor(mcp23017, middle_rig.pin, middle_rig.active_on_surface);
	Mcp23017DownwardSensor rear_sensor(mcp23017, rear_rig.pin, rear_rig.active_on_surface);

	if (!front_sensor.start() || !middle_sensor.start() || !rear_sensor.start())
	{
		std::cerr << "Failed to start one or more TCRT5000 sensors on MCP23017.\n";
		return 1;
	}

	std::cout << "Starting all TCRT5000 MCP23017 test\n";
	std::cout << "duration_s = " << duration_s << '\n';
	std::cout << "poll_ms    = " << poll_ms << '\n';
	std::cout << "front_active_on_surface  = " << (front_rig.active_on_surface ? 1 : 0) << '\n';
	std::cout << "middle_active_on_surface = " << (middle_rig.active_on_surface ? 1 : 0) << '\n';
	std::cout << "rear_active_on_surface   = " << (rear_rig.active_on_surface ? 1 : 0) << '\n';

	const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(duration_s);
	while (std::chrono::steady_clock::now() < deadline)
	{
		PrintReading("front", front_sensor.latest());
		PrintReading("middle", middle_sensor.latest());
		PrintReading("rear", rear_sensor.latest());
		std::cout << '\n';

		std::this_thread::sleep_for(std::chrono::milliseconds(poll_ms));
	}

	front_sensor.stop();
	middle_sensor.stop();
	rear_sensor.stop();

	std::cout << "All TCRT5000 MCP23017 test complete.\n";
	return 0;
}
