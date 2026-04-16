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
void PrintUsage(const char* program_name)
{
	std::cout
		<< "Usage:\n  " << program_name
		<< " [mcp_pin] [duration_s] [active_on_surface(0|1)]\n\n"
		<< "Default:\n"
		<< "  mcp_pin           = " << static_cast<int>(RobotConfig::MCP23017::FRONT_DOWNWARD_DO) << " (front TCRT5000 DO)\n"
		<< "  duration_s        = 30\n"
		<< "  active_on_surface = " << (RobotConfig::Sensors::FRONT_DOWNWARD_ACTIVE_ON_SURFACE ? 1 : 0) << "\n\n"
		<< "Recommended wiring:\n"
		<< "  Raspberry Pi 3.3V -> MCP23017 VDD and TCRT5000 VCC\n"
		<< "  Raspberry Pi GND  -> MCP23017 VSS and TCRT5000 GND\n"
		<< "  Raspberry Pi SDA  -> MCP23017 SDA\n"
		<< "  Raspberry Pi SCL  -> MCP23017 SCL\n"
		<< "  MCP23017 RESET    -> 3.3V\n"
		<< "  MCP23017 A0/A1/A2 -> GND (address 0x20)\n"
		<< "  Front TCRT5000 DO -> MCP23017 GPA0 (default)\n"
		<< "  Front TCRT5000 AO -> not connected\n\n"
		<< "This test prints the interpreted front downward state from the MCP23017-backed sensor.\n";
}

void PrintReading(const DownwardReading& reading)
{
	std::cout
		<< "valid=" << (reading.valid ? 1 : 0)
		<< " on_step_surface=" << (reading.on_step_surface ? 1 : 0)
		<< " drop_detected=" << (reading.drop_detected ? 1 : 0)
		<< " edge_detected=" << (reading.edge_detected ? 1 : 0)
		<< '\n';
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

	const std::uint8_t mcp_pin =
		(argc >= 2) ? static_cast<std::uint8_t>(std::strtoul(argv[1], nullptr, 10))
					: RobotConfig::MCP23017::FRONT_DOWNWARD_DO;
	const int duration_s = (argc >= 3) ? std::atoi(argv[2]) : 30;
	const bool active_on_surface =
		(argc >= 4) ? (std::atoi(argv[3]) != 0) : RobotConfig::Sensors::FRONT_DOWNWARD_ACTIVE_ON_SURFACE;

	if (duration_s <= 0)
	{
		std::cerr << "duration_s must be a positive integer.\n";
		PrintUsage(argv[0]);
		return 1;
	}

	auto mcp23017 = std::make_shared<Mcp23017Driver>();
	if (!mcp23017->start())
	{
		std::cerr << "Failed to start MCP23017. Check SDA/SCL/VDD/VSS wiring, RESET high, and address 0x20.\n";
		return 1;
	}

	Mcp23017DownwardSensor front_sensor(mcp23017, mcp_pin, active_on_surface);
	if (!front_sensor.start())
	{
		std::cerr << "Failed to start front downward sensor on MCP23017 pin " << static_cast<int>(mcp_pin) << ".\n";
		return 1;
	}

	std::cout << "Starting MCP23017 front IR sensor test\n";
	std::cout << "MCP23017 pin      = " << static_cast<int>(mcp_pin) << '\n';
	std::cout << "duration_s        = " << duration_s << '\n';
	std::cout << "active_on_surface = " << (active_on_surface ? 1 : 0) << '\n';

	DownwardReading last_reading = front_sensor.latest();
	std::cout << "[initial] ";
	PrintReading(last_reading);

	const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(duration_s);
	while (std::chrono::steady_clock::now() < deadline)
	{
		if (front_sensor.waitForEdge(std::chrono::milliseconds(500)))
		{
			last_reading = front_sensor.latest();
			std::cout << "[edge] ";
			PrintReading(last_reading);
		}
		else
		{
			last_reading = front_sensor.latest();
			std::cout << "[poll] ";
			PrintReading(last_reading);
		}
	}

	front_sensor.stop();
	std::cout << "MCP23017 front IR sensor test complete.\n";
	return 0;
}
