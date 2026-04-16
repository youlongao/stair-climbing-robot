#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <string>
#include <thread>

#include "config.h"
#include "mcp23017_driver.h"

namespace
{
struct Observation
{
	bool lower_seen{false};
	bool upper_seen{false};
	bool read_error{false};
};

void PrintUsage(const char* program_name)
{
	std::cout
		<< "Usage:\n  " << program_name << " [observe_seconds] [interval_ms]\n\n"
		<< "Default:\n"
		<< "  observe_seconds = 5\n"
		<< "  interval_ms     = 100\n\n"
		<< "This test checks whether the rear slide front/rear limit switches are swapped.\n"
		<< "It does not drive the rear slide motor. You press the physical switches by hand.\n\n"
		<< "Expected main-robot mapping:\n"
		<< "  Physical rear limit  -> slide2_lower -> MCP23017 pin "
		<< static_cast<int>(RobotConfig::MCP23017::SLIDE_2_LOWER_LIMIT)
		<< " (GPB0)\n"
		<< "  Physical front limit -> slide2_upper -> MCP23017 pin "
		<< static_cast<int>(RobotConfig::MCP23017::SLIDE_2_UPPER_LIMIT)
		<< " (GPA7)\n\n"
		<< "Signal meaning:\n"
		<< "  raw=1, triggered=0 means open/not pressed with pull-up.\n"
		<< "  raw=0, triggered=1 means active-low switch pressed to GND.\n";
}

bool ConfigureRearSlideInputs(Robot::Mcp23017Driver& driver)
{
	return driver.configureInput(RobotConfig::MCP23017::SLIDE_2_LOWER_LIMIT, true) &&
		   driver.configureInput(RobotConfig::MCP23017::SLIDE_2_UPPER_LIMIT, true);
}

bool ReadTriggered(Robot::Mcp23017Driver& driver, const std::uint8_t pin, bool& raw, bool& triggered)
{
	if (!driver.readPin(pin, raw))
	{
		return false;
	}

	triggered = !raw;
	return true;
}

Observation ObservePhase(Robot::Mcp23017Driver& driver,
						 const std::string& phase_name,
						 const int observe_seconds,
						 const int interval_ms)
{
	std::cout << "\n" << phase_name << "\n";
	std::cout << "Press and hold the requested physical limit switch, then press Enter...";
	std::cout.flush();
	std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

	Observation observation;
	const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(observe_seconds);

	while (std::chrono::steady_clock::now() < deadline)
	{
		bool lower_raw = false;
		bool lower_triggered = false;
		bool upper_raw = false;
		bool upper_triggered = false;

		const bool lower_ok = ReadTriggered(
			driver,
			RobotConfig::MCP23017::SLIDE_2_LOWER_LIMIT,
			lower_raw,
			lower_triggered);
		const bool upper_ok = ReadTriggered(
			driver,
			RobotConfig::MCP23017::SLIDE_2_UPPER_LIMIT,
			upper_raw,
			upper_triggered);

		if (!lower_ok || !upper_ok)
		{
			observation.read_error = true;
			std::cout << "read_error\n";
			std::this_thread::sleep_for(std::chrono::milliseconds(interval_ms));
			continue;
		}

		observation.lower_seen = observation.lower_seen || lower_triggered;
		observation.upper_seen = observation.upper_seen || upper_triggered;

		std::cout << "slide2_lower(raw=" << (lower_raw ? 1 : 0)
				  << ",triggered=" << (lower_triggered ? 1 : 0)
				  << ")  slide2_upper(raw=" << (upper_raw ? 1 : 0)
				  << ",triggered=" << (upper_triggered ? 1 : 0)
				  << ")\n";

		std::this_thread::sleep_for(std::chrono::milliseconds(interval_ms));
	}

	return observation;
}

void PrintPhaseVerdict(const char* physical_name,
					   const char* expected_signal,
					   const char* opposite_signal,
					   const bool expected_seen,
					   const bool opposite_seen)
{
	std::cout << physical_name << " result: ";
	if (expected_seen && !opposite_seen)
	{
		std::cout << "OK, triggered " << expected_signal << ".\n";
		return;
	}

	if (!expected_seen && opposite_seen)
	{
		std::cout << "SWAPPED, triggered " << opposite_signal << " instead of "
				  << expected_signal << ".\n";
		return;
	}

	if (expected_seen && opposite_seen)
	{
		std::cout << "AMBIGUOUS, both " << expected_signal << " and "
				  << opposite_signal << " triggered. Check shorts or both switches pressed.\n";
		return;
	}

	std::cout << "NO_TRIGGER, neither switch input changed. Check wiring, GND, or switch type.\n";
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

	const int observe_seconds = (argc >= 2) ? std::atoi(argv[1]) : 5;
	const int interval_ms = (argc >= 3) ? std::atoi(argv[2]) : 100;
	if (observe_seconds <= 0 || interval_ms <= 0)
	{
		std::cerr << "observe_seconds and interval_ms must be positive integers.\n";
		PrintUsage(argv[0]);
		return 1;
	}

	Robot::Mcp23017Driver driver;
	if (!driver.start())
	{
		std::cerr << "Failed to start MCP23017. Check SDA/SCL/VDD/VSS, RESET high, and address 0x20.\n";
		return 1;
	}

	if (!ConfigureRearSlideInputs(driver))
	{
		std::cerr << "Failed to configure rear slide limit inputs.\n";
		return 1;
	}

	PrintUsage(argv[0]);

	const auto rear_observation = ObservePhase(
		driver,
		"Step 1: press the PHYSICAL REAR limit switch.",
		observe_seconds,
		interval_ms);
	const auto front_observation = ObservePhase(
		driver,
		"Step 2: press the PHYSICAL FRONT limit switch.",
		observe_seconds,
		interval_ms);

	std::cout << "\nVerdict:\n";
	PrintPhaseVerdict(
		"Physical rear limit",
		"slide2_lower/GPB0",
		"slide2_upper/GPA7",
		rear_observation.lower_seen,
		rear_observation.upper_seen);
	PrintPhaseVerdict(
		"Physical front limit",
		"slide2_upper/GPA7",
		"slide2_lower/GPB0",
		front_observation.upper_seen,
		front_observation.lower_seen);

	if (!rear_observation.read_error && !front_observation.read_error &&
		!rear_observation.lower_seen && rear_observation.upper_seen &&
		!front_observation.upper_seen && front_observation.lower_seen)
	{
		std::cout << "Overall: rear/front limit switch inputs are likely swapped.\n";
	}
	else if (!rear_observation.read_error && !front_observation.read_error &&
			 rear_observation.lower_seen && !rear_observation.upper_seen &&
			 front_observation.upper_seen && !front_observation.lower_seen)
	{
		std::cout << "Overall: rear/front limit switch orientation matches config.h.\n";
	}
	else
	{
		std::cout << "Overall: not conclusive. Use the per-phase result above to inspect wiring.\n";
	}

	return (rear_observation.read_error || front_observation.read_error) ? 1 : 0;
}
