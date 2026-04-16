#include <chrono>
#include <cstdlib>
#include <iostream>
#include <string>
#include <thread>

#include "config.h"
#include "front_distance_sensor.h"

using namespace Robot;

namespace
{
void PrintUsage(const char* program_name)
{
	std::cout
		<< "Usage:\n  " << program_name
		<< " [duration_s] [timeout_us] [sample_interval_ms] [trig_gpio] [echo_gpio]\n\n"
		<< "Default:\n"
		<< "  duration_s         = 20\n"
		<< "  timeout_us         = " << RobotConfig::Sensors::ECHO_TIMEOUT_US << "\n"
		<< "  sample_interval_ms = " << RobotConfig::Realtime::ULTRASONIC_POLLING_MS << "\n"
		<< "  trig_gpio          = " << RobotConfig::GPIO::ULTRASONIC_TRIG << "\n"
		<< "  echo_gpio          = " << RobotConfig::GPIO::ULTRASONIC_ECHO << "\n\n"
		<< "Recommended wiring:\n"
		<< "  HC-SR04 VCC  -> 5V\n"
		<< "  HC-SR04 GND  -> Raspberry Pi GND\n"
		<< "  HC-SR04 TRIG -> configured TRIG GPIO\n"
		<< "  HC-SR04 ECHO -> configured ECHO GPIO through safe level shifting / divider\n\n"
		<< "This test repeatedly triggers the front ultrasonic sensor and prints the measured distance.\n";
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

	const int duration_s = (argc >= 2) ? std::atoi(argv[1]) : 20;
	const int timeout_us = (argc >= 3) ? std::atoi(argv[2]) : RobotConfig::Sensors::ECHO_TIMEOUT_US;
	const int sample_interval_ms =
		(argc >= 4) ? std::atoi(argv[3]) : RobotConfig::Realtime::ULTRASONIC_POLLING_MS;
	const unsigned int trig_gpio =
		(argc >= 5) ? static_cast<unsigned int>(std::strtoul(argv[4], nullptr, 10))
					: static_cast<unsigned int>(RobotConfig::GPIO::ULTRASONIC_TRIG);
	const unsigned int echo_gpio =
		(argc >= 6) ? static_cast<unsigned int>(std::strtoul(argv[5], nullptr, 10))
					: static_cast<unsigned int>(RobotConfig::GPIO::ULTRASONIC_ECHO);

	if (duration_s <= 0 || timeout_us <= 0 || sample_interval_ms <= 0)
	{
		std::cerr << "duration_s, timeout_us, and sample_interval_ms must be positive integers.\n";
		PrintUsage(argv[0]);
		return 1;
	}

	FrontDistanceSensor sensor(RobotConfig::Platform::GPIO_CHIP, trig_gpio, echo_gpio);

	std::cout << "Starting front ultrasonic test\n";
	std::cout << "TRIG GPIO          = " << trig_gpio << '\n';
	std::cout << "ECHO GPIO          = " << echo_gpio << '\n';
	std::cout << "duration_s         = " << duration_s << '\n';
	std::cout << "timeout_us         = " << timeout_us << '\n';
	std::cout << "sample_interval_ms = " << sample_interval_ms << '\n';

	const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(duration_s);
	while (std::chrono::steady_clock::now() < deadline)
	{
		const auto reading = sensor.readBlocking(std::chrono::microseconds(timeout_us));
		if (reading.valid)
		{
			std::cout << "[sample] valid=1 distance_m=" << reading.distance_m << '\n';
		}
		else
		{
			std::cout << "[sample] valid=0 distance_m=0\n";
		}

		std::this_thread::sleep_for(std::chrono::milliseconds(sample_interval_ms));
	}

	std::cout << "Front ultrasonic test complete.\n";
	return 0;
}
