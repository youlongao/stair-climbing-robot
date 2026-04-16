#include <chrono>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <string>
#include <thread>

#include <cerrno>

#include "config.h"

#include <gpiod.h>

namespace
{
constexpr const char* kGpioChip = "/dev/gpiochip0";
constexpr unsigned int kRearSlideIn1In3Gpio = RobotConfig::MotorGPIO::SLIDE_2_IN1_IN3; // Physical pin 32
constexpr unsigned int kRearSlideIn2In4Gpio = RobotConfig::MotorGPIO::SLIDE_2_IN2_IN4; // Physical pin 33

std::string FormatErrno(const std::string& prefix)
{
	return prefix + ": " + std::strerror(errno);
}

enum class DriveMode
{
	Coast,
	Forward,
	Reverse,
	Brake
};

void PrintUsage(const char* program_name)
{
	std::cout
		<< "Usage:\n  " << program_name
		<< " [forward_ms] [reverse_ms] [cycles]\n\n"
		<< "Default:\n"
		<< "  forward_ms = 800\n"
		<< "  reverse_ms = 800\n"
		<< "  cycles     = 1\n\n"
		<< "Direct GPIO wiring assumptions (rear slide only):\n"
		<< "  BCM12 -> Rear slide IN1 + IN3\n"
		<< "  BCM13 -> Rear slide IN2 + IN4\n"
		<< "  OUT1+OUT3 / OUT2+OUT4 -> rear slide motor terminals\n"
		<< "  Battery + -> DRV8833 VCC/VM\n"
		<< "  Battery - -> DRV8833 GND and Raspberry Pi GND\n"
		<< "  EEP/nSLEEP must be tied high (3.3V)\n\n"
		<< "Truth table used by this test:\n"
		<< "  Forward : IN1/3=1, IN2/4=0\n"
		<< "  Reverse : IN1/3=0, IN2/4=1\n"
		<< "  Coast   : IN1/3=0, IN2/4=0\n"
		<< "  Brake   : IN1/3=1, IN2/4=1\n";
}

bool ApplyMode(gpiod_line_request* request, const DriveMode mode)
{
	gpiod_line_value value_a = GPIOD_LINE_VALUE_INACTIVE;
	gpiod_line_value value_b = GPIOD_LINE_VALUE_INACTIVE;

	switch (mode)
	{
	case DriveMode::Forward:
		value_a = GPIOD_LINE_VALUE_ACTIVE;
		value_b = GPIOD_LINE_VALUE_INACTIVE;
		break;
	case DriveMode::Reverse:
		value_a = GPIOD_LINE_VALUE_INACTIVE;
		value_b = GPIOD_LINE_VALUE_ACTIVE;
		break;
	case DriveMode::Brake:
		value_a = GPIOD_LINE_VALUE_ACTIVE;
		value_b = GPIOD_LINE_VALUE_ACTIVE;
		break;
	case DriveMode::Coast:
	default:
		value_a = GPIOD_LINE_VALUE_INACTIVE;
		value_b = GPIOD_LINE_VALUE_INACTIVE;
		break;
	}

	if (gpiod_line_request_set_value(request, kRearSlideIn1In3Gpio, value_a) < 0)
	{
		std::cerr << FormatErrno("Failed to set rear slide IN1+IN3") << '\n';
		return false;
	}

	if (gpiod_line_request_set_value(request, kRearSlideIn2In4Gpio, value_b) < 0)
	{
		std::cerr << FormatErrno("Failed to set rear slide IN2+IN4") << '\n';
		return false;
	}

	return true;
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

	const int forward_ms = (argc >= 2) ? std::atoi(argv[1]) : 800;
	const int reverse_ms = (argc >= 3) ? std::atoi(argv[2]) : 800;
	const int cycles = (argc >= 4) ? std::atoi(argv[3]) : 1;

	if (forward_ms <= 0 || reverse_ms <= 0 || cycles <= 0)
	{
		std::cerr << "forward_ms, reverse_ms, and cycles must be positive integers.\n";
		PrintUsage(argv[0]);
		return 1;
	}

	gpiod_chip* chip = gpiod_chip_open(kGpioChip);
	if (chip == nullptr)
	{
		std::cerr << FormatErrno("Failed to open /dev/gpiochip0") << '\n';
		return 1;
	}

	auto* settings = gpiod_line_settings_new();
	auto* line_config = gpiod_line_config_new();
	auto* request_config = gpiod_request_config_new();

	if (settings == nullptr || line_config == nullptr || request_config == nullptr)
	{
		std::cerr << "Failed to allocate libgpiod request objects for rear slide test.\n";
		gpiod_line_settings_free(settings);
		gpiod_line_config_free(line_config);
		gpiod_request_config_free(request_config);
		gpiod_chip_close(chip);
		return 1;
	}

	gpiod_line_settings_set_direction(settings, GPIOD_LINE_DIRECTION_OUTPUT);
	gpiod_line_settings_set_drive(settings, GPIOD_LINE_DRIVE_PUSH_PULL);
	gpiod_line_settings_set_output_value(settings, GPIOD_LINE_VALUE_INACTIVE);
	gpiod_request_config_set_consumer(request_config, "rear-slide-test");

	const unsigned int offsets[2] = {kRearSlideIn1In3Gpio, kRearSlideIn2In4Gpio};
	gpiod_line_config_add_line_settings(line_config, offsets, 2, settings);
	gpiod_line_request* request = gpiod_chip_request_lines(chip, request_config, line_config);

	gpiod_line_settings_free(settings);
	gpiod_line_config_free(line_config);
	gpiod_request_config_free(request_config);

	if (request == nullptr)
	{
		std::cerr << FormatErrno("Failed to request rear slide GPIO lines") << '\n';
		gpiod_chip_close(chip);
		return 1;
	}

	std::cout << "Starting rear slide DRV8833 direct GPIO test\n";
	std::cout << "GPIO IN1+IN3 = " << kRearSlideIn1In3Gpio << '\n';
	std::cout << "GPIO IN2+IN4 = " << kRearSlideIn2In4Gpio << '\n';
	std::cout << "Forward time  = " << forward_ms << " ms\n";
	std::cout << "Reverse time  = " << reverse_ms << " ms\n";
	std::cout << "Cycles        = " << cycles << '\n';

	if (!ApplyMode(request, DriveMode::Coast))
	{
		gpiod_line_request_release(request);
		gpiod_chip_close(chip);
		return 1;
	}

	for (int cycle = 1; cycle <= cycles; ++cycle)
	{
		std::cout << "[cycle " << cycle << "] forward\n";
		if (!ApplyMode(request, DriveMode::Forward))
		{
			gpiod_line_request_release(request);
			gpiod_chip_close(chip);
			return 1;
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(forward_ms));

		std::cout << "[cycle " << cycle << "] stop\n";
		if (!ApplyMode(request, DriveMode::Coast))
		{
			gpiod_line_request_release(request);
			gpiod_chip_close(chip);
			return 1;
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(500));

		std::cout << "[cycle " << cycle << "] reverse\n";
		if (!ApplyMode(request, DriveMode::Reverse))
		{
			gpiod_line_request_release(request);
			gpiod_chip_close(chip);
			return 1;
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(reverse_ms));

		std::cout << "[cycle " << cycle << "] stop\n";
		if (!ApplyMode(request, DriveMode::Coast))
		{
			gpiod_line_request_release(request);
			gpiod_chip_close(chip);
			return 1;
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(500));
	}

	(void)ApplyMode(request, DriveMode::Coast);
	gpiod_line_request_release(request);
	gpiod_chip_close(chip);

	std::cout << "Rear slide test complete.\n";
	return 0;
}
