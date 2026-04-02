#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <string>
#include <thread>

#include <gpiod.h>

namespace
{
std::string FormatErrno(const std::string& prefix)
{
	return prefix + ": " + std::strerror(errno);
}

void PrintUsage(const char* program_name)
{
	std::cout
		<< "Usage:\n  " << program_name
		<< " <gpio_in1_in3> <gpio_in2_in4> [forward_ms] [reverse_ms] [cycles]\n\n"
		<< "Recommended wiring for one DRV8833 carrier in paralleled mode:\n"
		<< "  Raspberry Pi GPIO A -> IN1 + IN3\n"
		<< "  Raspberry Pi GPIO B -> IN2 + IN4\n"
		<< "  DRV8833 OUT1 + OUT3 -> Motor terminal A\n"
		<< "  DRV8833 OUT2 + OUT4 -> Motor terminal B\n"
		<< "  Battery + -> DRV8833 VCC/VM\n"
		<< "  Battery - -> DRV8833 GND and Raspberry Pi GND\n\n"
		<< "Truth table used by this test:\n"
		<< "  Forward : IN1/3=1, IN2/4=0\n"
		<< "  Reverse : IN1/3=0, IN2/4=1\n"
		<< "  Coast   : IN1/3=0, IN2/4=0\n"
		<< "  Brake   : IN1/3=1, IN2/4=1\n";
}

enum class DriveMode
{
	Coast,
	Forward,
	Reverse,
	Brake
};

bool ApplyMode(gpiod_line_request* request,
			   const unsigned int gpio_in1_in3,
			   const unsigned int gpio_in2_in4,
			   const DriveMode mode)
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

	if (gpiod_line_request_set_value(request, gpio_in1_in3, value_a) < 0)
	{
		std::cerr << FormatErrno("Failed to set GPIO for IN1+IN3") << '\n';
		return false;
	}

	if (gpiod_line_request_set_value(request, gpio_in2_in4, value_b) < 0)
	{
		std::cerr << FormatErrno("Failed to set GPIO for IN2+IN4") << '\n';
		return false;
	}

	return true;
}
}

int main(int argc, char* argv[])
{
	if (argc < 3)
	{
		PrintUsage(argv[0]);
		return 1;
	}

	const unsigned int gpio_in1_in3 = static_cast<unsigned int>(std::strtoul(argv[1], nullptr, 10));
	const unsigned int gpio_in2_in4 = static_cast<unsigned int>(std::strtoul(argv[2], nullptr, 10));
	const int forward_ms = (argc >= 4) ? std::atoi(argv[3]) : 1500;
	const int reverse_ms = (argc >= 5) ? std::atoi(argv[4]) : 1500;
	const int cycles = (argc >= 6) ? std::atoi(argv[5]) : 3;

	if (forward_ms <= 0 || reverse_ms <= 0 || cycles <= 0)
	{
		std::cerr << "forward_ms, reverse_ms, and cycles must be positive integers.\n";
		return 1;
	}

	gpiod_chip* chip = gpiod_chip_open("/dev/gpiochip0");
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
		std::cerr << "Failed to allocate libgpiod request objects.\n";
		gpiod_line_settings_free(settings);
		gpiod_line_config_free(line_config);
		gpiod_request_config_free(request_config);
		gpiod_chip_close(chip);
		return 1;
	}

	gpiod_line_settings_set_direction(settings, GPIOD_LINE_DIRECTION_OUTPUT);
	gpiod_line_settings_set_drive(settings, GPIOD_LINE_DRIVE_PUSH_PULL);
	gpiod_line_settings_set_output_value(settings, GPIOD_LINE_VALUE_INACTIVE);
	gpiod_request_config_set_consumer(request_config, "drv8833-direct-test");

	const unsigned int offsets[2] = {gpio_in1_in3, gpio_in2_in4};
	gpiod_line_config_add_line_settings(line_config, offsets, 2, settings);

	gpiod_line_request* request = gpiod_chip_request_lines(chip, request_config, line_config);

	gpiod_line_settings_free(settings);
	gpiod_line_config_free(line_config);
	gpiod_request_config_free(request_config);

	if (request == nullptr)
	{
		std::cerr << FormatErrno("Failed to request GPIO lines for DRV8833 test") << '\n';
		gpiod_chip_close(chip);
		return 1;
	}

	std::cout << "Starting DRV8833 direct-drive test\n";
	std::cout << "GPIO IN1+IN3 = " << gpio_in1_in3 << '\n';
	std::cout << "GPIO IN2+IN4 = " << gpio_in2_in4 << '\n';
	std::cout << "Forward time  = " << forward_ms << " ms\n";
	std::cout << "Reverse time  = " << reverse_ms << " ms\n";
	std::cout << "Cycles        = " << cycles << '\n';

	if (!ApplyMode(request, gpio_in1_in3, gpio_in2_in4, DriveMode::Coast))
	{
		gpiod_line_request_release(request);
		gpiod_chip_close(chip);
		return 1;
	}

	for (int cycle = 1; cycle <= cycles; ++cycle)
	{
		std::cout << "[Cycle " << cycle << "] Forward\n";
		if (!ApplyMode(request, gpio_in1_in3, gpio_in2_in4, DriveMode::Forward))
		{
			gpiod_line_request_release(request);
			gpiod_chip_close(chip);
			return 1;
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(forward_ms));

		std::cout << "[Cycle " << cycle << "] Coast\n";
		if (!ApplyMode(request, gpio_in1_in3, gpio_in2_in4, DriveMode::Coast))
		{
			gpiod_line_request_release(request);
			gpiod_chip_close(chip);
			return 1;
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(500));

		std::cout << "[Cycle " << cycle << "] Reverse\n";
		if (!ApplyMode(request, gpio_in1_in3, gpio_in2_in4, DriveMode::Reverse))
		{
			gpiod_line_request_release(request);
			gpiod_chip_close(chip);
			return 1;
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(reverse_ms));

		std::cout << "[Cycle " << cycle << "] Brake\n";
		if (!ApplyMode(request, gpio_in1_in3, gpio_in2_in4, DriveMode::Brake))
		{
			gpiod_line_request_release(request);
			gpiod_chip_close(chip);
			return 1;
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(500));
	}

	(void)ApplyMode(request, gpio_in1_in3, gpio_in2_in4, DriveMode::Coast);
	std::cout << "Test complete. Outputs returned to coast mode.\n";

	gpiod_line_request_release(request);
	gpiod_chip_close(chip);
	return 0;
}
