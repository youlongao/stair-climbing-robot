#include "mcp23017_driver.h"

#include <cstring>
#include <string>

#include "logger.h"

#include <cerrno>
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <poll.h>
#include <pthread.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <gpiod.h>

namespace Robot
{
namespace
{
// MCP23017 register addresses (BANK = 0, the power-on default)
constexpr std::uint8_t kIodirARegister   = 0x00;
constexpr std::uint8_t kIodirBRegister   = 0x01;
constexpr std::uint8_t kGpintenaRegister = 0x04;	// interrupt-on-change enable, port A
constexpr std::uint8_t kGpintenBRegister = 0x05;	// interrupt-on-change enable, port B
constexpr std::uint8_t kIoconRegister    = 0x0A;	// configuration (ODR, INTPOL, MIRROR …)
constexpr std::uint8_t kGppuARegister    = 0x0C;
constexpr std::uint8_t kGppuBRegister    = 0x0D;
constexpr std::uint8_t kGpioARegister    = 0x12;	// reading clears the interrupt latch
constexpr std::uint8_t kGpioBRegister    = 0x13;

std::string BuildI2cPath(const unsigned int bus_id)
{
	return std::string(RobotConfig::Platform::I2C_DEVICE_PREFIX) + std::to_string(bus_id);
}

std::string FormatErrno(const std::string& prefix)
{
	return prefix + ": " + std::strerror(errno);
}
}

Mcp23017Driver::Mcp23017Driver(const unsigned int bus_id, const std::uint8_t device_address)
	: bus_id_(bus_id),
	  device_address_(device_address)
{
}

Mcp23017Driver::~Mcp23017Driver()
{
	stop();
}

bool Mcp23017Driver::start()
{
	std::lock_guard<std::mutex> lock(mutex_);
	if (ready_)
	{
		return true;
	}

	if (!openDeviceLocked())
	{
		return false;
	}

	ready_ =
		writeRegisterLocked(kIodirARegister, iodira_) &&
		writeRegisterLocked(kIodirBRegister, iodirb_) &&
		writeRegisterLocked(kGppuARegister, gppua_) &&
		writeRegisterLocked(kGppuBRegister, gppub_);

	if (!ready_)
	{
		closeDeviceLocked();
	}

	return ready_;
}

void Mcp23017Driver::stop()
{
	// ── 1. Stop the interrupt thread first ───────────────────────────────
	if (interrupt_running_.exchange(false))
	{
		// Wake up the blocking poll() in interruptLoop
		if (stop_pipe_[1] >= 0)
		{
			const char byte = 0;
			(void)write(stop_pipe_[1], &byte, 1);
		}

		if (interrupt_thread_.joinable())
		{
			interrupt_thread_.join();
		}

		if (int_request_ != nullptr)
		{
			gpiod_line_request_release(int_request_);
			int_request_ = nullptr;
		}

		if (int_chip_ != nullptr)
		{
			gpiod_chip_close(int_chip_);
			int_chip_ = nullptr;
		}

		if (stop_pipe_[0] >= 0) { close(stop_pipe_[0]); stop_pipe_[0] = -1; }
		if (stop_pipe_[1] >= 0) { close(stop_pipe_[1]); stop_pipe_[1] = -1; }
	}

	// ── 2. Close the I2C device ──────────────────────────────────────────
	std::lock_guard<std::mutex> lock(mutex_);
	ready_ = false;
	closeDeviceLocked();
}

bool Mcp23017Driver::isReady() const
{
	std::lock_guard<std::mutex> lock(mutex_);
	return ready_;
}

bool Mcp23017Driver::configureInput(const std::uint8_t pin, const bool pullup_enabled)
{
	if (pin >= 16U || !start())
	{
		return false;
	}

	std::lock_guard<std::mutex> lock(mutex_);

	if (pin < 8U)
	{
		// Update the interrupt-enable cache so startInterrupts() can write the
		// full mask in one go after all devices have been initialised.
		gpintena_ |= static_cast<std::uint8_t>(1U << pin);

		return writeBitLocked(kIodirARegister, pin, true, iodira_) &&
			   writeBitLocked(kGppuARegister, pin, pullup_enabled, gppua_);
	}

	const std::uint8_t bit = static_cast<std::uint8_t>(pin - 8U);
	gpintenb_ |= static_cast<std::uint8_t>(1U << bit);

	return writeBitLocked(kIodirBRegister, bit, true, iodirb_) &&
		   writeBitLocked(kGppuBRegister, bit, pullup_enabled, gppub_);
}

bool Mcp23017Driver::readPin(const std::uint8_t pin, bool& value)
{
	if (pin >= 16U || !start())
	{
		return false;
	}

	std::lock_guard<std::mutex> lock(mutex_);
	std::uint8_t gpio_value = 0U;

	if (pin < 8U)
	{
		if (!readRegisterLocked(kGpioARegister, gpio_value))
		{
			return false;
		}

		value = (gpio_value & static_cast<std::uint8_t>(1U << pin)) != 0U;
		return true;
	}

	const std::uint8_t bit = static_cast<std::uint8_t>(pin - 8U);
	if (!readRegisterLocked(kGpioBRegister, gpio_value))
	{
		return false;
	}

	value = (gpio_value & static_cast<std::uint8_t>(1U << bit)) != 0U;
	return true;
}

bool Mcp23017Driver::readAllPins(std::uint8_t& gpioa, std::uint8_t& gpiob)
{
	if (!start())
	{
		return false;
	}

	std::lock_guard<std::mutex> lock(mutex_);
	return readRegisterLocked(kGpioARegister, gpioa) &&
		   readRegisterLocked(kGpioBRegister, gpiob);
}

bool Mcp23017Driver::startInterrupts(const unsigned int inta_gpio, const unsigned int intb_gpio)
{
	if (interrupt_running_.load())
	{
		return true;
	}

	// ── Create stop pipe ─────────────────────────────────────────────────
	if (pipe(stop_pipe_) != 0)
	{
		Logger::error(FormatErrno("MCP23017: failed to create interrupt stop pipe"));
		return false;
	}

	// ── Request GPIO lines for INTA and INTB ─────────────────────────────
	int_chip_ = gpiod_chip_open(RobotConfig::Platform::GPIO_CHIP);
	if (int_chip_ == nullptr)
	{
		Logger::error(FormatErrno("MCP23017: failed to open GPIO chip for interrupt lines"));
		close(stop_pipe_[0]); close(stop_pipe_[1]);
		stop_pipe_[0] = stop_pipe_[1] = -1;
		return false;
	}

	auto* settings      = gpiod_line_settings_new();
	auto* line_config   = gpiod_line_config_new();
	auto* request_config = gpiod_request_config_new();

	if (settings == nullptr || line_config == nullptr || request_config == nullptr)
	{
		Logger::error("MCP23017: failed to allocate libgpiod objects for interrupt lines.");
		gpiod_line_settings_free(settings);
		gpiod_line_config_free(line_config);
		gpiod_request_config_free(request_config);
		gpiod_chip_close(int_chip_); int_chip_ = nullptr;
		close(stop_pipe_[0]); close(stop_pipe_[1]);
		stop_pipe_[0] = stop_pipe_[1] = -1;
		return false;
	}

	// Active-low, falling-edge detection (MCP23017 INT pin drives low on change)
	gpiod_line_settings_set_direction(settings, GPIOD_LINE_DIRECTION_INPUT);
	gpiod_line_settings_set_bias(settings, GPIOD_LINE_BIAS_PULL_UP);
	gpiod_line_settings_set_edge_detection(settings, GPIOD_LINE_EDGE_FALLING);
	gpiod_request_config_set_consumer(request_config, "mcp23017-int");

	const unsigned int offsets[2] = {inta_gpio, intb_gpio};
	gpiod_line_config_add_line_settings(line_config, offsets, 2, settings);
	int_request_ = gpiod_chip_request_lines(int_chip_, request_config, line_config);

	gpiod_line_settings_free(settings);
	gpiod_line_config_free(line_config);
	gpiod_request_config_free(request_config);

	if (int_request_ == nullptr)
	{
		Logger::error(FormatErrno("MCP23017: failed to request interrupt GPIO lines"));
		gpiod_chip_close(int_chip_); int_chip_ = nullptr;
		close(stop_pipe_[0]); close(stop_pipe_[1]);
		stop_pipe_[0] = stop_pipe_[1] = -1;
		return false;
	}

	// ── Configure MCP23017 interrupt hardware ────────────────────────────
	bool config_ok = false;
	{
		std::lock_guard<std::mutex> lock(mutex_);
		// IOCON = 0x00: MIRROR=0 (separate INTA/INTB), ODR=0 (push-pull),
		//               INTPOL=0 (active-low) — matches falling-edge detection above.
		config_ok =
			writeRegisterLocked(kIoconRegister, 0x00) &&
			writeRegisterLocked(kGpintenaRegister, gpintena_) &&
			writeRegisterLocked(kGpintenBRegister, gpintenb_);

		if (config_ok)
		{
			// Clear any interrupt latch that may already be asserted on the MCP23017.
			// If INTA or INTB was already driven low before we configured
			// falling-edge detection on the RPi GPIO, libgpiod would never see a
			// falling edge and the interrupt thread would block forever.
			// Reading GPIOA and GPIOB unconditionally clears the latch and
			// de-asserts INT, ensuring the first real event produces a clean edge.
			// Non-fatal: log a warning but continue if the read fails.
			std::uint8_t latch_clear_a = 0;
			std::uint8_t latch_clear_b = 0;
			if (!readRegisterLocked(kGpioARegister, latch_clear_a) ||
				!readRegisterLocked(kGpioBRegister, latch_clear_b))
			{
				Logger::warn("MCP23017: interrupt latch clear failed — first interrupt event may be missed.");
			}
		}
	}

	if (!config_ok)
	{
		Logger::error("MCP23017: failed to write interrupt configuration registers over I2C.");
		gpiod_line_request_release(int_request_);
		int_request_ = nullptr;
		gpiod_chip_close(int_chip_);
		int_chip_ = nullptr;
		close(stop_pipe_[0]); close(stop_pipe_[1]);
		stop_pipe_[0] = stop_pipe_[1] = -1;
		return false;
	}

	interrupt_running_.store(true);
	interrupt_thread_ = std::thread(&Mcp23017Driver::interruptLoop, this);

	Logger::info(
		"MCP23017 interrupt thread started (INTA=BCM" + std::to_string(inta_gpio) +
		", INTB=BCM" + std::to_string(intb_gpio) + ").");
	return true;
}

std::size_t Mcp23017Driver::registerPinChangeCallback(PinChangeCallback callback)
{
	std::lock_guard<std::mutex> lock(callbacks_mutex_);
	const std::size_t token = next_token_++;
	callbacks_[token] = std::move(callback);
	return token;
}

void Mcp23017Driver::unregisterPinChangeCallback(const std::size_t token)
{
	// Acquire the same mutex used by notifyCallbacks() so this call blocks
	// until any in-progress invocation of that callback has fully returned.
	std::lock_guard<std::mutex> lock(callbacks_mutex_);
	callbacks_.erase(token);
}

// ── Private ──────────────────────────────────────────────────────────────────

void Mcp23017Driver::interruptLoop()
{
	const int gpio_fd = gpiod_line_request_get_fd(int_request_);
	gpiod_edge_event_buffer* event_buf = gpiod_edge_event_buffer_new(16);

	pollfd pfds[2]{};
	pfds[0].fd     = gpio_fd;
	pfds[0].events = POLLIN;
	pfds[1].fd     = stop_pipe_[0];
	pfds[1].events = POLLIN;

	while (interrupt_running_.load())
	{
		// Block here until the MCP23017 drives an INT line low, or stop() wakes
		// us via the pipe.  No sleep — pure blocking I/O.
		const int ret = poll(pfds, 2, -1);

		if (ret <= 0 || !interrupt_running_.load())
		{
			break;
		}

		if ((pfds[0].revents & POLLIN) != 0)
		{
			// Drain the libgpiod event queue (required to re-arm the fd)
			gpiod_line_request_read_edge_events(int_request_, event_buf, 16);

			// Read both GPIO ports from the MCP23017.
			// Reading GPIOA/GPIOB also clears the MCP23017 interrupt latch,
			// re-arming the chip for the next change.
			std::uint8_t gpioa = 0;
			std::uint8_t gpiob = 0;
			{
				std::lock_guard<std::mutex> lock(mutex_);
				if (!ready_ ||
					!readRegisterLocked(kGpioARegister, gpioa) ||
					!readRegisterLocked(kGpioBRegister, gpiob))
				{
					continue;
				}
			}

			notifyCallbacks(gpioa, gpiob);
		}
		// If only pfds[1] fired, the stop pipe was written — loop condition
		// will be false on the next iteration and we exit cleanly.
	}

	gpiod_edge_event_buffer_free(event_buf);
}

void Mcp23017Driver::notifyCallbacks(const std::uint8_t gpioa, const std::uint8_t gpiob)
{
	std::lock_guard<std::mutex> lock(callbacks_mutex_);
	for (auto& [token, cb] : callbacks_)
	{
		if (cb)
		{
			cb(gpioa, gpiob);
		}
	}
}

bool Mcp23017Driver::openDeviceLocked()
{
	device_fd_ = open(BuildI2cPath(bus_id_).c_str(), O_RDWR);
	if (device_fd_ < 0)
	{
		Logger::error("Failed to open I2C bus for MCP23017: " + std::string(std::strerror(errno)));
		return false;
	}

	if (ioctl(device_fd_, I2C_SLAVE, device_address_) < 0)
	{
		Logger::error("Failed to select MCP23017 I2C address: " + std::string(std::strerror(errno)));
		closeDeviceLocked();
		return false;
	}

	return true;
}

void Mcp23017Driver::closeDeviceLocked()
{
	if (device_fd_ >= 0)
	{
		close(device_fd_);
		device_fd_ = -1;
	}
}

bool Mcp23017Driver::writeRegisterLocked(const std::uint8_t reg, const std::uint8_t value)
{
	const std::uint8_t payload[2] = {reg, value};
	return write(device_fd_, payload, sizeof(payload)) == static_cast<ssize_t>(sizeof(payload));
}

bool Mcp23017Driver::readRegisterLocked(const std::uint8_t reg, std::uint8_t& value)
{
	if (write(device_fd_, &reg, 1) != 1)
	{
		return false;
	}

	return read(device_fd_, &value, 1) == 1;
}

bool Mcp23017Driver::writeBitLocked(const std::uint8_t reg,
									const std::uint8_t bit,
									const bool value,
									std::uint8_t& cache)
{
	const std::uint8_t mask = static_cast<std::uint8_t>(1U << bit);
	if (value)
	{
		cache = static_cast<std::uint8_t>(cache | mask);
	}
	else
	{
		cache = static_cast<std::uint8_t>(cache & ~mask);
	}

	return writeRegisterLocked(reg, cache);
}
}
