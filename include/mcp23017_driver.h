#ifndef MCP23017_DRIVER_H
#define MCP23017_DRIVER_H

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <map>
#include <mutex>
#include <string>
#include <thread>

#include "config.h"

// Forward-declare libgpiod types to avoid pulling the header into every TU
// that includes this file.
struct gpiod_chip;
struct gpiod_line_request;
struct gpiod_edge_event_buffer;

namespace Robot
{
class Mcp23017Driver
{
public:
	// Callback type invoked from the interrupt thread on every GPIO change.
	// gpioa / gpiob are the full 8-bit port values read immediately after the
	// MCP23017 interrupt fires (reading the GPIO registers also clears the latch).
	using PinChangeCallback = std::function<void(std::uint8_t gpioa, std::uint8_t gpiob)>;

	static constexpr std::size_t kInvalidToken = 0;

	explicit Mcp23017Driver(unsigned int bus_id = RobotConfig::I2C::BUS_ID,
							std::uint8_t device_address = RobotConfig::I2C::MCP23017_ADDR);
	~Mcp23017Driver();

	bool start();
	void stop();
	bool isReady() const;

	// Configure a pin as an input.  Must be called for every pin before
	// startInterrupts() so the interrupt-enable mask is built up correctly.
	bool configureInput(std::uint8_t pin, bool pullup_enabled);
	bool readPin(std::uint8_t pin, bool& value);
	bool readAllPins(std::uint8_t& gpioa, std::uint8_t& gpiob);

	// Open GPIO lines for INTA (inta_gpio) and INTB (intb_gpio), configure
	// the MCP23017 interrupt-on-change hardware, and start the blocking
	// interrupt thread.  Call this once after all devices have registered
	// their pins via configureInput().
	bool startInterrupts(unsigned int inta_gpio, unsigned int intb_gpio);

	// Register a callback that will be invoked (from the interrupt thread)
	// whenever any MCP23017 GPIO pin changes state.
	// Returns a token that must be passed to unregisterPinChangeCallback().
	std::size_t registerPinChangeCallback(PinChangeCallback callback);

	// Remove a previously registered callback.
	// This call blocks until any in-progress callback invocation has finished,
	// guaranteeing that the callback will never be called after this returns.
	void unregisterPinChangeCallback(std::size_t token);

private:
	// Interrupt thread body: blocks on poll() over the gpiod fd and the stop
	// pipe; never sleeps.
	void interruptLoop();

	// Fire all registered callbacks under callbacks_mutex_.
	void notifyCallbacks(std::uint8_t gpioa, std::uint8_t gpiob);

	// I2C helpers (all require mutex_ to be held by the caller)
	bool openDeviceLocked();
	void closeDeviceLocked();
	bool writeRegisterLocked(std::uint8_t reg, std::uint8_t value);
	bool readRegisterLocked(std::uint8_t reg, std::uint8_t& value);
	bool writeBitLocked(std::uint8_t reg, std::uint8_t bit, bool value, std::uint8_t& cache);

	// ── I2C state (protected by mutex_) ──────────────────────────────────
	mutable std::mutex mutex_;
	unsigned int bus_id_;
	std::uint8_t device_address_;
	bool ready_{false};
	int device_fd_{-1};

	std::uint8_t iodira_{0xFF};		// port-A direction cache (1 = input)
	std::uint8_t iodirb_{0xFF};		// port-B direction cache
	std::uint8_t gppua_{0x00};		// port-A pull-up cache
	std::uint8_t gppub_{0x00};		// port-B pull-up cache
	std::uint8_t gpintena_{0x00};	// port-A interrupt-enable cache (built by configureInput)
	std::uint8_t gpintenb_{0x00};	// port-B interrupt-enable cache

	// ── Interrupt thread ─────────────────────────────────────────────────
	std::thread interrupt_thread_;
	std::atomic_bool interrupt_running_{false};
	int stop_pipe_[2]{-1, -1};			// write end woken to stop the thread
	gpiod_chip* int_chip_{nullptr};
	gpiod_line_request* int_request_{nullptr};

	// ── Registered callbacks (protected by callbacks_mutex_) ─────────────
	std::mutex callbacks_mutex_;
	std::map<std::size_t, PinChangeCallback> callbacks_;
	std::size_t next_token_{1};
};
}

#endif
