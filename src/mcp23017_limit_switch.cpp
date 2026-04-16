#include "mcp23017_limit_switch.h"

#include <chrono>
#include <utility>

namespace Robot
{
Mcp23017LimitSwitch::Mcp23017LimitSwitch(std::shared_ptr<Mcp23017Driver> driver,
										 const std::uint8_t pin,
										 const LimitRole role,
										 const bool active_low)
	: driver_(std::move(driver)),
	  pin_(pin),
	  role_(role),
	  active_low_(active_low)
{
}

Mcp23017LimitSwitch::~Mcp23017LimitSwitch()
{
	stop();
}

bool Mcp23017LimitSwitch::start()
{
	if (running_.load())
	{
		return true;
	}

	if (!driver_ || !driver_->configureInput(pin_, active_low_))
	{
		return false;
	}

	// Read the current pin state synchronously so the initial value is valid
	// before the interrupt thread takes over delivery.
	bool raw_value = false;
	if (!driver_->readPin(pin_, raw_value))
	{
		return false;
	}

	updateFromValue(raw_value, false);
	last_raw_value_ = raw_value;
	last_raw_value_valid_ = true;
	running_.store(true);

	// Register an interrupt-driven callback with the driver.  The driver calls
	// this from its single interrupt thread whenever any MCP23017 pin changes
	// — no polling, no sleep.
	callback_token_ = driver_->registerPinChangeCallback(
		[this](const std::uint8_t gpioa, const std::uint8_t gpiob) {
			// Extract the bit for our specific pin from the appropriate port.
			const std::uint8_t port = (pin_ < 8U) ? gpioa : gpiob;
			const std::uint8_t bit  = (pin_ < 8U) ? pin_ : static_cast<std::uint8_t>(pin_ - 8U);
			const bool raw = ((port >> bit) & 1U) != 0U;

			bool changed = false;
			{
				std::lock_guard<std::mutex> lock(mutex_);
				changed = !last_raw_value_valid_ || (raw != last_raw_value_);
				if (changed)
				{
					last_raw_value_ = raw;
					last_raw_value_valid_ = true;
				}
			}

			if (changed)
			{
				updateFromValue(raw, true);
			}
		});

	return true;
}

void Mcp23017LimitSwitch::stop()
{
	if (!running_.exchange(false))
	{
		return;
	}

	// Unregister first: this blocks until any in-progress callback has finished,
	// guaranteeing the callback cannot fire after stop() returns.
	if (driver_ && callback_token_ != Mcp23017Driver::kInvalidToken)
	{
		driver_->unregisterPinChangeCallback(callback_token_);
		callback_token_ = Mcp23017Driver::kInvalidToken;
	}

	// Unblock any thread waiting in waitForTrigger().
	trigger_cv_.notify_all();
}

bool Mcp23017LimitSwitch::isTriggered() const
{
	return latestState().triggered;
}

bool Mcp23017LimitSwitch::isUpperLimit() const
{
	return role_ == LimitRole::Upper && isTriggered();
}

bool Mcp23017LimitSwitch::isLowerLimit() const
{
	return role_ == LimitRole::Lower && isTriggered();
}

LimitSwitchState Mcp23017LimitSwitch::latestState() const
{
	std::lock_guard<std::mutex> lock(mutex_);
	return state_;
}

bool Mcp23017LimitSwitch::waitForTrigger(const std::chrono::milliseconds timeout)
{
	std::unique_lock<std::mutex> lock(mutex_);
	const bool notified = trigger_cv_.wait_for(lock, timeout, [this]() {
		return pending_trigger_ || !running_.load();
	});

	if (!notified || !pending_trigger_)
	{
		return false;
	}

	pending_trigger_ = false;
	return true;
}

void Mcp23017LimitSwitch::setCallback(LimitSwitchCallback callback)
{
	std::lock_guard<std::mutex> lock(mutex_);
	callback_ = std::move(callback);
}

void Mcp23017LimitSwitch::updateFromValue(const bool raw_value, const bool notify_edge)
{
	LimitSwitchCallback callback;
	LimitSwitchState snapshot;

	{
		std::lock_guard<std::mutex> lock(mutex_);
		state_.triggered = active_low_ ? !raw_value : raw_value;
		state_.timestamp = SteadyClock::now();
		state_.valid = true;
		pending_trigger_ = state_.triggered && notify_edge;
		callback = callback_;
		snapshot = state_;
	}

	if (notify_edge)
	{
		trigger_cv_.notify_all();
	}

	if (callback)
	{
		callback(snapshot);
	}
}
}
