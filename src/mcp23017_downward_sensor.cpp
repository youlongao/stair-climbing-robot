#include "mcp23017_downward_sensor.h"

#include <chrono>
#include <utility>

namespace Robot
{
Mcp23017DownwardSensor::Mcp23017DownwardSensor(std::shared_ptr<Mcp23017Driver> driver,
											   const std::uint8_t pin,
											   const bool active_on_surface)
	: driver_(std::move(driver)),
	  pin_(pin),
	  active_on_surface_(active_on_surface)
{
}

Mcp23017DownwardSensor::~Mcp23017DownwardSensor()
{
	stop();
}

bool Mcp23017DownwardSensor::start()
{
	if (running_.load())
	{
		return true;
	}

	if (!driver_ || !driver_->configureInput(pin_, false))
	{
		return false;
	}

	// Synchronous read of the initial pin state before interrupt delivery begins.
	bool raw_value = false;
	if (!driver_->readPin(pin_, raw_value))
	{
		return false;
	}

	updateFromValue(raw_value, false);
	last_raw_value_ = raw_value;
	last_raw_value_valid_ = true;
	running_.store(true);

	// Register an interrupt-driven callback.  The driver wakes this from its
	// single interrupt thread whenever any MCP23017 pin changes — no polling,
	// no sleep.
	callback_token_ = driver_->registerPinChangeCallback(
		[this](const std::uint8_t gpioa, const std::uint8_t gpiob) {
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

void Mcp23017DownwardSensor::stop()
{
	if (!running_.exchange(false))
	{
		return;
	}

	// Unregister first so the callback cannot fire after stop() returns.
	if (driver_ && callback_token_ != Mcp23017Driver::kInvalidToken)
	{
		driver_->unregisterPinChangeCallback(callback_token_);
		callback_token_ = Mcp23017Driver::kInvalidToken;
	}

	// Unblock any thread waiting in waitForEdge().
	edge_cv_.notify_all();
}

DownwardReading Mcp23017DownwardSensor::latest() const
{
	std::lock_guard<std::mutex> lock(mutex_);
	return latest_reading_;
}

bool Mcp23017DownwardSensor::waitForEdge(const std::chrono::milliseconds timeout)
{
	std::unique_lock<std::mutex> lock(mutex_);
	const bool edge_seen = edge_cv_.wait_for(lock, timeout, [this]() {
		return pending_edge_ || !running_.load();
	});

	if (!edge_seen || !pending_edge_)
	{
		return false;
	}

	pending_edge_ = false;
	return true;
}

void Mcp23017DownwardSensor::setCallback(DownwardCallback callback)
{
	std::lock_guard<std::mutex> lock(mutex_);
	callback_ = std::move(callback);
}

void Mcp23017DownwardSensor::updateFromValue(const bool sensor_active, const bool notify_edge)
{
	DownwardCallback callback;
	DownwardReading snapshot;

	{
		std::lock_guard<std::mutex> lock(mutex_);
		latest_reading_.on_step_surface = (sensor_active == active_on_surface_);
		latest_reading_.drop_detected   = !latest_reading_.on_step_surface;
		latest_reading_.edge_detected   = latest_reading_.drop_detected;
		latest_reading_.timestamp       = SteadyClock::now();
		latest_reading_.valid           = true;
		pending_edge_                   = notify_edge;
		callback                        = callback_;
		snapshot                        = latest_reading_;
	}

	if (notify_edge)
	{
		edge_cv_.notify_all();
	}

	if (callback)
	{
		callback(snapshot);
	}
}
}
