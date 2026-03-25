#include "safety_manager.h"

#include <cmath>
#include <utility>

#include "config.h"
#include "logger.h"

namespace Robot
{
SafetyManager::SafetyManager(PoseMonitor* pose_monitor)
	: pose_monitor_(pose_monitor)
{
}

void SafetyManager::addRule(SafetyRule rule)
{
	std::lock_guard<std::mutex> lock(mutex_);
	rules_.push_back(std::move(rule));
}

void SafetyManager::addEmergencyStopHandler(std::function<void()> handler)
{
	std::lock_guard<std::mutex> lock(mutex_);
	emergency_stop_handlers_.push_back(std::move(handler));
}

SafetyStatus SafetyManager::checkAllSafetyConditions()
{
	{
		std::lock_guard<std::mutex> lock(mutex_);
		if (current_status_.latched)
		{
			return current_status_;
		}
	}

	if (pose_monitor_ != nullptr)
	{
		const auto pose = pose_monitor_->currentPose();
		if (!pose.valid)
		{
			emergencyStop(FaultCode::ImuFault, "Pose monitor has no valid IMU sample.");
			return currentStatus();
		}

		if (!pose_monitor_->isSafe())
		{
			if (std::fabs(pose.pitch_deg) > RobotConfig::Safety::MAX_SAFE_PITCH_DEG)
			{
				emergencyStop(FaultCode::OverPitch, "Robot pitch exceeded the safety threshold.");
			}
			else if (std::fabs(pose.roll_deg) > RobotConfig::Safety::MAX_SAFE_ROLL_DEG)
			{
				emergencyStop(FaultCode::OverRoll, "Robot roll exceeded the safety threshold.");
			}
			else
			{
				emergencyStop(FaultCode::ImuFault, "IMU data became stale.");
			}

			return currentStatus();
		}
	}

	std::vector<SafetyRule> rules_snapshot;
	{
		std::lock_guard<std::mutex> lock(mutex_);
		rules_snapshot = rules_;
	}

	for (const auto& rule : rules_snapshot)
	{
		if (!rule)
		{
			continue;
		}

		const auto status = rule();
		if (status.has_value() && status->level != SafetyLevel::Ok)
		{
			emergencyStop(status->fault, status->message);
			return currentStatus();
		}
	}

	SafetyStatus safe_status;
	safe_status.level = SafetyLevel::Ok;
	safe_status.fault = FaultCode::None;
	safe_status.message = "safe";
	safe_status.timestamp = SteadyClock::now();
	safe_status.latched = false;

	std::lock_guard<std::mutex> lock(mutex_);
	current_status_ = safe_status;
	return current_status_;
}

void SafetyManager::emergencyStop(FaultCode fault, std::string message)
{
	std::vector<std::function<void()>> handlers;
	const auto timestamp = SteadyClock::now();
	const std::string message_copy = message;

	{
		std::lock_guard<std::mutex> lock(mutex_);
		current_status_.level = SafetyLevel::Fault;
		current_status_.fault = fault;
		current_status_.message = std::move(message);
		current_status_.timestamp = timestamp;
		current_status_.latched = true;
		handlers = emergency_stop_handlers_;
	}

	for (const auto& handler : handlers)
	{
		if (handler)
		{
			handler();
		}
	}

	Logger::error("Emergency stop triggered: " + message_copy);
}

void SafetyManager::clearFault()
{
	std::lock_guard<std::mutex> lock(mutex_);
	current_status_ = {};
}

SafetyStatus SafetyManager::currentStatus() const
{
	std::lock_guard<std::mutex> lock(mutex_);
	return current_status_;
}
}
