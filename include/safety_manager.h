#ifndef SAFETY_MANAGER_H
#define SAFETY_MANAGER_H

#include <functional>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "pose_monitor.h"
#include "types.h"

namespace Robot
{
class SafetyManager
{
public:
	using SafetyRule = std::function<std::optional<SafetyStatus>()>;

	explicit SafetyManager(PoseMonitor* pose_monitor = nullptr);

	void addRule(SafetyRule rule);
	void addEmergencyStopHandler(std::function<void()> handler);

	SafetyStatus checkAllSafetyConditions();
	void emergencyStop(FaultCode fault = FaultCode::EmergencyStop,
					   std::string message = "Emergency stop requested.");
	void clearFault();
	SafetyStatus currentStatus() const;

private:
	PoseMonitor* pose_monitor_;

	mutable std::mutex mutex_;
	std::vector<SafetyRule> rules_;
	std::vector<std::function<void()>> emergency_stop_handlers_;
	SafetyStatus current_status_;
};
}

#endif
