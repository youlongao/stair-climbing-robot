#include "pose_monitor.h"

#include <cmath>

#include "utils.h"

namespace Robot
{
PoseMonitor::PoseMonitor(IImuSensor* imu_sensor)
{
	if (imu_sensor != nullptr)
	{
		bindToImu(*imu_sensor);
	}
}

void PoseMonitor::bindToImu(IImuSensor& imu_sensor)
{
	imu_sensor.setCallback([this](const PoseData& pose) {
		updatePose(pose);
	});
}

void PoseMonitor::updatePose(const PoseData& pose)
{
	std::lock_guard<std::mutex> lock(mutex_);
	latest_pose_ = pose;
}

PoseData PoseMonitor::currentPose() const
{
	std::lock_guard<std::mutex> lock(mutex_);
	return latest_pose_;
}

bool PoseMonitor::isSafe() const
{
	const auto pose = currentPose();
	if (!pose.valid)
	{
		return false;
	}

	if (!isFresh(
			pose.timestamp,
			std::chrono::milliseconds(RobotConfig::Sensors::SENSOR_STALE_MS)))
	{
		return false;
	}

	return !isOverTilt();
}

bool PoseMonitor::isOverTilt() const
{
	const auto pose = currentPose();
	if (!pose.valid)
	{
		return true;
	}

	return std::fabs(pose.pitch_deg) > RobotConfig::Safety::MAX_SAFE_PITCH_DEG ||
		   std::fabs(pose.roll_deg) > RobotConfig::Safety::MAX_SAFE_ROLL_DEG;
}
}
