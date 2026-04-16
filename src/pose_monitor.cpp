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

PoseMonitor::~PoseMonitor()
{
	if (imu_sensor_ != nullptr)
	{
		imu_sensor_->setCallback({});
	}
}

void PoseMonitor::bindToImu(IImuSensor& imu_sensor)
{
	if (imu_sensor_ != nullptr && imu_sensor_ != &imu_sensor)
	{
		imu_sensor_->setCallback({});
	}

	imu_sensor_ = &imu_sensor;
	imu_sensor.setCallback([this](const PoseData& pose) {
		updatePose(pose);
	});
}

void PoseMonitor::setUpdateCallback(PoseCallback callback)
{
	std::lock_guard<std::mutex> lock(mutex_);
	update_callback_ = std::move(callback);
}

void PoseMonitor::updatePose(const PoseData& pose)
{
	PoseCallback callback;
	{
		std::lock_guard<std::mutex> lock(mutex_);
		latest_pose_ = pose;
		callback = update_callback_;
	}

	if (callback)
	{
		callback(pose);
	}
}

PoseData PoseMonitor::currentPose() const
{
	std::lock_guard<std::mutex> lock(mutex_);
	auto pose = latest_pose_;
	if (RobotConfig::TestMode::BYPASS_IMU && pose.valid)
	{
		pose.timestamp = SteadyClock::now();
	}

	return pose;
}

bool PoseMonitor::isSafe() const
{
	const auto pose = currentPose();
	if (!pose.valid)
	{
		return false;
	}

	if (!RobotConfig::TestMode::BYPASS_IMU &&
		!isFresh(
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
