#ifndef POSE_MONITOR_H
#define POSE_MONITOR_H

#include <functional>
#include <mutex>

#include "config.h"
#include "hardware_interfaces.h"

namespace Robot
{
class PoseMonitor
{
public:
	explicit PoseMonitor(IImuSensor* imu_sensor = nullptr);

	void bindToImu(IImuSensor& imu_sensor);
	void setUpdateCallback(PoseCallback callback);
	void updatePose(const PoseData& pose);

	PoseData currentPose() const;
	bool isSafe() const;
	bool isOverTilt() const;

private:
	mutable std::mutex mutex_;
	PoseData latest_pose_;
	PoseCallback update_callback_;
};
}

#endif
