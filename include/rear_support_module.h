#ifndef REAR_SUPPORT_MODULE_H
#define REAR_SUPPORT_MODULE_H

#include <functional>

#include "config.h"
#include "hardware_interfaces.h"

namespace Robot
{
class RearSupportModule
{
public:
	/*
	rear_slide_axis: Linear actuator interface of the rear sliding module
	rear_lift_axis: Linear actuator interface of the second lifting module
	rear_support_confirmed: A callback function is used to confirm 
							whether the subsequent support wheel has truly landed on the new step surface
	*/
	RearSupportModule(ILinearAxis* rear_slide_axis = nullptr,
					  ILinearAxis* rear_lift_axis = nullptr,
					  std::function<bool()> rear_support_confirmed = {});

	// Assist in adjusting the overall machine attitude during the mid-section transfer phase
	void assistMiddleTransfer();

	// The rear support wheel is moved towards the new step surface
	bool transferSupportToStep();

	// Stop the movement of the rear support mechanism and maintain its current position
	void stabilizeSupport();

private:
	ILinearAxis* rear_slide_axis_;
	ILinearAxis* rear_lift_axis_;
	std::function<bool()> rear_support_confirmed_;
};
}

#endif
