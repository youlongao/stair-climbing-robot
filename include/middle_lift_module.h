#ifndef MIDDLE_LIFT_MODULE_H
#define MIDDLE_LIFT_MODULE_H

#include <functional>

#include "config.h"
#include "hardware_interfaces.h"

namespace Robot
{
class MiddleLiftModule
{
public:
	// lift_axis：Linear actuator interface of the first lifting module
	/* middle_support_confirmed: A callback used to confirm whether 
	 the mid-drive wheel has landed on the support surface */
	explicit MiddleLiftModule(ILinearAxis& lift_axis,
							  std::function<bool()> middle_support_confirmed = {});

	bool raiseBody();	// raise main body : front section and middle section
	bool lowerBody();	// lower the main body height, Used to restore posture after a step is completed,
						// or to allow the main body to return to the support surface.
	bool lowerUntilLowerLimit();	// retract the first lift until the lower limit is reached
	bool moveToHeight(float target_height_m);	// raise main body to target height
	void holdPosition();	// keep current height, no longer raise or lower body

private:
	ILinearAxis& lift_axis_;
	std::function<bool()> middle_support_confirmed_;
	float target_height_m_{0.0F};
};
}

#endif
