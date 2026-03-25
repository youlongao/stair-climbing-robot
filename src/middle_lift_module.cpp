#include "middle_lift_module.h"

#include <cmath>
#include <utility>

#include "logger.h"
#include "utils.h"

namespace Robot
{
MiddleLiftModule::MiddleLiftModule(ILinearAxis& lift_axis,
								   std::function<bool()> middle_support_confirmed)
	: lift_axis_(lift_axis),
	  middle_support_confirmed_(std::move(middle_support_confirmed))	// confirmation callback for middle segement landed 
{
}

bool MiddleLiftModule::raiseBody()	// raise main body action
{
	// if middle section is confirmed, the target finished
	// keep current position, and return true
	if (middle_support_confirmed_)
	{
		if (middle_support_confirmed_())
		{
			holdPosition();
			return true;
		}

		// if landing has been yet confirmed, check if it has touched the upper limit
		const auto axis_state = lift_axis_.getAxisState();
		if (axis_state.at_upper_limit)
		{
			// up to upper limit but failing to confirm landing, the action isn't finished as expected
			lift_axis_.stop();
			Logger::warn("Middle lift reached the upper limit before middle support was confirmed.");
			return false;
		}

		// continue to raise main body
		lift_axis_.moveNormalized(RobotConfig::Motion::BODY_LIFT_SPEED);
		return false;
	}
	// if there is no middle section support confirmation callback, 
	// it degenerates into a control method of raising to the maximum height
	return moveToHeight(RobotConfig::Geometry::BODY_LIFT_MAX_TRAVEL_M);
}

// lower main body
bool MiddleLiftModule::lowerBody()
{
	// lower main body to minimum height
	return moveToHeight(0.0F);
}

// raise main body to target height
bool MiddleLiftModule::moveToHeight(const float target_height_m)
{
	// target height range is limited in machine's allowale range
	target_height_m_ = clamp(target_height_m, 0.0F, RobotConfig::Geometry::BODY_LIFT_MAX_TRAVEL_M);

	// get lift shaft state
	const auto axis_state = lift_axis_.getAxisState();

	// calculate current height error 
	const float error = target_height_m_ - axis_state.position_m;

	// if error < allowable error -- up to the target position and keep it
	if (std::fabs(error) <= RobotConfig::Geometry::POSITION_TOLERANCE_M)
	{
		holdPosition();
		return true;
	}

	// if error > 0, need to continue raising body
	if (error > 0.0F)
	{
		// if reached upper limit, can not continue to raise body
		if (axis_state.at_upper_limit)
		{
			lift_axis_.stop();
			Logger::warn("Middle lift requested to move above upper limit.");
			return false;
		}

		// continue to raise main body at a lift speed
		lift_axis_.moveNormalized(RobotConfig::Motion::BODY_LIFT_SPEED);
		return false;
	}

	// if reached lower limit, can not continue to lower body
	if (axis_state.at_lower_limit)
	{
		lift_axis_.stop();
		Logger::warn("Middle lift requested to move below lower limit.");
		return false;
	}

	// continue to lower main body at a lower speed
	lift_axis_.moveNormalized(RobotConfig::Motion::BODY_LOWER_SPEED);
	return false;
}

// keep current height
void MiddleLiftModule::holdPosition()
{
	lift_axis_.holdPosition();
}
}
