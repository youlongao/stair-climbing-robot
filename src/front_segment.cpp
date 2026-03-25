#include "front_segment.h"

#include <utility>

#include "logger.h"

namespace Robot
{
//	 Bind the front-end drive, forward distance sensor, optional lifting axis, 
//	 and landing confirmation callback to the object.
FrontSegment::FrontSegment(IDriveSection& drive_section,
						   IFrontDistanceSensor& front_distance_sensor,
						   ILinearAxis* front_lift_axis,
						   std::function<bool()> surface_confirmed)
	: drive_section_(drive_section),
	  front_distance_sensor_(front_distance_sensor),
	  front_lift_axis_(front_lift_axis),
	  surface_confirmed_(std::move(surface_confirmed))
{
}

bool FrontSegment::approachStep()
{
	// The blocking method reads the forward distance once, and the waiting time is determined
	// by the ultrasonic echo timeout parameter
	const auto reading = front_distance_sensor_.readBlocking(
		std::chrono::microseconds(RobotConfig::Sensors::ECHO_TIMEOUT_US));

	// if the reading result is invalid, stop
	if (!reading.valid)
	{
		drive_section_.stop();
		Logger::warn("Front segment approach paused because the front distance sensor is invalid.");
		return false;
	}

	
	// if distance > stair's maximum detected distance -- move forward by approaching speed
	if (reading.distance_m > RobotConfig::Sensors::STEP_FACE_MAX_DISTANCE_M)
	{
		drive_section_.setNormalizedSpeed(RobotConfig::Motion::APPROACH_SPEED,
										  RobotConfig::Motion::APPROACH_SPEED);
		return false;
	}

	// enter the step detection range, stop the car first and then determine whether we actually reached 
	// the trigger window
	drive_section_.stop();

	// distance >= minimum distance threshold
	return reading.distance_m >= RobotConfig::Sensors::STEP_FACE_MIN_DISTANCE_M;
}

// raise the front section and clime step slowly
bool FrontSegment::liftFrontToStep()
{
	// if a front lifting shaft exists, first lift the front seciton upwards
	if (front_lift_axis_ != nullptr)
	{
		front_lift_axis_->moveNormalized(RobotConfig::Motion::BODY_LIFT_SPEED);
	}

	// at the same time, making front wheels move forward slowly
	drive_section_.setNormalizedSpeed(RobotConfig::Motion::CREEP_SPEED,
									  RobotConfig::Motion::CREEP_SPEED);
	
	// if a landing confirmation callback is provided, and the current confirmation indicates that 
	// the front wheels have landed on the step
	// stop the front wheels and keep the lifting axle in its current position
	if (surface_confirmed_ && surface_confirmed_())
	{
		drive_section_.stop();
		if (front_lift_axis_ != nullptr)
		{
			front_lift_axis_->holdPosition();
		}

		return true;
	}

	// the front wheel has not yet landed; continue the current action 
	return false;
}

bool FrontSegment::placeFrontOnStep()
{
	// If a lifting axis exists, 
	//gradually lower the front section from the raised state to the target position.
	if (front_lift_axis_ != nullptr)
	{
		// read lifting shaft current state
		const auto axis_state = front_lift_axis_->getAxisState();

		// if current position also higher than allowable error, continued slow decline
		if (axis_state.position_m > RobotConfig::Geometry::POSITION_TOLERANCE_M)
		{
			front_lift_axis_->moveNormalized(RobotConfig::Motion::BODY_LOWER_SPEED);
		}
		else
		{
			// if have approached target position, keeping current postion, no longer decline
			front_lift_axis_->holdPosition();
		}
	}

	// read the most recent forward distance to determine if there is suffcient clearance ahead
	const auto reading = front_distance_sensor_.latest();

	// If the ultrasound reading is invalid, the judgment will not be blocked;
	// If valid, the distance must be no less than the "completion stage clearance threshold"
	const bool clearance_ok =
		!reading.valid || reading.distance_m >= RobotConfig::Sensors::STEP_COMPLETION_CLEARANCE_M;

	// If no confirmation callback is provided,
	// it is assumed that the surface confirmation has been passed by default.
	// Otherwise, the callback must return true for the data to be considered successfully processed
	const bool surface_ok = !surface_confirmed_ || surface_confirmed_();

	// Once the front wheel has landed steadily and there is sufficient clearance in front,
	// the placement of the front section is considered complete
	if (surface_ok && clearance_ok)
	{
		drive_section_.stop();
		return true;
	}

	// If not yet complete, continue moving forward at a crawling speed with slight adjustments
	drive_section_.setNormalizedSpeed(RobotConfig::Motion::CREEP_SPEED,
									  RobotConfig::Motion::CREEP_SPEED);
	return false;
}

// stop all front section actions, including front drive wheels and lifting shaft
void FrontSegment::stopFrontSegment()
{
	drive_section_.stop();
	if (front_lift_axis_ != nullptr)
	{
		front_lift_axis_->stop();
	}
}
}
