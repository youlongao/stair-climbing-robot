#include "front_segment.h"

#include <cmath>
#include <string>
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
	// Read the most recent published ultrasonic sample.
	// The sensor driver now runs its own worker thread, so this stage no longer performs
	// blocking I/O inside the motion path.
	const auto reading = front_distance_sensor_.latest();

	// if the reading result is invalid, stop
	if (!reading.valid || !std::isfinite(reading.distance_m))
	{
		approach_close_sample_count_ = 0;
		approach_assist_speed_ = 0.0F;
		drive_section_.stop();
		return false;
	}

	if (reading.distance_m < RobotConfig::Sensors::STEP_FACE_MIN_DISTANCE_M)
	{
		approach_close_sample_count_ = 0;
		approach_assist_speed_ = 0.0F;
		drive_section_.stop();
		Logger::warn("Ignoring out-of-range front distance sample: " + std::to_string(reading.distance_m) + " m.");
		return false;
	}

	// Keep the whole vehicle driving until it is truly close enough to start the
	// climb sequence. STEP_FACE_MAX_DISTANCE_M is only the detection band; using
	// it as the stop threshold makes the robot prepare to lift too early.
	if (reading.distance_m > RobotConfig::Sensors::READY_TO_CLIMB_DISTANCE_M)
	{
		approach_close_sample_count_ = 0;
		approach_assist_speed_ =
			reading.distance_m <= RobotConfig::Sensors::STEP_FACE_MAX_DISTANCE_M ?
				RobotConfig::Motion::CREEP_SPEED :
				RobotConfig::Motion::APPROACH_SPEED;
		drive_section_.setNormalizedSpeed(approach_assist_speed_, approach_assist_speed_);
		return false;
	}

	++approach_close_sample_count_;
	if (approach_close_sample_count_ < RobotConfig::Sensors::APPROACH_CLOSE_CONFIRM_SAMPLES)
	{
		approach_assist_speed_ = RobotConfig::Motion::CREEP_SPEED;
		drive_section_.setNormalizedSpeed(approach_assist_speed_, approach_assist_speed_);
		return false;
	}

	// Once the robot reaches or passes the close-range threshold, stop driving and let
	// the state machine prepare the lift sequence instead of pushing into the step.
	approach_assist_speed_ = 0.0F;
	drive_section_.stop();
	Logger::info("Approach threshold confirmed at front distance " + std::to_string(reading.distance_m) + " m.");
	return true;
}

float FrontSegment::approachAssistSpeed() const
{
	return approach_assist_speed_;
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

bool FrontSegment::liftFrontUntilClearance()
{
	drive_section_.brake();

	if (front_lift_axis_ != nullptr)
	{
		front_lift_axis_->moveNormalized(RobotConfig::Motion::BODY_LIFT_SPEED);
	}

	const auto reading = front_distance_sensor_.latest();
	const bool clearance_ok =
		reading.valid && reading.distance_m >= RobotConfig::Sensors::STEP_COMPLETION_CLEARANCE_M;

	if (clearance_ok)
	{
		if (front_lift_axis_ != nullptr)
		{
			front_lift_axis_->holdPosition();
		}
		return true;
	}

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

void FrontSegment::driveForward(const float speed)
{
	drive_section_.setNormalizedSpeed(speed, speed);
}

void FrontSegment::stopDrive()
{
	drive_section_.stop();
}

void FrontSegment::brakeDrive()
{
	drive_section_.brake();
}

bool FrontSegment::isSurfaceConfirmed() const
{
	return surface_confirmed_ && surface_confirmed_();
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
