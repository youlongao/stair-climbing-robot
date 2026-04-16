#include "motion_coordinator.h"

#include <string>
#include <utility>

#include "config.h"
#include "logger.h"

namespace Robot
{
MotionCoordinator::MotionCoordinator(FrontSegment& front_segment,
									 MiddleLiftModule& middle_lift_module,
									 MiddleDriveModule& middle_drive_module,
									 RearSupportModule& rear_support_module,
									 IDriveSection* approach_assist_drive,
									 StateEntryCallback state_entry_callback)
	: front_segment_(front_segment),
	  middle_lift_module_(middle_lift_module),
	  middle_drive_module_(middle_drive_module),
	  rear_support_module_(rear_support_module),
	  approach_assist_drive_(approach_assist_drive),
	  state_entry_callback_(std::move(state_entry_callback))
{
}

bool MotionCoordinator::executeState(const MotionState current_state)
{
	if (!active_state_valid_ || active_state_ != current_state)
	{
		enterState(current_state);
	}

	state_complete_ = false;

	switch (current_state)
	{
	case MotionState::Idle:
		state_complete_ = true;
		break;

	case MotionState::ApproachingStep:
		if (checkActuatorTimeout("ApproachingStep", RobotConfig::Motion::APPROACH_TIMEOUT_S))
		{
			front_segment_.stopDrive();
			stopApproachAssist();
		}
		else
		{
			state_complete_ = front_segment_.approachStep();
			if (state_complete_)
			{
				stopApproachAssist();
			}
			else
			{
				driveApproachAssist(front_segment_.approachAssistSpeed());
			}
		}
		break;

	case MotionState::RearSliderBack:
		front_segment_.brakeDrive();
		middle_drive_module_.holdPosition();
		if (!checkActuatorTimeout("RearSliderBack", RobotConfig::Motion::ACTUATOR_CONFIRM_TIMEOUT_S))
		{
			state_complete_ = rear_support_module_.moveSlideBackwardUntilLimit();
		}
		break;

	case MotionState::FrontLift:
		middle_drive_module_.holdPosition();
		if (!checkActuatorTimeout("FrontLift", RobotConfig::Motion::ACTUATOR_CONFIRM_TIMEOUT_S))
		{
			state_complete_ = front_segment_.liftFrontUntilClearance();
		}
		break;

	case MotionState::MiddleDriveToFrontLanding:
		front_segment_.stopDrive();
		if (checkSensorTimeout("MiddleDriveToFrontLanding"))
		{
			stopApproachAssist();
		}
		else if (front_segment_.isSurfaceConfirmed())
		{
			stopApproachAssist();
			state_complete_ = true;
		}
		else
		{
			driveApproachAssist(RobotConfig::Motion::CREEP_SPEED);
		}
		break;

	case MotionState::MiddleClimb:
		front_segment_.brakeDrive();
		middle_drive_module_.holdPosition();
		if (!checkActuatorTimeout("MiddleClimb", RobotConfig::Motion::ACTUATOR_CONFIRM_TIMEOUT_S))
		{
			state_complete_ = middle_lift_module_.lowerUntilLowerLimit();
		}
		break;

	case MotionState::FrontDriveToMiddleLanding:
		stopApproachAssist();
		if (checkSensorTimeout("FrontDriveToMiddleLanding"))
		{
			front_segment_.stopDrive();
		}
		else if (middle_drive_module_.isSupportConfirmed())
		{
			front_segment_.stopDrive();
			state_complete_ = true;
		}
		else
		{
			front_segment_.driveForward(RobotConfig::Motion::CREEP_SPEED);
		}
		break;

	case MotionState::RearSliderForward:
		front_segment_.brakeDrive();
		middle_drive_module_.holdPosition();
		if (!checkActuatorTimeout("RearSliderForward", RobotConfig::Motion::ACTUATOR_CONFIRM_TIMEOUT_S))
		{
			state_complete_ = rear_support_module_.moveSlideForwardUntilLimit();
		}
		break;

	case MotionState::RearLift:
		front_segment_.brakeDrive();
		middle_drive_module_.holdPosition();
		if (!checkActuatorTimeout("RearLift", RobotConfig::Motion::ACTUATOR_CONFIRM_TIMEOUT_S))
		{
			state_complete_ = rear_support_module_.liftRearUntilUpperLimit();
		}
		break;

	case MotionState::FinalDriveToRearLanding:
		if (checkSensorTimeout("FinalDriveToRearLanding"))
		{
			front_segment_.stopDrive();
			stopApproachAssist();
		}
		else if (rear_support_module_.isSupportConfirmed())
		{
			front_segment_.stopDrive();
			stopApproachAssist();
			rear_support_module_.stabilizeSupport();
			state_complete_ = true;
		}
		else
		{
			front_segment_.driveForward(RobotConfig::Motion::CREEP_SPEED);
			driveApproachAssist(RobotConfig::Motion::CREEP_SPEED);
		}
		break;

	case MotionState::Completed:
	case MotionState::Fault:
	default:
		stopAll();
		state_complete_ = true;
		break;
	}

	return state_complete_;
}

bool MotionCoordinator::isPhaseComplete(const MotionState) const
{
	return state_complete_;
}

void MotionCoordinator::stopAll()
{
	front_segment_.stopFrontSegment();
	middle_lift_module_.holdPosition();
	middle_drive_module_.holdPosition();
	rear_support_module_.stabilizeSupport();
	resetPhases();
}

void MotionCoordinator::resetPhases()
{
	state_complete_ = false;
	active_state_valid_ = false;
	state_timed_out_ = false;
	state_timeout_message_.clear();
	state_timeout_kind_ = TimeoutKind::Sensor;
}

void MotionCoordinator::enterState(const MotionState next_state)
{
	active_state_ = next_state;
	active_state_valid_ = true;
	state_complete_ = false;
	state_entry_time_ = SteadyClock::now();
	state_timed_out_ = false;
	state_timeout_message_.clear();
	state_timeout_kind_ = TimeoutKind::Sensor;

	if (state_entry_callback_)
	{
		state_entry_callback_(next_state);
	}
}

bool MotionCoordinator::hasStateTimeout() const
{
	return state_timed_out_;
}

const std::string& MotionCoordinator::stateTimeoutMessage() const
{
	return state_timeout_message_;
}

MotionCoordinator::TimeoutKind MotionCoordinator::stateTimeoutKind() const
{
	return state_timeout_kind_;
}

bool MotionCoordinator::checkSensorTimeout(const char* state_name)
{
	if (state_timed_out_)
	{
		return true;
	}

	const auto elapsed = SteadyClock::now() - state_entry_time_;
	if (elapsed >= std::chrono::seconds(RobotConfig::Motion::SENSOR_CONFIRM_TIMEOUT_S))
	{
		state_timed_out_ = true;
		state_timeout_kind_ = TimeoutKind::Sensor;
		state_timeout_message_ =
			std::string("Sensor confirmation timeout after ") +
			std::to_string(RobotConfig::Motion::SENSOR_CONFIRM_TIMEOUT_S) +
			"s in state: " + state_name;
		return true;
	}

	return false;
}

bool MotionCoordinator::checkActuatorTimeout(const char* state_name, const int timeout_s)
{
	if (state_timed_out_)
	{
		return true;
	}

	const auto elapsed = SteadyClock::now() - state_entry_time_;
	if (elapsed >= std::chrono::seconds(timeout_s))
	{
		state_timed_out_ = true;
		state_timeout_kind_ = TimeoutKind::Actuator;
		state_timeout_message_ =
			std::string("Actuator confirmation timeout after ") +
			std::to_string(timeout_s) +
			"s in state: " + state_name;
		return true;
	}

	return false;
}

void MotionCoordinator::driveApproachAssist(const float speed)
{
	if (approach_assist_drive_ != nullptr)
	{
		approach_assist_drive_->setNormalizedSpeed(speed, speed);
	}
}

void MotionCoordinator::stopApproachAssist()
{
	if (approach_assist_drive_ != nullptr)
	{
		approach_assist_drive_->stop();
	}
}
}
