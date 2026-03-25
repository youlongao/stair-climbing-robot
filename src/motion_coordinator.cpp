#include "motion_coordinator.h"

namespace Robot
{
MotionCoordinator::MotionCoordinator(FrontSegment& front_segment,
									 MiddleLiftModule& middle_lift_module,
									 MiddleDriveModule& middle_drive_module,
									 RearSupportModule& rear_support_module)
	: front_segment_(front_segment),
	  middle_lift_module_(middle_lift_module),
	  middle_drive_module_(middle_drive_module),
	  rear_support_module_(rear_support_module)
{
}

bool MotionCoordinator::executeFrontPhase(const MotionState current_state)
{
	switch (current_state)
	{
	case MotionState::ApproachingStep:
		front_phase_complete_ = front_segment_.approachStep();
		break;
	case MotionState::FrontClimb:
		front_phase_complete_ = front_segment_.liftFrontToStep();
		break;
	default:
		front_phase_complete_ = false;
		break;
	}

	return front_phase_complete_;
}

bool MotionCoordinator::executeMiddleTransferPhase(const MotionState current_state)
{
	if (current_state == MotionState::MiddleTransfer)
	{
		rear_support_module_.assistMiddleTransfer();
		const bool lift_ready = middle_lift_module_.raiseBody();
		const bool drive_ready = middle_drive_module_.advanceToStep();
		middle_transfer_phase_complete_ = lift_ready && drive_ready;
	}
	else
	{
		middle_transfer_phase_complete_ = false;
	}

	return middle_transfer_phase_complete_;
}

bool MotionCoordinator::executeRearTransferPhase(const MotionState current_state)
{
	if (current_state == MotionState::RearTransfer)
	{
		rear_transfer_phase_complete_ = rear_support_module_.transferSupportToStep();
	}
	else
	{
		rear_transfer_phase_complete_ = false;
	}

	return rear_transfer_phase_complete_;
}

bool MotionCoordinator::isPhaseComplete(const MotionState current_state) const
{
	switch (current_state)
	{
	case MotionState::ApproachingStep:
	case MotionState::FrontClimb:
		return front_phase_complete_;
	case MotionState::MiddleTransfer:
		return middle_transfer_phase_complete_;
	case MotionState::RearTransfer:
		return rear_transfer_phase_complete_;
	default:
		return false;
	}
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
	front_phase_complete_ = false;
	middle_transfer_phase_complete_ = false;
	rear_transfer_phase_complete_ = false;
}
}
