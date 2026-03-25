#include "climbing_fsm.h"

namespace Robot
{
MotionState ClimbingFsm::updateState(const StepAssessment& step_assessment,
									 const SafetyStatus& safety_status,
									 const bool front_phase_complete,
									 const bool middle_transfer_phase_complete,
									 const bool rear_transfer_phase_complete)
{
	if (safety_status.level == SafetyLevel::Fault)
	{
		current_state_ = MotionState::Fault;
		return current_state_;
	}

	switch (current_state_)
	{
	case MotionState::Idle:
		return step_assessment.ready_for_climb ? MotionState::ApproachingStep : MotionState::Idle;
	case MotionState::ApproachingStep:
		return front_phase_complete ? MotionState::FrontClimb : MotionState::ApproachingStep;
	case MotionState::FrontClimb:
		return front_phase_complete ? MotionState::MiddleTransfer : MotionState::FrontClimb;
	case MotionState::MiddleTransfer:
		return middle_transfer_phase_complete ? MotionState::RearTransfer : MotionState::MiddleTransfer;
	case MotionState::RearTransfer:
		return rear_transfer_phase_complete ? MotionState::Completed : MotionState::RearTransfer;
	case MotionState::Completed:
		return MotionState::Completed;
	case MotionState::Fault:
	default:
		return MotionState::Fault;
	}
}

bool ClimbingFsm::transitionTo(const MotionState next_state)
{
	if (current_state_ == next_state)
	{
		return false;
	}

	current_state_ = next_state;
	return true;
}

void ClimbingFsm::handleError(const FaultCode fault)
{
	last_fault_ = fault;
	current_state_ = MotionState::Fault;
}

MotionState ClimbingFsm::getCurrentState() const
{
	return current_state_;
}
}
