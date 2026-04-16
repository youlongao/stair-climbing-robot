#ifndef CLIMBING_FSM_H
#define CLIMBING_FSM_H

#include "types.h"

namespace Robot
{
class ClimbingFsm
{
public:
	MotionState updateState(const SafetyStatus& safety_status, bool phase_complete);
	bool transitionTo(MotionState next_state);
	void handleError(FaultCode fault);
	MotionState getCurrentState() const;

private:
	MotionState current_state_{MotionState::Idle};
	FaultCode last_fault_{FaultCode::None};
	int climb_cycles_completed_{0};
};
}

#endif
