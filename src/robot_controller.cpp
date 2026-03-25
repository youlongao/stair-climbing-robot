#include "robot_controller.h"

#include <cmath>

namespace Robot
{
RobotController::RobotController(ClimbingFsm& climbing_fsm,
								 MotionCoordinator& motion_coordinator,
								 StepDetector& step_detector,
								 PoseMonitor& pose_monitor,
								 SafetyManager& safety_manager)
	: climbing_fsm_(climbing_fsm),
	  motion_coordinator_(motion_coordinator),
	  step_detector_(step_detector),
	  pose_monitor_(pose_monitor),
	  safety_manager_(safety_manager)
{
}

bool RobotController::init()
{
	motion_coordinator_.resetPhases();
	climbing_fsm_.transitionTo(MotionState::Idle);

	const auto safety_status = safety_manager_.checkAllSafetyConditions();
	{
		std::lock_guard<std::mutex> lock(mutex_);
		robot_state_.motion_state = climbing_fsm_.getCurrentState();
		robot_state_.safety_status = safety_status;
		robot_state_.timestamp = SteadyClock::now();
	}

	return safety_status.level != SafetyLevel::Fault;
}

void RobotController::update()
{
	const auto current_state = climbing_fsm_.getCurrentState();
	const auto safety_status = safety_manager_.checkAllSafetyConditions();
	const auto step_assessment = step_detector_.detectStepEdge();

	bool front_phase_complete = false;
	bool middle_transfer_phase_complete = false;
	bool rear_transfer_phase_complete = false;

	switch (current_state)
	{
	case MotionState::ApproachingStep:
	case MotionState::FrontClimb:
		front_phase_complete = motion_coordinator_.executeFrontPhase(current_state);
		break;
	case MotionState::MiddleTransfer:
		middle_transfer_phase_complete = motion_coordinator_.executeMiddleTransferPhase(current_state);
		break;
	case MotionState::RearTransfer:
		rear_transfer_phase_complete = motion_coordinator_.executeRearTransferPhase(current_state);
		break;
	case MotionState::Completed:
	case MotionState::Fault:
		stopAll();
		break;
	case MotionState::Idle:
	default:
		break;
	}

	const auto next_state = climbing_fsm_.updateState(
		step_assessment,
		safety_status,
		front_phase_complete,
		middle_transfer_phase_complete,
		rear_transfer_phase_complete);
	climbing_fsm_.transitionTo(next_state);

	if (next_state == MotionState::Fault)
	{
		stopAll();
	}

	RobotState snapshot;
	snapshot.motion_state = climbing_fsm_.getCurrentState();
	snapshot.step_assessment = step_assessment;
	snapshot.safety_status = safety_status;
	snapshot.sensor_data.pose = pose_monitor_.currentPose();
	snapshot.sensor_data.front_distance.distance_m = step_assessment.front_face_distance_m;
	snapshot.sensor_data.front_distance.valid = !std::isnan(step_assessment.front_face_distance_m);
	snapshot.sensor_data.front_distance.timestamp = step_assessment.timestamp;
	snapshot.timestamp = SteadyClock::now();

	std::lock_guard<std::mutex> lock(mutex_);
	robot_state_ = snapshot;
}

void RobotController::stopAll()
{
	motion_coordinator_.stopAll();
}

void RobotController::resetSystem()
{
	safety_manager_.clearFault();
	motion_coordinator_.resetPhases();
	climbing_fsm_.transitionTo(MotionState::Idle);

	std::lock_guard<std::mutex> lock(mutex_);
	robot_state_ = {};
}

RobotState RobotController::state() const
{
	std::lock_guard<std::mutex> lock(mutex_);
	return robot_state_;
}
}
