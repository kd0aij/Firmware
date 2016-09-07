#ifndef MC_SEQUENCER_H
#define MC_SEQUENCER_H

#include <px4_defines.h>
#include <math.h>
#include <drivers/drv_hrt.h>

#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/control_state.h>

#include <mathlib/mathlib.h>
#include <platforms/px4_defines.h>

void control_sequencer(
	struct control_state_s &_ctrl_state,
	struct vehicle_attitude_setpoint_s &_att_sp,
	struct manual_control_setpoint_s &_manual,
	math::Matrix<3, 3> &R_sp,
	float &rollRate, float &pitchRate, float &yawRate);

#endif
