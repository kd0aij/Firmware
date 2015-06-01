/****************************************************************************
 *
 *   Copyright (c) 2015 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file VTOL_pos_main.cpp
 *
 * @author Roman Bapst 	<bapstr@ethz.ch>
 *
 */

#include <nuttx/config.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <math.h>
#include <poll.h>
#include <drivers/drv_hrt.h>
#include <arch/board/board.h>
#include <uORB/uORB.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/actuator_armed.h>
#include <systemlib/param/param.h>
#include <systemlib/err.h>
#include <systemlib/perf_counter.h>
#include <systemlib/systemlib.h>
#include <systemlib/circuit_breaker.h>
#include <lib/mathlib/mathlib.h>
#include <lib/geo/geo.h>

extern "C" __EXPORT int vtol_pos_control_main(int argc, char *argv[]);

#define GRAVITY 9.81f

class VtolPositionControl
{
public:

	VtolPositionControl();
	~VtolPositionControl();

	int start();	// start the task and return OK on success


private:
//******************flags & handlers******************************************************
	bool _task_should_exit;
	int _control_task;		//task handle for VTOL position controller

	// handlers for subscriptions */
	int		_v_att_sub;				// vehicle attitude subscription
	int		_v_control_mode_sub;	// vehicle control mode subscription
	int		_params_sub;			// parameter updates subscription
	int		_manual_control_sp_sub;	// manual control setpoint subscription
	int 	_local_pos_sub;			// sensor subscription
	int 	_actuator_status_sub;

	orb_advert_t  	_att_sp_pub;
	orb_advert_t 	_local_pos_sp_pub;
//*******************data containers***********************************************************
	struct vehicle_attitude_s			_v_att;				// vehicle attitude
	struct vehicle_attitude_setpoint_s	_v_att_sp;			// vehicle attitude setpoint
	struct manual_control_setpoint_s	_manual_control_sp; // manual control setpoint
	struct vehicle_control_mode_s		_v_control_mode;	// vehicle control mode
	struct vehicle_local_position_s		_local_pos;
	struct vehicle_local_position_setpoint_s _local_pos_sp; // vehicle local position setpoint
	struct actuator_armed_s 				_actuator_status;

	struct {
		math::Vector<3> pos_p;
		math::Vector<3> vel_p;
		math::Vector<3> vel_d;
		math::Vector<3> vel_i;
		math::Vector<3> vel_ff;
		float vz_max;
		float mc_yaw_p;
		float man_roll_max;
		float man_pitch_max;
		float man_yaw_max;
		float acc_trans;
		float pitch_transit;
		float airspeed_transit;
		float angle_acc_trans;
		float thrust_scaling;
		float pitch_trim;
		float pitch_sensitivity;
		float fw_cruise_speed;
	} _params;

	struct {
		param_t pos_xy_p;
		param_t pos_z_p;
		param_t vel_xy_p;
		param_t vel_z_p;
		param_t vel_xy_d;
		param_t vel_z_d;
		param_t vel_xy_i;
		param_t vel_z_i;
		param_t vel_xy_ff;
		param_t vel_z_ff;
		param_t vz_max;
		param_t man_roll_max;
		param_t man_pitch_max;
		param_t man_yaw_max;
		param_t mc_yaw_p;
		param_t acc_trans;
		param_t pitch_transit;
		param_t airspeed_transit;
		param_t angle_acc_trans;
		param_t thrust_scaling;
		param_t pitch_trim;
		param_t pitch_sensitivity;
		param_t fw_cruise_speed;
	} _params_handles;

	perf_counter_t	_loop_perf;			// loop performance counter
	perf_counter_t	_nonfinite_input_perf;		// performance counter for non finite input

	typedef enum {
		MC_MODE=0,
		TRANS_MODE,
		FW_MODE
	}vtol_mode;

	struct
	{
		vtol_mode status;		// current mode
		vtol_mode status_prev;	// previous mode
		math::Quaternion q_trans;
		float vel_trans_abs;	// absolute xy velocity during transition
		float pitch_trans;
	}_vtol;

	math::Vector<3> _pos;
	math::Vector<3> _pos_sp;
	math::Vector<3> _vel;
	math::Vector<3> _vel_sp;
	math::Quaternion _q_sp;
	math::Matrix<3,3> _R;
	math::Matrix<3,3> _R_sp;
	math::Vector<3> _euler;
	math::Vector<3> _f_aero;
	math::Vector<3> _a_sp;
	math::Vector<3> _a_sp_prev;

	math::Vector<3> _vel_error_prev;
	math::Vector<3> _vel_error_integ;

	float _thrust_sp;
	float _yaw_sp;
	float _dt;
	bool _reset_pos;
	bool _reset_vel_xy_int;
	bool _reset_vel_z_int;
	bool _reset_yaw_sp;
	bool _reset_vel_error;
	bool _a_sp_prev_valid;
	float _gravity_factor;

//***************** Member functions ***********************************************************************

	void 		task_main();	// main task
	static void	task_main_trampoline(int argc, char *argv[]);	// Shim for calling task_main from task_create.

	void 		vehicle_attitude_poll();		// Check for changes in vehicle attitude
	void		vehicle_control_mode_poll();	// Check for changes in vehicle control mode.
	void		vehicle_manual_poll();			// Check for changes in manual inputs.
	void		arming_status_poll();			// Check for arming status updates.
	void 		vehicle_local_pos_poll();		// Check for changes in sensor values
	void 		vehicle_airspeed_poll();		// Check for changes in airspeed
	void 		parameters_update_poll();		// Check if parameters have changed
	void 		actuator_status_poll();			// Check if actuator status has changed
	int 		parameters_update();			// Update local paraemter cache

	void 		update_vtol_state();

	void 		control_mc();
	void 		control_manual();
	void 		control_attitude();
	void 		control_position();
	void 		get_desired_velocity();

	void 		control_fw();
	void 		calc_des_pos_and_vel();


	void 		control_trans();
	void 		control_position_step();

	void 		compute_des_yaw();
	void 		get_desired_acceleration();
	void 		get_aerodynamic_force();
	void 		run_controller();
	void 		publish_att_sp();
	void 		send_to_log();
	float 		get_feasible_yaw();
};

namespace VTOL_pos_control
{
VtolPositionControl *g_control;
}

/**
* Constructor
*/
VtolPositionControl::VtolPositionControl() :
	_task_should_exit(false),
	_control_task(-1),

	// init subscription handlers
	_v_att_sub(-1),
	_v_control_mode_sub(-1),
	_params_sub(-1),
	_manual_control_sp_sub(-1),
	_local_pos_sub(-1),
	_actuator_status_sub(-1),

	// init publication handlers
	_att_sp_pub(nullptr),
	_local_pos_sp_pub(nullptr),

	_loop_perf(perf_alloc(PC_ELAPSED, "vtol_pos_control")),
	_nonfinite_input_perf(perf_alloc(PC_COUNT, "vtol pos control nonfinite input"))
{
	memset(&_v_att, 0, sizeof(_v_att));
	memset(&_v_att_sp, 0, sizeof(_v_att_sp));
	memset(&_manual_control_sp, 0, sizeof(_manual_control_sp));
	memset(&_v_control_mode, 0, sizeof(_v_control_mode));
	memset(&_local_pos,0,sizeof(_local_pos));
	memset(&_local_pos_sp,0,sizeof(_local_pos_sp));
	memset(&_actuator_status,0,sizeof(_actuator_status));

	memset(&_params,0,sizeof(_params));

	_params_handles.pos_xy_p = param_find("VTP_POS_XY_P");
	_params_handles.pos_z_p = param_find("VTP_POS_Z_P");
	_params_handles.vel_xy_p = param_find("VTP_VEL_XY_P");
	_params_handles.vel_z_p = param_find("VTP_VEL_Z_P");
	_params_handles.vel_xy_d = param_find("VTP_VEL_XY_D");
	_params_handles.vel_z_d = param_find("VTP_VEL_Z_D");
	_params_handles.vel_xy_i = param_find("VTP_VEL_XY_I");
	_params_handles.vel_z_i = param_find("VTP_VEL_Z_I");
	_params_handles.vel_xy_ff = param_find("VTP_VEL_XY_FF");
	_params_handles.vel_z_ff = param_find("VTP_VEL_Z_FF");
	_params_handles.vz_max = param_find("VTP_VZ_MAX");

	_params_handles.mc_yaw_p = param_find("MC_YAW_P");
	_params_handles.man_roll_max = param_find("VTP_MAN_R_MAX");
	_params_handles.man_pitch_max = param_find("VTP_MAN_P_MAX");
	_params_handles.man_yaw_max = param_find("VTP_MAN_Y_MAX");

	_params_handles.acc_trans = param_find("VTP_ACC_TRANS");
	_params_handles.pitch_transit = param_find("VTP_PTCH_TRANSIT");
	_params_handles.airspeed_transit = param_find("VTP_SPD_TRANSIT");
	_params_handles.angle_acc_trans = param_find("VTP_ACC_ANG");
	_params_handles.thrust_scaling = param_find("VTP_THRUST_SCALE");
	_params_handles.pitch_trim = param_find("VTP_PITCH_TRIM");
	_params_handles.pitch_sensitivity = param_find("VTP_PITCH_SENS");
	_params_handles.fw_cruise_speed = param_find("VTP_CRUISE_SPD");


	_vtol.status = MC_MODE;
	_vtol.status_prev = MC_MODE;
	_vtol.q_trans.from_euler(0,0,0);
	_vtol.vel_trans_abs = 0.0f;
	_vtol.pitch_trans = 0.0f;
	
	_pos.zero();
	_pos_sp.zero();
	_vel.zero();
	_vel_sp.zero();
	_q_sp.from_euler(0,0,0);
	_R.set(_v_att.R);
	_R_sp.from_euler(0,0,0);
	_euler.zero();
	_f_aero.zero();
	_thrust_sp = 0.0f;
	_a_sp.zero();
	_a_sp_prev = {0,0,-1.0f};
	_vel_error_integ.zero();
	_vel_error_prev.zero();

	_dt = 0.01f;
	_yaw_sp = 0.0f;
	_gravity_factor = 1.0f;
	_reset_vel_xy_int = true;
	_reset_vel_z_int = true;
	_reset_pos = true;
	_reset_yaw_sp = true;
	_reset_vel_error = true;
	_a_sp_prev_valid = false;

	// fetch initial parameter values
	parameters_update();
}

/**
* Destructor
*/
VtolPositionControl::~VtolPositionControl()
{
	if (_control_task != -1) {
		/* task wakes up every 100ms or so at the longest */
		_task_should_exit = true;

		/* wait for a second for the task to quit at our request */
		unsigned i = 0;

		do {
			/* wait 20ms */
			usleep(20000);

			/* if we have given up, kill it */
			if (++i > 50) {
				task_delete(_control_task);
				break;
			}
		} while (_control_task != -1);
	}

	VTOL_pos_control::g_control = nullptr;
}

/**
* Check for changes in attitude
*/
void VtolPositionControl::vehicle_attitude_poll()
{
	bool updated;

	/* Check if vehicle attitude has changed */
	orb_check(_v_att_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_attitude), _v_att_sub, &_v_att);
	}
}

/**
* Check for changes in vehicle control mode.
*/
void VtolPositionControl::vehicle_control_mode_poll()
{
	bool updated;

	/* Check if vehicle control mode has changed */
	orb_check(_v_control_mode_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_control_mode), _v_control_mode_sub, &_v_control_mode);
	}
}

/**
* Check for changes in manual inputs.
*/
void VtolPositionControl::vehicle_manual_poll()
{
	bool updated;

	/* get pilots inputs */
	orb_check(_manual_control_sp_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(manual_control_setpoint), _manual_control_sp_sub, &_manual_control_sp);
	}
}

/**
* Check for parameter updates.
*/
void
VtolPositionControl::parameters_update_poll()
{
	bool updated;

	/* Check if parameters have changed */
	orb_check(_params_sub, &updated);

	if (updated) {
		struct parameter_update_s param_update;
		orb_copy(ORB_ID(parameter_update), _params_sub, &param_update);
		parameters_update();
	}
}

/**
* Check for local position updates.
*/
void
VtolPositionControl::vehicle_local_pos_poll()
{
	bool updated;
	/* Check if parameters have changed */
	orb_check(_local_pos_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_local_position), _local_pos_sub , &_local_pos);
	}
}

/**
* Check for actuator status updates.
*/
void
VtolPositionControl::actuator_status_poll()
{
	bool updated;
	/* Check if parameters have changed */
	orb_check(_actuator_status_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(actuator_armed), _actuator_status_sub , &_actuator_status);
	}
}

/**
* Update parameters.
*/
int
VtolPositionControl::parameters_update()
{
	float v;
	/* position error gain */
	param_get(_params_handles.pos_xy_p, &v);
	_params.pos_p(0) = v;
	_params.pos_p(1) = v;
	param_get(_params_handles.pos_z_p, &v);
	_params.pos_p(2) = v;

	/* xy velocity error gain */
	param_get(_params_handles.vel_xy_p, &v);
	_params.vel_p(0) = v;
	_params.vel_p(1) = v;
	param_get(_params_handles.vel_z_p, &v);
	_params.vel_p(2) = v;

	/* xy velocity error d gain */
	param_get(_params_handles.vel_xy_d, &v);
	_params.vel_d(0) = v;
	_params.vel_d(1) = v;
	param_get(_params_handles.vel_z_d, &v);
	_params.vel_d(2) = v;

	/* xy velocity error i gain */
	param_get(_params_handles.vel_xy_i, &v);
	_params.vel_i(0) = v;
	_params.vel_i(1) = v;
	param_get(_params_handles.vel_z_i, &v);
	_params.vel_i(2) = v;

	/* xy velocity feedforward gain */
	param_get(_params_handles.vel_xy_ff,&v);
	_params.vel_ff(0) = v;
	_params.vel_ff(1) = v;

	/* z velocity feedforward gain */
	param_get(_params_handles.vel_z_ff,&v);
	_params.vel_ff(2) = v;

	/* max maginitude of z velocity */
	param_get(_params_handles.vz_max, &v);
	_params.vz_max = v;

	/* attitude yaw error p gain */
	param_get(_params_handles.mc_yaw_p, &v);
	_params.mc_yaw_p = v;

	param_get(_params_handles.man_roll_max, &_params.man_roll_max);
	param_get(_params_handles.man_pitch_max, &_params.man_pitch_max);
	param_get(_params_handles.man_yaw_max, &_params.man_yaw_max);
	_params.man_roll_max = math::radians(_params.man_roll_max);
	_params.man_pitch_max = math::radians(_params.man_pitch_max);
	_params.man_yaw_max = math::radians(_params.man_yaw_max);

	param_get(_params_handles.acc_trans,&_params.acc_trans);
	param_get(_params_handles.pitch_transit,&_params.pitch_transit);
	_params.pitch_transit = math::radians(_params.pitch_transit);
	param_get(_params_handles.airspeed_transit,&_params.airspeed_transit);
	param_get(_params_handles.angle_acc_trans,&_params.angle_acc_trans);
	_params.angle_acc_trans = math::radians(_params.angle_acc_trans);
	param_get(_params_handles.thrust_scaling,&_params.thrust_scaling);
	param_get(_params_handles.pitch_trim,&_params.pitch_trim);
	param_get(_params_handles.pitch_sensitivity,&_params.pitch_sensitivity);
	param_get(_params_handles.fw_cruise_speed, &_params.fw_cruise_speed);

	return OK;
}

// velocity setpoint for fw control
void VtolPositionControl::calc_des_pos_and_vel() {
	// user can change yaw with roll stick
	float turn_angle = _manual_control_sp.y*0.5f;
	float vel_abs = _params.fw_cruise_speed + _manual_control_sp.z * 10.0f;
	_vel_sp(0) = vel_abs * cosf(_yaw_sp + turn_angle);
	_vel_sp(1) = vel_abs * sinf(_yaw_sp + turn_angle);
	_vel_sp(2) = _params.pitch_sensitivity*_manual_control_sp.x + _params.pitch_trim;


	// do not use integral for fw flight
	_reset_vel_xy_int = true;
	_reset_vel_z_int = true;
	_reset_vel_error = false;
}

// only used for mc control
void VtolPositionControl::get_desired_velocity() {
	if(_reset_pos) {
		_pos_sp = _pos;
		_reset_pos = false;
	}

	// commanded move rate by user
	math::Vector<3> move_rate = {0,0,0};

	if(_v_control_mode.flag_control_altitude_enabled && !_v_control_mode.flag_control_position_enabled) {
		_pos_sp(0) = _pos(0);
		_pos_sp(1) = _pos(1);
		_pos_sp(2) = _pos(2);
		_vel_sp(2) = -2*(_manual_control_sp.z - 0.5f)*_params.vz_max;
		_vel_sp(0) = 0.0f;
		_vel_sp(1) = 0.0f;
	}
	else if(_v_control_mode.flag_control_position_enabled) {
		move_rate(0) = _manual_control_sp.x;
		move_rate(1) = _manual_control_sp.y;
		move_rate(2) = _manual_control_sp.z;
		// rotate move rates with yaw
		math::Matrix<3,3> R_yaw;
		float yaw = get_feasible_yaw();
		R_yaw.from_euler(0.0f,0.0f,yaw);
		move_rate = R_yaw*move_rate;
		// user gives move rate with sticks
		_pos_sp(0) += move_rate(0)*_dt;
		_pos_sp(1) +=  move_rate(1)*_dt;

		if(_manual_control_sp.z < 0.35f || _manual_control_sp.z > 0.65f) {
			_pos_sp(2) -= 1.0f*(_manual_control_sp.z - 0.5f)*_dt;
		}

		math::Vector<3> pos_err = _pos_sp - _pos;
		_vel_sp = pos_err.emult(_params.pos_p) + move_rate.emult(_params.vel_ff);
	}
	// create velocity setpoint from position error
	// prevent throttle loss during high climb rate phase
	if(_vel_sp(2) - _vel(2) > 0.7f) {
		_vel_sp(2) = _vel(2) + 0.7f;
	}
	_gravity_factor = 1.0f;	// assume not aerodyanmic lift, need to compensate for gravity
	_a_sp_prev_valid = false;
}

// caluclate desired acceleration from desired velocity
void VtolPositionControl::get_desired_acceleration() {
	// provisional feedforward, not used yet
	math::Vector<3> acc_feedforward;
	memset(&acc_feedforward,0,sizeof(acc_feedforward));

	// calculate velocity error
	math::Vector<3> vel_error;
	// deadband for fixed wing mode
	if(_manual_control_sp.aux2 > 0.0f && fabsf(_vel(2)) < 1.0f) {
		math::Vector<3> helper;
		helper = _vel;
		helper(2) *= helper(2)*helper(2);
		vel_error = _vel_sp - helper;
	}
	else {
		vel_error = _vel_sp - _vel;	// in global frame
	}

	
	_vel_error_integ += vel_error*_dt;	// update integral

	if(_reset_vel_xy_int) {
		_vel_error_integ(0) = 0.0f;
		_vel_error_integ(1) = 0.0f;
		_reset_vel_xy_int = false;
	}
	if(_reset_vel_z_int) {
		_vel_error_integ(2) = 0.0f;
		_reset_vel_z_int = false;
	}

	// initialize D controller
	if(_reset_vel_error) {
		_vel_error_prev = vel_error;
		_reset_vel_error = false;
	}

	// limit integration error to prevent windup
	_vel_error_integ(0) = math::constrain(_vel_error_integ(0),-8.0f,8.0f);
	_vel_error_integ(1) = math::constrain(_vel_error_integ(1),-8.0f,8.0f);
	_vel_error_integ(2) = math::constrain(_vel_error_integ(2),-8.0f,8.0f);

	_a_sp = acc_feedforward + _params.vel_p.emult(vel_error) + _params.vel_d.emult(vel_error - _vel_error_prev)/_dt
		+ _params.vel_i.emult(_vel_error_integ);
	_vel_error_prev = vel_error;

	if(_manual_control_sp.aux2 > 0.0f) {
		float a_corr = _a_sp.length();
		_a_sp = a_corr > GRAVITY*0.6f ? _a_sp*GRAVITY*0.6f/a_corr : _a_sp;
		math::Vector<3> thrust_line = {0.0f,0.0f,-1.0f};
		_a_sp += _R*thrust_line*GRAVITY;
	}
	else {
		_a_sp(2) -= GRAVITY;
	}

		// reset previous acceleration setpoint if not valid
		/*
		if(!_a_sp_prev_valid) {
			_a_sp_prev = _a_sp;
			_a_sp_prev_valid = true;
		}

		float acc_norm = math::constrain(_a_sp.length(),GRAVITY,1.0f/_params.thrust_scaling);
		// limit rate of change of a_sp
		math::Vector<3> a_sp_norm = _a_sp.normalized();
		math::Vector<3> a_sp_prev_norm = _a_sp_prev.normalized();

		float inner_prod = math::constrain(a_sp_norm*a_sp_prev_norm,-1.0f,1.0f);
		float beta = acosf(inner_prod);
		if(fabsf(beta) > M_PI_F*_dt) {
			beta = M_PI_F*_dt;
			math::Vector<3> axis = a_sp_prev_norm % a_sp_norm;
			math::Quaternion q_limiter = {cosf(beta/2), sinf(beta/2)*axis(0),sinf(beta/2)*axis(1),sinf(beta/2)*axis(2)};
			a_sp_norm = q_limiter.rotate(a_sp_prev_norm);
		}
		*/

	//_a_sp = a_sp_norm*acc_norm;
	//_a_sp_prev = _a_sp;	// update
}

// not used for now
void VtolPositionControl::get_aerodynamic_force() {
	// for now pretend there are no aerodynamic forces
	memset(&_f_aero,0,sizeof(_f_aero));
}

// main controller, which generates quaternion setpoint
void VtolPositionControl::run_controller() {
	// get desired acceleration
	get_desired_acceleration();

	// substract estimated aerodynamic acceleration
	//get_aerodynamic_force();

	// rotate desired acceleration such that nose of vehicle is in flight direction
	math::Quaternion q_yaw;
	q_yaw.from_euler(0.0f,0.0f,_yaw_sp);
	math::Matrix<3,3> R_yaw = q_yaw.to_dcm();
	R_yaw = R_yaw.transposed();
	math::Vector<3> a_des_yaw = R_yaw*_a_sp;

	// compute desired rotation angle
	math::Vector<3> thrust_des_body = a_des_yaw.normalized();
	math::Vector<3> zi = {0.0f,0.0f,-1.0f};
	float inner_prod = zi*thrust_des_body;
	inner_prod = math::constrain(inner_prod,-1.0f,1.0f);
	float alpha = acosf(inner_prod);

	// compute axis of desired rotation
	math::Vector<3> rot_axis;
	if(fabsf(alpha) < 0.01f) {
		rot_axis.zero();
		rot_axis(1) = 1.0f;
	}
	else {
		rot_axis = zi%thrust_des_body;
	}

	rot_axis.normalize();

	// compute desired quaternion
	math::Quaternion q_xy;
	q_xy(0) = cosf(0.5f*alpha);
	rot_axis*= sinf(0.5f*alpha);
	q_xy(1) = rot_axis(0);
	q_xy(2) = rot_axis(1);
	q_xy(3) = rot_axis(2);

	_q_sp = q_yaw*q_xy;	// combine the two rotations
	_R_sp = _q_sp.to_dcm();

	// rotate body z-axis to global frame
	math::Vector<3> zb_des = thrust_des_body;
	zb_des = R_yaw.transposed()*zb_des;
	thrust_des_body = zb_des;

	// compute desired thrust (project desired to actual axis)
	math::Vector<3> zb = _R*zi;
	_thrust_sp = zb*zb_des;
	_thrust_sp *= _a_sp.length(); // this is not a force yet!
	// limit thrust, min value is imortant to avoid loss of pitch control
	_thrust_sp = math::constrain(_thrust_sp*_params.thrust_scaling,0.0f,1.0f);
}

float VtolPositionControl::get_feasible_yaw() {
	if(_vel(0)*_vel(0) + _vel(1)*_vel(1) > 16.0f) {
		return atan2f(_vel(1),_vel(0));
	} else {
		return _euler(2);
	}
}

// for mc mode only
void VtolPositionControl::control_manual() {
	math::Vector<3> euler_des;
	euler_des(0) = _manual_control_sp.y * _params.man_roll_max;
	euler_des(1) = -_manual_control_sp.x * _params.man_pitch_max;
	euler_des(2) = _yaw_sp;
	_thrust_sp = _manual_control_sp.z;
	_R_sp.from_euler(euler_des(0),euler_des(1),euler_des(2));
	_q_sp.from_euler(euler_des(0),euler_des(1),euler_des(2));
}

// for mc mode only
void VtolPositionControl::control_attitude() {
	get_desired_velocity();
	run_controller();

	// write attitude and thrust setpoint
	math::Vector<3> euler_des;
	euler_des(0) = _manual_control_sp.y * _params.man_roll_max;
	euler_des(1) = -_manual_control_sp.x * _params.man_pitch_max;
	euler_des(2) = _yaw_sp;
	_R_sp.from_euler(euler_des(0),euler_des(1),euler_des(2));
	_q_sp.from_euler(euler_des(0),euler_des(1),euler_des(2));
}

// for mc mode only
void VtolPositionControl::control_position() {
	get_desired_velocity();
	run_controller();
}

// publish the attitude setpoint
void VtolPositionControl::publish_att_sp() {
	// write desired attitude & thrust setpoint
	math::Vector<3> euler_des = _R_sp.to_euler();
	_v_att_sp.timestamp = hrt_absolute_time();
	memcpy(&_v_att_sp.R_body,_R_sp.data,sizeof(_v_att_sp.R_body));
	memcpy(&_v_att_sp.q_d[0],&_q_sp.data[0],sizeof(_v_att_sp.q_d));
	
	// compute quaternion error
	math::Quaternion q_sp_inv = {_q_sp(0),-_q_sp(1),-_q_sp(2),-_q_sp(3)};
	math::Quaternion q_error;
	math::Quaternion q;
	q.from_dcm(_R);
	q_error = q_sp_inv*q;
	memcpy(&_v_att_sp.q_e[0],&q_error.data[0],sizeof(_v_att_sp.q_e));
	_v_att_sp.R_valid = true;
	_v_att_sp.roll_body = euler_des(0);
	_v_att_sp.pitch_body = euler_des(1);
	_v_att_sp.yaw_body = _yaw_sp;
	_v_att_sp.thrust = _thrust_sp;

	// publish desired attitude and thrust
	if (_att_sp_pub != nullptr) {
		orb_publish(ORB_ID(vehicle_attitude_setpoint), _att_sp_pub, &_v_att_sp);
	} else {
		_att_sp_pub = orb_advertise(ORB_ID(vehicle_attitude_setpoint), &_v_att_sp);
	}
}

// do some special logging for developing purposes
void VtolPositionControl::send_to_log() {
	_local_pos_sp.timestamp = hrt_absolute_time();

	// fill local position setpoint
	_local_pos_sp.x = _pos_sp(0);
	_local_pos_sp.y = _pos_sp(1);
	_local_pos_sp.z = _pos_sp(2);
	_local_pos_sp.vx = _vel_sp(0);
	_local_pos_sp.vy = _vel_sp(1);
	_local_pos_sp.vz = _vel_sp(2);
	_local_pos_sp.acc_x = _a_sp(0);
	_local_pos_sp.acc_y = _a_sp(1);
	_local_pos_sp.acc_z = _a_sp(2);

	if(_local_pos_sp_pub != nullptr) {
		orb_publish(ORB_ID(vehicle_local_position_setpoint),_local_pos_sp_pub,&_local_pos_sp);
	}
	else {
		_local_pos_sp_pub = orb_advertise(ORB_ID(vehicle_local_position_setpoint),&_local_pos_sp);
	}
}

// control plane in mc mode for close to hover situations
void VtolPositionControl::control_mc() {
	// manual control
	if(!_v_control_mode.flag_control_altitude_enabled) {
		// full manual control
		control_manual();
		_reset_vel_z_int = true;
		_reset_vel_xy_int = true;
		_reset_pos = true;
		_reset_vel_error = true;
	}
	else if(_v_control_mode.flag_control_altitude_enabled && ! _v_control_mode.flag_control_position_enabled) {
		_reset_vel_xy_int = true;
		control_attitude();
	}
	else if(_v_control_mode.flag_control_altitude_enabled && _v_control_mode.flag_control_position_enabled) {
		control_position();
	}
}

// control plane in fw mode, thrust is done manually
void VtolPositionControl::control_fw() {
	// do normal roll, pitch control to tune low level controller
	calc_des_pos_and_vel();
	run_controller();
	/*
	math::Quaternion q_fw;
	float roll_des = _manual_control_sp.y;
	float pitch_des = -_manual_control_sp.x;
	math::Quaternion q_offset;
	q_offset.from_euler(0,-1.3f,0);
	math::Quaternion q;
	q.from_dcm(_R);
	q_fw = q*q_fw;
	math::Quaternion q_manual;
	q_manual.from_euler(roll_des,pitch_des,_yaw_sp);
	_q_sp = q_manual*q_offset;
	_thrust_sp = _manual_control_sp.z;
	*/
}

// not importatant for now
void VtolPositionControl::control_trans() {
	if(_vtol.status_prev == MC_MODE) {
		// do a front transition
		math::Quaternion q_pitch;
		q_pitch.from_euler(0,-_vtol.pitch_trans,0);
		_q_sp = _vtol.q_trans*q_pitch;
		_R_sp = _q_sp.to_dcm();
		_thrust_sp = _manual_control_sp.z;
		_vtol.pitch_trans += 0.2f*_dt;
		_vtol.pitch_trans = math::constrain(_vtol.pitch_trans,0.0f,1.0f);
	}
	else {
		// do a backtransition
		math::Quaternion q_pitch;
		q_pitch.from_euler(0,_vtol.pitch_trans,0);
		_q_sp = _vtol.q_trans*q_pitch;
		_R_sp = _q_sp.to_dcm();
		_thrust_sp = _manual_control_sp.z;
		_vtol.pitch_trans += 0.2f*_dt;
		_vtol.pitch_trans = math::constrain(_vtol.pitch_trans,0.0f,M_PI_2_F);
	}
}

void VtolPositionControl::control_position_step() {
	/* set target position to the position located 40 meters in front of the plane, 10 meters heigher up*/

}

void VtolPositionControl::compute_des_yaw() {
	// compute desired yaw for all modes
	if(_vtol.status == FW_MODE) {
		if(_vel(0)*_vel(0) + _vel(1)*_vel(1) > 16.0f) {
			_yaw_sp = atan2f(_vel(1),_vel(0));
		}
		else if(_vel(0)*_vel(0) + _vel(1)*_vel(1) < 16.0f) {
			_yaw_sp = _euler(2);
		}
	}
	else if(_vtol.status == TRANS_MODE) {
		
	}
	else {	// we are in multicopter mode, let user change yaw
		float yaw_sp_move_rate = _manual_control_sp.r * _params.man_yaw_max;
		_v_att_sp.yaw_sp_move_rate = yaw_sp_move_rate;
		_yaw_sp = _wrap_pi(_yaw_sp + yaw_sp_move_rate * _dt);
		float yaw_offs_max = _params.man_yaw_max / _params.mc_yaw_p;
		float yaw_offs = _wrap_pi(_yaw_sp - _v_att.yaw);
		if (yaw_offs < - yaw_offs_max) {
			_yaw_sp = _wrap_pi(_v_att.yaw - yaw_offs_max);
			} else if (yaw_offs > yaw_offs_max) {
			_yaw_sp = _wrap_pi(_v_att.yaw + yaw_offs_max);
		}
	}

	// reset yaw setpoint if necessary
	if(_reset_yaw_sp) {
		_yaw_sp = _euler(2);
	}
}

// for now we just want to test fw mode when we are in air, no other logic
void VtolPositionControl::update_vtol_state() {
	if(_manual_control_sp.aux2 > 0.0f) {
		_vtol.status = FW_MODE;
	}
	else {
		_vtol.status = MC_MODE;
	}
	// check if user wants to do transition
	/*
	if(_manual_control_sp.aux2 > 0.0f && _vtol.status == MC_MODE) {
		// check if possible to do front transtion
		bool allow = false;
		// only allow starting from position control mode
		allow = _v_control_mode.flag_control_position_enabled || _v_control_mode.flag_control_altitude_enabled;
		// only allow when vehicle is not moving too fast in xy_plane
		allow = (allow && fabsf(_vel(0)*_vel(0) + _vel(1)*_vel(1)) < 4.0f);
		// only allow if vehicle is more or less facing up
		float z_comp = _R(2,2);
		allow = (allow && z_comp > 0.9f);
		// set current yaw to desired yaw for transition
		if(allow) {
			_vtol.status_prev = _vtol.status;
			_vtol.status = TRANS_MODE;
			_vtol.q_trans.from_dcm(_R);
			_vtol.vel_trans_abs = 0.0f;
			_vtol.pitch_trans = 0.0f;
			_vel_sp.zero();
			_yaw_sp = _euler(2);	// reset yaw setpoint
		}
	}
	else if(_manual_control_sp.aux2 < 0 && _vtol.status == FW_MODE) {
		// do a backtransition
		_vtol.status_prev = _vtol.status;
		_vtol.status = TRANS_MODE;
		_vtol.pitch_trans = 0.0f;
		_vtol.q_trans.from_dcm(_R);
	}
	else if(_vtol.status == TRANS_MODE && _vtol.status_prev == MC_MODE) {
		if(_vtol.pitch_trans > 0.9f || fabsf(_vel(0)*_vel(0) + _vel(1)*_vel(1)) > 15.0f) {
			_vtol.status_prev = _vtol.status;
			_vtol.status = FW_MODE;
		}
	}
	else if(_vtol.status == TRANS_MODE && _vtol.status_prev == FW_MODE) {
		// get current angle of nose vector and earth z vector
		float angle = acosf(_R(2,2));
		if(fabsf(angle < 0.1f) || fabsf(_vel(0)*_vel(0) + _vel(1)*_vel(1)) < 3.0f) {
			_vtol.status = MC_MODE;
			_vtol.status_prev = MC_MODE;
			_vtol.vel_trans_abs = 0.0f;
			_reset_pos = true;
			_reset_vel_xy_int = true;
			_reset_vel_z_int = true;
		}
	}
	*/
}

void
VtolPositionControl::task_main_trampoline(int argc, char *argv[])
{
	VTOL_pos_control::g_control->task_main();
}

void VtolPositionControl::task_main()
{
	// do subscriptions
	_v_att_sub             = orb_subscribe(ORB_ID(vehicle_attitude));
	_v_control_mode_sub    = orb_subscribe(ORB_ID(vehicle_control_mode));
	_params_sub            = orb_subscribe(ORB_ID(parameter_update));
	_manual_control_sp_sub = orb_subscribe(ORB_ID(manual_control_setpoint));
	_local_pos_sub         = orb_subscribe(ORB_ID(vehicle_local_position));

	parameters_update();  // initialize parameter cache

	/* wakeup source*/
	struct pollfd fds[2];	// local position, parameters

	fds[0].fd     = _local_pos_sub;
	fds[0].events = POLLIN;
	fds[1].fd     = _params_sub;
	fds[1].events = POLLIN;

	hrt_abstime t_prev = 0;

	while (!_task_should_exit) {

		/* wait for up to 100ms for data */
		int pret = poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), 500);


		/* timed out - periodic check for _task_should_exit */
		if (pret == 0) {
			continue;
		}

		/* this is undesirable but not much we can do - might want to flag unhappy status */
		if (pret < 0) {
			warn("poll error %d, %d", pret, errno);
			/* sleep a bit before next try */
			usleep(100000);
			continue;
		}

		if (fds[1].revents & POLLIN) {	//parameters were updated, read them now
			/* read from param to clear updated flag */
			struct parameter_update_s update;
			orb_copy(ORB_ID(parameter_update), _params_sub, &update);

			/* update parameters from storage */
			parameters_update();
		}

		// poll topics
		vehicle_attitude_poll();
		vehicle_control_mode_poll();
		vehicle_manual_poll();
		parameters_update_poll();
		vehicle_local_pos_poll();

		// update data
		_pos(0) = _local_pos.x;
		_pos(1) = _local_pos.y;
		_pos(2) = _local_pos.z;

		_vel(0) = _local_pos.vx;
		_vel(1) = _local_pos.vy;
		_vel(2) = _local_pos.vz;

		_R.set(_v_att.R);
		_euler = _R.to_euler();
		// get usual dt estimate
		hrt_abstime t = hrt_absolute_time();
		_dt = t_prev != 0 ? (t - t_prev) * 0.000001f : 0.01f;
		t_prev = t;

		// reset yaw setpoint if not armed
		_reset_yaw_sp = false;

		compute_des_yaw();

		// failsave: user can switch to the fw controller, everything will be reset
		if(_manual_control_sp.aux1 <= 0.0f) {
		update_vtol_state();
		}
		else {
			// user has brutally switched to fw save mode, reset everything
			_vtol.status = MC_MODE;
			_vtol.status_prev = MC_MODE;
			_vtol.vel_trans_abs = 0.0f;
			_vtol.pitch_trans = 0.0f;
			_reset_pos = true;
			_reset_vel_xy_int = true;
			_reset_vel_z_int = true;
			_reset_vel_error = true;
		}

		if(_vtol.status == MC_MODE) {
			// fly as multicopter
			control_mc();
		}
		else if(_vtol.status == FW_MODE)
		{
			// fly as fw
			control_fw();
		}
		else {
			// do transition
			control_trans();
		}

		// send data to attitude controller, prevent double publishing with fw attitude controller(failsave)
		if(_manual_control_sp.aux1 < 0) {
			publish_att_sp();
		}
		// do some special logging for developing purposes
		send_to_log();
	}

	warnx("exit");
	_control_task = -1;
	_exit(0);
}

int
VtolPositionControl::start()
{
	ASSERT(_control_task == -1);

	/* start the task */
	_control_task = px4_task_spawn_cmd("vtol_pos_control",
				       SCHED_DEFAULT,
				       SCHED_PRIORITY_MAX - 10,
				       2048,
				       (px4_main_t)&VtolPositionControl::task_main_trampoline,
				       nullptr);

	if (_control_task < 0) {
		warn("task start failed");
		return -errno;
	}

	return OK;
}

int vtol_pos_control_main(int argc, char *argv[])
{
	if (argc < 1) {
		errx(1, "usage: vtol_pos_control {start|stop|status}");
	}

	if (!strcmp(argv[1], "start")) {

		if (VTOL_pos_control::g_control != nullptr) {
			errx(1, "already running");
		}

		VTOL_pos_control::g_control = new VtolPositionControl;

		if (VTOL_pos_control::g_control == nullptr) {
			errx(1, "alloc failed");
		}

		if (OK != VTOL_pos_control::g_control->start()) {
			delete VTOL_pos_control::g_control;
			VTOL_pos_control::g_control = nullptr;
			err(1, "start failed");
		}

		exit(0);
	}

	if (!strcmp(argv[1], "stop")) {
		if (VTOL_pos_control::g_control == nullptr) {
			errx(1, "not running");
		}

		delete VTOL_pos_control::g_control;
		VTOL_pos_control::g_control = nullptr;
		exit(0);
	}

	if (!strcmp(argv[1], "status")) {
		if (VTOL_pos_control::g_control) {
			errx(0, "running");

		} else {
			errx(1, "not running");
		}
	}

	warnx("unrecognized command");
	return 1;
}
