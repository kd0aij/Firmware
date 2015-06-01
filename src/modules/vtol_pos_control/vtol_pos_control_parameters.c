/****************************************************************************
 *
 *   Copyright (c) 2014 PX4 Development Team. All rights reserved.
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
 * @file vtol_att_control_params.c
 * Parameters for vtol attitude controller.
 *
 * @author Roman Bapst <bapstr@ethz.ch>
 */

#include <systemlib/param/param.h>

 /**
 * Vtol position error gain
 *
 *
 * @min 0.0
 * @group VTOL Position Control
 */
PARAM_DEFINE_FLOAT(VTP_POS_XY_P,1.0f);

 /**
 * Vtol position error gain
 *
 *
 * @min 0.0
 * @group VTOL Position Control
 */
PARAM_DEFINE_FLOAT(VTP_POS_Z_P,1.0f);

 /**
 * Vtol position error gain
 *
 *
 * @min 0.0
 * @group VTOL Position Control
 */
PARAM_DEFINE_FLOAT(VTP_VEL_XY_P,1.0f);

 /**
 * Vtol position error gain
 *
 *
 * @min 0.0
 * @group VTOL Position Control
 */
PARAM_DEFINE_FLOAT(VTP_VEL_Z_P,1.0f);

 /**
 * Vtol position error gain
 *
 *
 * @min 0.0
 * @group VTOL Position Control
 */
PARAM_DEFINE_FLOAT(VTP_VEL_XY_D,0.03f);

 /**
 * Vtol position error gain
 *
 *
 * @min 0.0
 * @group VTOL Position Control
 */
PARAM_DEFINE_FLOAT(VTP_VEL_Z_D,0.03f);

/**
 * Vtol position error gain
 *
 *
 * @min 0.0
 * @group VTOL Position Control
 */
PARAM_DEFINE_FLOAT(VTP_VEL_XY_I,0.0f);

/**
 * Vtol position error gain
 *
 *
 * @min 0.0
 * @group VTOL Position Control
 */
PARAM_DEFINE_FLOAT(VTP_VEL_Z_I,0.5f);

/**
 * Vtol xy velocity feedforward gain
 *
 *
 * @min 0.0
 * @group VTOL Position Control
 */
PARAM_DEFINE_FLOAT(VTP_VEL_XY_FF,0.5f);

/**
 * Vtol xy velocity feedforward gain
 *
 *
 * @min 0.0
 * @group VTOL Position Control
 */
PARAM_DEFINE_FLOAT(VTP_VEL_Z_FF,0.5f);

/**
 * Vtol position error gain
 *
 *
 * @min 0.0
 * @group VTOL Position Control
 */
PARAM_DEFINE_FLOAT(VTP_VZ_MAX,1.0f);

/**
 * Vtol acceleration during transition
 *
 *
 * @min 0.0
 * @group VTOL Position Control
 */
PARAM_DEFINE_FLOAT(VTP_ACC_TRANS,1.0f);

/**
 * Vtol angle at which ok to switch to fw mode
 *
 *
 * @min 0.0
 * @group VTOL Position Control
 */
PARAM_DEFINE_FLOAT(VTP_PTCH_TRANSIT,70.0f);

/**
 * Vtol airspeed at which ok to switch to fw mode
 *
 *
 * @min 0.0
 * @group VTOL Position Control
 */
PARAM_DEFINE_FLOAT(VTP_SPD_TRANSIT,12.0f);

/**
 * Vtol angular acceleration during backtransition
 *
 *
 * @min 0.0
 * @group VTOL Position Control
 */
PARAM_DEFINE_FLOAT(VTP_ACC_ANG,30.0f);

/**
 * Vtol scale from desired thrust acceleration to thrust signal
 *
 *
 * @min 0.0
 * @group VTOL Position Control
 */
PARAM_DEFINE_FLOAT(VTP_THRUST_SCALE,0.06f);

/**
 * Vtol scale from desired thrust acceleration to thrust signal
 *
 *
 * @min 0.0
 * @group VTOL Position Control
 */
PARAM_DEFINE_FLOAT(VTP_MAN_R_MAX,35.0f);

/**
 * Vtol scale from desired thrust acceleration to thrust signal
 *
 *
 * @min 0.0
 * @group VTOL Position Control
 */
PARAM_DEFINE_FLOAT(VTP_MAN_P_MAX,35.0f);

/**
 * Vtol scale from desired thrust acceleration to thrust signal
 *
 *
 * @min 0.0
 * @group VTOL Position Control
 */
PARAM_DEFINE_FLOAT(VTP_MAN_Y_MAX,120.0f);

/**
 * Pitch trim for fw flight
 *
 *
 * @min -2.0
 * @max 2.0
 * @group VTOL Position Control
 */
PARAM_DEFINE_FLOAT(VTP_PITCH_TRIM,0.0f);

/**
 * Pitch sensitivity to control stick for fw flight
 *
 *
 * @group VTOL Position Control
 */
PARAM_DEFINE_FLOAT(VTP_PITCH_SENS,1.0f);

/**
 * Target cruise speed in forward flight
 *
 *
 * @group VTOL Position Control
 */
PARAM_DEFINE_FLOAT(VTP_CRUISE_SPD,10.0f);
