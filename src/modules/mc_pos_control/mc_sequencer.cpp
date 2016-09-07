#include "mc_sequencer.h"

void control_sequencer(
	struct control_state_s &ctrl_state,
	struct vehicle_attitude_setpoint_s &att_sp,
	struct manual_control_setpoint_s &manual,
	math::Matrix<3, 3> &R_sp,
	float &rollRate, float &pitchRate, float &yawRate)
{

	enum Seq_state {
		IDLE, CLIMB, ROLL, PITCH, FINISH
	};
	static Seq_state cur_state = IDLE;
	static Seq_state last_state = FINISH;

	static math::Quaternion q_end;
	static math::Quaternion q_cur;
	static math::Vector<3> euler_end;
	static uint64_t start_finish = 0;
	const uint64_t climb_dur = 1.0 * 1000000;

	uint64_t cur_time = hrt_absolute_time();
	static uint64_t start_time = cur_time;

	/* if seq_switch just transitioned from off to on, begin substituting sequencer
	 * controls for manual controls. The sequencer could be a separate module publishing
	 * manual_control_setpoint messages, or a smaller message containing only
	 * x, y, z, r
	 */
//						uint8_t seq_switch = _manual.seq_switch;

	// for SITL, simulate seq_switch activation every 5 seconds
	static uint8_t seq_switch = manual_control_setpoint_s::SWITCH_POS_OFF;

	if ((cur_time - start_time) > 10000000) {
		if (seq_switch == manual_control_setpoint_s::SWITCH_POS_OFF) {
			seq_switch = manual_control_setpoint_s::SWITCH_POS_ON;
			PX4_INFO("seq_switch on: at %f", (double) start_time / 1e6);
		}

		start_time = cur_time;
	}

	// reduce thrust when inverted (Earth z points down in body frame)
	bool inversion = (att_sp.R_body[8] > 0.0f);
	static bool inverted = false;

	if (inversion != inverted) {
		inverted = inversion;

		// reduce thrust when inverted (z component of Earth z points down)
		if (inverted) {
			att_sp.thrust = 0.2f;

		} else {
			att_sp.thrust = 0.8f;
		}

		PX4_INFO("inversion at %6.3f, roll: %6.3f, pitch: %6.3f",
			 (double) cur_time / 1e6, (double) att_sp.roll_body,
			 (double) att_sp.pitch_body);
	}

	/* substitute attitude sequence for _manual_control_setpoint */
	if (cur_state != last_state) {
		PX4_INFO("state: %d at %6.3f", cur_state, (double) cur_time / 1e6);
		last_state = cur_state;
	}

	switch (cur_state) {

	case CLIMB: {

			rollRate = 0.0f;
			pitchRate = 0.0f;
			yawRate = 0.0f;
			att_sp.thrust = 0.9f;

			if ((cur_time - start_time) > climb_dur) {
				cur_state = PITCH;
			}

			break;
		}

	case ROLL: {
			rollRate = 1.0f;
			pitchRate = 0.0f;
			yawRate = 0.0f;

			q_cur.set(ctrl_state.q);
			math::Quaternion q_err = q_cur * q_end.conjugated();
			float error = acosf(fabsf(q_err.data[0]));

			if ((cur_time - start_time) > (climb_dur + 250000) && (error < 0.24f)) {
				rollRate = 0.0f;
				R_sp.from_euler(euler_end.data[0], euler_end.data[1],
						euler_end.data[2]);
				memcpy(&att_sp.R_body[0], R_sp.data, sizeof(att_sp.R_body));
				cur_state = FINISH;
				start_finish = cur_time;

				printf("finish roll: q_cur: ");
				q_cur.print();
				printf("q_end: ");
				q_end.print();
				printf("q_err: ");
				q_err.print();
				PX4_INFO("error %6.3f", (double) error);
			}

			break;
		}

	case PITCH: {
			rollRate = 0.0f;
			pitchRate = 1.0f;
			yawRate = 0.0f;

			q_cur.set(ctrl_state.q);
			math::Quaternion q_err = q_cur * q_end.conjugated();
			float error = acosf(fabsf(q_err.data[0]));

			if ((cur_time - start_time) > (climb_dur + 250000) && (error < 0.24f)) {
				pitchRate = 0.0f;
				R_sp.from_euler(euler_end.data[0], euler_end.data[1],
						euler_end.data[2]);
				memcpy(&att_sp.R_body[0], R_sp.data, sizeof(att_sp.R_body));
				cur_state = FINISH;
				start_finish = cur_time;

				printf("finish pitch: q_cur: ");
				q_cur.print();
				printf("q_end: ");
				q_end.print();
				printf("q_err: ");
				q_err.print();
				PX4_INFO("error %6.3f", (double) error);
			}

			break;
		}

	case FINISH: {
			// return to starting orientation
			q_cur.set(ctrl_state.q);
			math::Quaternion q_err = q_cur * q_end.conjugated();
			float error = acosf(fabsf(q_err.data[0]));

			if (error < 0.005f || (cur_time - start_finish) > 500000) {
				cur_state = IDLE;
				seq_switch = manual_control_setpoint_s::SWITCH_POS_OFF;
				PX4_INFO("sequence end at %6.3f, duration: %6.3f",
					 (double) cur_time / 1e6,
					 (double)(cur_time - start_time) / 1e6);
				printf("q_cur: ");
				q_cur.print();
				printf("q_end: ");
				q_end.print();
				printf("q_err: ");
				q_err.print();
				PX4_INFO("error %6.3f", (double) error);
				printf("final Euler angles: ");
				q_cur.to_euler().print();
			}

			break;
		}

	case IDLE: {
			// don't change manual inputs
			rollRate = manual.y;
			pitchRate = -manual.x;
			yawRate = manual.r;

			if (seq_switch == manual_control_setpoint_s::SWITCH_POS_ON) {
				cur_state = CLIMB;

				// initialize sequencer
				q_end.set(ctrl_state.q);
				euler_end = q_end.to_euler();
				printf("starting Euler angles: ");
				q_end.to_euler().print();
				printf("q_end: ");
				q_end.print();
			}

			break;
		}
	}

}
