// issue a sequence of "manual" control commands in (rate-only) ACRO mode
// Each step will hold a rate for a specified time or till a specified attitude is reached
// This module will override the manual control topic: manual_control_setpoint
// To avoid changing mc_pos_control, it would be nice to interrupt publishing by sensors.cpp,
// but that would require changing it... Perhaps best to add a new "flight mode" called
// "sequence" and add it to both commander and mc_pos_control.


