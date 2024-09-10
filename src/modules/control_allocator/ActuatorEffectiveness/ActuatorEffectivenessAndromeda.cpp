/****************************************************************************
 *
 *   Copyright (c) 2021-2023 PX4 Development Team. All rights reserved.
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

#include "ActuatorEffectivenessAndromeda.hpp"

using namespace matrix;

ActuatorEffectivenessAndromeda::ActuatorEffectivenessAndromeda(ModuleParams *parent)
	: ModuleParams(parent),
	  _mc_rotors_vertical(this, ActuatorEffectivenessRotors::AxisConfiguration::FixedUpwards, true),
	  _mc_rotors_lateral(this, ActuatorEffectivenessRotors::AxisConfiguration::FixedUpwards, true),
	  _tilts(this)
{
}

bool
ActuatorEffectivenessAndromeda::getEffectivenessMatrix(Configuration &configuration,
		EffectivenessUpdateReason external_update)
{
	if (external_update == EffectivenessUpdateReason::NO_EXTERNAL_UPDATE) {
		return false;
	}

	bool rotors_added_successfully = false;

	//Vertical forces matrix
	configuration.selected_matrix = 0;
	rotors_added_successfully = _mc_rotors_vertical.addActuators(configuration);

	// Lateral forces matrix
	configuration.selected_matrix = 1;
	rotors_added_successfully = _mc_rotors_lateral.addActuators(configuration);

	*configuration.num_actuators /=2;

	// Tilts
	configuration.selected_matrix = 0;
	_first_tilt_idx = configuration.num_actuators_matrix[configuration.selected_matrix];

	const bool tilts_added_successfully = _tilts.addActuators(configuration);

	return (rotors_added_successfully && tilts_added_successfully);
}

void ActuatorEffectivenessAndromeda::updateSetpoint(const matrix::Vector<float, NUM_AXES> &control_sp,
		int matrix_index, ActuatorVector &actuator_sp, const matrix::Vector<float, NUM_ACTUATORS> &actuator_min,
		const matrix::Vector<float, NUM_ACTUATORS> &actuator_max)
{

	for(int i=0; i<_servo_count; i++){

		actuator_sp(i) = actuator_sp(i) <  math::radians(_servo_param[i].angle_min) ?  math::radians(_servo_param[i].angle_min) : actuator_sp(i);
		actuator_sp(i) = actuator_sp(i) >  math::radians(_servo_param[i].angle_max) ?  math::radians(_servo_param[i].angle_max) : actuator_sp(i);
		// PX4_INFO("%d) tilt_sp: %f", i, (double)tilt_sp[i]);
	}

}

void ActuatorEffectivenessAndromeda::getUnallocatedControl(int matrix_index, control_allocator_status_s &status)
{
	// Note: the values '-1', '1' and '0' are just to indicate a negative,
	// positive or no saturation to the rate controller. The actual magnitude is not used.
	if (_yaw_tilt_saturation_flags.tilt_yaw_pos) {
		status.unallocated_torque[2] = 1.f;

	} else if (_yaw_tilt_saturation_flags.tilt_yaw_neg) {
		status.unallocated_torque[2] = -1.f;

	} else {
		status.unallocated_torque[2] = 0.f;
	}
}
