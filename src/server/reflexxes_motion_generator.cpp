/*
 * This file is part of humotion
 *
 * Copyright(c) sschulz <AT> techfak.uni-bielefeld.de
 * http://opensource.cit-ec.de/projects/humotion
 *
 * This file may be licensed under the terms of the
 * GNU Lesser General Public License Version 3 (the ``LGPL''),
 * or (at your option) any later version.
 *
 * Software distributed under the License is distributed
 * on an ``AS IS'' basis, WITHOUT WARRANTY OF ANY KIND, either
 * express or implied. See the LGPL for the specific language
 * governing rights and limitations.
 *
 * You should have received a copy of the LGPL along with this
 * program. If not, go to http://www.gnu.org/licenses/lgpl.html
 * or write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 *
 * The development of this software was supported by the
 * Excellence Cluster EXC 277 Cognitive Interaction Technology.
 * The Excellence Cluster EXC 277 is a grant of the Deutsche
 * Forschungsgemeinschaft (DFG) in the context of the German
 * Excellence Initiative.
 */

#include "humotion/server/eye_motion_generator.h"
#include <cassert>

using humotion::server::Config;
using humotion::server::ReflexxesMotionGenerator;

//! constructor
ReflexxesMotionGenerator::ReflexxesMotionGenerator(JointInterface* j, Config* c, int dof, float t) : MotionGenerator(j, c) {
	dof_count = dof;

	// create Reflexxes API for <dof> DOF actuator
	reflexxes_api = new ReflexxesAPI(dof, t);
	reflexxes_position_input = new RMLPositionInputParameters(dof);
	reflexxes_position_output = new RMLPositionOutputParameters(dof);

	// synchronize phase
	reflexxes_motion_flags.SynchronizationBehavior =
	   // RMLPositionFlags::NO_SYNCHRONIZATION;
	   RMLPositionFlags::PHASE_SYNCHRONIZATION_IF_POSSIBLE;
}

//! destructor
ReflexxesMotionGenerator::~ReflexxesMotionGenerator() {
}

//! feed motion generator with target data:
//! \param dof id
//! \param target angle
//! \param max_speed max reachable speed during accel
//! \param max_accel max allowable acceleration
void ReflexxesMotionGenerator::reflexxes_set_input(int dof, float target, float current_position, float current_velocity,
                                                   humotion::Timestamp timestamp, float max_speed, float max_accel) {
	assert(dof < dof_count);

	// set up reflexxes control loop
	reflexxes_position_input->TargetPositionVector->VecData[dof] = target;
	reflexxes_position_input->SelectionVector->VecData[dof] = true;
	reflexxes_position_input->MaxVelocityVector->VecData[dof] = max_speed;
	reflexxes_position_input->MaxAccelerationVector->VecData[dof] = max_accel;

	// target speed is zero (really?)
	reflexxes_position_input->TargetVelocityVector->VecData[dof] = 0.0;

	// safety: libreflexxes does not like zero accellerations...
	if (reflexxes_position_input->MaxAccelerationVector->VecData[dof] == 0.0) {
		reflexxes_position_input->MaxAccelerationVector->VecData[dof] = 0.0001;
	}
}

//! calculate motion profile

void ReflexxesMotionGenerator::reflexxes_calculate_profile() {
	int res = reflexxes_api->RMLPosition(*reflexxes_position_input, reflexxes_position_output, reflexxes_motion_flags);

	if (res < 0) {
		if (res == ReflexxesAPI::RML_ERROR_INVALID_INPUT_VALUES) {
			printf("> ReflexxesMotionGenerator --> RML_ERROR_INVALID_INPUT_VALUES error\n");
		}
		else {
			printf("> ReflexxesMotionGenerator --> UNKNOWN_ERROR: reflexxes error %d\n", res);
		}
	}

	// feed back values
	for (int i = 0; i < dof_count; i++) {
		reflexxes_position_input->CurrentPositionVector->VecData[i] = reflexxes_position_output->NewPositionVector->VecData[i];

		reflexxes_position_input->CurrentVelocityVector->VecData[i] = reflexxes_position_output->NewVelocityVector->VecData[i];

		reflexxes_position_input->CurrentAccelerationVector->VecData[i] =
		   reflexxes_position_output->NewAccelerationVector->VecData[i];
	}
}
