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

#include "humotion/server/eyebrow_motion_generator.h"
#include "humotion/server/server.h"

// using namespace std;
// using namespace humotion;
// using namespace humotion::server;

using humotion::server::EyebrowMotionGenerator;

//! constructor
EyebrowMotionGenerator::EyebrowMotionGenerator(JointInterface* j, Config* cfg) : MotionGenerator(j, cfg) {
}

//! destructor
EyebrowMotionGenerator::~EyebrowMotionGenerator() {
}

//! calculate joint targets
void EyebrowMotionGenerator::calculate_targets() {
	// printf("> humotion: calculating eyebrow targets\n");
	float eyebrow_left_target = requested_gaze_state_.eyebrow_left;
	float eyebrow_right_target = requested_gaze_state_.eyebrow_right;

	// store targets
	joint_interface_->set_target(JointInterface::ID_EYES_LEFT_BROW, eyebrow_left_target, 0.0);
	joint_interface_->set_target(JointInterface::ID_EYES_RIGHT_BROW, eyebrow_right_target, 0.0);
}

//! publish targets to motor boards:
void EyebrowMotionGenerator::publish_targets() {
	// publish values if there is an active gaze input within the last timerange
	if (gaze_target_input_active()) {
		joint_interface_->publish_target(JointInterface::ID_EYES_LEFT_BROW);
		joint_interface_->publish_target(JointInterface::ID_EYES_RIGHT_BROW);
	}
}
