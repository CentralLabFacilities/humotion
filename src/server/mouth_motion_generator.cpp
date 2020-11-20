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

#include "humotion/server/mouth_motion_generator.h"

using humotion::server::Config;
using humotion::server::MouthMotionGenerator;

// minimum mouth opening
const float MouthMotionGenerator::MOUTH_MIN_OPENING = 7.0; // mm

//! constructor
MouthMotionGenerator::MouthMotionGenerator(JointInterface* j, Config* cfg) : MotionGenerator(j, cfg) {
}

//! destructor
MouthMotionGenerator::~MouthMotionGenerator() {
}

//! calculate joint targets
void MouthMotionGenerator::calculate_targets() {
	// printf("> humotion: calculating mouth targets\n");
	update_mouth_target(JointInterface::ID_LIP_LEFT_UPPER, JointInterface::ID_LIP_LEFT_LOWER);
	update_mouth_target(JointInterface::ID_LIP_CENTER_UPPER, JointInterface::ID_LIP_CENTER_LOWER);
	update_mouth_target(JointInterface::ID_LIP_RIGHT_UPPER, JointInterface::ID_LIP_RIGHT_LOWER);
}

//! calculate mouth target angles for a given combination of upper/lower joints
//! \param int upper joint id
//! \param int lower joint id
void MouthMotionGenerator::update_mouth_target(int upper_id, int lower_id) {
	// fetch min/max for joints
	float min_upper = joint_interface_->get_joint_min(upper_id);
	float max_lower = joint_interface_->get_joint_max(lower_id);
	float min_opening = MOUTH_MIN_OPENING;

	// fetch position & opening for joint, parameter is only used to
	// determine LEFT/CENTER/RIGHT. upper/lower plays no role here
	float position = mouthstate_to_position(requested_mouth_target_, upper_id);
	float opening = min_opening + mouthstate_to_opening(requested_mouth_target_, upper_id);

	// check opening larger than minimum
	if (opening < min_opening) {
		// oops, not safe to move to this opening, abort move!
		// printf("> invalid opening (%f)\n", opening);
		opening = min_opening;
	}

	float unsafe_target_upper = position - opening / 2.0;
	float unsafe_target_lower = position + opening / 2.0;

	// now check if this would exceed the allowed limits for that joint
	float distance_min_upper = unsafe_target_upper - min_upper;
	if (distance_min_upper < 0) {
		// oops, we will get to close to the safe border, abort move!
		// printf("> collision on TOP [u=%4.2f l=%4.2f]\n",unsafe_target_upper,unsafe_target_lower);
		unsafe_target_upper = min_upper;
		unsafe_target_lower = unsafe_target_upper + opening;
	}

	float distance_max_lower = max_lower - unsafe_target_lower;
	if (distance_max_lower < 0) {
		// oops, we will get to close to the safe border, abort move!
		// printf("> collision on BOT[u=%4.2f l=%4.2f]\n",unsafe_target_upper,unsafe_target_lower);
		unsafe_target_lower = max_lower;
		unsafe_target_upper = unsafe_target_lower - opening;
	}

	// tell the joint about the new values
	joint_interface_->set_target(upper_id, unsafe_target_upper, 0.0);
	joint_interface_->set_target(lower_id, unsafe_target_lower, 0.0);
}

//! extract opening from mouth state for given joint enum id
//! \param MouthState m
//! \param int enum with joint id
float MouthMotionGenerator::mouthstate_to_opening(const MouthState& m, int e) {
	switch (e) {
		default:
			printf("> get_opening(0x%02X): invalid joint enum!\n", e);
			exit(EXIT_FAILURE);
		case (JointInterface::ID_LIP_LEFT_UPPER):
		case (JointInterface::ID_LIP_LEFT_LOWER):
			return m.opening_left;
		case (JointInterface::ID_LIP_CENTER_UPPER):
		case (JointInterface::ID_LIP_CENTER_LOWER):
			return m.opening_center;
		case (JointInterface::ID_LIP_RIGHT_UPPER):
		case (JointInterface::ID_LIP_RIGHT_LOWER):
			return m.opening_right;
	}
}

//! extract position from mouth state for given joint enum id
//! \param MouthState m
//! \param int enum with joint id
float MouthMotionGenerator::mouthstate_to_position(const MouthState& m, int e) {
	switch (e) {
		default:
			printf("> get_position(0x%02X): invalid joint enum!\n", e);
			exit(EXIT_FAILURE);
		case (JointInterface::ID_LIP_LEFT_UPPER):
		case (JointInterface::ID_LIP_LEFT_LOWER):
			return m.position_left;
		case (JointInterface::ID_LIP_CENTER_UPPER):
		case (JointInterface::ID_LIP_CENTER_LOWER):
			return m.position_center;
		case (JointInterface::ID_LIP_RIGHT_UPPER):
		case (JointInterface::ID_LIP_RIGHT_LOWER):
			return m.position_right;
	}
}

//! publish targets to motor boards:
void MouthMotionGenerator::publish_targets() {
	// publish values if there is an active gaze input within the last timerange
	if (mouth_target_input_active()) {
		joint_interface_->publish_target(JointInterface::ID_LIP_LEFT_UPPER);
		joint_interface_->publish_target(JointInterface::ID_LIP_LEFT_LOWER);
		joint_interface_->publish_target(JointInterface::ID_LIP_CENTER_UPPER);
		joint_interface_->publish_target(JointInterface::ID_LIP_CENTER_LOWER);
		joint_interface_->publish_target(JointInterface::ID_LIP_RIGHT_UPPER);
		joint_interface_->publish_target(JointInterface::ID_LIP_RIGHT_LOWER);
	}
}
