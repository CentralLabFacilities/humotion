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

#include <cassert>
#include <string>
#include <utility>

#include "humotion/server/motion_generator.h"

using humotion::server::Config;
using humotion::server::debug_data_t;
using humotion::server::MotionGenerator;

//! constructor
MotionGenerator::MotionGenerator(JointInterface* j, Config* cfg) {
	config = cfg;
	joint_interface_ = j;
	last_mouth_target_update_ = last_mouth_target_update_.min();
	last_gaze_target_update_ = last_mouth_target_update_.min();
}

//! destructor
MotionGenerator::~MotionGenerator() {
}

//! return all accumulated debug data
debug_data_t MotionGenerator::get_debug_data() {
	return debug_data_;
}

//! store debug data
void MotionGenerator::store_debug_data(const std::string& name, float value) {
	debug_data_[name] = value;
}

//! fetch the latest position and velocity and return the timestamp of that dataset
//! \param joint id
//! \param pointer to position variable
//! \param pointer to velocity variable
//! \return Timestamp of this dataset
humotion::Timestamp MotionGenerator::get_timestamped_state(int joint_id, float* position, float* velocity) {
	humotion::Timestamp stamp_p = joint_interface_->get_ts_position(joint_id).get_last_timestamp();
	humotion::Timestamp stamp_v = joint_interface_->get_ts_speed(joint_id).get_last_timestamp();
	humotion::Timestamp stamp;

	if (stamp_v < stamp_p) {
		// right now there is no velocity with that timestamp yet, therefore use the velocity ts
		// for both:
		stamp = stamp_v;
	}
	else {
		// both are available at the position ts, use that one
		stamp = stamp_p;
	}

	// fetch data
	*position = joint_interface_->get_ts_position(joint_id).get_interpolated_value(stamp);
	*velocity = joint_interface_->get_ts_speed(joint_id).get_interpolated_value(stamp);
	return stamp;
}

//! fetch the latest (=current) speed of a joint
//! \param joint_id
//! \return float value of joint speed
float MotionGenerator::get_current_speed(int joint_id) {
	return joint_interface_->get_ts_speed(joint_id).get_newest_value();
}

//! fetch the latest (=current) position of a joint
//! \param joint_id
//! \return float value of joint position
float MotionGenerator::get_current_position(int joint_id) {
	/*Timestamp tsl = joint_interface->get_ts_position(joint_id).get_last_timestamp();
	Timestamp now = Timestamp::now();
	Timestamp diff=now-tsl;
	printf("TIME DIFF %fs %fns\n",diff.sec, diff.nsec);*/

	return joint_interface_->get_ts_position(joint_id).get_newest_value();
}

//! update gaze target:
//! \param GazeState with target values for the overall gaze
void MotionGenerator::set_gaze_target(const GazeState& new_gaze_target) {
	// store value for next iteration
	requested_gaze_state_ = new_gaze_target;

	// keep track if the gaze targets are comming in regulary
	last_gaze_target_update_ = std::chrono::steady_clock::now();
}

//! update mouth state:
//! \param MouthState with target values for the mouth joints
void MotionGenerator::set_mouth_target(const MouthState& s) {
	// store value
	requested_mouth_target_ = s;

	// keep track if the mouth targets are comming in regulary
	last_mouth_target_update_ = std::chrono::steady_clock::now();
}

//! was there incoming gaze data the last second?
//! \return true if there was data incoming in the last second, false otherwise
bool MotionGenerator::gaze_target_input_active() {
	if (last_gaze_target_update_ + std::chrono::seconds(1) > std::chrono::steady_clock::now()) {
		// incoming data -> if gaze is disabled, enable it!
		joint_interface_->enable_gaze_joints();
		return true;
	}

	// else: no incoming data, disable!
	joint_interface_->disable_gaze_joints();
	return false;
}

//! was there incoming mouth data the last second?
//! \return true if there was data incoming in the last second, false otherwise
bool MotionGenerator::mouth_target_input_active() {
	if (last_mouth_target_update_ + std::chrono::seconds(1) > std::chrono::steady_clock::now()) {
		// incoming data -> if mouth is disabled, enable it!
		joint_interface_->enable_mouth_joints();
		return true;
	}

	// else: no incoming data, disable!
	joint_interface_->disable_mouth_joints();
	return false;
}

//! limit target to min/max bounds:
float MotionGenerator::limit_target(int joint_id, float val) {
	assert(joint_id < JointInterface::JOINT_ID_ENUM_SIZE);

	// fetch min/max for joint:
	float min = joint_interface_->get_joint_min(joint_id);
	float max = joint_interface_->get_joint_max(joint_id);

	if (max < min) {
		printf("> ERROR: how can min (%4.2f) be bigger than max (%4.2f)?? EXITING NOW\n", min, max);
		printf("> HINT : did you initialize the joints' min/max positions properly?\n");
		exit(0);
	}

	float limit_factor = 1.0;
	switch (joint_id) {
		default:
			limit_factor = 1.0;
			break;

		case (JointInterface::ID_EYES_LEFT_LR):
		case (JointInterface::ID_EYES_RIGHT_LR):
		case (JointInterface::ID_EYES_BOTH_UD):
			limit_factor = config->limit_omr_eye;
			break;

		case (JointInterface::ID_NECK_PAN):
		case (JointInterface::ID_NECK_TILT):
		case (JointInterface::ID_NECK_ROLL):
			limit_factor = config->limit_mr_neck;
			break;
	}

	float center = (min + max) / 2;
	float diff_min = center - min;
	float min_new = -limit_factor * diff_min + center;
	float diff_max = max - center;
	float max_new = limit_factor * diff_max + center;
	// printf("LIMIT: in [%3.1f:%3.1f] --> [%3.1f:%3.1f]\n",min,max,min_new,max_new);
	min = min_new;
	max = max_new;

	val = fmin(val, max);
	val = fmax(val, min);

	return val;
}
