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

#include <string>

#include "humotion/server/controller.h"
#include "humotion/server/eye_motion_generator.h"
#include "humotion/server/eyebrow_motion_generator.h"
#include "humotion/server/eyelid_motion_generator.h"
#include "humotion/server/mouth_motion_generator.h"
#include "humotion/server/neck_motion_generator.h"
#include "humotion/timestamp.h"

// using namespace std;
// using namespace humotion;
// using namespace humotion::server;

using humotion::server::Config;
using humotion::server::Controller;
using humotion::server::debug_data_t;

//! constructor
Controller::Controller(JointInterface* j) {
	activated_ = false;
	joint_interface_ = j;

	config_ = new Config();
}

//! destructor
Controller::~Controller() {
}

//! initialise motion generators
void Controller::init_motion_generators() {
	// NOTE: the order of these generators is important!
	//       (i.e. the neck generator must be added after the eye generator!)

	// eye motion generation:
	add_motion_generator(new EyeMotionGenerator(joint_interface_, config_));

	// eyelid motion generator
	add_motion_generator(new EyelidMotionGenerator(joint_interface_, config_));

	// neck motion generator
	add_motion_generator(new NeckMotionGenerator(joint_interface_, config_));

	// mouth motion generator
	add_motion_generator(new MouthMotionGenerator(joint_interface_, config_));

	// eyebrow motion generator
	add_motion_generator(new EyebrowMotionGenerator(joint_interface_, config_));
}

//! add a single motion genrator
void Controller::add_motion_generator(MotionGenerator* m) {
	motion_generator_vector_.push_back(m);
}

//! calculate target angles for all motion generators:
void Controller::calculate_targets() {
	Controller::motion_generator_vector_t::iterator it;
	for (it = motion_generator_vector_.begin(); it < motion_generator_vector_.end(); it++) {
		MotionGenerator* mg = *it;
		// calculate targets
		mg->calculate_targets();
	}
}

debug_data_t Controller::get_debug_data() {
	debug_data_t debug_data;

	Controller::motion_generator_vector_t::iterator it;
	for (it = motion_generator_vector_.begin(); it < motion_generator_vector_.end(); it++) {
		MotionGenerator* mg = *it;
		// fetch and append debug data
		debug_data_t dataset = mg->get_debug_data();
		debug_data.insert(dataset.begin(), dataset.end());
	}

	// fetch data from controller as well
	debug_data_t controller_dataset = debug_data_;
	debug_data.insert(controller_dataset.begin(), controller_dataset.end());

	return debug_data;
}

//! store debug data
void Controller::store_debug_data(std::string name, float value) {
	debug_data_[name] = value;
}

//! publish all target angles to the devices:
//! NOTE: this is done in an extra loop to have a low delay between consequent sets:
void Controller::publish_targets() {
	Controller::motion_generator_vector_t::iterator it;
	for (it = motion_generator_vector_.begin(); it < motion_generator_vector_.end(); it++) {
		(*it)->publish_targets();
	}
}

humotion::GazeState Controller::relative_gaze_to_absolute_gaze(humotion::GazeState relative) {
	double pan, tilt, roll;
	double neck_pan = 0.0;
	double neck_tilt = 0.0;

	humotion::GazeState absolute_gaze = relative;

	// incoming gaze state wants to set a relative gaze angle
	// in order to calc the new absolute gaze, we need to go back
	// in time and find out where the head was pointing at that specific time:
	Timestamp relative_target_timestamp = relative.timestamp;

	// check if this timestamp allows a valid conversion:
	Timestamp history_begin = joint_interface_->get_ts_position(JointInterface::ID_NECK_PAN).get_first_timestamp();
	Timestamp history_end = joint_interface_->get_ts_position(JointInterface::ID_NECK_PAN).get_last_timestamp();

	// printf("> incoming: %f, history is %f to %f\n",relative_target_timestamp.to_seconds(),
	// history_begin.to_seconds(), history_end.to_seconds());

	// our history keeps the last n elements in a timestamped list
	if ((relative_target_timestamp < history_begin) || (history_begin.is_null())) {
		// when the incoming data is older than that it makes no sense
		// to do any guesswork and try to calculate a valid absolute target
		// therefore we will use the last known targets (see below)
		// in case we did not see this timestamp before, show a warning:
		if (last_known_absolute_timestamp_ != relative_target_timestamp) {
			printf("> WARNING: restored/guessed absolute target for unknown timestamp "
			       "%f (tsmap = [%f - %f]) [this should only happen during startup]\n",
			       relative_target_timestamp.to_seconds(), history_begin.to_seconds(), history_end.to_seconds());
			last_known_absolute_target_pan_ = 0.0;
			last_known_absolute_target_tilt_ = 0.0;
			last_known_absolute_target_roll_ = 0.0;
		}
	}
	else {
		// all fine, we can reconstruct the absolute target
		// fetch head / camera pose during that timestamp
		neck_pan =
		   joint_interface_->get_ts_position(JointInterface::ID_NECK_PAN).get_interpolated_value(relative_target_timestamp);
		double eye_l_pan =
		   joint_interface_->get_ts_position(JointInterface::ID_EYES_LEFT_LR).get_interpolated_value(relative_target_timestamp);
		double eye_r_pan = joint_interface_->get_ts_position(JointInterface::ID_EYES_RIGHT_LR)
		                      .get_interpolated_value(relative_target_timestamp);
		last_known_absolute_target_pan_ = neck_pan + (eye_l_pan + eye_r_pan) / 2.0;
		//
		neck_tilt =
		   joint_interface_->get_ts_position(JointInterface::ID_NECK_TILT).get_interpolated_value(relative_target_timestamp);
		double eye_tilt =
		   joint_interface_->get_ts_position(JointInterface::ID_EYES_BOTH_UD).get_interpolated_value(relative_target_timestamp);
		last_known_absolute_target_tilt_ = neck_tilt + eye_tilt;
		//
		last_known_absolute_target_roll_ =
		   joint_interface_->get_ts_position(JointInterface::ID_NECK_ROLL).get_interpolated_value(relative_target_timestamp);

		// safe this timestamp as known:
		last_known_absolute_timestamp_ = relative_target_timestamp;
	}

	pan = last_known_absolute_target_pan_;
	tilt = last_known_absolute_target_tilt_;
	roll = last_known_absolute_target_roll_;

	// substract offsets
	pan -= relative.pan_offset;
	tilt -= relative.tilt_offset;
	roll -= relative.roll_offset;

	// build up absolute target
	absolute_gaze.gaze_type = GazeState::GAZETYPE_ABSOLUTE;
	absolute_gaze.pan = pan + relative.pan;
	absolute_gaze.tilt = tilt + relative.tilt;
	absolute_gaze.roll = roll + relative.roll;
	// printf("pan  now = %4.1f, rel=%4.1f ===> %4.2f\n", pan, relative.pan, absolute_gaze.pan);
	// printf("tilt now = %4.1f, rel=%4.1f ===> %4.2f\n", tilt, relative.tilt, absolute_gaze.tilt);

	// update the timestamp. this gaze target is now absolute, set appropriate timestamp
	// printf("%f %f %f 333\n", absolute_gaze.pan, absolute_gaze.tilt, absolute_gaze.roll);
	// absolute_gaze.tilt = 0.0;
	// absolute_gaze.roll = 0.0;

	// absolute_gaze.roll = absolute_gaze.pan; //0.012;
	absolute_gaze.timestamp = Timestamp::now();

	// store debug data:
	// this is the position we had at the ts of the relative target
	store_debug_data("controller/pan_neck_position_at_relative_ts", neck_pan);
	store_debug_data("controller/pan_overall_position_at_relative_ts", last_known_absolute_target_pan_);
	// this is the relative movement that was requested
	store_debug_data("controller/pan_target_relative", relative.pan);
	// this is the calculated overall target
	store_debug_data("controller/pan_target", absolute_gaze.pan);

	// FIXME: use ros TF for that calculation...
	// see http://wiki.ros.org/tf/Tutorials/Time%20travel%20with%20tf%20%28C%2B%2B%29
	// ros::Time past = now - ros::Duration(5.0);
	// listener.waitForTransform("/turtle2", now,J "/turtle1", past, "/world", ros::Duration(1.0));
	// listener.lookupTransform("/turtle2", now, "/turtle1", past, "/world", transform);
	// absolute_gaze.dump();

	return absolute_gaze;
}

//! activate controller
void Controller::set_activated(void) {
	activated_ = true;
}

//! update gaze target:
//! \param GazeState with target values for the overall gaze
void Controller::set_gaze_target(humotion::GazeState new_gaze_target) {
	if (!activated_) {
		// not yet initialized, ignore incoming targets
		return;
	}

	humotion::GazeState target_gaze;
	// new_gaze_target.dump();

	// relative or absolute gaze update?
	if (new_gaze_target.gaze_type == GazeState::GAZETYPE_RELATIVE) {
		// relative gaze target -> calculate target angles
		target_gaze = relative_gaze_to_absolute_gaze(new_gaze_target);
	}
	else {
		// already absolute gaze, set this
		target_gaze = new_gaze_target;
	}

	Controller::motion_generator_vector_t::iterator it;
	for (it = motion_generator_vector_.begin(); it < motion_generator_vector_.end(); it++) {
		(*it)->set_gaze_target(target_gaze);
	}
}

//! update mouth state:
//! \param MouthState with target values for the mouth joints
void Controller::set_mouth_target(MouthState s) {
	if (!activated_) {
		// not yet initialized, ignore incoming targets
		return;
	}

	Controller::motion_generator_vector_t::iterator it;
	for (it = motion_generator_vector_.begin(); it < motion_generator_vector_.end(); it++) {
		(*it)->set_mouth_target(s);
	}
}

//! access the configuration
Config* Controller::get_config() {
	return config_;
}
