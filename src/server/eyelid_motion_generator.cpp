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

#include "humotion/server/eyelid_motion_generator.h"
#include "humotion/server/server.h"
#include <random>

using humotion::server::Config;
using humotion::server::EyelidMotionGenerator;

static std::minstd_rand RANDOM_ENGINE;

//! constructor
EyelidMotionGenerator::EyelidMotionGenerator(JointInterface* j, Config* cfg) : EyeMotionGenerator(j, cfg) {
	saccade_blink_active_ = false;
	saccade_blink_requested_ = false;

	for (int i = 0; i < 2; i++) {
		eyelid_closed_[i] = false;
		eyeblink_active_[i] = false;
		eyeblink_timeout_[i] = std::chrono::steady_clock::now();
	}

	eyeblink_blocked_timeout_ = std::chrono::steady_clock::now();
}

//! destructor
EyelidMotionGenerator::~EyelidMotionGenerator() {
}

//! calculate joint targets
//! \TODO: make up motion twice as slow as down motion
void EyelidMotionGenerator::calculate_targets() {
	// printf("> humotion: calculating eyelid targets\n");

	// fetch current angles
	float eye_tilt_now = get_current_position(JointInterface::ID_EYES_BOTH_UD);

	float eyelid_target = 0.0;
	if (config->eyelids_follow_eyemotion) {
		// the eyelids shoudl follow the eye motion
		// therefore copy the eyeballs tilt position
		eyelid_target = eye_tilt_now;
	}

	// calculate left eyelid targets
	float eyelid_upper_left_target = eyelid_target + requested_gaze_state_.eyelid_opening_upper;
	float eyelid_lower_left_target = eyelid_target - requested_gaze_state_.eyelid_opening_lower;

	// calculate right eyelid targets
	float eyelid_upper_right_target = eyelid_target + requested_gaze_state_.eyelid_opening_upper;
	float eyelid_lower_right_target = eyelid_target - requested_gaze_state_.eyelid_opening_lower;

	// limit target angles
	eyelid_upper_left_target = limit_target(JointInterface::ID_EYES_LEFT_LID_UPPER, eyelid_upper_left_target);
	eyelid_lower_left_target = limit_target(JointInterface::ID_EYES_LEFT_LID_LOWER, eyelid_lower_left_target);
	eyelid_upper_right_target = limit_target(JointInterface::ID_EYES_RIGHT_LID_UPPER, eyelid_upper_right_target);
	eyelid_lower_right_target = limit_target(JointInterface::ID_EYES_RIGHT_LID_LOWER, eyelid_lower_right_target);

	// (temporarily) store the target
	joint_interface_->set_target(JointInterface::ID_EYES_LEFT_LID_UPPER, eyelid_upper_left_target, 0.0);
	joint_interface_->set_target(JointInterface::ID_EYES_LEFT_LID_LOWER, eyelid_lower_left_target, 0.0);
	joint_interface_->set_target(JointInterface::ID_EYES_RIGHT_LID_UPPER, eyelid_upper_right_target, 0.0);
	joint_interface_->set_target(JointInterface::ID_EYES_RIGHT_LID_LOWER, eyelid_lower_right_target, 0.0);

	// check for saccade
	check_for_saccade();

	// is there a blink request?
	start_external_eyeblinks(requested_gaze_state_.eyeblink_request_left, requested_gaze_state_.eyeblink_request_right);

	// do we have a saccade & do we want to exec an eyeblink?
	process_saccadic_eyeblinks();

	// execute for periodic eyeblinks every ? ms
	process_periodic_eyeblinks();

	// close eyes again after a given timeout
	handle_eyeblink_timeout();

	// eyeblinks override position target (if necessary)
	override_lids_for_eyeblink();
}

//! process any external blink requests
void EyelidMotionGenerator::start_external_eyeblinks(int duration_left, int duration_right) {
	// manual eyeblinks will ALWAYs get executed as we use
	// a negative block timeout (=timeout has already passed)
	if ((duration_left != 0) || (duration_right != 0)) {
		eyeblink_blocked_timeout_ = std::chrono::steady_clock::now() - std::chrono::seconds(100);
	}

	if (duration_left == 0) {
		// nothing to do
	}
	else if (duration_left < 0) {
		// infinite sleep -> close
		eyelid_closed_[LEFT] = true;
	}
	else {
		// timeout sleep
		start_eyeblink(LEFT, duration_left);
		eyelid_closed_[LEFT] = false;
	}

	if (duration_right == 0) {
		// nothing to do
	}
	else if (duration_right < 0) {
		// infinite sleep -> close
		eyelid_closed_[RIGHT] = true;
	}
	else {
		// timeout sleep
		start_eyeblink(RIGHT, duration_right);
		eyelid_closed_[RIGHT] = false;
	}
}

//! process saccadic blink requests
//! -> when we do a saccade, the chances to blink are much higher
void EyelidMotionGenerator::process_saccadic_eyeblinks() {
	if (saccade_blink_requested_) {
		// every n-th's saccade requests an eyeblink
		if (std::bernoulli_distribution(config->eyeblink_probability_after_saccade)(RANDOM_ENGINE)) {
			printf("> saccadic eyeblink:\n");
			start_eyeblink(LEFT, config->eyeblink_duration * 1000.0);
			start_eyeblink(RIGHT, config->eyeblink_duration * 1000.0);
		}
		saccade_blink_requested_ = false;
	}
}

//! process periodic blink requests
//! -> we want to have an eyeblink every n...m seconds
void EyelidMotionGenerator::process_periodic_eyeblinks() {
	if (eyeblink_active_[LEFT] || eyeblink_active_[RIGHT]) {
		std::uniform_real_distribution<float> uniform(config->eyeblink_periodic_distribution_lower,
		                                              config->eyeblink_periodic_distribution_upper);
		// random timeout for a new periodic eyeblink
		std::chrono::duration<float> seconds_to_next_blink(uniform(RANDOM_ENGINE));
		periodic_blink_start_time_ =
		   std::chrono::steady_clock::now() + std::chrono::duration_cast<std::chrono::milliseconds>(seconds_to_next_blink);
	}

	if (std::chrono::steady_clock::now() > periodic_blink_start_time_) {
		// printf("> periodic eyeblink:\n");
		start_eyeblink(LEFT, config->eyeblink_duration * 1000.0);
		start_eyeblink(RIGHT, config->eyeblink_duration * 1000.0);
	}
}

//! handle eyeblink timeouts.
//! this function will actually re-open the eyes after a predefined time
void EyelidMotionGenerator::handle_eyeblink_timeout() {
	auto now = std::chrono::steady_clock::now();

	// take care of re-opening eye timeout
	for (int i = LEFT; i <= RIGHT; i++) {
		if (eyeblink_active_[i]) {
			if (now >= eyeblink_timeout_[i]) {
				eyeblink_active_[i] = false;
			}
		}
	}

	// take care of blocking time
	if (eyeblink_active_[LEFT] || eyeblink_active_[RIGHT]) {
		eyeblink_blocked_timeout_ = now + std::chrono::duration_cast<std::chrono::milliseconds>(
		                                     std::chrono::duration<float>(config->eyeblink_blocked_time));
	}
}

//! override eyelid positions
//! this will override (=close) the eyelids for all possible eyeblink requests
void EyelidMotionGenerator::override_lids_for_eyeblink() {
	// close the requested eyelids
	if (eyeblink_active_[LEFT] || eyelid_closed_[LEFT]) {
		close_eyelid(JointInterface::ID_EYES_LEFT_LID_UPPER);
		close_eyelid(JointInterface::ID_EYES_LEFT_LID_LOWER);
	}
	if (eyeblink_active_[RIGHT] || eyelid_closed_[RIGHT]) {
		close_eyelid(JointInterface::ID_EYES_RIGHT_LID_UPPER);
		close_eyelid(JointInterface::ID_EYES_RIGHT_LID_LOWER);
	}
}

//! overwrite the intermediate target to close the given eyelid:
void EyelidMotionGenerator::close_eyelid(int joint_id) {
	float value = 0.0;

	// set upper to minimal allowed value (=as closed as possible) + a small offset
	// set lower to the same value (instead of max) in order to avoid collisions
	switch (joint_id) {
		default:
			// no eyelid -> return
			return;

		case (JointInterface::ID_EYES_LEFT_LID_UPPER):
		case (JointInterface::ID_EYES_LEFT_LID_LOWER):
			// use the upper value + 10 deg as close state:
			value = joint_interface_->get_joint_min(JointInterface::ID_EYES_LEFT_LID_UPPER) + 10.0;
			// overwrite last target_
			joint_interface_->set_target(joint_id, value, 0.0);
			break;

		case (JointInterface::ID_EYES_RIGHT_LID_UPPER):
		case (JointInterface::ID_EYES_RIGHT_LID_LOWER):
			// use the upper value + 10 deg as close state:
			value = joint_interface_->get_joint_min(JointInterface::ID_EYES_RIGHT_LID_UPPER) + 10.0;
			// overwrite last target_
			joint_interface_->set_target(joint_id, value, 0.0);
			break;
	}
}

//! start an eyeblink request
//! this will actually set a flag and a timeout for reopening the given eyelid pair again
//! \param int id of side (LEFT or RIGHT)
void EyelidMotionGenerator::start_eyeblink(int side, int duration) {
	// cout << "BLOCKED UNTIL " << eyeblink_blocked_timeout << "\n";
	if (std::chrono::steady_clock::now() < eyeblink_blocked_timeout_) {
		// return if we are still in the block time
		return;
	}
	// request for n ms eyeblink:
	eyeblink_timeout_[side] = std::chrono::steady_clock::now() + std::chrono::milliseconds(duration);

	eyeblink_active_[side] = true;
}

//! check if there is an ongoing saccade:
void EyelidMotionGenerator::check_for_saccade() {
	if (get_eye_saccade_active()) {
		// this is a saccade, check if not already in saccade:
		if (!saccade_blink_active_) {
			// this is a new saccade! restart blink timer
			saccade_blink_requested_ = true;
			saccade_blink_active_ = true;
		}
	}
	else {
		saccade_blink_active_ = false;
	}
}

//! publish targets to motor boards:
void EyelidMotionGenerator::publish_targets() {
	// publish values if there is an active gaze input within the last timerange
	if (gaze_target_input_active()) {
		joint_interface_->publish_target(JointInterface::ID_EYES_LEFT_LID_UPPER);
		joint_interface_->publish_target(JointInterface::ID_EYES_LEFT_LID_LOWER);
		joint_interface_->publish_target(JointInterface::ID_EYES_RIGHT_LID_UPPER);
		joint_interface_->publish_target(JointInterface::ID_EYES_RIGHT_LID_LOWER);
	}
}
