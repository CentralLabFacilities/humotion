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

#include "humotion/server/gaze_motion_generator.h"
#include "humotion/server/server.h"

using humotion::server::Config;
using humotion::server::GazeMotionGenerator;

//! constructor
GazeMotionGenerator::GazeMotionGenerator(JointInterface* j, Config* cfg, int dof, float t)
   : ReflexxesMotionGenerator(j, cfg, dof, t) {
}

//! destructor
GazeMotionGenerator::~GazeMotionGenerator() {
}

//! update gaze target:
//! \param GazeState with target values for the overall gaze
void GazeMotionGenerator::set_gaze_target(GazeState new_gaze_target) {
	if (requested_gaze_state_.gaze_type == GazeState::GAZETYPE_RELATIVE) {
		printf("> ERROR: gaze targets should be converted to absolute before calling this\n");
		exit(EXIT_FAILURE);
	}

	// check magnitude of gaze change to detect eye-neck saccades
	float dist = fabs(requested_gaze_state_.distance_pt_abs(new_gaze_target));

	// check requested speed
	float speed = fabs(requested_gaze_state_.distance_pt_abs(new_gaze_target)) /
	              (new_gaze_target.timestamp.to_seconds() - requested_gaze_state_.timestamp.to_seconds());

	// check magnitude and speed of gaze change to detect eye-neck saccades
	if (dist > config->threshold_angle_neck_saccade) {
		// the next saccade has to use neck motion as well
		if (speed > config->threshold_velocity_eye_saccade) {
			neck_saccade_requested = true;
		}
	}
	else {
		neck_saccade_requested = false;
	}

	// check for eye getting close to ocolomotor range
	// actually it makes more sense to trigger the neck saccade based on
	// the target getting out of the OMR
	float eye_target_l = joint_interface_->get_target_position(JointInterface::ID_EYES_LEFT_LR);
	float eye_target_r = joint_interface_->get_target_position(JointInterface::ID_EYES_RIGHT_LR);
	float eye_target_ud = joint_interface_->get_target_position(JointInterface::ID_EYES_BOTH_UD);

	// min/max bounds
	float left_min = config->threshold_angle_omr_limit * joint_interface_->get_joint_min(JointInterface::ID_EYES_LEFT_LR);
	float left_max = config->threshold_angle_omr_limit * joint_interface_->get_joint_max(JointInterface::ID_EYES_LEFT_LR);
	float right_min = config->threshold_angle_omr_limit * joint_interface_->get_joint_min(JointInterface::ID_EYES_RIGHT_LR);
	float right_max = config->threshold_angle_omr_limit * joint_interface_->get_joint_max(JointInterface::ID_EYES_RIGHT_LR);
	float ud_min = config->threshold_angle_omr_limit * joint_interface_->get_joint_min(JointInterface::ID_EYES_BOTH_UD);
	float ud_max = config->threshold_angle_omr_limit * joint_interface_->get_joint_max(JointInterface::ID_EYES_BOTH_UD);

	if ((eye_target_l < left_min) || (eye_target_l > left_max) || (eye_target_r < right_min) || (eye_target_r > right_max) ||
	    (eye_target_ud < ud_min) || (eye_target_ud > ud_max)) {
		// the eyeball gets close to OMR, activate a neck compensation motion
		neck_saccade_omr = true;
	}
	else {
		neck_saccade_omr = false;
	}

	// use base class set method
	MotionGenerator::set_gaze_target(new_gaze_target);
}

//! check if an eye saccade is active:
bool GazeMotionGenerator::get_eye_saccade_active() {
	bool saccade_active;

	float speed_left = joint_interface_->get_ts_speed(JointInterface::ID_EYES_LEFT_LR).get_newest_value();
	float speed_right = joint_interface_->get_ts_speed(JointInterface::ID_EYES_RIGHT_LR).get_newest_value();
	float speed_tilt = joint_interface_->get_ts_speed(JointInterface::ID_EYES_BOTH_UD).get_newest_value();

	float speed_total_l = sqrt(speed_left * speed_left + speed_tilt * speed_tilt);
	float speed_total_r = sqrt(speed_right * speed_right + speed_tilt * speed_tilt);

	// thresholding
	if ((speed_total_l > config->threshold_velocity_eye_saccade) || (speed_total_r > config->threshold_velocity_eye_saccade)) {
		// this is a saccade
		saccade_active = true;
	}
	else {
		saccade_active = false;
	}

	return saccade_active;
}

//! get overall gaze
humotion::GazeState GazeMotionGenerator::get_current_gaze() {
	// shortcut, makes the following easier to read
	JointInterface* j = joint_interface_;

	GazeState gaze = requested_gaze_state_;

	gaze.pan = j->get_ts_position(JointInterface::ID_NECK_PAN).get_newest_value() +
	           (j->get_ts_position(JointInterface::ID_EYES_LEFT_LR).get_newest_value() +
	            j->get_ts_position(JointInterface::ID_EYES_RIGHT_LR).get_newest_value()) /
	              2.0;
	gaze.tilt = j->get_ts_position(JointInterface::ID_NECK_TILT).get_newest_value() +
	            j->get_ts_position(JointInterface::ID_EYES_BOTH_UD).get_newest_value();
	gaze.roll = j->get_ts_position(JointInterface::ID_NECK_ROLL).get_newest_value();

	gaze.vergence = j->get_ts_position(JointInterface::ID_EYES_LEFT_LR).get_newest_value() -
	                j->get_ts_position(JointInterface::ID_EYES_RIGHT_LR).get_newest_value();

	return gaze;
}
