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
#include "humotion/server/server.h"

// using namespace std;
// using namespace humotion;
// using namespace humotion::server;

using humotion::server::EyeMotionGenerator;

//! constructor
EyeMotionGenerator::EyeMotionGenerator(JointInterface *j, Config *cfg) :
    GazeMotionGenerator(j, cfg, 3, 1.0/Server::MOTION_UPDATERATE) {
}


//! destructor
EyeMotionGenerator::~EyeMotionGenerator() {
}

//! set up an eyemotion profile
//! this will use speed and acceleration calc formulas from literature:
//! \param dof id of joint
//! \param target angle
//! \param current angle
void EyeMotionGenerator::setup_eyemotion(int dof, float target,
                                         float current_position,
                                         float current_velocity,
                                         humotion::Timestamp timestamp) {
    // get distance to target:
    float distance_abs = fabs(target - current_position);

    // get max speed: factor can be found in encyc britannica:
    // "linear.... being 300° per second for 10° and 500° per second for 30°" (max=700)
    float max_velocity = fmin(700.0, 10.0*distance_abs + 200.0);

    // scale and limit max speed
    max_velocity = max_velocity * config->scale_velocity_eye;
    max_velocity = fmin(max_velocity, config->limit_velocity_eye);

    // max accel: use data from:
    // "Speed and Accuracy of Saccadic Eye Movements:
    //      Characteristics of Impulse Variability in the Oculomotor System"
    // http://www-personal.umich.edu/~kornblum/files/journal_exp_psych_HPP_15-3.pdf [table 2]
    float max_accel = fmin(80000.0, 1526.53*distance_abs + 10245.4);

    // scale and limit max acceleration
    max_accel = max_accel * config->scale_acceleration_eye;
    max_accel = fmin(max_accel, config->limit_acceleration_eye);

    // feed reflexxes api with data
    reflexxes_set_input(dof, target, current_position, current_velocity,
                        timestamp, max_velocity, max_accel);
}

//! calculate joint targets
void EyeMotionGenerator::calculate_targets() {
    // use the target values for a faster response
    float neck_pan_now  = 0.0;
    float neck_tilt_now = 0.0;

    if (config->use_neck_target_instead_of_position_eye) {
        // position calc based on target
        neck_pan_now = joint_interface_->get_target_position(JointInterface::ID_NECK_PAN);
        neck_tilt_now = joint_interface_->get_target_position(JointInterface::ID_NECK_TILT);
    } else {
        // position calc based on real position
        neck_pan_now =
                joint_interface_->get_ts_position(JointInterface::ID_NECK_PAN).get_newest_value();
        neck_tilt_now =
                joint_interface_->get_ts_position(JointInterface::ID_NECK_TILT).get_newest_value();
    }

    // calculate target angles for the eyes
    // right eye is dominant -> direct output
    float eye_pan_r_target = (requested_gaze_state_.pan + requested_gaze_state_.vergence/2.0)
            - (neck_pan_now);

    // left eye is non dominant -> filtered output: TODO: activate low pass filtered output
    // FIXME: USE LOWPASS FILTER HERE --> output_angle[angle_names::EYE_PAN_L]
    //        + 0.1 * (eye_pan_l_target - output_angle[angle_names::EYE_PAN_L]) etc;
    float eye_pan_l_target = (requested_gaze_state_.pan - requested_gaze_state_.vergence/2.0)
            - (neck_pan_now);

    // tilt
    float eye_tilt_target = requested_gaze_state_.tilt - (neck_tilt_now);

    // check and take care of limits
    eye_pan_l_target = limit_target(JointInterface::ID_EYES_LEFT_LR, eye_pan_l_target);
    eye_pan_r_target = limit_target(JointInterface::ID_EYES_RIGHT_LR, eye_pan_r_target);
    eye_tilt_target = limit_target(JointInterface::ID_EYES_BOTH_UD, eye_tilt_target);

    // fetch current dataset
    float eye_pan_l_now, eye_pan_r_now, eye_tilt_now;
    float eye_pan_l_speed, eye_pan_r_speed, eye_tilt_speed;

    humotion::Timestamp eye_pan_l_ts = get_timestamped_state(
                JointInterface::ID_EYES_LEFT_LR,
                &eye_pan_l_now,
                &eye_pan_l_speed);

    humotion::Timestamp eye_pan_r_ts = get_timestamped_state(
                JointInterface::ID_EYES_RIGHT_LR,
                &eye_pan_r_now,
                &eye_pan_r_speed);

    humotion::Timestamp eye_tilt_ts = get_timestamped_state(
                JointInterface::ID_EYES_BOTH_UD,
                &eye_tilt_now,
                &eye_tilt_speed);

    // pass paramaters to reflexxes api
    setup_eyemotion(0, eye_pan_l_target, eye_pan_l_now, eye_pan_l_speed, eye_pan_l_ts);
    setup_eyemotion(1, eye_pan_r_target, eye_pan_r_now, eye_pan_r_speed, eye_pan_r_ts);
    setup_eyemotion(2, eye_tilt_target, eye_tilt_now, eye_tilt_speed,    eye_tilt_ts);

    // cout << "EYE MOTION 2 " << eye_tilt_target << " now=" << eye_tilt_now << "\n";

    // call reflexxes to handle profile calculation
    reflexxes_calculate_profile();

    // tell the joint about the new values
    joint_interface_->set_target(JointInterface::ID_EYES_LEFT_LR,
                                reflexxes_position_output->NewPositionVector->VecData[0],
                                reflexxes_position_output->NewVelocityVector->VecData[0]);
    joint_interface_->set_target(JointInterface::ID_EYES_RIGHT_LR,
                                reflexxes_position_output->NewPositionVector->VecData[1],
                                reflexxes_position_output->NewVelocityVector->VecData[1]);
    joint_interface_->set_target(JointInterface::ID_EYES_BOTH_UD,
                                reflexxes_position_output->NewPositionVector->VecData[2],
                                reflexxes_position_output->NewVelocityVector->VecData[2]);

    // store debug data:
    store_debug_data("eye/neck_pan_now", neck_pan_now);
    store_debug_data("eye/neck_tilt_now", neck_tilt_now);
    store_debug_data("eye/eye_pan_r_target", eye_pan_r_target);
    store_debug_data("eye/eye_pan_l_target", eye_pan_l_target);
    store_debug_data("eye/eye_tilt_target", eye_tilt_target);

    store_debug_data("eye/gaze_target_pan", requested_gaze_state_.pan);
    store_debug_data("eye/gaze_target_tilt", requested_gaze_state_.tilt);

    store_debug_data("eye/eye_pan_r_now", eye_pan_r_now);
    store_debug_data("eye/eye_pan_l_now", eye_pan_l_now);
    store_debug_data("eye/eye_tilt_now", eye_tilt_now);

    store_debug_data("eye/output_l_lr", reflexxes_position_output->NewPositionVector->VecData[0]);
    store_debug_data("eye/output_r_lr", reflexxes_position_output->NewPositionVector->VecData[1]);
    store_debug_data("eye/output_ud",   reflexxes_position_output->NewPositionVector->VecData[2]);
}


//! pubish the calculated targets to the joint subsystem
void EyeMotionGenerator::publish_targets() {
    // publish values if there is an active gaze input within the last timerange
    if (gaze_target_input_active()) {
        joint_interface_->publish_target(JointInterface::ID_EYES_LEFT_LR);
        joint_interface_->publish_target(JointInterface::ID_EYES_RIGHT_LR);
        joint_interface_->publish_target(JointInterface::ID_EYES_BOTH_UD);
    }
}
