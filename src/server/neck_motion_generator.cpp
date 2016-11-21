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

#include <cmath>

#include "humotion/server/gaze_motion_generator.h"
#include "humotion/server/neck_motion_generator.h"
#include "humotion/server/server.h"

using humotion::server::NeckMotionGenerator;
using humotion::server::Config;

//! constructor
NeckMotionGenerator::NeckMotionGenerator(JointInterface *j, Config *cfg) :
    GazeMotionGenerator(j, cfg, 3, 1.0/Server::MOTION_UPDATERATE) {
    breath_time_ = 0.0;
}


//! destructor
NeckMotionGenerator::~NeckMotionGenerator() {
}

//! get a breath offset angle
//! @return float of breath offset value
float NeckMotionGenerator::get_breath_offset() {
    // we want to have a constant acceleration
    // -> triangular wave as speeds -> (x<0.5)? 2*x*x:  1- 2*(1-x)**2 = 4x - 2x**2 - 1
    float breath_offset = 0.0;

    // 0...1 -> move up, 1..2 -> return, 2..3 -> still
    float breath_time_normalized = (breath_time_ * 3)/config->breath_period;

    if (breath_time_normalized <= 0.5) {
        // accelerated motion
        breath_offset = config->breath_amplitude * (2.0 * pow(breath_time_normalized, 2));
    } else if (breath_time_normalized <= 1.0) {
        // deaccelerate
        breath_offset = config->breath_amplitude * (1.0 - 2.0
                                                    * pow(1.0 - breath_time_normalized, 2));
    } else if (breath_time_normalized <= 1.5) {
        // accelerate again
        breath_offset = config->breath_amplitude * (1.0 - (2.0 * pow(breath_time_normalized-1, 2)));
    } else if (breath_time_normalized <= 2.0) {
        breath_offset = config->breath_amplitude * (2.0 * pow(2.0 - breath_time_normalized, 2));
    } else if (breath_time_normalized <= 3.0) {
        // pause for some time
        breath_offset = 0;
    }

    // fetch next time
    breath_time_ += 1.0/Server::MOTION_UPDATERATE;

    if (breath_time_ >= config->breath_period) {
        breath_time_ -= config->breath_period;
    }

    return breath_offset;
}


//! calculate joint targets
void NeckMotionGenerator::calculate_targets() {
    // fetch current dataset
    float neck_pan_now, neck_tilt_now, neck_roll_now;
    float neck_pan_speed, neck_tilt_speed, neck_roll_speed;

    humotion::Timestamp neck_pan_ts = get_timestamped_state(JointInterface::ID_NECK_PAN,
                                                            &neck_pan_now,
                                                            &neck_pan_speed);

    humotion::Timestamp neck_tilt_ts = get_timestamped_state(JointInterface::ID_NECK_TILT,
                                                            &neck_tilt_now,
                                                            &neck_tilt_speed);

    humotion::Timestamp neck_roll_ts = get_timestamped_state(JointInterface::ID_NECK_ROLL,
                                                            &neck_roll_now,
                                                            &neck_roll_speed);

    // reached target?
    float goal_diff   = fabs(get_current_gaze().distance_pt_abs(requested_gaze_state_));
    float target_diff = fabs(requested_gaze_state_.distance_pt_abs(previous_neck_target_));

    // printf("GOAL DIFF = %f TARGET DIFF = %f\n",goal_diff,target_diff);
    // get_current_gaze().dump();
    // requested_gaze_state.dump();

    // check if new target
    // close to goal?
    if ( (neck_saccade_active_) && (goal_diff < 1.0) ) {
        neck_saccade_reached_goal_ = true;
    }

    if (neck_saccade_active_) {
        previous_neck_target_ = requested_gaze_state_;
    }

    // if we get a new target now, we can stop the neck saccade
    if (target_diff > .1) {
        if (neck_saccade_reached_goal_) {
            // joint_interface->neck_saccade_done();
            neck_saccade_active_ = false;
            neck_saccade_reached_goal_ = false;
        }
    }

    if (neck_saccade_requested) {
        neck_saccade_active_ = true;
    }

    // check if this is a small or big saccade
    if (neck_saccade_active_ || neck_saccade_omr) {
        // full saccade with neck motion -> update neck target
        requested_neck_state_ = requested_gaze_state_;
    }

    // get targets: this is the sum of stored neck target and up-to-date offset:
    float neck_pan_target  = requested_neck_state_.pan  + requested_gaze_state_.pan_offset;
    float neck_tilt_target = requested_neck_state_.tilt + requested_gaze_state_.tilt_offset;

    // roll is always equal to requested gaze (not neck) state
    float neck_roll_target = requested_gaze_state_.roll + requested_gaze_state_.roll_offset;

    // add breath wave to tilt
    neck_tilt_target += get_breath_offset();

    // neck_roll_target = 0.0;
    // printf("%f %f %f %f ROLL\n", neck_pan_target,
    // neck_roll_target, neck_roll_now, neck_roll_speed);

    // pass parameters to reflexxes api
    setup_neckmotion(0, neck_pan_target,  neck_pan_now,  neck_pan_speed,  neck_pan_ts);
    setup_neckmotion(1, neck_tilt_target, neck_tilt_now, neck_tilt_speed, neck_tilt_ts);
    setup_neckmotion(2, neck_roll_target, neck_roll_now, neck_roll_speed, neck_roll_ts);


    // call reflexxes to handle profile calculation
    reflexxes_calculate_profile();

    // tell the joint if about the new values
    joint_interface_->set_target(JointInterface::ID_NECK_PAN,
                                reflexxes_position_output->NewPositionVector->VecData[0],
                                reflexxes_position_output->NewVelocityVector->VecData[0]);

    joint_interface_->set_target(JointInterface::ID_NECK_TILT,
                                reflexxes_position_output->NewPositionVector->VecData[1],
                                reflexxes_position_output->NewVelocityVector->VecData[1]);

    joint_interface_->set_target(JointInterface::ID_NECK_ROLL,
                                reflexxes_position_output->NewPositionVector->VecData[2],
                                reflexxes_position_output->NewVelocityVector->VecData[2]);

    // printf("%f %f %f RRR\n", );

    /*printf("\n%f %f %f %f %f DBG\n",
            neck_pan_now, neck_pan_target,
            reflexxes_position_output->NewPositionVector->VecData[0],
            joint_interface_->get_ts_speed(JointInterface::ID_NECK_PAN).get_newest_value(),
            reflexxes_position_output->NewVelocityVector->VecData[0]
            );*/

    /*printf("\n%f %f %f %f %f #GAZELOG\n",
            boost::get_system_time().time_of_day().total_milliseconds()/1000.0,
            requested_gaze_state_.pan + requested_gaze_state_.pan_offset,
            get_current_gaze().pan,
            (get_current_position(JointInterface::ID_EYES_LEFT_LR) + get_current_position(JointInterface::ID_EYES_RIGHT_LR))/2.0,
            get_current_position(JointInterface::ID_NECK_PAN));*/

    // store debug data:
    store_debug_data("neck/neck_pan_now", neck_pan_now);
    store_debug_data("neck/neck_tilt_now", neck_tilt_now);
    store_debug_data("neck/neck_pan_target", neck_pan_target);
    store_debug_data("neck/neck_tilt_target", neck_tilt_target);

    store_debug_data("neck/gaze_target_pan", requested_gaze_state_.pan);
    store_debug_data("neck/gaze_target_tilt", requested_gaze_state_.tilt);

    store_debug_data("neck/output_pan", reflexxes_position_output->NewPositionVector->VecData[0]);
    store_debug_data("neck/output_tilt", reflexxes_position_output->NewPositionVector->VecData[1]);
}

//! publish targets to motor boards:
void NeckMotionGenerator::publish_targets() {
    // publish values if there is an active gaze input within the last timerange
    if (gaze_target_input_active()) {
        joint_interface_->publish_target(JointInterface::ID_NECK_PAN);
        joint_interface_->publish_target(JointInterface::ID_NECK_TILT);
        joint_interface_->publish_target(JointInterface::ID_NECK_ROLL);
    }
}


//! set up neck motion profile
//! this will use speed and acceleration calc formulas from literature:
//! \param dof id of joint
//! \param target angle
//! \param current angle
void NeckMotionGenerator::setup_neckmotion(int dof, float target, float current_position,
                                           float current_velocity, humotion::Timestamp timestamp) {
    // get distance to target
    float distance_abs = fabs(target - current_position);


    // get max speed: according to the equation Hmax from [guitton87] there is a linear relation
    // between distance_abs and v_max_head:
    // v_max = 4.39 * d_total + 106.0 (in degrees)
    float max_velocity = 4.39 * distance_abs + 106.0;

    // scale and limit max speed:
    max_velocity = max_velocity * config->scale_velocity_neck;
    max_velocity = fmin(max_velocity, config->limit_velocity_neck);

    // max accel: assuming linear acceleration we have:

    /* v ^  _
    *   |  / \
    *   | /   \
    *   |/_____\___> t
    */
    // d_total = 2 * 1/2 * a * (t_total/2)^2 = 1/4 * a * t_total^2
    // as we use linear accel we have
    // v_max = a * t_total/2  --> t_total = 2*v_max / a
    // combine both
    // d_total = 1/4 * a * 4 * vmax^2 / a^2 = v_max^2 / a
    // d_total = a * 2 * d_total / (v_max^2)
    // and therefore
    // a = v_max^2 / d_total
    float max_accel = 1.0;

    if (distance_abs > 0.0) {
        max_accel = pow(max_velocity, 2) / distance_abs;
    }

    // scale and limit acceleration
    max_accel = max_accel * config->scale_acceleration_neck;
    max_accel = fmin(max_accel, config->limit_acceleration_neck);

    // printf("MAX SPEED %4.2f / max accel %4.2f\n",max_speed, max_accel);
    // printf("%f %f %f ", distance_abs, max_velocity, max_accel);

    // feed reflexxes api with data
    reflexxes_set_input(dof, target, current_position, current_velocity,
                        timestamp, max_velocity, max_accel);
}
