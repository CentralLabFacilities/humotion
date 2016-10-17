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

#include "humotion/server/config.h"

using humotion::server::Config;

Config::Config() {
    init_defaults();
}


Config::~Config() {
}

void Config::init_defaults() {
    // publish internal debug data as topics?
    publish_internals = false;

    // saccade detection thresholds:
    // 1) velocity threshold, values above this value trigger an eye saccade
    //    value is given in deg/s
    threshold_velocity_eye_saccade = 15.0;

    // 2) angular threshold, a target change higher than this value will trigger a neck saccade
    //    value is given in deg
    threshold_angle_neck_saccade = 15.0;

    // 3) eyes reaching ocolumotor limits will trigger correction saccade
    //    value given in percent (NOTE: 1.0 = 100%)
    threshold_angle_omr_limit = 0.35;


    // neck motion generation configuration
    // humotion calculates neck velocities based on the linear equation Hmax from
    // [Guitton87] "Gaze control in humans: eye-head coordination during orienting movements ..."
    // In order to allow better adaption to the robot capabilities humotion allows to
    // scale the calculated velocity, value is given in percent (NOTE: 1.0 = 100% human velocity)
    scale_velocity_neck = 0.45;

    // scale acceleration
    scale_acceleration_neck = 0.55;

    // additionally humotion allows to limit the maximum velocity, value is given in deg/s
    limit_velocity_neck = 700.0;

    // limit the maximum acceleration, value is given in deg/s^2
    limit_acceleration_neck = 1000.0;

    // eye motion generation configuration
    // scale the calculated velocity, value is given in percent (NOTE: 1.0 = 100% human velocity)
    scale_velocity_eye = 0.45;

    // scale acceleration
    scale_acceleration_eye = 1.0;

    // additionally humotion allows to limit the maximum velocity, value is given in deg/s
    limit_velocity_eye = 700.0;

    // limit the maximum acceleration, value is given in deg/s^2
    limit_acceleration_eye = 80000;

    // calculate the overall target distance based on neck target instead of using the real neck
    // position
    use_neck_target_instead_of_position_eye = true;

    // parameters fo the breathing pattern
    // healthy adult human: 12-15 breaths/min (see e.g. "Ganong's review of medical physiology")
    // total breathe: 60/12-15 = 3-5s
    // inhale 1.5-2s
    // exhale 1.5-2s
    // pause      2s
    // overall period given in seconds
    breath_period = 4.5;  // = 1.5 + 1.5 + 1.5 for inhale, pause & exhale
    // amplitude given in degrees
    breath_amplitude = 1.0;


    // params for eyelid motion
    // should the eyelids follow the eyeball?
    eyelids_follow_eyemotion = true;


    // parameters for eye blinking
    // duration for one eyeblink, value is given in seconds
    eyeblink_duration = 0.15;

    // occurance of periodic eyeblinks, uniformly distributed over the given range
    // typical values for a human are one blink every 2...10s
    // values are given in seconds
    eyeblink_periodic_distribution_lower = 2.0;
    eyeblink_periodic_distribution_upper = 10.0;

    // probability that one eye saccade causes an eyeblink, value given in percent
    // for humans this is up to 95%, this gets quite annoying on the robot so the default is lower
    eyeblink_probability_after_saccade = 0.33;

    // blocking time where further eyeblinks are suppressed, value given in seconds
    eyeblink_blocked_time = 1.0;
}
