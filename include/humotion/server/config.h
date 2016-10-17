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


#ifndef INCLUDE_HUMOTION_SERVER_CONFIG_H_
#define INCLUDE_HUMOTION_SERVER_CONFIG_H_

#include <map>
#include <string>

namespace humotion {
namespace server {

typedef std::map<std::string, float> debug_data_t;

class Config {
 public:
    Config();
    ~Config();
    void init_defaults();

    // ********************************************************
    // NOTE: See config.cpp for explanations and default values
    // ********************************************************

    // publish debug information
    bool publish_internals;

    // saccade detection thresholds
    float threshold_velocity_eye_saccade;
    float threshold_angle_neck_saccade;
    float threshold_angle_omr_limit;


    // neck motion generation configuration
    float scale_velocity_neck;
    float scale_acceleration_neck;
    float limit_velocity_neck;
    float limit_acceleration_neck;

    // eye motion generation configuration
    float scale_velocity_eye;
    float scale_acceleration_eye;
    float limit_velocity_eye;
    float limit_acceleration_eye;
    bool use_neck_target_instead_of_position_eye;

    // parameters fo the breathing pattern
    float breath_period;
    float breath_amplitude;

    // parameters for eyelids
    bool eyelids_follow_eyemotion;

    // parameters for eye blinking
    float eyeblink_duration;
    float eyeblink_periodic_distribution_lower;
    float eyeblink_periodic_distribution_upper;
    float eyeblink_probability_after_saccade;
    float eyeblink_blocked_time;
};

}  // namespace server
}  // namespace humotion

#endif  // INCLUDE_HUMOTION_SERVER_CONFIG_H_
