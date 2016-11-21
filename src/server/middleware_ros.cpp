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

#include <boost/algorithm/string/classification.hpp>
#include <std_msgs/Float32.h>

#include <string>
#include <utility>

#include "humotion/server/middleware_ros.h"

using humotion::server::MiddlewareROS;

//! constructor
MiddlewareROS::MiddlewareROS(std::string scope, Controller *c)
    : Middleware(scope, c) {
    ROS_DEBUG_STREAM_NAMED("MiddlewareROS", "will use ros middleware");
    debug_initialized_ = false;

    // start ros core?
    if (ros::isInitialized()) {
        // oh another process is doing ros comm, fine, we do not need to call it
        tick_necessary_ = false;
    } else {
        // we have to take care of ros
        ROS_DEBUG_STREAM_NAMED("MiddlewareROS", "no active ros middleware, will call ros::init() "
               "now and we will call tick() periodically!\n");

        std::string node_name = scope;
        node_name.erase(std::remove(node_name.begin(), node_name.end(), '/'), node_name.end());

        ROS_DEBUG_STREAM_NAMED("MiddlewareROS", "registering on ROS as node " << node_name.c_str());

        ros::M_string no_remapping;
        ros::init(no_remapping, node_name);
        tick_necessary_ = true;
    }

    // create main node handle
    nh_ = new ros::NodeHandle(scope + "/humotion");

    // create node handle
    ros::NodeHandle pnh("~");

    // set up subscribers
    mouth_target_subscriber_ = nh_->subscribe("mouth/target", 150,
                                          &MiddlewareROS::incoming_mouth_target ,
                                          this, ros::TransportHints().unreliable());
    gaze_target_subscriber_  = nh_->subscribe("gaze/target",  150,
                                          &MiddlewareROS::incoming_gaze_target,
                                          this, ros::TransportHints().unreliable());

    // set up dynamic reconfiguration service
    attach_to_reconfiguration_server(pnh);
}

//! callback for incoming dynamic reconfigure requests:
void MiddlewareROS::dynamic_reconfigure_callback(const humotion::humotionConfig &dyn_config,
                                                  uint32_t level) {
    // fetch config
    humotion::server::Config *config = controller_->get_config();

    // debug
    config->publish_internals = dyn_config.publish_internals;

    // saccade detection thresholds
    config->threshold_velocity_eye_saccade = dyn_config.threshold_velocity_eye_saccade;
    config->threshold_angle_neck_saccade = dyn_config.threshold_angle_neck_saccade;
    config->threshold_angle_omr_limit = dyn_config.threshold_angle_omr_limit;

    config->use_neck_target_instead_of_position_eye =
            dyn_config.use_neck_target_instead_of_position_eye;

    // neck motion generation configuration
    config->scale_velocity_neck = dyn_config.scale_velocity_neck;
    config->scale_acceleration_neck = dyn_config.scale_acceleration_neck;
    config->limit_velocity_neck = dyn_config.limit_velocity_neck;
    config->limit_acceleration_neck = dyn_config.limit_acceleration_neck;

    // eye motion generation configuration
    config->scale_velocity_eye = dyn_config.scale_velocity_eye;
    config->scale_acceleration_eye = dyn_config.scale_acceleration_eye;
    config->limit_velocity_eye = dyn_config.limit_velocity_eye;
    config->limit_acceleration_eye = dyn_config.limit_acceleration_eye;

    // parameters fo the breathing pattern
    config->breath_period = dyn_config.breath_period;
    config->breath_amplitude = dyn_config.breath_amplitude;

    // parameters for eyelids
    config->eyelids_follow_eyemotion = dyn_config.eyelids_follow_eyemotion;

    // parameters for eye blinking
    config->eyeblink_duration = dyn_config.eyeblink_duration;
    config->eyeblink_periodic_distribution_lower = dyn_config.eyeblink_periodic_distribution_lower;
    config->eyeblink_periodic_distribution_upper = dyn_config.eyeblink_periodic_distribution_upper;
    config->eyeblink_probability_after_saccade = dyn_config.eyeblink_probability_after_saccade;
    config->eyeblink_blocked_time = dyn_config.eyeblink_blocked_time;
}

//! attach to dynamic reconfigure server
void MiddlewareROS::attach_to_reconfiguration_server(ros::NodeHandle priv_nodehandle) {
    ROS_DEBUG_STREAM_NAMED("MiddlewareROS", "connecting to dynamic reconfiguration server");

    ros::NodeHandle reconf_node(priv_nodehandle, "humotion/configuration");

    reconf_server_ = new dynamic_reconfigure::Server<humotion::humotionConfig>(reconf_node);
    reconf_server_->setCallback(boost::bind(
                                    &MiddlewareROS::dynamic_reconfigure_callback, this, _1, _2));
}

//! destructor
MiddlewareROS::~MiddlewareROS() {
}

//! connection ok?
//! \return true if conn is alive
bool MiddlewareROS::ok() {
    return ros::ok();
}

//! do a single tick
void MiddlewareROS::tick() {
    if (tick_necessary_) {
        ros::spinOnce();
    }
}

//! init debug dataset publishers
void MiddlewareROS::debug_initialize() {
    if (debug_initialized_) {
        return;
    }

    debug_initialized_ = true;
}

//! publish a debug dataset
void MiddlewareROS::publish_debug_dataset(debug_data_t debug_data) {
    if (!controller_->get_config()->publish_internals) {
        // no debugging data enabled, return
        return;
    }

    debug_initialize();

    // iterate over all debug variables and publish them
    debug_data_t::iterator it;
    for (it = debug_data.begin(); it != debug_data.end(); it++) {
        publish_debug_data(it->first, it->second);
    }
}

void MiddlewareROS::publish_debug_data(std::string name, float value) {
    debug_topic_map_t::iterator it = debug_topic_map.find(name);
    if (it == debug_topic_map.end()) {
        // we have no publisher for this dataset, create one:
        ROS_DEBUG_STREAM("creating debug output stream " << name);
        ros::Publisher pub =  nh_->advertise<std_msgs::Float32>("debug_data/" + name, 1000);

        std::pair<debug_topic_map_t::iterator, bool> ret;
        ret = debug_topic_map.insert(std::pair<std::string, ros::Publisher>(name, pub));
        it = ret.first;
    }

    // in any case, it is now the publisher we want  so publish data now
    std_msgs::Float32 msg;
    msg.data = value;

    // std::cout  << "DEBUG HUMOTION " << name << " " << value << std::endl;

    // send it
    it->second.publish(msg);
}

//! callback to handle incoming mouth  target
void MiddlewareROS::incoming_mouth_target(const humotion::mouth::ConstPtr& msg) {
    // printf("> incoming mouth_target\n");
    MouthState mouth_state;

    mouth_state.position_left   = msg->position.left;
    mouth_state.position_center = msg->position.center;
    mouth_state.position_right  = msg->position.right;

    mouth_state.opening_left   = msg->opening.left;
    mouth_state.opening_center = msg->opening.center;
    mouth_state.opening_right  = msg->opening.right;

    controller_->set_mouth_target(mouth_state);
}


//! callback to handle incoming gaze  target
void MiddlewareROS::incoming_gaze_target(const humotion::gaze::ConstPtr& msg) {
    GazeState gaze_state;
    // printf("> incoming gaze_target [P:%3.1f, T:%3.1f, R:%3.1f] %d = %s\n",
    //          msg->pan, msg->tilt, msg->roll, msg->type,
    //          msg->type==humotion::gaze::GAZETYPE_ABSOLUTE?"ABSOLUTE":"RELATIVE");

    gaze_state.pan  = msg->pan;
    gaze_state.tilt = msg->tilt;
    gaze_state.roll = msg->roll;
    gaze_state.vergence = msg->vergence;

    gaze_state.pan_offset  = msg->pan_offset;
    gaze_state.tilt_offset = msg->tilt_offset;
    gaze_state.roll_offset = msg->roll_offset;

    gaze_state.eyelid_opening_upper = msg->eyelid_opening_upper;
    gaze_state.eyelid_opening_lower = msg->eyelid_opening_lower;

    gaze_state.eyebrow_left = msg->eyebrow_left;
    gaze_state.eyebrow_right = msg->eyebrow_right;

    gaze_state.eyeblink_request_left = msg->eyeblink_request_left;
    gaze_state.eyeblink_request_right = msg->eyeblink_request_right;

    if (msg->gaze_type == humotion::gaze::GAZETYPE_ABSOLUTE) {
        gaze_state.gaze_type = GazeState::GAZETYPE_ABSOLUTE;
    } else {
        gaze_state.gaze_type = GazeState::GAZETYPE_RELATIVE;
    }

    gaze_state.timestamp.set(msg->gaze_timestamp.sec, msg->gaze_timestamp.nsec);

    controller_->set_gaze_target(gaze_state);
}

