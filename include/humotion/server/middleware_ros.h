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

#ifndef INCLUDE_HUMOTION_SERVER_MIDDLEWARE_ROS_H_
#define INCLUDE_HUMOTION_SERVER_MIDDLEWARE_ROS_H_

#include <boost/shared_ptr.hpp>
#include <dynamic_reconfigure/server.h>
#include <ros/ros.h>

#include <map>
#include <string>
#include <utility>

#include "humotion/gaze.h"
#include "humotion/humotionConfig.h"
#include "humotion/mouth.h"
#include "humotion/server/middleware.h"

namespace humotion {
namespace server {

class MiddlewareROS : public Middleware {
 public:
    MiddlewareROS(std::string name, Controller *c);
    ~MiddlewareROS();
    bool ok();
    void tick();

    void dynamic_reconfigure_callback(const humotion::humotionConfig &config, uint32_t level);
    void publish_debug_dataset(debug_data_t);

 private:
    void publish_debug_data(std::string name, float value);

    void incoming_mouth_target(const humotion::mouth::ConstPtr& msg);
    void incoming_gaze_target(const humotion::gaze::ConstPtr& msg);
    void attach_to_reconfiguration_server(ros::NodeHandle priv_nodehandle);
    void debug_initialize();

    bool tick_necessary_;
    bool debug_initialized_;

    ros::Subscriber mouth_target_subscriber_;
    ros::Subscriber gaze_target_subscriber_;

    ros::NodeHandle nh_;

    typedef std::map<std::string, ros::Publisher> debug_topic_map_t;
    debug_topic_map_t debug_topic_map;

    dynamic_reconfigure::Server<humotion::humotionConfig> *reconf_server_;
};

}  // namespace server
}  // namespace humotion

#endif  // INCLUDE_HUMOTION_SERVER_MIDDLEWARE_ROS_H_
