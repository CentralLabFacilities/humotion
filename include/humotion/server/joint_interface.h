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

#ifndef INCLUDE_HUMOTION_SERVER_JOINT_INTERFACE_H_
#define INCLUDE_HUMOTION_SERVER_JOINT_INTERFACE_H_

#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>
#include <boost/thread/thread_time.hpp>

#include <cstdint>
#include <cstdio>
#include <map>
#include <string>

#include "humotion/gaze_state.h"
#include "humotion/mouth_state.h"
#include "humotion/timestamped_list.h"

namespace humotion {
namespace server {

// forward declaration to solve include loop
class Controller;


class JointInterface {
 public:
    JointInterface();
    ~JointInterface();

    void set_target(int joint_id, float position, float velocity);
    float get_target_position(int joint_id);
    float get_target_velocity(int joint_id);

    void publish_target(int joint_id);
    virtual void publish_target(int joint_id, float position, float velocity) = 0;
    virtual void execute_motion() = 0;

    typedef std::map<int, TimestampedList> joint_tsl_map_t;

    TimestampedList get_ts_position(int joint_id);
    TimestampedList get_ts_speed(int joint_id);

    void set_framerate(float f);

    void enable_mouth_joints();
    void disable_mouth_joints();
    bool mouth_target_input_active();

    void enable_gaze_joints();
    void disable_gaze_joints();
    bool gaze_target_input_active();

    float get_joint_min(int joint_id);
    float get_joint_max(int joint_id);

    void dump_angles();

    unsigned int get_and_clear_incoming_position_count();

    enum JOINT_ID_ENUM{
        ZERO = 0,
        ID_LIP_LEFT_UPPER,
        ID_LIP_LEFT_LOWER,
        ID_LIP_CENTER_UPPER,
        ID_LIP_CENTER_LOWER,
        ID_LIP_RIGHT_UPPER,
        ID_LIP_RIGHT_LOWER,
        ID_NECK_PAN,
        ID_NECK_TILT,
        ID_NECK_ROLL,
        ID_EYES_LEFT_LR,
        ID_EYES_RIGHT_LR,
        ID_EYES_BOTH_UD,
        ID_EYES_LEFT_LID_LOWER,
        ID_EYES_LEFT_LID_UPPER,
        ID_EYES_RIGHT_LID_LOWER,
        ID_EYES_RIGHT_LID_UPPER,
        ID_EYES_LEFT_BROW,
        ID_EYES_RIGHT_BROW,
        ID_DUMMY_0,
        ID_DUMMY_1,
        ID_DUMMY_2,
        JOINT_ID_ENUM_SIZE
    };

    bool get_joint_position_map_empty();

    void store_incoming_position(int joint_id, float position, Timestamp timestamp);
    void store_incoming_velocity(int joint_id, float velocity, Timestamp timestamp);

 protected:
    virtual void enable_joint(int id) = 0;
    virtual void disable_joint(int id) = 0;
    float framerate;

    float joint_min[JOINT_ID_ENUM_SIZE];
    float joint_max[JOINT_ID_ENUM_SIZE];



 private:
    float joint_target_position_[JOINT_ID_ENUM_SIZE];
    float joint_target_velocity_[JOINT_ID_ENUM_SIZE];

    boost::mutex joint_ts_position_map_access_mutex_;
    boost::mutex joint_ts_speed_map_access_mutex_;
    joint_tsl_map_t joint_ts_position_map_;
    joint_tsl_map_t joint_ts_speed_map_;

    bool mouth_enabled_;
    bool gaze_enabled_;
    unsigned int incoming_position_count_;
};

}  // namespace server
}  // namespace humotion

#endif  // INCLUDE_HUMOTION_SERVER_JOINT_INTERFACE_H_
