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

#include "humotion/server/joint_interface.h"
#include "humotion/server/controller.h"

using boost::mutex;
using humotion::server::JointInterface;

//! constructor
JointInterface::JointInterface() {
    framerate = 50.0;
    mouth_enabled_ = false;
    gaze_enabled_  = false;
}

//! destructor
JointInterface::~JointInterface() {
}

//! set joint target position
//! \param joint_id of joint
//! \param float position
void JointInterface::set_target(int joint_id, float position, float velocity) {
    assert(joint_id < JOINT_ID_ENUM_SIZE);

    // update current value
    joint_target_position_[joint_id] = position;
    joint_target_velocity_[joint_id] = velocity;
}

//! fetch target position
//! \param joint_id of joint
float JointInterface::get_target_position(int joint_id) {
    assert(joint_id < JOINT_ID_ENUM_SIZE);
    return joint_target_position_[joint_id];
}

//! fetch target velocity
//! \param joint_id of joint
float JointInterface::get_target_velocity(int joint_id) {
    assert(joint_id < JOINT_ID_ENUM_SIZE);
    return joint_target_velocity_[joint_id];
}

//! incoming position data
//! \param joint name
//! \param position
//! \param timestamp when the position was measured
void JointInterface::store_incoming_position(int joint_id, float position, Timestamp timestamp) {
    // lock the tsd_list for this access. by doing this we assure
    // that no other thread accessing this element can diturb the
    // following atomic instructions:
    mutex::scoped_lock sl(joint_ts_position_map_access_mutex_);

    // printf("> humotion: incoming joint position for joint id 0x%02X "
    // "= %4.2f (ts=%.2f)\n",joint_id,position,timestamp.to_seconds());
    joint_ts_position_map_[joint_id].insert(timestamp, position);

    incoming_position_count_++;
}

//! return incoming position data count & clear this counter
//! this can be used as a keep alive status check
//! \return number of incoming joint positions since the last call
unsigned int JointInterface::get_and_clear_incoming_position_count() {
    unsigned int i = incoming_position_count_;
    incoming_position_count_ = 0;
    return i;
}

//! incoming speed data
//! \param joint name
//! \param speed
//! \param timestamp when the position was measured
void JointInterface::store_incoming_velocity(int joint_id, float velocity, Timestamp timestamp) {
    // lock the tsd_list for this access. by doing this we assure
    // that no other thread accessing this element can disturb the
    // following atomic instructions:
    mutex::scoped_lock scoped_lock(joint_ts_speed_map_access_mutex_);

    // printf("> humotion: incoming joint velocity for joint id 0x%02X = %4.2f "
    // "(ts=%.2f)\n",joint_id,velocity,timestamp.to_seconds());
    joint_ts_speed_map_[joint_id].insert(timestamp, velocity);
}

//! return the timestamped float for the given joints position
humotion::TimestampedList JointInterface::get_ts_position(int joint_id) {
    // lock the tsd_list for this access. by doing this we assure
    // that no other thread accessing this element can disturb the
    // following atomic instructions
    mutex::scoped_lock sl(joint_ts_position_map_access_mutex_);

    // search map
    joint_tsl_map_t::iterator it = joint_ts_position_map_.find(joint_id);

    if (it == joint_ts_position_map_.end()) {
        printf("> humotion: no ts_position for joint id 0x%02X found\n", joint_id);
        return TimestampedList();
    }

    // ok fine, we found the requested value
    return it->second;
}

//! return the timestamped float for the given joints speed
humotion::TimestampedList JointInterface::get_ts_speed(int joint_id) {
    // lock the tsd_list for this access. by doing this we assure
    // that no other thread accessing this element can diturb the
    // following atomic instructions
    mutex::scoped_lock sl(joint_ts_speed_map_access_mutex_);

    // search map
    joint_tsl_map_t::iterator it = joint_ts_speed_map_.find(joint_id);

    if (it == joint_ts_speed_map_.end()) {
        printf("> humotion: no ts_speed for joint id 0x%02X found\n", joint_id);
        return humotion::TimestampedList();
    }

    // ok fine, we found our value
    return it->second;
}

//! set framerate
void JointInterface::set_framerate(float f) {
    framerate = f;
}

//! enable all mouth joints
void JointInterface::enable_mouth_joints() {
    // already enabled? skip this
    if (mouth_enabled_) {
        return;
    }

    printf("> humotion: ENABLING MOUTH JOINTS\n");
    enable_joint(ID_LIP_LEFT_UPPER);
    enable_joint(ID_LIP_LEFT_LOWER);
    enable_joint(ID_LIP_CENTER_UPPER);
    enable_joint(ID_LIP_CENTER_LOWER);
    enable_joint(ID_LIP_RIGHT_UPPER);
    enable_joint(ID_LIP_RIGHT_LOWER);
    mouth_enabled_ = true;
}


//! disable all mouth joints
void JointInterface::disable_mouth_joints() {
    // already disabled? skip this
    if (!mouth_enabled_) {
        return;
    }

    printf("> humotion: DISABLING MOUTH JOINTS\n");
    disable_joint(ID_LIP_LEFT_UPPER);
    disable_joint(ID_LIP_LEFT_LOWER);
    disable_joint(ID_LIP_CENTER_UPPER);
    disable_joint(ID_LIP_CENTER_LOWER);
    disable_joint(ID_LIP_RIGHT_UPPER);
    disable_joint(ID_LIP_RIGHT_LOWER);
    mouth_enabled_ = false;
}

//! enable all gaze joints
void JointInterface::enable_gaze_joints() {
    // already enabled? skip this
    if (gaze_enabled_) {
        return;
    }

    printf("> humotion: ENABLING GAZE JOINTS\n");
    enable_joint(ID_EYES_LEFT_LR);
    enable_joint(ID_EYES_RIGHT_LR);
    enable_joint(ID_EYES_BOTH_UD);

    enable_joint(ID_EYES_LEFT_LID_UPPER);
    enable_joint(ID_EYES_LEFT_LID_LOWER);
    enable_joint(ID_EYES_RIGHT_LID_UPPER);
    enable_joint(ID_EYES_RIGHT_LID_LOWER);

    enable_joint(ID_EYES_LEFT_BROW);
    enable_joint(ID_EYES_RIGHT_BROW);

    enable_joint(ID_NECK_PAN);
    enable_joint(ID_NECK_TILT);
    enable_joint(ID_NECK_ROLL);

    gaze_enabled_ = true;
}

//! disable all gaze joints
void JointInterface::disable_gaze_joints() {
    // already disabled? skip this
    if (!gaze_enabled_) {
        return;
    }

    printf("> humotion: DISABLING GAZE JOINTS\n");
    disable_joint(ID_EYES_LEFT_LR);
    disable_joint(ID_EYES_RIGHT_LR);
    disable_joint(ID_EYES_BOTH_UD);

    disable_joint(ID_EYES_LEFT_LID_UPPER);
    disable_joint(ID_EYES_LEFT_LID_LOWER);
    disable_joint(ID_EYES_RIGHT_LID_UPPER);
    disable_joint(ID_EYES_RIGHT_LID_LOWER);

    disable_joint(ID_EYES_LEFT_BROW);
    disable_joint(ID_EYES_RIGHT_BROW);

    disable_joint(ID_NECK_PAN);
    disable_joint(ID_NECK_TILT);
    disable_joint(ID_NECK_ROLL);

    gaze_enabled_ = false;
}

//! fetch maximum allowed joint position
//! \return max position
float JointInterface::get_joint_max(int joint_id) {
    assert((joint_id > 0) && (joint_id < JOINT_ID_ENUM_SIZE));
    return joint_max[joint_id];
}

//! fetch minimum allowed joint position
//! \return min position
float JointInterface::get_joint_min(int joint_id) {
    assert((joint_id > 0) && (joint_id < JOINT_ID_ENUM_SIZE));
    return joint_min[joint_id];
}

//! check if joint position map is empty
//! \return true if empty
bool JointInterface::get_joint_position_map_empty() {
    return (joint_ts_position_map_.empty());
}

//! call the virtual store function with given position and velocities
void JointInterface::publish_target(int joint_id) {
    publish_target(joint_id, joint_target_position_[joint_id], joint_target_velocity_[joint_id]);
}
