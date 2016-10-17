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

#ifndef EXAMPLES_YARP_ICUB_INCLUDE_HUMOTION_YARP_ICUB_ICUB_JOINTINTERFACE_H_
#define EXAMPLES_YARP_ICUB_INCLUDE_HUMOTION_YARP_ICUB_ICUB_JOINTINTERFACE_H_

#include "humotion_yarp_icub/icub_data_receiver.h"

#include <humotion/server/joint_interface.h>
#include <humotion/server/server.h>

#include <boost/bimap.hpp>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/IControlLimits2.h>
#include <yarp/dev/IControlMode.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/Network.h>
#include <yarp/os/Port.h>
#include <yarp/os/Property.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Time.h>
#include <yarp/sig/Vector.h>

#include <string>
#include <vector>

class iCubDataReceiver;
class iCubFaceInterface;

class iCubJointInterface : public humotion::server::JointInterface{
 public:
    explicit iCubJointInterface(std::string scope);
    ~iCubJointInterface();

    void run();

    int convert_icub_jointid_to_humotion(int id);
    int convert_humotion_jointid_to_icub(int id);

    enum JOINT_ID_ENUM{
        ICUB_ID_NECK_TILT = 0,
        ICUB_ID_NECK_ROLL = 1,
        ICUB_ID_NECK_PAN  = 2,
        ICUB_ID_EYES_BOTH_UD = 3,
        ICUB_ID_EYES_PAN = 4,
        ICUB_ID_EYES_VERGENCE = 5,
        ICUB_ID_EYES_LEFT_LID_LOWER,
        ICUB_ID_EYES_LEFT_LID_UPPER,
        ICUB_ID_EYES_RIGHT_LID_LOWER,
        ICUB_ID_EYES_RIGHT_LID_UPPER,
        ICUB_ID_EYES_LEFT_BROW,
        ICUB_ID_EYES_RIGHT_BROW,
        ICUB_ID_LIP_LEFT_UPPER,
        ICUB_ID_LIP_LEFT_LOWER,
        ICUB_ID_LIP_CENTER_UPPER,
        ICUB_ID_LIP_CENTER_LOWER,
        ICUB_ID_LIP_RIGHT_UPPER,
        ICUB_ID_LIP_RIGHT_LOWER,
        ICUB_JOINT_ID_ENUM_SIZE
    };

    yarp::dev::PolyDriver *get_yarp_polydriver() { return &yarp_polydriver_; }

    static const int MAIN_LOOP_FREQUENCY = 50;

 protected:
    void disable_joint(int e);
    void publish_target(int e, float position, float velocity);
    void enable_joint(int e);
    void execute_motion();

 private:
    bool running_in_simulation_;
    yarp::dev::PolyDriver yarp_polydriver_;

    std::vector<double> pv_mix_last_error_;
    std::vector<double> pv_mix_pid_p_;
    std::vector<double> pv_mix_pid_d_;

    // yarp views
    yarp::dev::IVelocityControl *yarp_ivel_;
    yarp::dev::IPositionControl *yarp_ipos_;
    yarp::dev::IControlLimits *yarp_ilimits_;
    yarp::dev::IAmplifierControl *yarp_amp_;
    yarp::dev::IPidControl *yarp_pid_;
    yarp::dev::IControlMode *yarp_icontrol_;

    yarp::sig::Vector yarp_commands_;

    float target_angle_[ICUB_JOINT_ID_ENUM_SIZE];
    float target_velocity_[ICUB_JOINT_ID_ENUM_SIZE];


    void init_controller();
    void init_id_map();
    void init_pv_mix_pid();
    void insert_icupid_to_humotionid_mapping(int icubid, int humotionid);


    void store_icub_joint_target(int icub_id, float position, float velocity);


    void set_joint_enable_state(int e, bool enabled);

    iCubDataReceiver *icub_data_receiver;
    void init_joints();

    std::string scope;

    void store_min_max(yarp::dev::IControlLimits *ilimits, int icub_id);

    void set_target_in_velocitymode(int id);

    iCubFaceInterface *face_interface_;

    typedef boost::bimap<int, int > enum_id_bimap_t;
    typedef enum_id_bimap_t::value_type enum_id_bimap_entry_t;
    enum_id_bimap_t enum_id_bimap;
};

#endif  // EXAMPLES_YARP_ICUB_INCLUDE_HUMOTION_YARP_ICUB_ICUB_JOINTINTERFACE_H_
