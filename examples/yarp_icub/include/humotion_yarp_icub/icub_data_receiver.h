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

#ifndef EXAMPLES_YARP_ICUB_INCLUDE_HUMOTION_YARP_ICUB_ICUB_DATA_RECEIVER_H_
#define EXAMPLES_YARP_ICUB_INCLUDE_HUMOTION_YARP_ICUB_ICUB_DATA_RECEIVER_H_

#include <humotion/server/joint_interface.h>
#include <humotion/server/server.h>

#include <boost/bimap.hpp>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/IControlLimits2.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/Network.h>
#include <yarp/os/Property.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Time.h>
#include <yarp/sig/Vector.h>

#include "humotion_yarp_icub/icub_jointinterface.h"

// forward declaration to solve loop
class iCubJointInterface;

class iCubDataReceiver : public yarp::os::RateThread{
 public:
    iCubDataReceiver(int period, iCubJointInterface *icub_jointinterface);
    bool threadInit();
    void threadRelease();
    void run();

 private:
    void store_incoming_position(int icub_id, double value, humotion::Timestamp timestamp);
    void store_incoming_velocity(int icub_id, double velocity, humotion::Timestamp timestamp);
    yarp::sig::Vector calculate_velocities(yarp::sig::Vector positions,
                              yarp::sig::Vector timestamps);

    void dump_incoming_data();

    float target_eye_pan_;
    float target_eye_vergence_;

    float target_eye_pan_velocity_;
    float target_eye_vergence_velocity_;

    yarp::sig::Vector positions_;
    yarp::sig::Vector timestamps_;
    yarp::sig::Vector velocities_;
    yarp::sig::Vector commands_;

    yarp::sig::Vector previous_positions_;
    yarp::sig::Vector previous_timestamps_;

    iCubJointInterface *icub_jointinterface_;
    yarp::dev::IEncodersTimed *iencs_;
};

#endif  // EXAMPLES_YARP_ICUB_INCLUDE_HUMOTION_YARP_ICUB_ICUB_DATA_RECEIVER_H_
