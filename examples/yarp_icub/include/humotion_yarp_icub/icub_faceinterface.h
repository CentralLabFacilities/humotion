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
#ifndef EXAMPLES_YARP_ICUB_INCLUDE_HUMOTION_YARP_ICUB_ICUB_FACEINTERFACE_H_
#define EXAMPLES_YARP_ICUB_INCLUDE_HUMOTION_YARP_ICUB_ICUB_FACEINTERFACE_H_

#include "humotion_yarp_icub/icub_jointinterface.h"

#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/os/Network.h>

#include <string>

class iCubFaceInterface {
 public:
    explicit iCubFaceInterface(std::string scope);
    ~iCubFaceInterface();


    void set_eyelid_angle(float angle);
    void set_eyebrow_angle(int id, float *target_angle);
    void set_mouth(float *target_angle);

 private:
    double lid_angle;
    int lid_opening_previous;
    int previous_mouth_state;
    double target_angle_previous[iCubJointInterface::ICUB_JOINT_ID_ENUM_SIZE];
    std::string scope;
    yarp::os::BufferedPort<yarp::os::Bottle> emotion_port;
};

#endif  // EXAMPLES_YARP_ICUB_INCLUDE_HUMOTION_YARP_ICUB_ICUB_FACEINTERFACE_H_
