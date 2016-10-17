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

#include "humotion_yarp_icub/icub_faceinterface.h"

#include <boost/format.hpp>
#include <algorithm>
#include <string>

using yarp::os::Network;
using yarp::os::Bottle;
using std::cout;
using std::cerr;
using std::string;

//! constructor
iCubFaceInterface::iCubFaceInterface(std::string _scope) {
    scope = _scope;

    // attach to facial expressions:
    std::string emotion_scope = scope + "/face/raw/in";
    cout << "opening connection to '"<< emotion_scope << "'\n";

    bool init_ok = true;
    // open emotion port
    std::string emotion_port_out = "/emotionwriter";
    if (!emotion_port.open(emotion_port_out.c_str())) {
        cerr << "ERROR: failed to open emotion port '" << emotion_port_out << "'\n";
        init_ok = false;
    }
    if (!Network::connect(emotion_port_out.c_str(), emotion_scope.c_str())) {
        cerr << "ERROR: failed to connect to emotion port '" << emotion_port_out << "'\n";
        init_ok = false;
    }

    if (!init_ok) {
        cerr << "ERROR: failed to set up emotion component\n";
        cerr << "       please make sure that the faceExpressions yarpdev is started\n";
        cerr << "       (e.g.  yarpdev --name /icub/face/raw --device serial ... is running)\n";
        exit(EXIT_FAILURE);
    }
}

//! destructor
iCubFaceInterface::~iCubFaceInterface() {
}

//! special command to set eyelid angle
//! \param angle in degrees
void iCubFaceInterface::set_eyelid_angle(float angle) {
    if (emotion_port.getOutputCount() > 0) {
        // try to set the value based on the upper one
        // some guesses from the sim: S30 = 0° / S40 = 10°
        int opening = (25.0 + 0.8*angle);
        opening = std::min(48, std::max(24, opening));

        if (opening == lid_opening_previous) {
            // no update necessary
            return;
        }

        lid_angle = angle;
        lid_opening_previous = opening;

        char buf[20];
        snprintf(buf, sizeof(buf), "S%2d", opening);

        // cout << "SETTING EYELID '" << buf << "' (" << angle << " -> " << opening << "\n";
        Bottle &cmd = emotion_port.prepare();
        cmd.clear();
        cmd.addString(buf);
        // NOTE: writeStrict is important in order not to loose packets
        emotion_port.writeStrict();
    } else {
        cerr << "ERROR: no icub emotion output\n";
        exit(EXIT_FAILURE);
    }
}

//! special command to set the eyebrow angle
//! \param id {0=left, 1=right)
//! \param angle in degrees
void iCubFaceInterface::set_eyebrow_angle(int id, float *target_angle) {
    if (emotion_port.getOutputCount() > 0) {
        double angle = target_angle[id];
        int icub_val = 0;

        // swap rotation direction for eyebrow
        if (id == iCubJointInterface::ICUB_ID_EYES_LEFT_BROW) {
            angle = -angle;
        }

        // convert to icub representation
        if (angle < -20) {
            icub_val = 1;
        } else if (angle < 10) {
            icub_val = 2;
        } else if (angle < 20) {
            icub_val = 4;
        } else {
            icub_val = 8;
        }

        // make sure to update only on new values
        if (icub_val == target_angle_previous[id]) {
                // no updata necessary
                return;
        }

        // store actual value
        target_angle_previous[id] = icub_val;


        std::string cmd_s;
        if (id == iCubJointInterface::ICUB_ID_EYES_LEFT_BROW) {
            cmd_s = "L0" + boost::lexical_cast<std::string>(icub_val);
        } else {
            cmd_s = "R0" + boost::lexical_cast<std::string>(icub_val);
        }

        // cout << "SETTING EYEBROW " << id << " (" << angle << " -> " << cmd_s << ")\n";

        Bottle &cmd = emotion_port.prepare();
        cmd.clear();
        cmd.addString(cmd_s);
        // NOTE: writeStrict is important in order not to loose packets
        emotion_port.writeStrict();
    } else {
        cerr << "ERROR: no icub emotion output\n";
        exit(EXIT_FAILURE);
    }
}

void iCubFaceInterface::set_mouth(float *target_angle) {
    // convert from 6DOF mouth displacement to icub leds:
    int led_value = 0;

    // fetch center opening
    double center_opening = target_angle[iCubJointInterface::ICUB_ID_LIP_CENTER_LOWER] -
            target_angle[iCubJointInterface::ICUB_ID_LIP_CENTER_UPPER];
    bool mouth_open = (center_opening > 15.0) ? true : false;

    // side of mouth high or low?
    double center_avg = (target_angle[iCubJointInterface::ICUB_ID_LIP_CENTER_LOWER] +
            target_angle[iCubJointInterface::ICUB_ID_LIP_CENTER_UPPER])/2.0;
    double left_avg   = (target_angle[iCubJointInterface::ICUB_ID_LIP_LEFT_LOWER] +
            target_angle[iCubJointInterface::ICUB_ID_LIP_LEFT_UPPER])/2.0;
    double right_avg  = (target_angle[iCubJointInterface::ICUB_ID_LIP_RIGHT_LOWER] +
            target_angle[iCubJointInterface::ICUB_ID_LIP_RIGHT_UPPER])/2.0;

    // happy, neutral or sad?
    double diff_l = center_avg - left_avg;
    double diff_r = center_avg - right_avg;
    double diff   = (diff_l+diff_r)/2.0;

    if (diff > 2.0) {
        if (mouth_open) {
            led_value = 0x14;
        } else {
            if (diff > 2.6) {
                led_value = 0x0A;
            } else {
                led_value = 0x0B;
            }
        }
    } else if (diff < -3.0) {
        if (mouth_open) {
            led_value = 0x06;
        } else {
            led_value = 0x18;
        }
    } else if (diff < -2.0) {
        if (mouth_open) {
            led_value = 0x04;  // 0x25;
        } else {
            led_value = 0x08;
        }
    } else {
        if (mouth_open) {
            led_value = 0x16;
        } else {
            led_value = 0x08;
        }
    }


    if (led_value == previous_mouth_state) {
        // no update necessary
        return;
    }

    previous_mouth_state = led_value;

    // convert to string
    char buf[10];
    snprintf(buf, sizeof(buf), "M%02X", led_value);

    /*
    cout << "sending mouth " << buf << "\n";
    cout << boost::format("  mouth angles: %3.2f %3.2f %3.2f\n")
            % target_angle[ICUB_ID_LIP_LEFT_UPPER]
            % target_angle[ICUB_ID_LIP_CENTER_UPPER]
            % target_angle[ICUB_ID_LIP_RIGHT_UPPER];
    cout << boost::format("  mouth         %3.2f %3.2f %3.2f\n")
            % target_angle[ICUB_ID_LIP_LEFT_LOWER]
            % target_angle[ICUB_ID_LIP_CENTER_LOWER]
            % target_angle[ICUB_ID_LIP_RIGHT_LOWER];
    cout << boost::format("  mouth  open=%3.2f diff=%3.2f\n")
            % center_opening
            % diff;
    */

    // add mouth
    Bottle &cmd = emotion_port.prepare();
    cmd.clear();
    cmd.addString(buf);
    // NOTE: writeStrict is important in order not to loose packets
    emotion_port.writeStrict();

/*
    //store joint values which we do not handle on icub here:
    double timestamp = get_timestamp_ms();
    JointInterface::store_incoming_position(ID_LIP_LEFT_UPPER,   target_angle[ICUB_ID_LIP_LEFT_UPPER], timestamp);
    JointInterface::store_incoming_position(ID_LIP_LEFT_LOWER,   target_angle[ICUB_ID_LIP_LEFT_LOWER], timestamp);
    JointInterface::store_incoming_position(ID_LIP_CENTER_UPPER, target_angle[ICUB_ID_LIP_CENTER_UPPER], timestamp);
    JointInterface::store_incoming_position(ID_LIP_CENTER_LOWER, target_angle[ICUB_ID_LIP_CENTER_LOWER], timestamp);
    JointInterface::store_incoming_position(ID_LIP_RIGHT_UPPER,  target_angle[ICUB_ID_LIP_RIGHT_UPPER], timestamp);
    JointInterface::store_incoming_position(ID_LIP_RIGHT_LOWER,  target_angle[ICUB_ID_LIP_RIGHT_LOWER], timestamp);
*/
}
