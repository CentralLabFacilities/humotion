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
#include <string>

#include "humotion/client/middleware_ros.h"
#include "humotion/gaze.h"
#include "humotion/mouth.h"

// using namespace std;
// using namespace boost;
// using namespace humotion;
using humotion::client::MiddlewareROS;

//! constructor
MiddlewareROS::MiddlewareROS(const std::string& scope) : Middleware(scope) {
	// start ros core
	if (!ros::isInitialized()) {
		tick_necessary_ = true;
		std::string node_name = "humotion_client__" + base_scope_;
		node_name.erase(std::remove(node_name.begin(), node_name.end(), '/'), node_name.end());

		ros::M_string no_remapping;
		ros::init(no_remapping, node_name);
	}
	else {
		// another ros thread already takes care of spinning
		tick_necessary_ = false;
	}

	// create node handle
	ros::NodeHandle n;

	// set up publishers
	mouth_target_publisher_ = n.advertise<humotion::mouth>(base_scope_ + "/humotion/mouth/target", 100);
	gaze_target_publisher_ = n.advertise<humotion::gaze>(base_scope_ + "/humotion/gaze/target", 100);
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

//! send mouth target to server
void MiddlewareROS::send_mouth_target() {
	// build target packet
	humotion::mouth msg;

	// set timestamp
	msg.header.stamp = ros::Time::now();

	msg.position.left = mouth_state_.position_left;
	msg.position.center = mouth_state_.position_center;
	msg.position.right = mouth_state_.position_right;

	msg.opening.left = mouth_state_.opening_left;
	msg.opening.center = mouth_state_.opening_center;
	msg.opening.right = mouth_state_.opening_right;

	// add position to send queue
	mouth_target_publisher_.publish(msg);

	// allow ros to handle data
	tick();
}

//! send mouth target to server
void MiddlewareROS::send_gaze_target() {
	// build target packet
	humotion::gaze msg;

	// set timestamp
	msg.header.stamp = ros::Time::now();

	msg.pan = gaze_state_.pan;
	msg.tilt = gaze_state_.tilt;
	msg.roll = gaze_state_.roll;
	msg.vergence = gaze_state_.vergence;

	msg.pan_offset = gaze_state_.pan_offset;
	msg.tilt_offset = gaze_state_.tilt_offset;
	msg.roll_offset = gaze_state_.roll_offset;

	msg.eyelid_opening_upper = gaze_state_.eyelid_opening_upper;
	msg.eyelid_opening_lower = gaze_state_.eyelid_opening_lower;

	msg.eyebrow_left = gaze_state_.eyebrow_left;
	msg.eyebrow_right = gaze_state_.eyebrow_right;

	msg.eyeblink_request_left = gaze_state_.eyeblink_request_left;
	msg.eyeblink_request_right = gaze_state_.eyeblink_request_right;

	if (gaze_state_.gaze_type == GazeState::GAZETYPE_ABSOLUTE) {
		msg.gaze_type = humotion::gaze::GAZETYPE_ABSOLUTE;
	}
	else {
		msg.gaze_type = humotion::gaze::GAZETYPE_RELATIVE;
	}
	msg.gaze_timestamp.sec = gaze_state_.timestamp.sec;
	msg.gaze_timestamp.nsec = gaze_state_.timestamp.nsec;

	// add position to send queue
	gaze_target_publisher_.publish(msg);

	// allow ros to handle data
	tick();
}
