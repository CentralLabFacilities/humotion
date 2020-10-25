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

#include <boost/algorithm/string.hpp>
#include <ros/ros.h>
#include <string>
#include <chrono>
#include <thread>

#include "humotion/server/middleware_ros.h"
#include "humotion/server/server.h"

using humotion::server::Server;

//! set up constant for updaterate (50Hz)
const float Server::MOTION_UPDATERATE = 50.0;

//! constructor
//! open a new server instance.
Server::Server(std::string scope, std::string mw, JointInterface* _joint_interface) {
	// convert mw to uppercase
	boost::to_upper(mw);

	printf("> initializing humotion server (on %s, middleware=%s)\n", scope.c_str(), mw.c_str());

	// store pointer to joint interface
	joint_interface_ = _joint_interface;

	// tell joint interface our framerate
	joint_interface_->set_framerate(MOTION_UPDATERATE);

	// create controller
	controller_ = new Controller(joint_interface_);
	controller_->init_motion_generators();

	// start middleware
	if (mw == "ROS") {
		middleware_ = new MiddlewareROS(scope, controller_);
	}
	else {
		printf("> ERROR: invalid mw '%s' given. RSB support was dropped. Please use ROS\n\n", mw.c_str());
		exit(EXIT_FAILURE);
	}

	// start motion generation thread
	start_motion_generation_thread();
}

//! destructor
Server::~Server() {
}

//! middleware still running?
bool Server::ok() {
	return middleware_->ok();
}

//! start main thread
void Server::start_motion_generation_thread() {
	motion_generation_thread_ptr_ = new boost::thread(boost::bind(&Server::motion_generation_thread, this));
}

//! main thread that handles all data in/out
//! this thread will take care of gathering all input data and
//! then generate the output targets at a fixed rate of \sa MOTION_UPDATERATE
void Server::motion_generation_thread() {
	unsigned int incoming_data_count_invalid = 0;

	printf("> started motion generation thread\n");

	// calculate loop period
	const auto loop_period =
	   std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::duration<float>(1.0 / MOTION_UPDATERATE));
	auto timeout = std::chrono::steady_clock::now() + loop_period;
	printf("> one loop = %ldms\n", loop_period.count());

	// wait for incoming joint data
	while (middleware_->ok()) {
		// mw tick
		middleware_->tick();

		unsigned int incoming_data_count = joint_interface_->get_and_clear_incoming_position_count();

		if (incoming_data_count == 0) {
			incoming_data_count_invalid++;
			if (incoming_data_count_invalid >= MOTION_UPDATERATE) {
				printf("> waiting for valid incoming joint data (position+velocity)\n");
				incoming_data_count_invalid = 0;
			}
		}
		else {
			// fine, joint data is arriving, exit waiting loop
			break;
		}

		std::this_thread::sleep_until(timeout);
		timeout += loop_period;
	}

	printf("> joint data arrived, control loop active.\n");
	controller_->set_activated();

	// fine, data is arriving, activate and run control loop
	while (middleware_->ok()) {
		// mw tick
		middleware_->tick();

		// calculate all targets
		controller_->calculate_targets();

		// do some logging
		middleware_->publish_debug_dataset(controller_->get_debug_data());

		// publish data to joints
		controller_->publish_targets();

		// finally execute the movement
		joint_interface_->execute_motion();

		// check if we received joint information in the last step
		unsigned int incoming_data_count = joint_interface_->get_and_clear_incoming_position_count();

		if (incoming_data_count == 0) {
			// printf("> WARNING: no incoming joint data during the last iteration...\n");
			incoming_data_count_invalid++;
			if (incoming_data_count_invalid >= MOTION_UPDATERATE) {
				printf("> ERROR: no incoming joint data for >1 second -> will exit now\n");
				exit(EXIT_FAILURE);
			}
		}
		else {
			incoming_data_count_invalid = 0;
		}

		std::this_thread::sleep_until(timeout);
		timeout += loop_period;
	}

	printf("> motion generation thread exited.\n");
}
