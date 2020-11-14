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

#include <humotion/server/server.h>
#include <stdio.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/Network.h>
#include <yarp/os/Property.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Time.h>
#include <yarp/sig/Vector.h>

#include <iostream>
#include <string>

#include "humotion_yarp_icub/icub_data_receiver.h"
#include "humotion_yarp_icub/icub_jointinterface.h"

using std::cerr;
using yarp::os::Network;
using yarp::os::Property;

// using namespace yarp::dev;
// using namespace yarp::sig;
// using namespace yarp::os;
// using namespace std;

int main(int argc, char* argv[]) {
	Network yarp;

	if (!yarp.checkNetwork()) {
		cerr << "no yarp network, exiting\n";
		return EXIT_FAILURE;
	}

	Property params;
	params.fromCommand(argc, argv);

	if (!params.check("robot")) {
		cerr << "Please specify the name of the robot\n";
		cerr << "--robot name (e.g. icub or icubSim)\n";
		return EXIT_FAILURE;
	}

	std::string robotName = params.find("robot").asString().c_str();
	std::string scope = "/" + robotName;

	// create humotion interface
	iCubJointInterface* icub_jointinterface = new iCubJointInterface(scope);
	humotion::server::Server* humotion_server = new humotion::server::Server(scope, "ROS", icub_jointinterface);

	icub_jointinterface->run();

	// pre-/re-configure some dyn reconfig variables
	int syscall_result;
	syscall_result =
	   std::system(("rosrun dynamic_reconfigure dynparam set " + scope + "/humotion/configuration " +
	                " \"{'use_neck_target_instead_of_position_eye':false, " + "'eyelids_follow_eyemotion':false}\"")
	                  .c_str());

	if (syscall_result != 0) {
		std::cerr << "ERROR: failed to set dynamic reconf parameters!" << std::endl;
		return EXIT_FAILURE;
	}

	while (humotion_server->ok()) {
		usleep(1000);
	}

	return EXIT_SUCCESS;
}
