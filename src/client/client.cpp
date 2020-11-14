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
#include <string>

#include "humotion/client/client.h"
#include "humotion/client/middleware_ros.h"

// using namespace std;
// using namespace boost;
// using namespace humotion;
using humotion::client::Client;

//! constructor
//! open a new client instance.
Client::Client(std::string scope, std::string mw) {
	// convert mw to uppercase
	boost::to_upper(mw);

	printf("> initializing humotion client (on %s, middleware=%s)\n", scope.c_str(), mw.c_str());

	// start middleware
	if (mw == "ROS") {
		middleware_ = new humotion::client::MiddlewareROS(scope);
	}
	else {
		printf("> ERROR: invalid mw '%s' given. RSB support was droppd. please use ROS\n\n", mw.c_str());
		exit(EXIT_FAILURE);
	}
}

//! destructor
Client::~Client() {
}

//! check if connection is ok
//! \return true if conn is alive
bool Client::ok() {
	return middleware_->ok();
}

//! do a single middleware tick:
void Client::tick() {
	middleware_->tick();
}

//! set mouth position
//! \param MouthState m to set
//! \param send data to server (optional, use manual call to send_*() to trigger update on server)
void Client::update_mouth_target(MouthState m, bool send) {
	middleware_->update_mouth_target(m, send);
}

//! set gaze direction
//! \param GazeState m to set
//! \param send data to server (optional, use manual call to send_*() to trigger update on server)
void Client::update_gaze_target(GazeState s, bool send) {
	middleware_->update_gaze_target(s, send);
}

//! send all targets to server
void Client::send_all() {
	middleware_->send_all();
}
