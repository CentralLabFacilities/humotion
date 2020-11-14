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

#include "humotion/client/middleware.h"
#include <string>

// using namespace std;
// using namespace humotion;
// using namespace humotion::client;

using humotion::client::Middleware;

//! constructor
//! open a new Middleware instance.
Middleware::Middleware(std::string scope) {
	base_scope_ = scope;
}

//! destructor
Middleware::~Middleware() {
}

//! set mouth position
//! \param MouthState m to set
//! \param send data to server (optional, use manual call to send_*() to trigger update on server)
void Middleware::update_mouth_target(MouthState m, bool send) {
	mouth_state_ = m;
	if (send) {
		send_mouth_target();
	}
}

//! set gaze target
//! \param GazeState m to set
//! \param send data to server (optional, use manual call to send_*() to trigger update on server)
void Middleware::update_gaze_target(GazeState s, bool send) {
	gaze_state_ = s;
	if (send) {
		send_gaze_target();
	}
}

//! send all targets to server
void Middleware::send_all() {
	send_mouth_target();
	send_gaze_target();
}
