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

#ifndef INCLUDE_HUMOTION_SERVER_SERVER_H_
#define INCLUDE_HUMOTION_SERVER_SERVER_H_

#include <cstdint>
#include <cstdio>
#include <string>

#include "humotion/server/controller.h"
#include "humotion/server/middleware.h"

namespace humotion {
namespace server {

class Server {
public:
	Server(std::string name, std::string mw, JointInterface* _joint_interface);
	~Server();
	bool ok();
	static const float MOTION_UPDATERATE;

private:
	void start_motion_generation_thread();
	void motion_generation_thread();
	boost::thread* motion_generation_thread_ptr_;

	Middleware* middleware_;
	Controller* controller_;
	JointInterface* joint_interface_;
};

} // namespace server
} // namespace humotion

#endif // INCLUDE_HUMOTION_SERVER_SERVER_H_
