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

#ifndef INCLUDE_HUMOTION_CLIENT_MIDDLEWARE_H_
#define INCLUDE_HUMOTION_CLIENT_MIDDLEWARE_H_

#include <cstdint>
#include <cstdio>
#include <string>

#include "humotion/gaze_state.h"
#include "humotion/mouth_state.h"

namespace humotion {
namespace client {

class Middleware {
 public:
    explicit Middleware(std::string name);
    ~Middleware();

    void update_mouth_target(MouthState s, bool send);
    void update_gaze_target(GazeState s, bool send);
    void send_all();

    virtual void send_mouth_target() = 0;
    virtual void send_gaze_target() = 0;
    virtual bool ok() = 0;
    virtual void tick() = 0;

 protected:
    MouthState mouth_state_;
    GazeState gaze_state_;
    std::string base_scope_;
};

}  // namespace client
}  // namespace humotion

#endif  // INCLUDE_HUMOTION_CLIENT_MIDDLEWARE_H_
