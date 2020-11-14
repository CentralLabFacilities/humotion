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

#ifndef INCLUDE_HUMOTION_SERVER_CONTROLLER_H_
#define INCLUDE_HUMOTION_SERVER_CONTROLLER_H_

#include <string>
#include <vector>

#include "humotion/gaze_state.h"
#include "humotion/mouth_state.h"
#include "humotion/server/config.h"
#include "humotion/server/joint_interface.h"
#include "humotion/server/motion_generator.h"
#include "humotion/timestamp.h"

namespace humotion {
namespace server {

class Controller {
public:
	explicit Controller(JointInterface* j);
	~Controller();

	void init_motion_generators();
	void calculate_targets();
	void publish_targets();
	debug_data_t get_debug_data();

	void set_gaze_target(GazeState s);
	void set_mouth_target(MouthState s);
	void set_activated(void);
	Config* get_config();

private:
	int bla;
	debug_data_t debug_data_;
	void store_debug_data(std::string name, float value);

	void add_motion_generator(MotionGenerator* m);
	GazeState relative_gaze_to_absolute_gaze(GazeState relative);

	typedef std::vector<MotionGenerator*> motion_generator_vector_t;
	motion_generator_vector_t motion_generator_vector_;

	JointInterface* joint_interface_;
	Config* config_;

	bool activated_;

	Timestamp last_known_absolute_timestamp_;
	double last_known_absolute_target_pan_;
	double last_known_absolute_target_tilt_;
	double last_known_absolute_target_roll_;
};

} // namespace server
} // namespace humotion

#endif // INCLUDE_HUMOTION_SERVER_CONTROLLER_H_
