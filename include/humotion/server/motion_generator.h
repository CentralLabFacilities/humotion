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

#ifndef INCLUDE_HUMOTION_SERVER_MOTION_GENERATOR_H_
#define INCLUDE_HUMOTION_SERVER_MOTION_GENERATOR_H_

#include <boost/date_time/posix_time/posix_time.hpp>

#include <map>
#include <string>

#include "humotion/server/config.h"
#include "humotion/server/joint_interface.h"

namespace humotion {
namespace server {

class MotionGenerator {
public:
	MotionGenerator(JointInterface* j, Config* cfg);
	~MotionGenerator();

	virtual void calculate_targets() = 0;
	virtual void publish_targets() = 0;

	virtual void set_gaze_target(GazeState s);
	virtual void set_mouth_target(MouthState s);

	debug_data_t get_debug_data();

protected:
	Config* config;
	debug_data_t debug_data_;
	void store_debug_data(std::string name, float value);

	float get_current_position(int joint_id);
	float get_current_speed(int joint_id);
	humotion::Timestamp get_timestamped_state(int joint_id, float* position, float* velocity);

	float limit_target(int joint_id, float val);
	bool mouth_target_input_active();
	bool gaze_target_input_active();

	JointInterface* joint_interface_;

	// gaze
	GazeState requested_gaze_state_;
	boost::system_time last_gaze_target_update_;

	// mouth
	MouthState requested_mouth_target_;
	boost::system_time last_mouth_target_update_;
};

} // namespace server
} // namespace humotion

#endif // INCLUDE_HUMOTION_SERVER_MOTION_GENERATOR_H_
