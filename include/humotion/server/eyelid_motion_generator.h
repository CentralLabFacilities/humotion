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

#ifndef INCLUDE_HUMOTION_SERVER_EYELID_MOTION_GENERATOR_H_
#define INCLUDE_HUMOTION_SERVER_EYELID_MOTION_GENERATOR_H_

#include <chrono>

#include "humotion/server/eye_motion_generator.h"

namespace humotion {
namespace server {

class EyelidMotionGenerator : public EyeMotionGenerator {
public:
	EyelidMotionGenerator(JointInterface* j, Config* cfg);
	~EyelidMotionGenerator();

	void calculate_targets();
	void publish_targets();

private:
	void start_external_eyeblinks(int duration_left, int duration_right);
	void process_saccadic_eyeblinks();
	void process_periodic_eyeblinks();
	void handle_eyeblink_timeout();
	void override_lids_for_eyeblink();
	void check_for_saccade();
	void start_eyeblink(int side, int duration);

	void close_eyelid(int joint_id);

	enum SIDE_ID
	{
		LEFT = 0,
		RIGHT
	};

	bool saccade_blink_active_;
	bool saccade_blink_requested_;

	bool eyeblink_active_[2];
	bool eyelid_closed_[2];

	std::chrono::time_point<std::chrono::steady_clock> periodic_blink_start_time_;
	std::chrono::time_point<std::chrono::steady_clock> eyeblink_timeout_[2];
	std::chrono::time_point<std::chrono::steady_clock> eyeblink_blocked_timeout_;
};

} // namespace server
} // namespace humotion

#endif // INCLUDE_HUMOTION_SERVER_EYELID_MOTION_GENERATOR_H_
