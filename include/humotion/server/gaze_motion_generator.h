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

#ifndef INCLUDE_HUMOTION_SERVER_GAZE_MOTION_GENERATOR_H_
#define INCLUDE_HUMOTION_SERVER_GAZE_MOTION_GENERATOR_H_

#include "humotion/server/reflexxes_motion_generator.h"
#include "humotion/timestamped_list.h"

namespace humotion {
namespace server {

class GazeMotionGenerator : public ReflexxesMotionGenerator {
public:
	GazeMotionGenerator(JointInterface* j, Config* cfg, int dof, float t);
	~GazeMotionGenerator();

	void set_gaze_target(GazeState s);

protected:
	GazeState get_current_gaze();

	bool get_eye_saccade_active();
	bool neck_saccade_requested;
	bool neck_saccade_omr;
};

} // namespace server
} // namespace humotion

#endif // INCLUDE_HUMOTION_SERVER_GAZE_MOTION_GENERATOR_H_
