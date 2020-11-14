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

#ifndef INCLUDE_HUMOTION_SERVER_REFLEXXES_MOTION_GENERATOR_H_
#define INCLUDE_HUMOTION_SERVER_REFLEXXES_MOTION_GENERATOR_H_

#include <ReflexxesAPI.h>
#include <RMLPositionFlags.h>
#include <RMLPositionInputParameters.h>
#include <RMLPositionOutputParameters.h>

#include "humotion/server/motion_generator.h"

namespace humotion {
namespace server {

class ReflexxesMotionGenerator : public MotionGenerator {
public:
	ReflexxesMotionGenerator(JointInterface* j, Config* cfg, int dof, float t);
	~ReflexxesMotionGenerator();

protected:
	void reflexxes_set_input(int dof, float target, float current_position, float current_velocity, Timestamp timestamp,
	                         float max_speed, float max_accel);
	void reflexxes_calculate_profile();

	//*****************************************
	ReflexxesAPI* reflexxes_api;
	RMLPositionInputParameters* reflexxes_position_input;
	RMLPositionOutputParameters* reflexxes_position_output;
	RMLPositionFlags reflexxes_motion_flags;
	//*****************************************
	int dof_count;

private:
};

} // namespace server
} // namespace humotion

#endif // INCLUDE_HUMOTION_SERVER_REFLEXXES_MOTION_GENERATOR_H_
