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

#ifndef INCLUDE_HUMOTION_GAZE_STATE_H_
#define INCLUDE_HUMOTION_GAZE_STATE_H_

#include <math.h>

#include "humotion/timestamp.h"

namespace humotion {

class GazeState {
 public:
        GazeState();
        ~GazeState();

        void dump();

        // pan tilt roll
        float pan;
        float tilt;
        float roll;

        float pan_offset;
        float tilt_offset;
        float roll_offset;

        // pan,tilt,roll can be relative or absolute
        int gaze_type;

        // when was this target requested?
        Timestamp timestamp;

        // is this relative or absolute
        enum GAZE_STATE_TYPE{
            GAZETYPE_ABSOLUTE = 0,
            GAZETYPE_RELATIVE = 1,
            GAZETYPE_OVERRIDE = 2
        };

        // eye vergence
        float vergence;

        // eyelid opening angle
        float eyelid_opening_upper;
        float eyelid_opening_lower;

        // eyebrow angles
        float eyebrow_left;
        float eyebrow_right;

        // eyeblink request
        int eyeblink_request_left;
        int eyeblink_request_right;
        static const int EYEBLINK_TIME_DEFAULT = 150;  // in ms

        float distance_pt_abs(GazeState b) {
            float dist_pan = (b.pan + b.pan_offset) - (pan + pan_offset);
            float dist_tilt = (b.tilt + b.tilt_offset) - (tilt + tilt_offset);
            return sqrt(pow(dist_pan, 2.0) + pow(dist_tilt, 2.0));
        }

        float distance_tilt_abs(GazeState b) {
            float dist_tilt = (b.tilt + b.tilt_offset) - (tilt + tilt_offset);
            return fabs(dist_tilt);
        }

        float distance_pan_abs(GazeState b) {
            float dist_pan = (b.pan + b.pan_offset) - (pan + pan_offset);
            return fabs(dist_pan);
        }
};

}  // namespace humotion

#endif  // INCLUDE_HUMOTION_GAZE_STATE_H_
