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

#include <stdio.h>

#include "humotion/gaze_state.h"

using humotion::GazeState;

GazeState::GazeState() {
    pan = 0.0;
    tilt = 0.0;
    roll = 0.0;
    vergence = 0.0;

    pan_offset = 0.0;
    tilt_offset = 0.0;
    roll_offset = 0.0;

    eyelid_opening_upper = 35.0;
    eyelid_opening_lower = 35.0;

    eyebrow_left   = -3.0;
    eyebrow_right  = 3.0;

    gaze_type = GAZETYPE_ABSOLUTE;

    eyeblink_request_right = 0;
    eyeblink_request_left  = 0;
}

GazeState::~GazeState() {
}

void GazeState::dump() {
    // dump values to stdout
    printf("> GAZE STATE: %4.2f [%2.1f] %4.2f [%2.1f] %4.2f [%2.1f] (PTR) vergence=%4.2f "
           "eyelid_opening=%4.2f/%4.2f eyebrows = %4.2f %4.2f type=%s [ts=%.3f] "
           "eyeblink_requests=%d %d\n",
           pan, pan_offset, tilt, tilt_offset, roll, roll_offset, vergence,
           eyelid_opening_upper, eyelid_opening_lower,
           eyebrow_left, eyebrow_right,
           (gaze_type == GAZETYPE_ABSOLUTE?"absolute":(gaze_type == GAZETYPE_RELATIVE?"relative":
                (gaze_type == GAZETYPE_OVERRIDE?"override":"!INVALID!"))),
           timestamp.to_seconds(),
           eyeblink_request_left, eyeblink_request_right);
}
