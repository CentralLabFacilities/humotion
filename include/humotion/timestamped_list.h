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

#ifndef INCLUDE_HUMOTION_TIMESTAMPED_LIST_H_
#define INCLUDE_HUMOTION_TIMESTAMPED_LIST_H_

#include <boost/thread/mutex.hpp>

#include <list>

#include "humotion/timestamp.h"
#include "humotion/timestamped_float.h"

namespace humotion {
class TimestampedList {
 public:
    // defaults to 10 seconds, this is way too much but should do
    explicit TimestampedList(unsigned int size = 10*100);
    TimestampedList(TimestampedList const &l);

    typedef std::list<TimestampedFloat> timestamped_float_list_t;

    void insert(Timestamp ts, float val);
    float get_interpolated_value(Timestamp target_ts);
    float get_newest_value();
    static void run_tests();
    void copy_tsf_list_to(timestamped_float_list_t *target);
    Timestamp get_last_timestamp();
    Timestamp get_first_timestamp();

 private:
    mutable boost::mutex access_mutex_;
    float interpolate(TimestampedFloat a, TimestampedFloat b, Timestamp timestamp);
    timestamped_float_list_t tsf_list_;
};

}  // namespace humotion

#endif  // INCLUDE_HUMOTION_TIMESTAMPED_LIST_H_
