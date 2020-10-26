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

#include <assert.h>

#include "humotion/timestamped_list.h"

using humotion::TimestampedList;
using std::mutex;

TimestampedList::TimestampedList(unsigned int s) {
	// initialize the list to its desired size:
	TimestampedFloat now(Timestamp::now(), 0.0);
	tsf_list_.resize(s, now);
}

TimestampedList::TimestampedList(TimestampedList const& l) {
	// lock the tsf_list for this access. by doing this we assure
	// that no other thread accessing this element can diturb the
	// following atomic instructions:
	const std::lock_guard<std::mutex> lock(l.access_mutex_);

	// now do a deep copy with locking!
	tsf_list_ = l.tsf_list_;
}

void TimestampedList::copy_tsf_list_to(timestamped_float_list_t* target) {
	// lock the tsf_list for this access. by doing this we assure
	// that no other thread accessing this element can diturb the
	// following atomic instructions:
	const std::lock_guard<std::mutex> lock(access_mutex_);

	*target = tsf_list_;
}

humotion::Timestamp TimestampedList::get_first_timestamp() {
	// lock the tsf_list for this access. by doing this we assure
	// that no other thread accessing this element can diturb the
	// following atomic instructions:
	const std::lock_guard<std::mutex> lock(access_mutex_);

	if (tsf_list_.empty()) {
		return Timestamp(0, 0);
	}

	timestamped_float_list_t::iterator it = tsf_list_.begin();
	return it->timestamp;
}

humotion::Timestamp TimestampedList::get_last_timestamp() {
	// lock the tsf_list for this access. by doing this we assure
	// that no other thread accessing this element can diturb the
	// following atomic instructions:
	const std::lock_guard<std::mutex> lock(access_mutex_);

	if (tsf_list_.empty()) {
		return Timestamp(0, 0);
	}

	timestamped_float_list_t::iterator it = tsf_list_.end();
	it--;
	return it->timestamp;
}

void TimestampedList::insert(Timestamp timestamp, float val) {
	// erase first element:
	tsf_list_.pop_front();
	tsf_list_.push_back(TimestampedFloat(timestamp, val));
	// printf("insert [%5.3f] = %5.1f\n",timestamp,val);
}

float TimestampedList::get_newest_value() {
	// lock the tsf_list for this access. by doing this we assure
	// that no other thread accessing this element can diturb the
	// following atomic instructions:
	const std::lock_guard<std::mutex> lock(access_mutex_);

	if (tsf_list_.empty()) {
		printf("> WARNING: requested newest value from empty list, returning 0.0\n");
		return 0.0;
	}

	timestamped_float_list_t::iterator it = tsf_list_.end();
	it--;
	return it->value;
}

float TimestampedList::get_interpolated_value(Timestamp target_ts) {
	// lock the tsf_list for this access. by doing this we assure
	// that no other thread accessing this element can diturb the
	// following atomic instructions:
	const std::lock_guard<std::mutex> lock(access_mutex_);

	TimestampedFloat previous;
	// printf("> latency %3.2fms\n", (Timestamped().to_seconds() - target_ts.to_seconds())*1000.0);

	for (timestamped_float_list_t::iterator it = tsf_list_.begin(); it != tsf_list_.end(); ++it) {
		if (it->timestamp == target_ts) {
			// perfect match, return this value
			return it->value;
		}
		else if (it->timestamp > target_ts) {
			// ok found close target
			if (it == tsf_list_.begin()) {
				// no preceding element
				printf("> warning, timestamp %6.3f smaller than first element %6.3f in timestamped"
				       "list. this should not happen (increase ts buffer?)\n",
				       target_ts.to_seconds(), tsf_list_.begin()->timestamp.to_seconds());
				return it->value;
			}
			else {
				// do interpolation
				return interpolate(*it, previous, target_ts);
			}
		}
		previous = *it;
	}

	// we reached the end, return the last value
	printf("> warning: found no timestamp >= than %f in timestamped list...\n", target_ts.to_seconds());
	printf("           this should not happen as images will always be behind\n");
	printf("           the motor data. returning most recent value (ts=%f)\n", previous.timestamp.to_seconds());

	return previous.value;
}

float TimestampedList::interpolate(TimestampedFloat a, TimestampedFloat b, Timestamp timestamp) {
	// a->timestamp < timestamp <= b->timestamp
	double dist_a = timestamp.to_seconds() - a.timestamp.to_seconds();
	double dist_b = b.timestamp.to_seconds() - timestamp.to_seconds();
	double dist = dist_a + dist_b;

	float interpolation = a.value + (dist_a / dist) * (b.value - a.value);
	return interpolation;
}

// tests
void TimestampedList::run_tests() {
	int size = 10;
	TimestampedList list(size);

	for (int i = 0; i < size; i++) {
		list.insert(Timestamp(i * 100.0, 0), i * 10.0);
	}

	// test algorithm:

	// test exact match:
	for (int i = 0; i < size; i++) {
		Timestamp ts(i * 100.0, 0);
		printf("> testing get_interpolated_value(%f) == %f (value read back = %f)\n", ts.to_seconds(), i * 10.0,
		       list.get_interpolated_value(ts));
		assert(list.get_interpolated_value(ts) == i * 10.0);
	}
	printf("passed test 1\n");

	assert(list.get_interpolated_value(Timestamp(0.0, 0)) == 0.0);
	assert(list.get_interpolated_value(Timestamp(110.0, 0)) == 11.0);
	assert(list.get_interpolated_value(Timestamp(150.0, 0)) == 15.0);
	assert(list.get_interpolated_value(Timestamp(999990.0, 0)) == 90.0);

	printf("passed test 2\n");

	list.insert(Timestamp(1000.0, 0), 200.0);
	list.insert(Timestamp(1300.0, 0), -100.0);
	assert(list.get_interpolated_value(Timestamp(1100, 0)) == 100.0);
	assert(list.get_interpolated_value(Timestamp(1200, 0)) == 0.0);
	assert(list.get_interpolated_value(Timestamp(1300, 0)) == -100.0);
	assert(list.get_interpolated_value(Timestamp(1250, 0)) == -50.0);

	printf("passed test 3\n");
	exit(EXIT_SUCCESS);
}
