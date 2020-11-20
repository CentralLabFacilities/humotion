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

#include <math.h>

#include "humotion/timestamp.h"

using humotion::Timestamp;

Timestamp::Timestamp() {
	// init of an empty timestamp will be assigned to the current system time
	set(now());
}

Timestamp::Timestamp(uint32_t _sec, uint32_t _nsec) {
	set(_sec, _nsec);
}

Timestamp::Timestamp(double dsec) {
	double fsec, fnsec;
	fnsec = modf(dsec, &fsec);
	sec = fsec;
	nsec = fnsec * 1000000000.0;
}

Timestamp Timestamp::now() {
	struct timespec tp;
	clock_gettime(CLOCK_REALTIME, &tp);
	return Timestamp(tp.tv_sec, tp.tv_nsec);
}

void Timestamp::set(uint32_t _sec, uint32_t _nsec) {
	sec = _sec;
	nsec = _nsec;
}

void Timestamp::set(Timestamp a) {
	set(a.sec, a.nsec);
}

double Timestamp::to_seconds() const {
	return sec + (static_cast<double>(nsec)) / 1000000000.0;
}

bool Timestamp::operator<=(const Timestamp& cmp) const {
	if (sec < cmp.sec) {
		return true;
	}
	else if (sec > cmp.sec) {
		return false;
	}
	else { // (a.sec == b.sec)
		// seconds are equal, check nsec:
		return (nsec <= cmp.nsec);
	}
}

bool Timestamp::is_null() const {
	return (sec == 0) && (nsec == 0);
}

bool Timestamp::operator<(const Timestamp& cmp) const {
	if (sec < cmp.sec) {
		return true;
	}
	else if (sec > cmp.sec) {
		return false;
	}
	else { // (a.sec == b.sec)
		// seconds are equal, check nsec:
		return (nsec < cmp.nsec);
	}
}

bool Timestamp::operator>=(const Timestamp& cmp) const {
	if (sec > cmp.sec) {
		return true;
	}
	else if (sec < cmp.sec) {
		return false;
	}
	else { // (a.sec == b.sec)
		// seconds are equal, check nsec:
		return (nsec >= cmp.nsec);
	}
}

bool Timestamp::operator>(const Timestamp& cmp) const {
	if (sec > cmp.sec) {
		return true;
	}
	else if (sec < cmp.sec) {
		return false;
	}
	else { // (a.sec == b.sec)
		// seconds are equal, check nsec:
		return (nsec > cmp.nsec);
	}
}

bool Timestamp::operator==(const Timestamp& cmp) const {
	return (sec == cmp.sec) && (nsec == cmp.nsec);
}

bool Timestamp::operator!=(const Timestamp& cmp) const {
	return !(*this == cmp);
}
