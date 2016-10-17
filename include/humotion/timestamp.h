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

#ifndef INCLUDE_HUMOTION_TIMESTAMP_H_
#define INCLUDE_HUMOTION_TIMESTAMP_H_

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include <string>

namespace humotion {

class Timestamp {
 public:
    Timestamp(void);
    Timestamp(uint32_t _sec, uint32_t _nsec);
    explicit Timestamp(double dsec);

    void set(uint32_t _sec, uint32_t _nsec);
    void set(Timestamp a);

    static Timestamp now(void);

    bool is_null(void);
    double to_seconds(void);

    bool operator>  (const Timestamp &cmp) const;
    bool operator<= (const Timestamp &cmp) const;
    bool operator<  (const Timestamp &cmp) const;
    bool operator>= (const Timestamp &cmp) const;
    bool operator== (const Timestamp &cmp) const;
    bool operator!= (const Timestamp &cmp) const;

    uint32_t sec;
    uint32_t nsec;
};

}  // namespace humotion

#endif  // INCLUDE_HUMOTION_TIMESTAMP_H_
