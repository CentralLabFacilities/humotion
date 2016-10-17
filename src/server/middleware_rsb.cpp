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

#include "server/middleware_rsb.h"

#ifdef RSB_SUPPORT


#define BOOST_SIGNALS_NO_DEPRECATION_WARNING 1
#include <rsb/converter/ProtocolBufferConverter.h>
#include <rsb/converter/Repository.h>
#include <rsb/Factory.h>
#include <rsb/Listener.h>
#include <rsb/MetaData.h>
#include <rsb/patterns/RemoteServer.h>

#include <string>

// using namespace std;
// using namespace boost;
// using namespace humotion;
// using namespace humotion::server;
// using namespace rsb;
// using namespace rsb::patterns;

WARNING:
// RSB interface might be deprtecated and needs some backporting
// from the ROS code. [todo]

//! constructor
MiddlewareRSB::MiddlewareRSB(string scope, Controller *c) : Middleware(scope, c) {
    printf("> using RSB middleware\n");
    printf("> registering converters\n");

    // converter for GazeTarget
    rsb::converter::Converter<string>::Ptr gazeTargetConverter(
                new rsb::converter::ProtocolBufferConverter<rst::robot::HumotionGazeTarget>());
    rsb::converter::converterRepository<string>()->registerConverter(gazeTargetConverter);

    // converter for MouthTarget
    rsb::converter::Converter<string>::Ptr mouthTargetConverter(
                new rsb::converter::ProtocolBufferConverter<rst::robot::MouthTarget>());
    rsb::converter::converterRepository<string>()->registerConverter(mouthTargetConverter);

    // first get a factory instance that is used to create RSB domain objects
    Factory &factory = getFactory();

    // create listeners
    mouth_target_listener = factory.createListener(base_scope + "/humotion/mouth/target");
    mouth_target_listener->addHandler(HandlerPtr(
                                  new DataFunctionHandler<rst::robot::MouthTarget>(
                                  boost::bind(&MiddlewareRSB::incoming_mouth_target, this, _1))));

    gaze_target_listener  = factory.createListener(base_scope + "/humotion/gaze/target");
    gaze_target_listener->addHandler(HandlerPtr(
                                  new EventFunctionHandler(
                                  boost::bind(&MiddlewareRSB::incoming_gaze_target, this, _1))));

    // informer->publish(gaze_target);
    printf("> MiddlewareRSB initialised\n");
}


//! destructor
MiddlewareRSB::~MiddlewareRSB() {
}

//! connection ok?
//! \return true if conn is alive
bool MiddlewareRSB::ok() {
    return true;
}

//! do a single tick
void MiddlewareRSB::tick() {
    // nothing to do
}

//! callback to handle incoming mouth  target
void MiddlewareRSB::incoming_mouth_target(boost::shared_ptr<rst::robot::MouthTarget> msg) {
    // printf("> incoming mouth_target\n");
    MouthState mouth_state;

    mouth_state.position_left   = msg->position_left();
    mouth_state.position_center = msg->position_center();
    mouth_state.position_right  = msg->position_right();

    mouth_state.opening_left   = msg->opening_left();
    mouth_state.opening_center = msg->opening_center();
    mouth_state.opening_right  = msg->opening_right();

    controller->set_mouth_target(mouth_state);
}

//! callback to handle incoming gaze  target
void MiddlewareRSB::incoming_gaze_target(rsb::EventPtr event) {
    // printf("> incoming gaze_target [P:%3.1f, T:%3.1f, R:%3.1f]\n",
    // msg->pan(), msg->tilt(), msg->roll());

    boost::shared_ptr<void> ev_data = event->getData();
    rst::robot::HumotionGazeTarget *msg = (rst::robot::HumotionGazeTarget*) ev_data.get();

    GazeState gaze_state;

    if (msg->gaze_type() == rst::robot::HumotionGazeTarget::ABSOLUTE) {
        gaze_state.gaze_type = GazeState::ABSOLUTE;
    } else {
        gaze_state.gaze_type = GazeState::RELATIVE;
    }

    gaze_state.pan  = msg->pan();
    gaze_state.tilt = msg->tilt();
    gaze_state.roll = msg->roll();
    gaze_state.vergence = msg->vergence();

    gaze_state.pan_offset  = msg->pan_offset();
    gaze_state.tilt_offset = msg->tilt_offset();
    gaze_state.roll_offset = msg->roll_offset();

    gaze_state.eyelid_opening_upper = msg->eyelid_opening_upper();
    gaze_state.eyelid_opening_lower = msg->eyelid_opening_lower();

    gaze_state.eyebrow_left = msg->eyebrow_left();
    gaze_state.eyebrow_right = msg->eyebrow_right();

    gaze_state.eyeblink_request_left = msg->eyeblink_request_left();
    gaze_state.eyeblink_request_right = msg->eyeblink_request_right();

    gaze_state.timestamp = msg->gaze_timestamp();
    // FIXME: convert RSB timestamp to our representation
    // event->getMetaData().getCreateTime() / 1000.0;
    // createTime() returns milliseconds -> convert to seconds

    controller->set_gaze_target(gaze_state);
}

#endif
