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

#include "client/middleware_rsb.h"

#ifdef RSB_SUPPORT

#define BOOST_SIGNALS_NO_DEPRECATION_WARNING 1
#include <rsb/converter/ProtocolBufferConverter.h>
#include <rsb/converter/Repository.h>
#include <rsb/Factory.h>
#include <rsb/Listener.h>
#include <rsb/MetaData.h>
#include <rsb/patterns/RemoteServer.h>
#include <string>

#include "humotion/gaze.h"
#include "humotion/mouth.h"

// using namespace std;
// using namespace rsb;
// using namespace boost;
// using namespace humotion;
using humotion::client::MiddlewareRSB;

//! constructor
MiddlewareRSB::MiddlewareRSB(const string& scope) : Middleware(scope) {
	printf("> registering converters\n");

	try {
		// converter for GazeTarget
		rsb::converter::Converter<string>::Ptr gazeTargetConverter(
		   new rsb::converter::ProtocolBufferConverter<rst::robot::HumotionGazeTarget>());
		rsb::converter::converterRepository<string>()->registerConverter(gazeTargetConverter);
	}
	catch (std::invalid_argument e) {
		printf("> it seems like a converter for rst::robot::HumotionGazeTarget "
		       "is already registered. fine, will not add another converter\n");
	}

	try {
		// converter for MouthTarget
		rsb::converter::Converter<string>::Ptr mouthTargetConverter(
		   new rsb::converter::ProtocolBufferConverter<rst::robot::MouthTarget>());
		rsb::converter::converterRepository<std::string>()->registerConverter(mouthTargetConverter);
	}
	catch (std::invalid_argument e) {
		printf("> it seems like a converter for rst::robot::MouthTarget is "
		       "already registered. fine, will not add another converter\n");
	}

	// first get a factory instance that is used to create RSB domain objects
	Factory& factory = getFactory();

	// create informer
	mouth_target_informer = factory.createInformer<rst::robot::MouthTarget>(base_scope + "/humotion/mouth/target");
	gaze_target_informer = factory.createInformer<rst::robot::HumotionGazeTarget>(base_scope + "/humotion/gaze/target");

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

//! send mouth target to server
void MiddlewareRSB::send_mouth_target() {
	// build target packet
	std::shared_ptr<rst::robot::MouthTarget> request(new rst::robot::MouthTarget());

	request->set_position_left(mouth_state.position_left);
	request->set_position_center(mouth_state.position_center);
	request->set_position_right(mouth_state.position_right);

	request->set_opening_left(mouth_state.opening_left);
	request->set_opening_center(mouth_state.opening_center);
	request->set_opening_right(mouth_state.opening_right);

	mouth_target_informer->publish(request);
}

//! send mouth target to server
void MiddlewareRSB::send_gaze_target(int gaze_type) {
	// build target packet
	std::shared_ptr<rst::robot::HumotionGazeTarget> request(new rst::robot::HumotionGazeTarget());

	request->set_pan(gaze_state.pan);
	request->set_tilt(gaze_state.tilt);
	request->set_roll(gaze_state.roll);
	request->set_vergence(gaze_state.vergence);

	request->set_pan_offset(gaze_state.pan_offset);
	request->set_tilt_offset(gaze_state.tilt_offset);
	request->set_roll_offset(gaze_state.roll_offset);

	request->set_eyelid_opening_upper(gaze_state.eyelid_opening_upper);
	request->set_eyelid_opening_lower(gaze_state.eyelid_opening_lower);

	request->set_eyebrow_left(gaze_state.eyebrow_left);
	request->set_eyebrow_right(gaze_state.eyebrow_right);

	request->set_eyeblink_request_left(gaze_state.eyeblink_request_left);
	request->set_eyeblink_request_right(gaze_state.eyeblink_request_right);

	if (gaze_state.type == GazeState::ABSOLUTE) {
		request->set_type(rst::robot::HumotionGazeTarget::ABSOLUTE);
	}
	else {
		request->set_type(rst::robot::HumotionGazeTarget::RELATIVE);
	}

	// add position to send queue
	gaze_target_informer->publish(request);
}

#endif
