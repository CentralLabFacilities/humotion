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

#include <boost/format.hpp>
#include <humotion/server/joint_interface.h>
#include <yarp/os/Property.h>

#include <string>

#include "humotion_yarp_icub/icub_data_receiver.h"

using std::cerr;
using std::cout;
using std::string;

using humotion::Timestamp;
using humotion::server::JointInterface;

using yarp::dev::IEncodersTimed;
using yarp::sig::Vector;

#define ICUB_DATA_RECEIVER_USE_ENCODERSPEED 0
#define ICUB_DATA_RECEIVER_DUMP_DATA 0

//! constructor
//! \param period_ms for the yarp rate thread
//! \param icub_jointinterface
iCubDataReceiver::iCubDataReceiver(int period_ms, iCubJointInterface* icub_jointinterface)
   : yarp::os::RateThread(period_ms) {
	// store pointer to icub jointinterface
	icub_jointinterface_ = icub_jointinterface;

	// fetch iencs view from yarp
	yarp::dev::PolyDriver* poly_driver = icub_jointinterface->get_yarp_polydriver();
	bool success = poly_driver->view(iencs_);
	if (!success) {
		cerr << "ERROR: polydriver failed to init iencs view\n";
		exit(EXIT_FAILURE);
	}

	// resize data storage vectors to match the number of axes
	int joints;
	iencs_->getAxes(&joints);
	positions_.resize(joints);
	velocities_.resize(joints);
	timestamps_.resize(joints);
}

//! yarp rate thread initializer
bool iCubDataReceiver::threadInit() {
	return true;
}

//! yarp thread release function
void iCubDataReceiver::threadRelease() {
}

//! manully calculate joint velocities (instead of using joint encoder speeds)
//! \param positions vector with current encoder position
//! \param timestamps vector with the associated timestamps
//! \return velocities as vector
Vector iCubDataReceiver::calculate_velocities(Vector positions, Vector timestamps) {
	Vector velocities;
	velocities.resize(positions.size());

	if (previous_positions_.size() == 0) {
		// first run, no valid old position available, return zero velocities
		// by setting all all elements to zero
		velocities = 0.0;
	}
	else {
		// calculate speed based on positions:
		for (int i = 0; i < positions.size(); i++) {
			float diff = positions[i] - previous_positions_[i];
			float timediff = timestamps[i] - previous_timestamps_[i];
			// calc speed:
			velocities[i] = diff / timediff;
		}
	}

	previous_positions_ = positions;
	previous_timestamps_ = timestamps;

	return velocities;
}

//! main loop routine, called by yarp rate thread
void iCubDataReceiver::run() {
	float velocity;
	Timestamp timestamp;

	// grab pos+vel data:
	iencs_->getEncodersTimed(positions_.data(), timestamps_.data());

#if ICUB_DATA_RECEIVER_USE_ENCODERSPEED
	// fetch data from icub. NOTE: make sure to enable the vel broadcast in the ini file!
	if (!iencs_->getEncoderSpeeds(velocities_.data())) {
		cout << "Failed to fetch encoder speeds...\n";
		return;
	}
#else
	// manually calculate the speed based on old position:
	velocities_ = calculate_velocities(positions_, timestamps_);
#endif

	// publish data to humotion
	for (int i = 0; i < positions_.size(); i++) {
		// convert to humotion timestamp
		timestamp = Timestamp(timestamps_[i]);
		// store position values
		store_incoming_position(i, positions_[i], timestamp);
		// store velocity
		store_incoming_velocity(i, velocities_[i], timestamp);
	}

	// small hack to tell humotion to update the lid angle
	// fixme: use real id
	timestamp = Timestamp::now();
	store_incoming_position(100, 0.0, timestamp);

#if ICUB_DATA_RECEIVER_DUMP_DATA
	dump_incoming_data();
#endif
}

//! store incoming position for a given icub joint
//! \param icub _id icub joint id
//! \param position
//! \param timestamp
void iCubDataReceiver::store_incoming_position(int icub_id, double position, Timestamp timestamp) {
	// cout << "store_incoming_position(icub=" << icub_id << ", " << position << ")\n";

	// store joint position in humotion backend
	if ((icub_id == iCubJointInterface::ICUB_ID_EYES_PAN) || (icub_id == iCubJointInterface::ICUB_ID_EYES_VERGENCE)) {
		// the icub handles eyes differently
		// instead of using seperate left/right pan the icub uses
		// a combined pan angle and vergence. therfore we have to convert this here:
		if (icub_id == iCubJointInterface::ICUB_ID_EYES_PAN) {
			target_eye_pan_ = position;
		}
		else {
			target_eye_vergence_ = -position;
		}

		float right = target_eye_pan_ + target_eye_vergence_ / 2.0;
		float left = target_eye_pan_ - target_eye_vergence_ / 2.0;

		icub_jointinterface_->store_incoming_position(JointInterface::ID_EYES_LEFT_LR, left, timestamp);
		icub_jointinterface_->store_incoming_position(JointInterface::ID_EYES_RIGHT_LR, right, timestamp);
	}
	else if (icub_id == 100) {
		// HACK
		// icub_jointinterface->store_incoming_position(ID_EYES_RIGHT_LID_UPPER,
		//                                             lid_angle, timestamp);
	}
	else {
		if (icub_id == iCubJointInterface::ID_NECK_PAN) {
			// icub uses an inverted neck pan specification
			position = -position;
		}

		// known configured mapping between joint ids
		int humotion_id = icub_jointinterface_->convert_icub_jointid_to_humotion(icub_id);
		icub_jointinterface_->store_incoming_position(humotion_id, position, timestamp);
	}
}

//! store incoming velocity for a given icub joint
//! \param icub_id icub joint id
//! \param velocity
//! \param timestamp
void iCubDataReceiver::store_incoming_velocity(int icub_id, double velocity, Timestamp timestamp) {
	// cout << "store_incoming_velocity(icub=" << icub_id << ", " << velocity << ")\n";

	// store joint position in humotion backend
	if ((icub_id == iCubJointInterface::ICUB_ID_EYES_PAN) || (icub_id == iCubJointInterface::ICUB_ID_EYES_VERGENCE)) {
		// the icub handles eyes differently
		// instead of using seperate left/right pan the icub uses
		// a combined pan angle and vergence. therfore we have to convert this here:
		if (icub_id == iCubJointInterface::ICUB_ID_EYES_PAN) {
			target_eye_pan_velocity_ = velocity;
		}
		else {
			target_eye_vergence_velocity_ = -velocity;
		}

		float right = target_eye_pan_velocity_ + target_eye_vergence_velocity_ / 2.0;
		float left = target_eye_pan_velocity_ - target_eye_vergence_velocity_ / 2.0;

		icub_jointinterface_->store_incoming_velocity(JointInterface::ID_EYES_LEFT_LR, left, timestamp);
		icub_jointinterface_->store_incoming_velocity(JointInterface::ID_EYES_RIGHT_LR, right, timestamp);
	}
	else if (icub_id == 100) {
		// HACK
		// icub_jointinterface->store_incoming_position(ID_EYES_RIGHT_LID_UPPER,
		//                                             lid_angle, timestamp);
	}
	else {
		if (icub_id == iCubJointInterface::ID_NECK_PAN) {
			// icub uses an inverted neck pan specification
			velocity = -velocity;
		}

		// known configured mapping between joint ids
		int humotion_id = icub_jointinterface_->convert_icub_jointid_to_humotion(icub_id);
		icub_jointinterface_->store_incoming_velocity(humotion_id, velocity, timestamp);
	}
}

//! helper for debugging purposes, feed this data into gnuplot for visual inspection
void iCubDataReceiver::dump_incoming_data() {
	// use gnuplot for viz:
	// ./icub_humotion_server --robot icub | grep "INCOMING" |tee  log
	// @gnuplot: plot "log" using 0:1 w l t "p neck tilt", "log" using 0:2 w l t "v neck tilt", \
    //                "log" using 0:5 w l t "p neck pan", "log" using 0:6 w l t "v neck pan", \
    //                "log" using 0:7 w l t "p eyes ud", "log" using 0:8 w l t "v eyes ud", \
    //                "log" using 0:9 w l t "p eyes vergence", "log" using 0:10 w l t "v eyes verg"
	cout << "\n";
	// publish data to humotion
	for (int i = 0; i < positions_.size(); i++) {
		cout << positions_[i] << " ";
		cout << velocities_[i] << " ";
	}
	cout << " #INCOMING_DATA_DUMP\n";
}
