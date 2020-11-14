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

#include "humotion_yarp_icub/icub_faceinterface.h"
#include "humotion_yarp_icub/icub_jointinterface.h"

#include <yarp/os/Property.h>
#include <string>

using std::cerr;
using std::cout;
using std::string;

//! constructor
iCubJointInterface::iCubJointInterface(string _scope) : humotion::server::JointInterface() {
	scope = _scope;

	// running in sim?
	if (scope == "/icubSim") {
		cout << "> icub simulation detected\n";
		running_in_simulation_ = true;
	}
	else {
		running_in_simulation_ = false;
	}

	// add mappings from icub ids to humotion ids
	init_id_map();

	// initialise the pd controller for the velocity and position mixer
	init_pv_mix_pid();

	// intantiate the face interface
	face_interface_ = new iCubFaceInterface(scope);

	// intantiate the polydriver
	yarp::os::Property options;
	options.put("device", "remote_controlboard");
	options.put("local", "/local/head");
	options.put("remote", scope + "/head");
	yarp_polydriver_.open(options);

	// fetch yarp views:
	bool success = true;
	// success &= yarp_polydriver_.view(yarp_iencs_);
	success &= yarp_polydriver_.view(yarp_ipos_);
	success &= yarp_polydriver_.view(yarp_ivel_);
	success &= yarp_polydriver_.view(yarp_ilimits_);
	success &= yarp_polydriver_.view(yarp_pid_);
	success &= yarp_polydriver_.view(yarp_amp_);

	if (!success) {
		cout << "ERROR: failed to fetch one or more yarp views... exiting\n";
		exit(EXIT_FAILURE);
	}

	// tell humotion about min/max joint values:
	init_joints();

	// initialise joint controller
	init_controller();
}

//! destructor
iCubJointInterface::~iCubJointInterface() {
	// stop controller on exit
	yarp_ivel_->stop();
	yarp_ipos_->stop();
}

//! init the controller that allows to write target angles or velocities
void iCubJointInterface::init_controller() {
	int number_of_joints;

	if (running_in_simulation_) {
		// running in simulation, use position control mode as velocity seems unsupported right now
		yarp_ipos_->getAxes(&number_of_joints);

		// prepare to set ref accel
		// yarp_commands_.resize(number_of_joints);
		// yarp_commands_ = 200000.0;
		// yarp_ipos_->setRefAccelerations(yarp_commands_.data());

		// enable position control
		yarp_ipos_->setPositionMode();
	}
	else {
		// use velocity controller, first fetch no of axes:
		yarp_ivel_->getAxes(&number_of_joints);

		// set ref acceleration to a value for all axes:
		yarp_commands_.resize(number_of_joints);
		yarp_commands_ = 1e6;
		yarp_ivel_->setRefAccelerations(yarp_commands_.data());

		// finally enable velocity control mode
		yarp_ivel_->setVelocityMode();
	}
}

//! initialise icub joint id to humotion joint id mappings
void iCubJointInterface::init_id_map() {
	insert_icupid_to_humotionid_mapping(ICUB_ID_LIP_LEFT_UPPER, ID_LIP_LEFT_UPPER);
	insert_icupid_to_humotionid_mapping(ICUB_ID_LIP_LEFT_LOWER, ID_LIP_LEFT_LOWER);
	insert_icupid_to_humotionid_mapping(ICUB_ID_LIP_CENTER_UPPER, ID_LIP_CENTER_UPPER);
	insert_icupid_to_humotionid_mapping(ICUB_ID_LIP_CENTER_LOWER, ID_LIP_CENTER_LOWER);
	insert_icupid_to_humotionid_mapping(ICUB_ID_LIP_RIGHT_UPPER, ID_LIP_RIGHT_UPPER);
	insert_icupid_to_humotionid_mapping(ICUB_ID_LIP_RIGHT_LOWER, ID_LIP_RIGHT_LOWER);
	insert_icupid_to_humotionid_mapping(ICUB_ID_NECK_PAN, ID_NECK_PAN);
	insert_icupid_to_humotionid_mapping(ICUB_ID_NECK_TILT, ID_NECK_TILT);
	insert_icupid_to_humotionid_mapping(ICUB_ID_NECK_ROLL, ID_NECK_ROLL);
	//
	insert_icupid_to_humotionid_mapping(ICUB_ID_EYES_PAN, ID_EYES_LEFT_LR);
	insert_icupid_to_humotionid_mapping(ICUB_ID_EYES_VERGENCE, ID_EYES_RIGHT_LR);
	insert_icupid_to_humotionid_mapping(ICUB_ID_EYES_BOTH_UD, ID_EYES_BOTH_UD);
	insert_icupid_to_humotionid_mapping(ICUB_ID_EYES_LEFT_LID_LOWER, ID_EYES_LEFT_LID_LOWER);
	insert_icupid_to_humotionid_mapping(ICUB_ID_EYES_LEFT_LID_UPPER, ID_EYES_LEFT_LID_UPPER);
	insert_icupid_to_humotionid_mapping(ICUB_ID_EYES_LEFT_BROW, ID_EYES_LEFT_BROW);
	insert_icupid_to_humotionid_mapping(ICUB_ID_EYES_RIGHT_LID_LOWER, ID_EYES_RIGHT_LID_LOWER);
	insert_icupid_to_humotionid_mapping(ICUB_ID_EYES_RIGHT_LID_UPPER, ID_EYES_RIGHT_LID_UPPER);
	insert_icupid_to_humotionid_mapping(ICUB_ID_EYES_RIGHT_BROW, ID_EYES_RIGHT_BROW);
}

//! initialize the position and velocity mixer PD controller
void iCubJointInterface::init_pv_mix_pid() {
	// init control variables and last error variable for the internal
	// position and velocity mixer PD controller:
	pv_mix_pid_p_.resize(ICUB_JOINT_ID_ENUM_SIZE);
	pv_mix_pid_d_.resize(ICUB_JOINT_ID_ENUM_SIZE);
	pv_mix_last_error_.resize(ICUB_JOINT_ID_ENUM_SIZE);

	enum_id_bimap_t::const_iterator it;
	for (it = enum_id_bimap.begin(); it != enum_id_bimap.end(); ++it) {
		int id = it->left;
		pv_mix_pid_p_[id] = 5.5;
		pv_mix_pid_d_[id] = 0.3;
		pv_mix_last_error_[id] = 0.0;
	}
}

//! add mapping from icub joint ids to humotion ids
//! this might look strange at the first sight but we need to have a generic
//! way to acces joints from libhumotion. therefore the lib uses its enum with ID_* enum ids
//! to access the joints. now we need to define a mapping to map those to the icub motor ids.
void iCubJointInterface::insert_icupid_to_humotionid_mapping(int icubid, int humotionid) {
	enum_id_bimap.insert(enum_id_bimap_entry_t(icubid, humotionid));
}

void iCubJointInterface::run() {
	float loop_duration_ms = 1000.0 / MAIN_LOOP_FREQUENCY;
	iCubDataReceiver* data_receiver = new iCubDataReceiver(loop_duration_ms, this);
	data_receiver->start();
}

//! stores the target position & velocity of a given joint
//! \param enum id of joint
//! \param float value
void iCubJointInterface::publish_target(int humotion_id, float position, float velocity) {
	// special handler for eye joints
	if ((humotion_id == JointInterface::ID_EYES_LEFT_LR) || (humotion_id == JointInterface::ID_EYES_RIGHT_LR)) {
		// the icub has a combined pan angle for both eyes, so seperate this:
		float target_position_left = get_target_position(JointInterface::ID_EYES_LEFT_LR);
		float target_position_right = get_target_position(JointInterface::ID_EYES_RIGHT_LR);
		float target_velocity_left = get_target_velocity(JointInterface::ID_EYES_LEFT_LR);
		float target_velocity_right = get_target_velocity(JointInterface::ID_EYES_RIGHT_LR);

		// calculate target angles
		float target_position_pan = (target_position_right + target_position_left) / 2;
		float target_position_vergence = (target_position_right - target_position_left);

		/*cout << "LR " << target_position_left << " " << target_position_right <<
		        " PAN " << target_position_pan << " VERGENCE " << target_position_vergence << "\n";*/

		// calculate target velocities
		// for now just use the same velocity for pan and vergence
		float target_velocity_pan = (target_velocity_left + target_velocity_right) / 2.0;
		float target_velocity_vergence = target_velocity_left - target_velocity_right;

		store_icub_joint_target(ICUB_ID_EYES_PAN, target_position_pan, target_velocity_pan);
		store_icub_joint_target(ICUB_ID_EYES_VERGENCE, target_position_vergence, target_velocity_vergence);
	}
	else {
		// convert to icub joint id
		int icub_id = convert_humotion_jointid_to_icub(humotion_id);
		// store target data
		store_icub_joint_target(icub_id, position, velocity);
	}
}

//! set the target data for a given icub joint
//! \param id of joint
//! \param float value of position
void iCubJointInterface::store_icub_joint_target(int icub_id, float position, float velocity) {
	// cout << "store_icub_joint_target(" << icub_id << ", " << position << ", ..)\n";

	if ((icub_id == ICUB_ID_NECK_PAN) || (icub_id == ICUB_ID_EYES_VERGENCE)) {
		// icub uses an inverted neck pan specification
		position = -position;
		velocity = -velocity;
	}

	// store values
	target_angle_[icub_id] = position;
	target_velocity_[icub_id] = velocity;
}

//! execute a move in velocity mode
//! \param id of joint
//! \param angle
void iCubJointInterface::set_target_in_velocitymode(int icub_id) {
	// fetch humotion id from icub joint id
	int humotion_id = convert_icub_jointid_to_humotion(icub_id);

	// fetch the target velocity
	float target_velocity = target_velocity_[icub_id];

	float vmax = 350.0;
	if (target_velocity > vmax)
		target_velocity = vmax;
	if (target_velocity < -vmax)
		target_velocity = -vmax;

	// execute:
	if ((icub_id != ICUB_ID_NECK_PAN) && (icub_id != ICUB_ID_EYES_BOTH_UD) && (icub_id != ICUB_ID_NECK_TILT) &&
	    (icub_id != ICUB_ID_EYES_PAN) && (icub_id != ICUB_ID_EYES_VERGENCE)
	    // && (icub_id != ICUB_ID_NECK_TILT)
	) {
		// limit to some joints for debugging...
		return;
	}

	// we now add a pd control loop for velocity moves in order to handle position errors
	// FIXME: add position interpolation into future. this requires to enable the velocity
	//        broadcast in the torso and head ini file and fetch that velocity in the
	//        icub_data_receiver as well. TODO: check if the can bus has enough bandwidth for that

	// first: fetch the timstamp of the last known position
	humotion::Timestamp data_ts = get_ts_position(humotion_id).get_last_timestamp();

	// fetch current position & velocity
	float current_position = get_ts_position(humotion_id).get_interpolated_value(data_ts);
	float current_velocity = get_ts_speed(humotion_id).get_interpolated_value(data_ts);

	// the icub has a combined eye pan actuator, therefore
	// we have to calculate pan angle and vergence based on the left and right target angles:
	if (icub_id == ICUB_ID_EYES_PAN) {
		// fetch timestamp
		data_ts = get_ts_position(ID_EYES_LEFT_LR).get_last_timestamp();

		// get the left and right positions
		float pos_left = get_ts_position(ID_EYES_LEFT_LR).get_interpolated_value(data_ts);
		float pos_right = get_ts_position(ID_EYES_RIGHT_LR).get_interpolated_value(data_ts);
		current_position = (pos_left + pos_right) / 2.0;

		// same for velocities:
		float vel_left = get_ts_speed(ID_EYES_LEFT_LR).get_interpolated_value(data_ts);
		float vel_right = get_ts_speed(ID_EYES_RIGHT_LR).get_interpolated_value(data_ts);
		current_velocity = (vel_left + vel_right) / 2.0;
	}
	else if (icub_id == ICUB_ID_EYES_VERGENCE) {
		// fetch timestamp
		data_ts = get_ts_position(ID_EYES_LEFT_LR).get_last_timestamp();

		// get the left and right positions
		float pos_left = get_ts_position(ID_EYES_LEFT_LR).get_interpolated_value(data_ts);
		float pos_right = get_ts_position(ID_EYES_RIGHT_LR).get_interpolated_value(data_ts);

		current_position = pos_left - pos_right;

		// same for velocities:
		float vel_left = get_ts_speed(ID_EYES_LEFT_LR).get_interpolated_value(data_ts);
		float vel_right = get_ts_speed(ID_EYES_RIGHT_LR).get_interpolated_value(data_ts);
		current_velocity = (vel_left - vel_right);
	}

	// calculate position error:
	float position_error = target_angle_[icub_id] - current_position;

	// calculate d term
	float error_d = (position_error - pv_mix_last_error_[icub_id]) / (framerate * 1000.0);
	pv_mix_last_error_[icub_id] = position_error;

	// finally do a PD loop to get the target velocity
	float pv_mix_velocity = pv_mix_pid_p_[icub_id] * position_error + pv_mix_pid_p_[icub_id] * error_d + target_velocity;

	/*cout << "\n"
	     << current_position << " "
	     << target_angle_[icub_id] << " "
	     << current_velocity << " "
	     << pv_mix_velocity  << " "
	     << target_velocity  << " "
	     << position_error << " "
	     << " PID" << icub_id << "\n";*/

	if (icub_id > ICUB_ID_EYES_VERGENCE) {
		cerr << "> ERROR: set_target_positionmode(id=" << icub_id << ", ...) not supported\n";
		return;
	}

	// execute velocity move
	bool success;
	int count = 0;
	do {
		if (running_in_simulation_) {
			// running in sim, use position control
			success = yarp_ipos_->positionMove(icub_id, target_angle_[icub_id]);
		}
		else {
			// use smooth velocity control on real robot
			success = yarp_ivel_->velocityMove(icub_id, pv_mix_velocity);
		}

		if (count++ >= 10) {
			cerr << "ERROR: failed to send motion command! gave up after 10 trials\n";
			yarp_ivel_->stop();
			yarp_ipos_->stop();
			exit(EXIT_FAILURE);
		}
	} while (!success);
}

//! actually execute the scheduled motion commands
void iCubJointInterface::execute_motion() {
	// set up neck and eye motion commands in velocitymode
	for (int i = ICUB_ID_NECK_TILT; i <= ICUB_ID_EYES_VERGENCE; i++) {
		set_target_in_velocitymode(i);
	}

	// eyelids: unfortuantely the icub has only 1dof for eyelids
	// therefore we can only use an opening value
	float opening_left = target_angle_[ICUB_ID_EYES_LEFT_LID_UPPER] - target_angle_[ICUB_ID_EYES_LEFT_LID_LOWER];
	float opening_right = target_angle_[ICUB_ID_EYES_RIGHT_LID_UPPER] - target_angle_[ICUB_ID_EYES_RIGHT_LID_LOWER];
	float opening = (opening_left + opening_right) / 2.0;
	// send it to icub face if
	face_interface_->set_eyelid_angle(opening);

	// eyebrows are set using a special command as well:
	face_interface_->set_eyebrow_angle(ICUB_ID_EYES_LEFT_BROW, target_angle_);
	face_interface_->set_eyebrow_angle(ICUB_ID_EYES_RIGHT_BROW, target_angle_);

	// mouth
	face_interface_->set_mouth(target_angle_);

	// store joint values which we do not handle on icub here:
	humotion::Timestamp timestamp = humotion::Timestamp::now();

	JointInterface::store_incoming_position(ID_LIP_LEFT_UPPER, target_angle_[ICUB_ID_LIP_LEFT_UPPER], timestamp);
	JointInterface::store_incoming_position(ID_LIP_LEFT_LOWER, target_angle_[ICUB_ID_LIP_LEFT_LOWER], timestamp);
	JointInterface::store_incoming_position(ID_LIP_CENTER_UPPER, target_angle_[ICUB_ID_LIP_CENTER_UPPER], timestamp);
	JointInterface::store_incoming_position(ID_LIP_CENTER_LOWER, target_angle_[ICUB_ID_LIP_CENTER_LOWER], timestamp);
	JointInterface::store_incoming_position(ID_LIP_RIGHT_UPPER, target_angle_[ICUB_ID_LIP_RIGHT_UPPER], timestamp);
	JointInterface::store_incoming_position(ID_LIP_RIGHT_LOWER, target_angle_[ICUB_ID_LIP_RIGHT_LOWER], timestamp);
}

void iCubJointInterface::set_joint_enable_state(int humotion_id, bool enable) {
	int icub_jointid = convert_humotion_jointid_to_icub(humotion_id);
	/*
	    switch(e){
	        default:
	            break;

	    case(ID_NECK_PAN):
	        icub_jointid = ICUB_ID_NECK_PAN;
	        break;

	    case(ID_NECK_TILT):
	        icub_jointid = ICUB_ID_NECK_TILT;
	        break;

	    case(ID_NECK_ROLL):
	        icub_jointid = ICUB_ID_NECK_ROLL;
	        break;

	    case(ID_EYES_BOTH_UD):
	        icub_jointid = ICUB_ID_EYES_BOTH_UD;
	        break;

	    // icub handles eyes as pan angle + vergence...
	    // -> hack: left eye enables pan and right eye enables vergence
	    case(ID_EYES_LEFT_LR):
	        icub_jointid = ICUB_ID_EYES_PAN;
	        break;

	    case(ID_EYES_RIGHT_LR):
	        icub_jointid = ICUB_ID_EYES_VERGENCE;
	        break;
	    }
	    */

	if ((icub_jointid != -1) && (icub_jointid <= ICUB_ID_EYES_VERGENCE)) {
		if (!running_in_simulation_) {
			// only set amp status on real robot. simulation crashes during this call ... :-X
			if (enable) {
				yarp_amp_->enableAmp(icub_jointid);
				yarp_pid_->enablePid(icub_jointid);
			}
			else {
				yarp_pid_->disablePid(icub_jointid);
				yarp_amp_->disableAmp(icub_jointid);
			}
		}
	}
}

//! prepare and enable a joint
//! NOTE: this should also prefill the min/max positions for this joint
//! \param the enum id of a joint
void iCubJointInterface::enable_joint(int e) {
	set_joint_enable_state(e, true);
}

//! shutdown and disable a joint
//! \param the enum id of a joint
void iCubJointInterface::disable_joint(int e) {
	set_joint_enable_state(e, false);
}

void iCubJointInterface::store_min_max(yarp::dev::IControlLimits* ilimits, int icub_id) {
	double min, max;
	int humotion_id = convert_icub_jointid_to_humotion(icub_id);

	if (!ilimits->getLimits(icub_id, &min, &max)) {
		cerr << "ERROR: failed to call getLimits for icub joint " << icub_id << "\n";
		exit(EXIT_FAILURE);
	}

	joint_min[humotion_id] = min;
	joint_max[humotion_id] = max;
}

//! initialise a joint (set up controller mode etc)
void iCubJointInterface::init_joints() {
	store_min_max(yarp_ilimits_, ICUB_ID_NECK_TILT);
	store_min_max(yarp_ilimits_, ICUB_ID_NECK_ROLL);
	store_min_max(yarp_ilimits_, ICUB_ID_NECK_PAN);
	store_min_max(yarp_ilimits_, ICUB_ID_EYES_BOTH_UD);

	// icub handles eyes differently, we have to set pan angle + vergence
	double pan_min, pan_max, vergence_min, vergence_max;
	yarp_ilimits_->getLimits(ICUB_ID_EYES_PAN, &pan_min, &pan_max);
	yarp_ilimits_->getLimits(ICUB_ID_EYES_VERGENCE, &vergence_min, &vergence_max);

	// this is not 100% correct, should be fixed
	joint_min[ID_EYES_LEFT_LR] = pan_min; // - vergence_max/2;
	joint_max[ID_EYES_LEFT_LR] = pan_max; // - vergence_max/2;
	joint_min[ID_EYES_RIGHT_LR] = joint_min[ID_EYES_LEFT_LR];
	joint_max[ID_EYES_RIGHT_LR] = joint_max[ID_EYES_LEFT_LR];

	// eyelids
	joint_min[ID_EYES_RIGHT_LID_UPPER] = -50; // 24-30;
	joint_max[ID_EYES_RIGHT_LID_UPPER] = 50;  // 48-30;
	// lid_angle = joint_max[ID_EYES_RIGHT_LID_UPPER];

	// eyebrows
	joint_min[ID_EYES_LEFT_BROW] = -50;
	joint_max[ID_EYES_LEFT_BROW] = 50;
	joint_min[ID_EYES_RIGHT_BROW] = joint_min[ID_EYES_LEFT_BROW];
	joint_max[ID_EYES_RIGHT_BROW] = joint_max[ID_EYES_LEFT_BROW];

	// mouth
	joint_min[ID_LIP_CENTER_UPPER] = 5;
	joint_max[ID_LIP_CENTER_UPPER] = 50;
	joint_min[ID_LIP_CENTER_LOWER] = 5;
	joint_max[ID_LIP_CENTER_LOWER] = 50;
	joint_min[ID_LIP_LEFT_UPPER] = 5;
	joint_max[ID_LIP_LEFT_UPPER] = 50;
	joint_min[ID_LIP_LEFT_LOWER] = 5;
	joint_max[ID_LIP_LEFT_LOWER] = 50;
	joint_min[ID_LIP_RIGHT_UPPER] = 5;
	joint_max[ID_LIP_RIGHT_UPPER] = 50;
	joint_min[ID_LIP_RIGHT_LOWER] = 5;
	joint_max[ID_LIP_RIGHT_LOWER] = 50;
}

//! conversion table for humotion joint id to icub joint id
//! \param int value for humotion joint id from JointInterface::JOINT_ID_ENUM
//! \return int value of icub joint id
int iCubJointInterface::convert_humotion_jointid_to_icub(int humotion_id) {
	enum_id_bimap_t::right_const_iterator it = enum_id_bimap.right.find(humotion_id);
	if (it == enum_id_bimap.right.end()) {
		// key does not exist
		cerr << "ERROR: invalid humotion joint id (" << humotion_id << ") given. can not convert this. exiting\n";
		exit(EXIT_FAILURE);
	}
	return it->second;
}

//! conversion table for icub joint id to humotion joint id
//! \param  int value of icub joint id
//! \return int value of humotion joint id from JointInterface::JOINT_ID_ENUM
int iCubJointInterface::convert_icub_jointid_to_humotion(int icub_id) {
	enum_id_bimap_t::left_const_iterator it = enum_id_bimap.left.find(icub_id);
	if (it == enum_id_bimap.left.end()) {
		// key does not exist
		cout << "ERROR: invalid icub joint id given. can not convert this. exiting\n";
		exit(EXIT_FAILURE);
	}
	return it->second;
}
