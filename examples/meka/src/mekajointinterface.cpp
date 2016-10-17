#include "mekajointinterface.h"

using std::cout;
using std::cerr;

using humotion::Timestamp;

//WARNING: DO NOT CHANGE THIS; VELOCITYMODE IS NOT YET IMPLEMENTED
#define POSITION_CONTROL 1

void MekaJointInterface::incoming_controlstate(const m3meka_msgs::M3ControlStates &control_state){
    //incoming controller status
    for (unsigned int i = 0; i<control_state.group_name.size(); i++){
        if (control_state.group_name[i] == "head"){
            //enable/disable based on controller status
            if (control_state.state[i] == m3meka_msgs::M3ControlStates::START){
                //controller up and running
                if (!controller_enabled){
                    printf("> incoming control state (%d), enabling jointstate output\n",control_state.state[i]);
                    controller_enabled = true;
                }
            }else{
                //controller in estop/stopped etc
                if (controller_enabled){
                    printf("> incoming control state (%d), DISABLING jointstate output\n",control_state.state[i]);
                    controller_enabled = false;
                }
            }
        }
    }
}

void MekaJointInterface::incoming_jointstates(const sensor_msgs::JointState & msg){
    //fetch current timestamp
    Timestamp timestamp = Timestamp(msg.header.stamp.toSec());

    //iterate through incoming joints and filter out joints we need:
    for(int i=0; i<msg.name.size(); i++){

        std::string name = msg.name[i];
        //printf("incoming data for joint '%s'\n", name.c_str());

        int id = -1;
        if (name == "head_j1"){
            id = ID_NECK_PAN;
        }else if(name == "head_j0"){
            id = ID_NECK_TILT;
        }

        //store data:
        if (id != -1){
            //printf("> storing joint data for joint id %d\n", id);
            if (i >= msg.position.size()){
                printf("> joint state msg is missing position data for joint '%s'...\n", name.c_str());
                return;
            }
            if (i >= msg.velocity.size()){
                //printf("> joint state msg is missing velocity data for joint '%s'...\n", name.c_str());
                //exit(EXIT_FAILURE);
                return;
            }
            //ok, safe to access data
            if (id == ID_NECK_PAN){
                //joint is inverted
                JointInterface::store_incoming_position(id, -180.0 / M_PI * msg.position[i], timestamp);
                JointInterface::store_incoming_velocity(id, -180.0 / M_PI * msg.velocity[i], timestamp);
            }else if (id == ID_NECK_TILT){
                JointInterface::store_incoming_position(id, 180.0 / M_PI * msg.position[i], timestamp);
                JointInterface::store_incoming_velocity(id, 180.0 / M_PI * msg.velocity[i], timestamp);
            }
        }
    }

    //dummy data uses current time
    timestamp = Timestamp::now();

    //store dummy positions for joints we do not know about:
    store_dummy_data(ID_LIP_LEFT_UPPER, timestamp);
    store_dummy_data(ID_LIP_LEFT_LOWER, timestamp);
    store_dummy_data(ID_LIP_CENTER_UPPER, timestamp);
    store_dummy_data(ID_LIP_CENTER_LOWER, timestamp);
    store_dummy_data(ID_LIP_RIGHT_UPPER, timestamp);
    store_dummy_data(ID_LIP_RIGHT_LOWER, timestamp);

    store_dummy_data(ID_NECK_ROLL, timestamp);
    store_dummy_data(ID_EYES_BOTH_UD, timestamp);
    store_dummy_data(ID_EYES_LEFT_LR, timestamp);
    store_dummy_data(ID_EYES_RIGHT_LR, timestamp);
    store_dummy_data(ID_EYES_LEFT_LID_LOWER, timestamp);
    store_dummy_data(ID_EYES_LEFT_LID_UPPER, timestamp);
    store_dummy_data(ID_EYES_LEFT_BROW, timestamp);
    store_dummy_data(ID_EYES_RIGHT_LID_LOWER, timestamp);
    store_dummy_data(ID_EYES_RIGHT_LID_UPPER, timestamp);
    store_dummy_data(ID_EYES_RIGHT_BROW, timestamp);

}

void MekaJointInterface::store_dummy_data(int id, Timestamp timestamp){
    JointInterface::store_incoming_position(id, 0.0, timestamp);
    JointInterface::store_incoming_velocity(id, 0.0, timestamp);
}

//! constructor
MekaJointInterface::MekaJointInterface(std::string _input_scope, std::string control_scope, std::string _output_scope) : humotion::server::JointInterface(){
    input_scope = _input_scope;
    output_scope = _output_scope;

    controller_enabled = false;

    //subscribe to meka joint states
    int argc = 0;
    ros::init(argc, (char**)NULL, "meka_humotion");
    ros::NodeHandle n;

    printf("> listening on jointstates on '%s'\n",input_scope.c_str());
    joint_state_subscriber = n.subscribe(input_scope, 150, &MekaJointInterface::incoming_jointstates, this);

    printf("> listening on controlstates on '%s'\n",control_scope.c_str());
    control_state_subscriber = n.subscribe(control_scope, 150, &MekaJointInterface::incoming_controlstate, this);

    printf("> sending targets on '%s'\n", output_scope.c_str());
    target_publisher = n.advertise<trajectory_msgs::JointTrajectory>(output_scope, 100);

    //tell humotion about min/max joint values:
    init_joints();
}

//! destructor
MekaJointInterface::~MekaJointInterface(){
}



void MekaJointInterface::run(){
   ros::spin();
}

//! set the target position of a joint
//! \param enum id of joint
//! \param float value
void MekaJointInterface::publish_target(int e, float position, float velocity){
   //we do this in execute motion for all joints at once...
}


//! actually execute the scheduled motion commands
void MekaJointInterface::execute_motion(){
    if (controller_enabled){
        //build msg
        trajectory_msgs::JointTrajectory msg;
        msg.joint_names.push_back("head_j0");
        msg.joint_names.push_back("head_j1");

        trajectory_msgs::JointTrajectoryPoint p;
        p.positions.push_back(get_target_position(ID_NECK_TILT) * M_PI / 180.0);
        //pan joint is inverted!
        p.positions.push_back(-get_target_position(ID_NECK_PAN) * M_PI / 180.0);
        //printf("targets pan=%4.1f tilt=%4.1f (eye p %4.1f t %4.2f)\n",joint_target[ID_NECK_TILT],joint_target[ID_NECK_PAN],joint_target[ID_EYES_LEFT_LR],joint_target[ID_EYES_BOTH_UD]);

        p.time_from_start = ros::Duration(1.2 * 1.0 / humotion::server::Server::MOTION_UPDATERATE);

        msg.points.push_back(p);

        target_publisher.publish(msg);
    }
}


//! prepare and enable a joint
//! NOTE: this should also prefill the min/max positions for this joint
//! \param the enum id of a joint
void MekaJointInterface::enable_joint(int e){
    //meka does not support this, joints are always enabled
}

//! shutdown and disable a joint
//! \param the enum id of a joint
void MekaJointInterface::disable_joint(int e){
    //meka does not support this, joints are always enabled
}

void MekaJointInterface::store_min_max(int id, float min, float max){
    joint_min[id] = min;
    joint_max[id] = max;
}

void MekaJointInterface::init_joints(){
    store_min_max(ID_NECK_TILT, -37, 1);
    store_min_max(ID_NECK_PAN, -70, 70);

    store_min_max(ID_NECK_ROLL, -1, 1);
    store_min_max(ID_EYES_BOTH_UD, -1, 1);
    store_min_max(ID_EYES_LEFT_LR, -1, 1);
    store_min_max(ID_EYES_RIGHT_LR, -1, 1);
    store_min_max(ID_EYES_LEFT_LID_UPPER, -1, 1);
    store_min_max(ID_EYES_LEFT_LID_LOWER, -1, 1);
    store_min_max(ID_EYES_RIGHT_LID_UPPER, -1, 1);
    store_min_max(ID_EYES_RIGHT_LID_LOWER, -1, 1);
    store_min_max(ID_EYES_LEFT_BROW, -1, 1);
    store_min_max(ID_EYES_RIGHT_BROW, -1, 1);
    store_min_max(ID_LIP_CENTER_UPPER, -1, 1);
    store_min_max(ID_LIP_CENTER_LOWER, -1, 1);
    store_min_max(ID_LIP_LEFT_UPPER, -1, 1);
    store_min_max(ID_LIP_LEFT_LOWER, -1, 1);
    store_min_max(ID_LIP_RIGHT_UPPER, -1, 1);
    store_min_max(ID_LIP_RIGHT_LOWER, -1, 1);
}

