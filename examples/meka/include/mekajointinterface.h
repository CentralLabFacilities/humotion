#pragma once
#include <humotion/server/joint_interface.h>
#include <humotion/server/server.h>
#include <boost/bimap.hpp>

#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "trajectory_msgs/JointTrajectory.h"

#include <m3meka_msgs/M3ControlStates.h>

class MekaJointInterface : public humotion::server::JointInterface{
public:
    MekaJointInterface(std::string _input_scope, std::string control_scope, std::string _output_scope);
    ~MekaJointInterface();

    //void fetch_position(Device *dev, double timestamp);
    //void fetch_speed(Device *dev, double timestamp);
    //void fetch_position(int id, double value, double timestamp);
    //void fetch_speed(int id, double value, double timestamp);
    void run();

    static const int MAIN_LOOP_FREQUENCY = 50;

protected:
    void disable_joint(int e);
    void publish_target(int e, float position, float velocity);
    void enable_joint(int e);
    void execute_motion();

private:
    void incoming_controlstate(const m3meka_msgs::M3ControlStates &control_state);
    void incoming_jointstates(const sensor_msgs::JointState & msg);
    void store_dummy_data(int id, humotion::Timestamp timestamp);
    void store_min_max(int id, float min, float max);
    ros::Subscriber joint_state_subscriber;
    ros::Subscriber control_state_subscriber;
    ros::Publisher target_publisher;

    void set_eyelid_angle(double angle);
    void set_eyebrow_angle(int id);
    void set_mouth();

    //iCubDataReceiver *icub_data_receiver;
    void init_joints();
    double lid_angle;
    int lid_opening_previous;
    int previous_mouth_state;

    std::string input_scope;
    std::string control_scope;
    std::string output_scope;

    float last_pos_eye_vergence;
    float last_pos_eye_pan;
    float last_vel_eye_vergence;
    float last_vel_eye_pan;

    bool controller_enabled;

    void store_joint(int id, float value);
    void set_target_in_positionmode(int id, double value);
    void set_target_in_velocitymode(int id, double value);


    int convert_enum_to_motorid(int e);
    int convert_motorid_to_enum(int id);


    typedef boost::bimap<int, int > enum_id_bimap_t;
    typedef enum_id_bimap_t::value_type enum_id_bimap_entry_t;
    enum_id_bimap_t enum_id_bimap;
};
