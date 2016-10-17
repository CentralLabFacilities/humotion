#include <stdio.h>
#include <humotion/server/server.h>
#include <string>
#include <iostream>
//#include "meka_data_receiver.h"
#include "mekajointinterface.h"

using namespace std;

int main(int argc, char *argv[]){
    /*Property params;
    params.fromCommand(argc, argv);

    if (!params.check("robot")){
        fprintf(stderr, "Please specify the name of the robot\n");
        fprintf(stderr, "--robot name (e.g. icub or icubSim)\n");
        return -1;
    }

    string robotName=params.find("robot").asString().c_str();
    string scope="/"+robotName;

*/

    if (argc != 5){
        printf("> ERROR: invalid number of parameters passed to server!\n\n");
        printf("usage   : %s <humotion base topic> <jointstates topic> <controlstates topic> <control output topic>\n\n",argv[0]);
        printf("example : %s /meka /joint_states  /meka_roscontrol_state_manager/state /meka_roscontrol/head_position_trajectory_controller/command)\n\n",argv[0]);
        exit(EXIT_FAILURE);
    }

    //create humotion interface
    string humotion_scope      = argv[1];
    string jointstates_scope   = argv[2];
    string controlstates_scope = argv[3];
    string control_scope       = argv[4];


    MekaJointInterface *jointinterface = new MekaJointInterface(jointstates_scope, controlstates_scope, control_scope);
    humotion::server::Server *humotion_server = new humotion::server::Server(humotion_scope, "ROS", jointinterface);

    //finally run it
    jointinterface->run();

    while(humotion_server->ok()){
        usleep(100000);
    }

    printf("> ros connection died, will exit now\n");

    return 0;
}
