#include "ros/ros.h"
#include <iostream>
#include "common_srv/ROSUnit_Factory.hpp"
#include "std_logger.hpp"
#include "RestrictedNormWaypointRefGenerator.hpp"
#include "ROSUnit_RestNormSettings.hpp"
#include "ROSUnit_Optitrack.hpp"
#include "Global2Inertial.hpp"

int main(int argc, char **argv){

    ros::init(argc, argv, "global2inertial_node");

    ros::NodeHandle nh;
    ros::Rate rate(120);
    ROSUnit_Factory ROSUnit_Factory_main{nh};

    ROSUnit* myROSOptitrack = new ROSUnit_Optitrack(nh);
    ROSUnit* myCameraPosition =  ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Subscriber, 
                                                                    ROSUnit_msg_type::ROSUnit_Float,
                                                                    "camera_provider");
    ROSUnit* rosunit_camera_enable = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Server,
                                                                            ROSUnit_msg_type::ROSUnit_Int,
                                                                            "set_camera_status");
    ROSUnit* rosunit_g2i_position = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Publisher, 
                                                                    ROSUnit_msg_type::ROSUnit_Point,
                                                                    "global2inertial/position");
    ROSUnit* rosunit_g2i_orientation = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Publisher, 
                                                                    ROSUnit_msg_type::ROSUnit_Point,
                                                                    "global2inertial/orientation");                                                                
    ROSUnit* rosunit_set_height_offset = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Server,
                                                                            ROSUnit_msg_type::ROSUnit_Float,
                                                                            "set_height_offset");
    
    
    Global2Inertial* myGlobal2Inertial = new Global2Inertial();

    myCameraPosition->setEmittingChannel((int)Global2Inertial::receiving_channels::ch_Camera);
    rosunit_camera_enable->setEmittingChannel((int)Global2Inertial::receiving_channels::ch_Camera);
    myCameraPosition->addCallbackMsgReceiver((MsgReceiver*)myGlobal2Inertial);
    rosunit_camera_enable->addCallbackMsgReceiver((MsgReceiver*)myGlobal2Inertial);
    myROSOptitrack->addCallbackMsgReceiver((MsgReceiver*)myGlobal2Inertial);
    myGlobal2Inertial->addCallbackMsgReceiver((MsgReceiver*)rosunit_g2i_position, (int)Global2Inertial::unicast_addresses::uni_Optitrack_pos);
    myGlobal2Inertial->addCallbackMsgReceiver((MsgReceiver*)rosunit_g2i_orientation, (int)Global2Inertial::unicast_addresses::uni_Optitrack_heading);
    rosunit_set_height_offset->addCallbackMsgReceiver((MsgReceiver*)myGlobal2Inertial);

    std::cout  << "###### GLOBAL2INERTIAL NODE ######" "\n";

    Timer tempo;
    int i = 0;
    while(ros::ok()){
        tempo.tick();

        ros::spinOnce();

        int gone = tempo.tockMicroSeconds();
        if(gone > 8333) {
            std::cout  << i <<  " G2I: " << gone << "\n";
        }
        i++;
        rate.sleep();

    }

    return 0;
}