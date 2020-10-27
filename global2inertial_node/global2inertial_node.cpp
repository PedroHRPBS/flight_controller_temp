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

    myROSOptitrack->getPorts()[(int)ROSUnit_Optitrack::ports_id::OP_0_OPT]->connect(myGlobal2Inertial->getPorts()[(int)Global2Inertial::ports_id::IP_0_OPTI_MSG]);
    myGlobal2Inertial->getPorts()[(int)Global2Inertial::ports_id::OP_0_OPTIPOS]->connect(rosunit_g2i_position);
    myGlobal2Inertial->getPorts()[(int)Global2Inertial::ports_id::OP_1_OPTIHEADING]->connect(rosunit_g2i_orientation);
    rosunit_set_height_offset->connect(myGlobal2Inertial->getPorts()[(int)Global2Inertial::ports_id::IP_1_FLOAT_DATA]);

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