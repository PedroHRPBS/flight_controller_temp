#include "ros/ros.h"
#include <iostream>
#include "common_srv/ROSUnit_Factory.hpp"
#include "std_logger.hpp"
#include "RestrictedNormWaypointRefGenerator.hpp"
#include "ROSUnit_RestNormSettings.hpp"

// int main(int argc, char **argv){

//     ros::init(argc, argv, "waypoint_reference_node");
   
//     ros::NodeHandle nh;
//     ros::Rate rate(120);
//     ROSUnit_Factory ROSUnit_Factory_main{nh};

//     ROSUnit* rosunit_restricted_norm_settings = new ROSUnit_RestNormSettings(nh);
//     ROSUnit* rosunit_uav_control_set_path = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Server,
//                                                                                 ROSUnit_msg_type::ROSUnit_Poses,
//                                                                                 "uav_control/set_path");
//     ROSUnit* rosunit_g2i_position = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Subscriber, 
//                                                                     ROSUnit_msg_type::ROSUnit_Point,
//                                                                     "global2inertial/position");
//     ROSUnit* rosunit_waypoint_counter = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Publisher, 
//                                                                     ROSUnit_msg_type::ROSUnit_Float,
//                                                                     "waypoint_reference/counter");
//     ROSUnit* rosunit_waypoint_x = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Publisher, 
//                                                                     ROSUnit_msg_type::ROSUnit_Float,
//                                                                     "waypoint_reference/x");
//     ROSUnit* rosunit_waypoint_y = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Publisher, 
//                                                                     ROSUnit_msg_type::ROSUnit_Float,
//                                                                     "waypoint_reference/y");
//     ROSUnit* rosunit_waypoint_z = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Publisher, 
//                                                                     ROSUnit_msg_type::ROSUnit_Float,
//                                                                     "waypoint_reference/z");
//     ROSUnit* rosunit_waypoint_yaw = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Publisher, 
//                                                                     ROSUnit_msg_type::ROSUnit_Float,
//                                                                     "waypoint_reference/yaw");

//     RestrictedNormWaypointRefGenerator* waypoint_generator = new RestrictedNormWaypointRefGenerator();

//     rosunit_uav_control_set_path->addCallbackMsgReceiver((MsgReceiver*)waypoint_generator->getPorts()[(int)RestrictedNormWaypointRefGenerator::ports_id::IP_0_WAYPOINT]);
//     rosunit_restricted_norm_settings->getPorts()[(int)ROSUnit_RestNormSettings::ports_id::OP_0_DATA]->addCallbackMsgReceiver((MsgReceiver*)waypoint_generator->getPorts()[(int)RestrictedNormWaypointRefGenerator::ports_id::IP_1_SETTINGS]);
//     rosunit_g2i_position->addCallbackMsgReceiver((MsgReceiver*)waypoint_generator->getPorts()[(int)RestrictedNormWaypointRefGenerator::ports_id::IP_2_DATA]); 

//     waypoint_generator->getPorts()[(int)RestrictedNormWaypointRefGenerator::ports_id::OP_4_COUNTER]->addCallbackMsgReceiver((MsgReceiver*)rosunit_waypoint_counter);

//     //******************SETTING TRAJECTORY GENERATION TOOL******************

//     waypoint_generator->getPorts()[(int)RestrictedNormWaypointRefGenerator::ports_id::OP_0_X]->addCallbackMsgReceiver((MsgReceiver*)rosunit_waypoint_x);
//     waypoint_generator->getPorts()[(int)RestrictedNormWaypointRefGenerator::ports_id::OP_1_Y]->addCallbackMsgReceiver((MsgReceiver*)rosunit_waypoint_y);
//     waypoint_generator->getPorts()[(int)RestrictedNormWaypointRefGenerator::ports_id::OP_2_Z]->addCallbackMsgReceiver((MsgReceiver*)rosunit_waypoint_z);
//     waypoint_generator->getPorts()[(int)RestrictedNormWaypointRefGenerator::ports_id::OP_3_YAW]->addCallbackMsgReceiver((MsgReceiver*)rosunit_waypoint_yaw);

//     std::cout  << "###### WAYPOINT REFERENCE NODE ######" "\n";

//     Timer tempo;
//     int i = 0;
//     while(ros::ok()){
//         tempo.tick();

//         ros::spinOnce();

//         int gone = tempo.tockMicroSeconds();
//         if(gone > 8333) {
//             std::cout  << i << " WP: " << gone << "\n";
//         }
//         i++;
//         rate.sleep();

//     }

    return 0;
}