//PROVIDERS_NODE V1.0.2
// 18 June 2020
// Pedro Henrique Silva
// YawRate from Xsens
#include "ros/ros.h"
#include <iostream>
#include "common_srv/ROSUnit_Factory.hpp"
#include "std_logger.hpp"
#include "Global2Inertial.hpp"
#include "PVConcatenator.hpp"
#include "Differentiator.hpp"
#include "WrapAroundFunction.hpp"
#include "ROSUnit_Optitrack.hpp"
#include "ROSUnit_Xsens.hpp"
#include "common_srv/Timer.hpp"

const int OPTITRACK_FREQUENCY = 120;

int main(int argc, char **argv){

    ros::init(argc, argv, "providers_node");

    ros::NodeHandle nh;
    ros::Rate rate(200);
    
    ROSUnit_Factory ROSUnit_Factory_main{nh};

    ROSUnit* rosunit_x_provider_pub = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Publisher, 
                                                                    ROSUnit_msg_type::ROSUnit_Point,
                                                                    "/providers/x");
    ROSUnit* rosunit_y_provider_pub = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Publisher, 
                                                                    ROSUnit_msg_type::ROSUnit_Point,
                                                                    "/providers/y");
    ROSUnit* rosunit_z_provider_pub = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Publisher, 
                                                                    ROSUnit_msg_type::ROSUnit_Point,
                                                                    "/providers/z");
    ROSUnit* rosunit_roll_provider_pub = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Publisher, 
                                                                    ROSUnit_msg_type::ROSUnit_Point,
                                                                    "/providers/roll");
    ROSUnit* rosunit_pitch_provider_pub = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Publisher, 
                                                                    ROSUnit_msg_type::ROSUnit_Point,
                                                                    "/providers/pitch");
    ROSUnit* rosunit_yaw_provider_pub = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Publisher, 
                                                                    ROSUnit_msg_type::ROSUnit_Point,
                                                                    "/providers/yaw");
    ROSUnit* rosunit_yaw_rate_provider_pub = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Publisher, 
                                                                    ROSUnit_msg_type::ROSUnit_Point,
                                                                    "/providers/yaw_rate");
    //TODO PROVIDER CAMERA PUBLISHER
    //TODO RECEIVED CAMERA DATA SUBSCRIBER

    // ROSUnit* rosunit_g2i_x = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Subscriber, 
    //                                                                 ROSUnit_msg_type::ROSUnit_Point,
    //                                                                 "global2inertial/x");
    // ROSUnit* rosunit_g2i_y = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Subscriber, 
    //                                                                 ROSUnit_msg_type::ROSUnit_Point,
    //                                                                 "global2inertial/y");
    // ROSUnit* rosunit_g2i_z = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Subscriber, 
    //                                                                 ROSUnit_msg_type::ROSUnit_Point,
    //                                                                 "global2inertial/z");                                                                     
    ROSUnit* rosunit_g2i_position = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Subscriber, 
                                                                    ROSUnit_msg_type::ROSUnit_Point,
                                                                    "global2inertial/position");
    ROSUnit* rosunit_g2i_orientation = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Subscriber, 
                                                                    ROSUnit_msg_type::ROSUnit_Point,
                                                                    "global2inertial/orientation");
   
    //***********************ADDING SENSORS********************************
    ROSUnit* myROSUnit_Xsens = new ROSUnit_Xsens(nh);

    //***********************SETTING PROVIDERS**********************************
    PVConcatenator* CsX_PVConcatenator = new PVConcatenator(PVConcatenator::concatenation_axes::conc_x_axis, act_on::pv);
    PVConcatenator* CsY_PVConcatenator = new PVConcatenator(PVConcatenator::concatenation_axes::conc_y_axis, act_on::pv);
    PVConcatenator* CsZ_PVConcatenator = new PVConcatenator(PVConcatenator::concatenation_axes::conc_z_axis, act_on::pv);
    PVConcatenator* CsRoll_PVConcatenator = new PVConcatenator(PVConcatenator::concatenation_axes::conc_x_axis, act_on::pv);
    PVConcatenator* CsPitch_PVConcatenator = new PVConcatenator(PVConcatenator::concatenation_axes::conc_y_axis, act_on::pv);
    PVConcatenator* CsYaw_PVConcatenator = new PVConcatenator(PVConcatenator::concatenation_axes::conc_z_axis, act_on::pv);
    PVConcatenator* CsYawRate_PVConcatenator = new PVConcatenator(PVConcatenator::concatenation_axes::conc_z_axis, act_on::pv);

    WrapAroundFunction* wrap_around_yaw = new WrapAroundFunction();
    wrap_around_yaw->assignParametersRange(-M_PI, M_PI);
    
    Differentiator* velocityFromPosition = new Differentiator(1./OPTITRACK_FREQUENCY);
    velocityFromPosition->setEmittingChannel((int)PVConcatenator::receiving_channels::ch_pv_dot);
    
    // Differentiator* yawRateFromYaw = new Differentiator(1./OPTITRACK_FREQUENCY);
    // yawRateFromYaw->setEmittingChannel((int)PVConcatenator::receiving_channels::ch_pv);
    
    rosunit_g2i_position->setEmittingChannel((int)PVConcatenator::receiving_channels::ch_pv);
    // rosunit_g2i_x->setEmittingChannel((int)PVConcatenator::receiving_channels::ch_pv);
    // rosunit_g2i_y->setEmittingChannel((int)PVConcatenator::receiving_channels::ch_pv);
    // rosunit_g2i_z->setEmittingChannel((int)PVConcatenator::receiving_channels::ch_pv);

    rosunit_g2i_orientation->setEmittingChannel((int)PVConcatenator::receiving_channels::ch_pv);

    rosunit_g2i_position->addCallbackMsgReceiver((MsgReceiver*)velocityFromPosition);
    // rosunit_g2i_orientation->addCallbackMsgReceiver((MsgReceiver*)yawRateFromYaw);
    velocityFromPosition->addCallbackMsgReceiver((MsgReceiver*)CsX_PVConcatenator);
    velocityFromPosition->addCallbackMsgReceiver((MsgReceiver*)CsY_PVConcatenator);
    velocityFromPosition->addCallbackMsgReceiver((MsgReceiver*)CsZ_PVConcatenator);
    // yawRateFromYaw->addCallbackMsgReceiver((MsgReceiver*)CsYawRate_PVConcatenator);
    rosunit_g2i_position->addCallbackMsgReceiver((MsgReceiver*)CsX_PVConcatenator);
    rosunit_g2i_position->addCallbackMsgReceiver((MsgReceiver*)CsY_PVConcatenator);
    rosunit_g2i_position->addCallbackMsgReceiver((MsgReceiver*)CsZ_PVConcatenator);
    rosunit_g2i_orientation->addCallbackMsgReceiver((MsgReceiver*)wrap_around_yaw);

    myROSUnit_Xsens->getPorts()[(int)ROSUnit_Xsens::ports_id::OP_4_YAW_RATE]->addCallbackMsgReceiver((MsgReceiver*)CsYawRate_PVConcatenator);
    wrap_around_yaw->addCallbackMsgReceiver((MsgReceiver*)CsYaw_PVConcatenator);
    
    myROSUnit_Xsens->getPorts()[(int)ROSUnit_Xsens::ports_id::OP_2_ROLL_RATE]->addCallbackMsgReceiver((MsgReceiver*)CsRoll_PVConcatenator);
    myROSUnit_Xsens->getPorts()[(int)ROSUnit_Xsens::ports_id::OP_3_PITCH_RATE]->addCallbackMsgReceiver((MsgReceiver*)CsPitch_PVConcatenator);
    myROSUnit_Xsens->getPorts()[(int)ROSUnit_Xsens::ports_id::OP_0_ROLL]->addCallbackMsgReceiver((MsgReceiver*)CsRoll_PVConcatenator);
    myROSUnit_Xsens->getPorts()[(int)ROSUnit_Xsens::ports_id::OP_1_PITCH]->addCallbackMsgReceiver((MsgReceiver*)CsPitch_PVConcatenator);

//     //TODO after adding G2I
//     //
    
//     //******************PROVIDERS TO CONTROL SYSTEMS******************************

//     CsX_PVConcatenator->addCallbackMsgReceiver((MsgReceiver*)rosunit_x_provider_pub);
//     CsY_PVConcatenator->addCallbackMsgReceiver((MsgReceiver*)rosunit_y_provider_pub);
//     CsZ_PVConcatenator->addCallbackMsgReceiver((MsgReceiver*)rosunit_z_provider_pub);
//     CsPitch_PVConcatenator->addCallbackMsgReceiver((MsgReceiver*)rosunit_pitch_provider_pub);
//     CsRoll_PVConcatenator->addCallbackMsgReceiver((MsgReceiver*)rosunit_roll_provider_pub);
//     CsYaw_PVConcatenator->addCallbackMsgReceiver((MsgReceiver*)rosunit_yaw_provider_pub);
//     CsYawRate_PVConcatenator->addCallbackMsgReceiver((MsgReceiver*)rosunit_yaw_rate_provider_pub);

//     std::cout  << "###### PROVIDERS NODE ######" "\n";
    
//     Timer tempo;
//     int i = 0;
//     while(ros::ok()){
//         tempo.tick();

//         ros::spinOnce();
       
//         int gone = tempo.tockMicroSeconds();
//         if(gone > 5000) {
//              std::cout  << i << " PROV: " << gone << "\n";
//         }
//         i++;

//         rate.sleep();

//     }

    return 0;
}