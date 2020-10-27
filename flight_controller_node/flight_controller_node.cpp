//Flight Controller V2.0.1
//16 June 2020
//Pedro Henrique Silva
#include <iostream>
#include <vector>
#include "ROSUnit_Optitrack.hpp"
#include "std_logger.hpp"
#include "HexaActuationSystem.hpp"
#include "QuadActuationSystem.hpp"
#include "ESCMotor.hpp"
#include "ROSUnit_Arm.hpp"
#include "ROSUnit_UpdateController.hpp"
#include "ROSUnit_ResetController.hpp"
#include "ROSUnit_BroadcastData.hpp"
#include "ROSUnit_SwitchBlock.hpp"
#include "MRFTController.hpp"
#include "ROSUnit_Xsens.hpp"
#include "Transform_InertialToBody.hpp"
#include "RestrictedNormWaypointRefGenerator.hpp"
#include "ROSUnit_RestNormSettings.hpp"
#include "Saturation.hpp"
#include "CircularProcessVariableReference.hpp"
#include "Global2Inertial.hpp"
#include "common_srv/ROSUnit_Factory.hpp"
#include "BatteryMonitor.hpp"
#include "ROSUnit_RTK.hpp"
#include "Differentiator.hpp"
#include "PVConcatenator.hpp"
#include "WrapAroundFunction.hpp"
#include <pthread.h>
#include <sched.h>
#include "BoundingBoxController.hpp"
#include "PIDplusMRFTController.hpp"
#include "Switch.hpp"
#include "Sum.hpp"
#include "Mux3D.hpp"
#include "Demux3D.hpp"
#include "InvertedSwitch.hpp"

#define XSENS_OVER_ROS
#define OPTITRACK
#undef BATTERY_MONITOR


const int PWM_FREQUENCY = 200;
const float SATURATION_VALUE_XY = 0.2617; //TODO trajectory following 0.5 before
const float SATURATION_VALUE_YAW = 0.2617;
const float SATURATION_VALUE_YAWRATE = 0.3;

void set_realtime_priority();

int main(int argc, char** argv) {
    // //TODO remove SwitchOut Message
    // std::cout << "Hello Flight Controller!" << std::endl;

    // //*****************************LOGGER********************************** 
    // Logger::assignLogger(new StdLogger());
    
    // //****************************ROS UNITS*******************************

    // ros::init(argc, argv, "flight_controller_node");

    // ros::NodeHandle nh;
    // ros::Rate rate(200);
    // ROSUnit_Factory ROSUnit_Factory_main{nh};

    
    // ROSUnit* myROSArm = new ROSUnit_Arm(nh);
    // ROSUnit* myROSUpdateController = new ROSUnit_UpdateController(nh);
    // ROSUnit* myROSResetController = new ROSUnit_ResetController(nh);
    // ROSUnit* myROSBroadcastData = new ROSUnit_BroadcastData(nh);
    // ROSUnit* myROSSwitchBlock = new ROSUnit_SwitchBlock(nh);
    
    // ROSUnit* rosunit_x_provider = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Subscriber, 
    //                                                                 ROSUnit_msg_type::ROSUnit_Point,
    //                                                                 "/providers/x");
    // ROSUnit* rosunit_y_provider = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Subscriber, 
    //                                                                 ROSUnit_msg_type::ROSUnit_Point,
    //                                                                 "/providers/y");
    // ROSUnit* rosunit_z_provider = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Subscriber, 
    //                                                                 ROSUnit_msg_type::ROSUnit_Point,
    //                                                                 "/providers/z");
    // ROSUnit* rosunit_roll_provider = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Subscriber, 
    //                                                                 ROSUnit_msg_type::ROSUnit_Point,
    //                                                                 "/providers/roll");
    // ROSUnit* rosunit_pitch_provider = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Subscriber, 
    //                                                                 ROSUnit_msg_type::ROSUnit_Point,
    //                                                                 "/providers/pitch");
    // ROSUnit* rosunit_yaw_provider = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Subscriber, 
    //                                                                 ROSUnit_msg_type::ROSUnit_Point,
    //                                                                 "/providers/yaw");
    // ROSUnit* rosunit_yaw_rate_provider = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Subscriber, 
    //                                                                 ROSUnit_msg_type::ROSUnit_Point,
    //                                                                 "/providers/yaw_rate");
    // ROSUnit* rosunit_waypoint_x = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Subscriber, 
    //                                                                 ROSUnit_msg_type::ROSUnit_Float,
    //                                                                 "waypoint_reference/x");
    // ROSUnit* rosunit_waypoint_y = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Subscriber, 
    //                                                                 ROSUnit_msg_type::ROSUnit_Float,
    //                                                                 "waypoint_reference/y");
    // ROSUnit* rosunit_waypoint_z = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Subscriber, 
    //                                                                 ROSUnit_msg_type::ROSUnit_Float,
    //                                                                 "waypoint_reference/z");
    // ROSUnit* rosunit_waypoint_yaw = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Subscriber, 
    //                                                                 ROSUnit_msg_type::ROSUnit_Float,
    //                                                                 "waypoint_reference/yaw");                                                            

    // //**************************SETTING BLOCKS**********************************

    // Block* PID_x = new PIDController(block_id::PID_X);
    // Block* PID_pitch = new PIDController(block_id::PID_PITCH);
    // Block* PV_Ref_x = new ProcessVariableReference(block_id::REF_X);
    // Block* PV_Ref_pitch = new ProcessVariableReference(block_id::REF_PITCH);
    // Block* PID_y = new PIDController(block_id::PID_Y);
    // Block* PID_roll = new PIDController(block_id::PID_ROLL);
    // Block* PV_Ref_y = new ProcessVariableReference(block_id::REF_Y);
    // Block* PV_Ref_roll = new ProcessVariableReference(block_id::REF_ROLL);
    // Block* PID_z = new PIDController(block_id::PID_Z);
    // Block* PID_z_identification = new PIDController(block_id::PID_Z_ID);
    // Block* PID_yaw = new PIDController(block_id::PID_YAW);
    // Block* PID_yaw_rate = new PIDController(block_id::PID_YAW_RATE);
    // Block* PV_Ref_z = new ProcessVariableReference(block_id::REF_Z);
    // Block* PV_Ref_yaw = new CircularProcessVariableReference(block_id::REF_YAW);
    // Block* PV_Ref_yaw_rate = new ProcessVariableReference(block_id::REF_YAW_RATE);

    // Block* MRFT_x = new MRFTController(block_id::MRFT_X);
    // Block* MRFT_y = new MRFTController(block_id::MRFT_Y);
    // Block* MRFT_z = new MRFTController(block_id::MRFT_Z);
    // Block* MRFT_roll = new MRFTController(block_id::MRFT_ROLL);
    // Block* MRFT_pitch = new MRFTController(block_id::MRFT_PITCH);
    // Block* MRFT_yaw = new MRFTController(block_id::MRFT_YAW);
    // Block* MRFT_yaw_rate = new MRFTController(block_id::MRFT_YAW_RATE);

    // Block* SM_x = new SlidingModeController(block_id::SM_X);
    // Block* SM_y = new SlidingModeController(block_id::SM_Y);

    // Block* PIDplusMRFT_z = new PIDplusMRFTController(block_id::PID_MRFT_Z, (PIDController*)PID_z, (MRFTController*)MRFT_z);

    // Transform_InertialToBody* transform_X_InertialToBody = new Transform_InertialToBody(control_system::x);
    // Transform_InertialToBody* transform_Y_InertialToBody = new Transform_InertialToBody(control_system::y);

    // Saturation* X_Saturation = new Saturation(SATURATION_VALUE_XY);
    // Saturation* Y_Saturation = new Saturation(SATURATION_VALUE_XY);
    // Saturation* Yaw_Saturation = new Saturation(SATURATION_VALUE_YAW);
    // Saturation* YawRate_Saturation = new Saturation(SATURATION_VALUE_YAWRATE);


    // //***********************SETTING CONTROL SYSTEMS***************************

    // ControlSystem* X_ControlSystem = new ControlSystem(control_system::x, block_frequency::hz120);
    // X_ControlSystem->addBlock(PID_x);
    // X_ControlSystem->addBlock(MRFT_x);
    // X_ControlSystem->addBlock(SM_x);
    // X_ControlSystem->addBlock(PV_Ref_x);

    // ControlSystem* Pitch_ControlSystem = new ControlSystem(control_system::pitch, block_frequency::hz200);
    // Pitch_ControlSystem->addBlock(PID_pitch);
    // Pitch_ControlSystem->addBlock(MRFT_pitch);
    // Pitch_ControlSystem->addBlock(PV_Ref_pitch);
    
    // ControlSystem* Y_ControlSystem = new ControlSystem(control_system::y, block_frequency::hz120);
    // Y_ControlSystem->addBlock(PID_y);
    // Y_ControlSystem->addBlock(MRFT_y);
    // Y_ControlSystem->addBlock(SM_y);
    // Y_ControlSystem->addBlock(PV_Ref_y);

    // ControlSystem* Roll_ControlSystem = new ControlSystem(control_system::roll, block_frequency::hz200);
    // Roll_ControlSystem->addBlock(PID_roll);
    // Roll_ControlSystem->addBlock(MRFT_roll);
    // Roll_ControlSystem->addBlock(PV_Ref_roll);
    
    // // ControlSystem* Z_ControlSystem = new ControlSystem(control_system::z, block_frequency::hz120);
    // // Z_ControlSystem->addBlock(PID_z);
    // // Z_ControlSystem->addBlock(MRFT_z);
    // // Z_ControlSystem->addBlock(PIDplusMRFT_z);
    // // Z_ControlSystem->addBlock(PV_Ref_z);

    // ControlSystem* Yaw_ControlSystem = new ControlSystem(control_system::yaw, block_frequency::hz120);
    // Yaw_ControlSystem->addBlock(PID_yaw);
    // Yaw_ControlSystem->addBlock(MRFT_yaw);
    // Yaw_ControlSystem->addBlock(PV_Ref_yaw);

    // ControlSystem* YawRate_ControlSystem = new ControlSystem(control_system::yaw_rate, block_frequency::hz200);
    // YawRate_ControlSystem->addBlock(PID_yaw_rate);
    // YawRate_ControlSystem->addBlock(MRFT_yaw_rate);
    // YawRate_ControlSystem->addBlock(PV_Ref_yaw_rate);

    // //*********************SETTING ACTUATION SYSTEMS************************
    
    // Actuator* M1 = new ESCMotor(0, PWM_FREQUENCY);
    // Actuator* M2 = new ESCMotor(1, PWM_FREQUENCY);
    // Actuator* M3 = new ESCMotor(2, PWM_FREQUENCY);
    // Actuator* M4 = new ESCMotor(3, PWM_FREQUENCY);
    // Actuator* M5 = new ESCMotor(4, PWM_FREQUENCY);
    // Actuator* M6 = new ESCMotor(5, PWM_FREQUENCY);

    // std::vector<Actuator*> actuators{M1, M2, M3, M4, M5, M6};

    // ActuationSystem* myActuationSystem = new HexaActuationSystem(actuators);
    // // ActuationSystem* myActuationSystem = new QuadActuationSystem(actuators);


    // //***********************************SETTING CONNECTIONS***********************************
    // //========                                                                             =============
    // //|      |-------------->X_Control_System-->RM_X-->Saturation-->Roll_Control_System--->|           |
    // //| USER |-------------->Y_Control_System-->RM_Y-->Saturation-->Pitch_Control_System-->| Actuation |
    // //|      |-------------->Z_Control_System--------------------------------------------->|  System   |
    // //|      |-------------->Yaw_Control_System-->Saturation--->YawRate_Control_System---->|           |
    // //========                                                                             =============
    
    // rosunit_waypoint_x->setEmittingChannel((int)ControlSystem::receiving_channels::ch_reference);
    // rosunit_waypoint_y->setEmittingChannel((int)ControlSystem::receiving_channels::ch_reference);
    // // rosunit_waypoint_z->setEmittingChannel((int)ControlSystem::receiving_channels::ch_reference);
    // rosunit_waypoint_yaw->setEmittingChannel((int)ControlSystem::receiving_channels::ch_reference);
    // Roll_ControlSystem->setEmittingChannel((int)HexaActuationSystem::receiving_channels::ch_roll);
    // Pitch_ControlSystem->setEmittingChannel((int)HexaActuationSystem::receiving_channels::ch_pitch);
    // // Z_ControlSystem->setEmittingChannel((int)HexaActuationSystem::receiving_channels::ch_throttle);
    // YawRate_ControlSystem->setEmittingChannel((int)HexaActuationSystem::receiving_channels::ch_yaw);

    // rosunit_waypoint_x->addCallbackMsgReceiver((MsgReceiver*)X_ControlSystem);
    // X_ControlSystem->addCallbackMsgReceiver((MsgReceiver*)transform_X_InertialToBody, (int)ControlSystem::unicast_addresses::unicast_control_system);
    // transform_X_InertialToBody->addCallbackMsgReceiver((MsgReceiver*)X_Saturation);
    // X_Saturation->addCallbackMsgReceiver((MsgReceiver*)Roll_ControlSystem);
    // Roll_ControlSystem->addCallbackMsgReceiver((MsgReceiver*)myActuationSystem, (int)ControlSystem::unicast_addresses::unicast_actuation_system);
    
    // rosunit_waypoint_y->addCallbackMsgReceiver((MsgReceiver*)Y_ControlSystem);
    // Y_ControlSystem->addCallbackMsgReceiver((MsgReceiver*)transform_Y_InertialToBody, (int)ControlSystem::unicast_addresses::unicast_control_system);
    // transform_Y_InertialToBody->addCallbackMsgReceiver((MsgReceiver*)Y_Saturation);
    // Y_Saturation->addCallbackMsgReceiver((MsgReceiver*)Pitch_ControlSystem);
    // Pitch_ControlSystem->addCallbackMsgReceiver((MsgReceiver*)myActuationSystem, (int)ControlSystem::unicast_addresses::unicast_actuation_system);
    
    // // rosunit_waypoint_z->addCallbackMsgReceiver((MsgReceiver*)Z_ControlSystem);
    // // Z_ControlSystem->addCallbackMsgReceiver((MsgReceiver*)myActuationSystem, (int)ControlSystem::unicast_addresses::unicast_actuation_system);
    
    // //*******************************************************************************************************************
    // // REFACTORING //

    // InvertedSwitch* ID_switch_z = new InvertedSwitch(std::equal_to<float>(), 2.0);
    // Switch* PID_MRFT_switch_z = new Switch(std::greater_equal<float>(), 0.119842615399054);
    // Sum* sum_PID_MRFT_z = new Sum(std::plus<float>());
    // Sum* sum_ref_z = new Sum(std::minus<float>());
    // Sum* sum_ref_dot_z = new Sum(std::minus<float>());
    // Sum* sum_ref_dot_dot_z = new Sum(std::minus<float>());
    // Demux3D* prov_demux_z = new Demux3D();
    // Mux3D* error_mux_z = new Mux3D();

    // myROSSwitchBlock->addCallbackMsgReceiver((MsgReceiver*)ID_switch_z->getPorts()[InvertedSwitch::ports_id::IP_1_TRIGGER]);
    // rosunit_waypoint_z->addCallbackMsgReceiver((MsgReceiver*)sum_ref_z->getPorts()[(int)Sum::ports_id::IP_0_DATA]);
    // rosunit_z_provider->addCallbackMsgReceiver((MsgReceiver*)prov_demux_z->getPorts()[(int)Demux3D::ports_id::IP_0_DATA]);

    // prov_demux_z->getPorts()[(int)Demux3D::ports_id::OP_0_DATA]->addCallbackMsgReceiver((MsgReceiver*)sum_ref_z->getPorts()[(int)Sum::ports_id::IP_1_DATA]);
    // prov_demux_z->getPorts()[(int)Demux3D::ports_id::OP_1_DATA]->addCallbackMsgReceiver((MsgReceiver*)sum_ref_dot_z->getPorts()[(int)Sum::ports_id::IP_1_DATA]);
    // prov_demux_z->getPorts()[(int)Demux3D::ports_id::OP_2_DATA]->addCallbackMsgReceiver((MsgReceiver*)sum_ref_dot_dot_z->getPorts()[(int)Sum::ports_id::IP_1_DATA]);
    // sum_ref_z->getPorts()[(int)Sum::ports_id::OP_0_DATA]->addCallbackMsgReceiver((MsgReceiver*)error_mux_z->getPorts()[(int)Mux3D::ports_id::IP_0_DATA]);
    // sum_ref_dot_z->getPorts()[(int)Sum::ports_id::OP_0_DATA]->addCallbackMsgReceiver((MsgReceiver*)error_mux_z->getPorts()[(int)Mux3D::ports_id::IP_1_DATA]);
    // sum_ref_dot_dot_z->getPorts()[(int)Sum::ports_id::OP_0_DATA]->addCallbackMsgReceiver((MsgReceiver*)error_mux_z->getPorts()[(int)Mux3D::ports_id::IP_2_DATA]);
    // error_mux_z->getPorts()[(int)Mux3D::ports_id::OP_0_DATA]->addCallbackMsgReceiver((MsgReceiver*)((PIDController*)PID_z)->getPorts()[(int)PIDController::ports_id::IP_0_DATA]);
    // error_mux_z->getPorts()[(int)Mux3D::ports_id::OP_0_DATA]->addCallbackMsgReceiver((MsgReceiver*)((PIDController*)PID_z_identification)->getPorts()[(int)PIDController::ports_id::IP_0_DATA]);
    // error_mux_z->getPorts()[(int)Mux3D::ports_id::OP_0_DATA]->addCallbackMsgReceiver((MsgReceiver*)((MRFTController*)MRFT_z)->getPorts()[(int)MRFTController::ports_id::IP_0_DATA]);
    
    // ((PIDController*)PID_z)->getPorts()[(int)PIDController::ports_id::OP_0_DATA]->addCallbackMsgReceiver((MsgReceiver*)ID_switch_z->getPorts()[(int)InvertedSwitch::ports_id::IP_2_DATA]);
    // ((PIDController*)PID_z_identification)->getPorts()[(int)PIDController::ports_id::OP_0_DATA]->addCallbackMsgReceiver((MsgReceiver*)PID_MRFT_switch_z->getPorts()[(int)Switch::ports_id::IP_0_DATA]);
    // prov_demux_z->getPorts()[(int)Demux3D::ports_id::OP_0_DATA]->addCallbackMsgReceiver((MsgReceiver*)PID_MRFT_switch_z->getPorts()[(int)Switch::ports_id::IP_1_TRIGGER]);
    // PID_MRFT_switch_z->getPorts()[(int)Switch::ports_id::OP_0_DATA_DEFAULT]->addCallbackMsgReceiver((MsgReceiver*)sum_PID_MRFT_z->getPorts()[(int)Sum::ports_id::IP_0_DATA]);
    // ((MRFTController*)MRFT_z)->getPorts()[(int)MRFTController::ports_id::OP_0_DATA]->addCallbackMsgReceiver((MsgReceiver*)sum_PID_MRFT_z->getPorts()[(int)Sum::ports_id::IP_1_DATA]);
    // sum_PID_MRFT_z->getPorts()[(int)Sum::ports_id::OP_0_DATA]->addCallbackMsgReceiver((MsgReceiver*)ID_switch_z->getPorts()[(int)InvertedSwitch::ports_id::IP_0_DATA_DEFAULT]);
    // ID_switch_z->getPorts()[(int)InvertedSwitch::ports_id::OP_0_DATA]->addCallbackMsgReceiver((MsgReceiver*)((HexaActuationSystem*)myActuationSystem)->getPorts()[(int)HexaActuationSystem::ports_id::IP_3_DATA_Z]);
    // ID_switch_z->getPorts()[(int)InvertedSwitch::ports_id::OP_0_DATA]->addCallbackMsgReceiver((MsgReceiver*)((ROSUnit_BroadcastData*)myROSBroadcastData)->getPorts()[(int)ROSUnit_BroadcastData::ports_id::IP_0_DATA]);
    
    
    // //*******************************************************************************************************************

    // rosunit_waypoint_yaw->addCallbackMsgReceiver((MsgReceiver*)Yaw_ControlSystem);
    // Yaw_ControlSystem->addCallbackMsgReceiver((MsgReceiver*)Yaw_Saturation, (int)ControlSystem::unicast_addresses::unicast_control_system);
    // Yaw_Saturation->addCallbackMsgReceiver((MsgReceiver*)YawRate_ControlSystem);
    // YawRate_ControlSystem->addCallbackMsgReceiver((MsgReceiver*)myActuationSystem, (int)ControlSystem::unicast_addresses::unicast_actuation_system);

    // //******************PROVIDERS TO CONTROL SYSTEMS******************************

    // //TODO remove this later, after everything is working, don't forget to change te receiving function
    // rosunit_x_provider->setEmittingChannel((int)ROSUnit_BroadcastData::ros_broadcast_channels::x);
    // rosunit_y_provider->setEmittingChannel((int)ROSUnit_BroadcastData::ros_broadcast_channels::y);
    // rosunit_z_provider->setEmittingChannel((int)ROSUnit_BroadcastData::ros_broadcast_channels::z);
    // rosunit_roll_provider->setEmittingChannel((int)ROSUnit_BroadcastData::ros_broadcast_channels::roll);  
    // rosunit_pitch_provider->setEmittingChannel((int)ROSUnit_BroadcastData::ros_broadcast_channels::pitch);
    // rosunit_yaw_provider->setEmittingChannel((int)ROSUnit_BroadcastData::ros_broadcast_channels::yaw);
    // rosunit_yaw_rate_provider->setEmittingChannel((int)ROSUnit_BroadcastData::ros_broadcast_channels::yaw_rate);

    // rosunit_x_provider->addCallbackMsgReceiver((MsgReceiver*)X_ControlSystem);
    // rosunit_y_provider->addCallbackMsgReceiver((MsgReceiver*)Y_ControlSystem);
    // // rosunit_z_provider->addCallbackMsgReceiver((MsgReceiver*)Z_ControlSystem);
    // rosunit_pitch_provider->addCallbackMsgReceiver((MsgReceiver*)Pitch_ControlSystem);
    // rosunit_roll_provider->addCallbackMsgReceiver((MsgReceiver*)Roll_ControlSystem);
    // rosunit_yaw_provider->addCallbackMsgReceiver((MsgReceiver*)Yaw_ControlSystem);
    // rosunit_yaw_rate_provider->addCallbackMsgReceiver((MsgReceiver*)YawRate_ControlSystem);

    // //This is only needed for the /uav_control/uav_position. Refactor.
    // rosunit_x_provider->addCallbackMsgReceiver((MsgReceiver*)myROSBroadcastData);
    // rosunit_y_provider->addCallbackMsgReceiver((MsgReceiver*)myROSBroadcastData);
    // rosunit_z_provider->addCallbackMsgReceiver((MsgReceiver*)myROSBroadcastData);
    // rosunit_roll_provider->addCallbackMsgReceiver((MsgReceiver*)myROSBroadcastData);
    // rosunit_pitch_provider->addCallbackMsgReceiver((MsgReceiver*)myROSBroadcastData);
    // rosunit_yaw_provider->addCallbackMsgReceiver((MsgReceiver*)myROSBroadcastData);
    // rosunit_yaw_rate_provider->addCallbackMsgReceiver((MsgReceiver*)myROSBroadcastData);

    // //***********************SETTING FLIGHT SCENARIO INPUTS****************************
    // myROSUpdateController->addCallbackMsgReceiver((MsgReceiver*)PID_x, (int)ROSUnit_UpdateController::unicast_addresses::pid);
    // myROSUpdateController->addCallbackMsgReceiver((MsgReceiver*)PID_y, (int)ROSUnit_UpdateController::unicast_addresses::pid);
    // myROSUpdateController->addCallbackMsgReceiver((MsgReceiver*)PID_z, (int)ROSUnit_UpdateController::unicast_addresses::pid);
    // myROSUpdateController->addCallbackMsgReceiver((MsgReceiver*)PID_z_identification, (int)ROSUnit_UpdateController::unicast_addresses::pid);
    // myROSUpdateController->addCallbackMsgReceiver((MsgReceiver*)PID_roll, (int)ROSUnit_UpdateController::unicast_addresses::pid);
    // myROSUpdateController->addCallbackMsgReceiver((MsgReceiver*)PID_pitch, (int)ROSUnit_UpdateController::unicast_addresses::pid);
    // myROSUpdateController->addCallbackMsgReceiver((MsgReceiver*)PID_yaw, (int)ROSUnit_UpdateController::unicast_addresses::pid);
    // myROSUpdateController->addCallbackMsgReceiver((MsgReceiver*)PID_yaw_rate, (int)ROSUnit_UpdateController::unicast_addresses::pid);

    // myROSUpdateController->addCallbackMsgReceiver((MsgReceiver*)MRFT_x, (int)ROSUnit_UpdateController::unicast_addresses::mrft);
    // myROSUpdateController->addCallbackMsgReceiver((MsgReceiver*)MRFT_y, (int)ROSUnit_UpdateController::unicast_addresses::mrft);
    // myROSUpdateController->addCallbackMsgReceiver((MsgReceiver*)MRFT_z, (int)ROSUnit_UpdateController::unicast_addresses::mrft);
    // myROSUpdateController->addCallbackMsgReceiver((MsgReceiver*)MRFT_roll, (int)ROSUnit_UpdateController::unicast_addresses::mrft);
    // myROSUpdateController->addCallbackMsgReceiver((MsgReceiver*)MRFT_pitch, (int)ROSUnit_UpdateController::unicast_addresses::mrft);
    // myROSUpdateController->addCallbackMsgReceiver((MsgReceiver*)MRFT_yaw, (int)ROSUnit_UpdateController::unicast_addresses::mrft);
    // myROSUpdateController->addCallbackMsgReceiver((MsgReceiver*)MRFT_yaw_rate, (int)ROSUnit_UpdateController::unicast_addresses::mrft);

    // myROSUpdateController->addCallbackMsgReceiver((MsgReceiver*)SM_x, (int)ROSUnit_UpdateController::unicast_addresses::sm);
    // myROSUpdateController->addCallbackMsgReceiver((MsgReceiver*)SM_y, (int)ROSUnit_UpdateController::unicast_addresses::sm);

    // myROSResetController->addCallbackMsgReceiver((MsgReceiver*)PID_x);
    // myROSResetController->addCallbackMsgReceiver((MsgReceiver*)PID_y);
    // myROSResetController->addCallbackMsgReceiver((MsgReceiver*)PID_z);
    // myROSResetController->addCallbackMsgReceiver((MsgReceiver*)PID_z_identification);
    // // myROSResetController->addCallbackMsgReceiver((MsgReceiver*)((PIDController*)PID_z_identification)->getPorts()[(int)PIDController::ports_id::IP_2_RESET]);

    // myROSResetController->addCallbackMsgReceiver((MsgReceiver*)PID_roll);
    // myROSResetController->addCallbackMsgReceiver((MsgReceiver*)PID_pitch);
    // myROSResetController->addCallbackMsgReceiver((MsgReceiver*)PID_yaw);
    // myROSResetController->addCallbackMsgReceiver((MsgReceiver*)PID_yaw_rate);

    // myROSResetController->addCallbackMsgReceiver((MsgReceiver*)MRFT_x);
    // myROSResetController->addCallbackMsgReceiver((MsgReceiver*)MRFT_y);
    // myROSResetController->addCallbackMsgReceiver((MsgReceiver*)MRFT_z);
    // myROSResetController->addCallbackMsgReceiver((MsgReceiver*)MRFT_roll);
    // myROSResetController->addCallbackMsgReceiver((MsgReceiver*)MRFT_pitch);
    // myROSResetController->addCallbackMsgReceiver((MsgReceiver*)MRFT_yaw);
    // myROSResetController->addCallbackMsgReceiver((MsgReceiver*)MRFT_yaw_rate);

    // myROSSwitchBlock->addCallbackMsgReceiver((MsgReceiver*)X_ControlSystem);
    // myROSSwitchBlock->addCallbackMsgReceiver((MsgReceiver*)Y_ControlSystem);
    // // myROSSwitchBlock->addCallbackMsgReceiver((MsgReceiver*)Z_ControlSystem);
    // myROSSwitchBlock->addCallbackMsgReceiver((MsgReceiver*)Roll_ControlSystem);
    // myROSSwitchBlock->addCallbackMsgReceiver((MsgReceiver*)Pitch_ControlSystem);
    // myROSSwitchBlock->addCallbackMsgReceiver((MsgReceiver*)Yaw_ControlSystem);
    // myROSSwitchBlock->addCallbackMsgReceiver((MsgReceiver*)YawRate_ControlSystem);

    // myROSArm->addCallbackMsgReceiver((MsgReceiver*) myActuationSystem);
    
    // //********************SETTING FLIGHT SCENARIO OUTPUTS***************************

    // myActuationSystem->addCallbackMsgReceiver((MsgReceiver*)myROSBroadcastData, (int)HexaActuationSystem::unicast_addresses::unicast_ActuationSystem_commands);
    // myActuationSystem->addCallbackMsgReceiver((MsgReceiver*)myROSBroadcastData, (int)HexaActuationSystem::unicast_addresses::unicast_ActuationSystem_armed);

    // X_ControlSystem->addCallbackMsgReceiver((MsgReceiver*)myROSBroadcastData);
    // Y_ControlSystem->addCallbackMsgReceiver((MsgReceiver*)myROSBroadcastData);
    // // Z_ControlSystem->addCallbackMsgReceiver((MsgReceiver*)myROSBroadcastData);
    // Roll_ControlSystem->addCallbackMsgReceiver((MsgReceiver*)myROSBroadcastData);
    // Pitch_ControlSystem->addCallbackMsgReceiver((MsgReceiver*)myROSBroadcastData);
    // Yaw_ControlSystem->addCallbackMsgReceiver((MsgReceiver*)myROSBroadcastData);
    // YawRate_ControlSystem->addCallbackMsgReceiver((MsgReceiver*)myROSBroadcastData);
    
    // #ifdef BATTERY_MONITOR
    // myBatteryMonitor->addCallbackMsgReceiver((MsgReceiver*)myROSBroadcastData);
    // #endif
    // MsgEmitter error_emitter;
    // error_emitter.addCallbackMsgReceiver((MsgReceiver*)myROSBroadcastData);
    // //***********************INERTIAL TO BODY PROVIDER*****************************
 
    // rosunit_yaw_provider->addCallbackMsgReceiver((MsgReceiver*)transform_X_InertialToBody);
    // rosunit_yaw_provider->addCallbackMsgReceiver((MsgReceiver*)transform_Y_InertialToBody);

    // //***********************SETTING PID INITIAL VALUES*****************************
    // ControllerMessage ctrl_msg;
    // PID_parameters pid_para_init;

    // pid_para_init.id = block_id::PID_X;
    // ctrl_msg.setPIDParam(pid_para_init);
    // ctrl_msg.set_dt(X_ControlSystem->get_dt());
    // ((PIDController*)PID_x)->initialize(ctrl_msg.getPIDParam());

    // pid_para_init.id = block_id::PID_Y;
    // ctrl_msg.setPIDParam(pid_para_init);
    // ctrl_msg.set_dt(Y_ControlSystem->get_dt());
    // ((PIDController*)PID_y)->initialize(ctrl_msg.getPIDParam());

    // pid_para_init.id = block_id::PID_Z;
    // ctrl_msg.setPIDParam(pid_para_init);
    // ctrl_msg.set_dt(1./120);
    // ((PIDController*)PID_z)->initialize(ctrl_msg.getPIDParam());

    // pid_para_init.id = block_id::PID_Z_ID;
    // ctrl_msg.setPIDParam(pid_para_init);
    // ctrl_msg.set_dt(1./120);
    // ((PIDController*)PID_z_identification)->initialize(ctrl_msg.getPIDParam());
    
    // pid_para_init.id = block_id::PID_ROLL;
    // ctrl_msg.setPIDParam(pid_para_init);
    // ctrl_msg.set_dt(Roll_ControlSystem->get_dt());
    // ((PIDController*)PID_roll)->initialize(ctrl_msg.getPIDParam());

    // pid_para_init.id = block_id::PID_PITCH;
    // ctrl_msg.setPIDParam(pid_para_init);
    // ctrl_msg.set_dt(Pitch_ControlSystem->get_dt());
    // ((PIDController*)PID_pitch)->initialize(ctrl_msg.getPIDParam());
    
    // pid_para_init.id = block_id::PID_YAW;
    // ctrl_msg.setPIDParam(pid_para_init);
    // ctrl_msg.set_dt(Yaw_ControlSystem->get_dt());
    // ((PIDController*)PID_yaw)->initialize(ctrl_msg.getPIDParam());
    
    // pid_para_init.id = block_id::PID_YAW_RATE;
    // ctrl_msg.setPIDParam(pid_para_init);
    // ctrl_msg.set_dt(YawRate_ControlSystem->get_dt());
    // ((PIDController*)PID_yaw_rate)->initialize(ctrl_msg.getPIDParam());
    
    // //***********************SETTING MRFT INITIAL VALUES*****************************

    // MRFT_parameters mrft_para_init;

    // mrft_para_init.id = block_id::MRFT_X;
    // ctrl_msg.setMRFTParam(mrft_para_init);
    // ctrl_msg.set_dt(X_ControlSystem->get_dt());
    // myROSUpdateController->emitMsgUnicast((DataMessage*) &ctrl_msg, (int)ROSUnit_UpdateController::unicast_addresses::mrft);

    // mrft_para_init.id = block_id::MRFT_Y;
    // ctrl_msg.setMRFTParam(mrft_para_init);
    // ctrl_msg.set_dt(Y_ControlSystem->get_dt());
    // myROSUpdateController->emitMsgUnicast((DataMessage*) &ctrl_msg, (int)ROSUnit_UpdateController::unicast_addresses::mrft);

    // mrft_para_init.id = block_id::MRFT_Z;
    // ctrl_msg.setMRFTParam(mrft_para_init);
    // ctrl_msg.set_dt(1/120);
    // myROSUpdateController->emitMsgUnicast((DataMessage*) &ctrl_msg, (int)ROSUnit_UpdateController::unicast_addresses::mrft);

    // mrft_para_init.id = block_id::MRFT_ROLL;
    // ctrl_msg.setMRFTParam(mrft_para_init);
    // ctrl_msg.set_dt(Roll_ControlSystem->get_dt());
    // myROSUpdateController->emitMsgUnicast((DataMessage*) &ctrl_msg, (int)ROSUnit_UpdateController::unicast_addresses::mrft);

    // mrft_para_init.id = block_id::MRFT_PITCH;
    // ctrl_msg.setMRFTParam(mrft_para_init);
    // ctrl_msg.set_dt(Pitch_ControlSystem->get_dt());
    // myROSUpdateController->emitMsgUnicast((DataMessage*) &ctrl_msg, (int)ROSUnit_UpdateController::unicast_addresses::mrft);

    // mrft_para_init.id = block_id::MRFT_YAW;
    // ctrl_msg.setMRFTParam(mrft_para_init);
    // ctrl_msg.set_dt(Yaw_ControlSystem->get_dt());
    // myROSUpdateController->emitMsgUnicast((DataMessage*) &ctrl_msg, (int)ROSUnit_UpdateController::unicast_addresses::mrft);

    // mrft_para_init.id = block_id::MRFT_YAW_RATE;
    // ctrl_msg.setMRFTParam(mrft_para_init);
    // ctrl_msg.set_dt(YawRate_ControlSystem->get_dt());
    // myROSUpdateController->emitMsgUnicast((DataMessage*) &ctrl_msg, (int)ROSUnit_UpdateController::unicast_addresses::mrft);

    // //***********************SETTING SM INITIAL VALUES*****************************

    // SM_parameters sm_para_init;

    // sm_para_init.id = block_id::SM_X;
    // ctrl_msg.setSMParam(sm_para_init);
    // ctrl_msg.set_dt(X_ControlSystem->get_dt());
    // myROSUpdateController->emitMsgUnicast((DataMessage*) &ctrl_msg, (int)ROSUnit_UpdateController::unicast_addresses::sm);

    // sm_para_init.id = block_id::SM_Y;
    // ctrl_msg.setSMParam(sm_para_init);
    // ctrl_msg.set_dt(Y_ControlSystem->get_dt());
    // myROSUpdateController->emitMsgUnicast((DataMessage*) &ctrl_msg, (int)ROSUnit_UpdateController::unicast_addresses::sm);

    // //***********************SETTING PID+MRFT BLOCK*******************************

    // rosunit_z_provider->addCallbackMsgReceiver((MsgReceiver*)PIDplusMRFT_z);

    // set_realtime_priority();

    // Timer tempo;
    // while(ros::ok()){
    //     tempo.tick();

    //     ros::spinOnce();

    //     int gone = tempo.tockMicroSeconds();
    //     if(gone > 5000) {
    //         std::cout  << "FC over 5000: " << gone << "\n";
    //     }
    //     rate.sleep();

    // }

    return 0;

}

void set_realtime_priority(){
    int ret;

    pthread_t this_thread = pthread_self();

    struct sched_param params;

    params.__sched_priority = sched_get_priority_max(SCHED_FIFO);

    std::cout << "Trying to set thread realtime prio = " << params.__sched_priority << std::endl;

    ret = pthread_setschedparam(this_thread, SCHED_FIFO, &params);

    if (ret != 0){
        std::cout << "Unsuccessful in setting thread realtime prio" << std::endl;
        return;
    }

    int policy = 0;

    ret = pthread_getschedparam(this_thread, &policy, &params);
    if (ret != 0){
        std::cout << "Couldn't retrieve reeal-time scheduling parameters" << std::endl;
        return;
    }

    if (policy != SCHED_FIFO){
        std::cout << "Scheduling is NOT SCHED_FIFO!" << std::endl;
    } else {
        std::cout << "SCHED_FIFO OK" << std::endl;
    }

    std::cout << "Thread priority is " << params.__sched_priority << std::endl;

}
