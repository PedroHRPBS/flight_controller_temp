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
const float SATURATION_VALUE_XY = 0.2617; 
const float SATURATION_VALUE_YAW = 0.2617;
const float SATURATION_VALUE_YAWRATE = 0.3;

void set_realtime_priority();

int main(int argc, char** argv) {
    // std::cout << "Hello Flight Controller!" << std::endl;

    // //*****************************LOGGER********************************** 
    // Logger::assignLogger(new StdLogger());
    
    // //****************************ROS UNITS*******************************

    ros::init(argc, argv, "flight_controller_node");
// 
    ros::NodeHandle nh;
    ros::Rate rate(200);
    ROSUnit_Factory ROSUnit_Factory_main{nh};

    
    ROSUnit* myROSArm = new ROSUnit_Arm(nh);
    ROSUnit* myROSUpdateController = new ROSUnit_UpdateController(nh);
    ROSUnit* myROSResetController = new ROSUnit_ResetController(nh);
    ROSUnit* myROSBroadcastData = new ROSUnit_BroadcastData(nh);
    
    ROSUnit* rosunit_x_provider = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Subscriber, 
                                                                    ROSUnit_msg_type::ROSUnit_Point,
                                                                    "/providers/x");
    ROSUnit* rosunit_y_provider = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Subscriber, 
                                                                    ROSUnit_msg_type::ROSUnit_Point,
                                                                    "/providers/y");
    ROSUnit* rosunit_z_provider = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Subscriber, 
                                                                    ROSUnit_msg_type::ROSUnit_Point,
                                                                    "/providers/z");
    ROSUnit* rosunit_roll_provider = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Subscriber, 
                                                                    ROSUnit_msg_type::ROSUnit_Point,
                                                                    "/providers/roll");
    ROSUnit* rosunit_pitch_provider = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Subscriber, 
                                                                    ROSUnit_msg_type::ROSUnit_Point,
                                                                    "/providers/pitch");
    ROSUnit* rosunit_yaw_provider = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Subscriber, 
                                                                    ROSUnit_msg_type::ROSUnit_Point,
                                                                    "/providers/yaw");
    ROSUnit* rosunit_yaw_rate_provider = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Subscriber, 
                                                                    ROSUnit_msg_type::ROSUnit_Point,
                                                                    "/providers/yaw_rate");
    ROSUnit* rosunit_waypoint_x = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Subscriber, 
                                                                    ROSUnit_msg_type::ROSUnit_Float,
                                                                    "waypoint_reference/x");
    ROSUnit* rosunit_waypoint_y = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Subscriber, 
                                                                    ROSUnit_msg_type::ROSUnit_Float,
                                                                    "waypoint_reference/y");
    ROSUnit* rosunit_waypoint_z = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Subscriber, 
                                                                    ROSUnit_msg_type::ROSUnit_Float,
                                                                    "waypoint_reference/z");
    ROSUnit* rosunit_waypoint_yaw = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Subscriber, 
                                                                    ROSUnit_msg_type::ROSUnit_Float,
                                                                    "waypoint_reference/yaw");                                                            
    ROSUnit* rosunit_ID_switch_x_trigger = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Server, 
                                                                    ROSUnit_msg_type::ROSUnit_Int,
                                                                    "/switch/ID_switch_x");
    ROSUnit* rosunit_bounding_box_switch_x_trigger = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Server, 
                                                                    ROSUnit_msg_type::ROSUnit_Int,
                                                                    "/switch/bounding_box_switch_x");
    ROSUnit* rosunit_ID_switch_roll_trigger = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Server, 
                                                                    ROSUnit_msg_type::ROSUnit_Int,
                                                                    "/switch/ID_switch_roll");
    ROSUnit* rosunit_ID_switch_y_trigger = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Server, 
                                                                    ROSUnit_msg_type::ROSUnit_Int,
                                                                    "/switch/ID_switch_y");
    ROSUnit* rosunit_bounding_box_switch_y_trigger = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Server, 
                                                                    ROSUnit_msg_type::ROSUnit_Int,
                                                                    "/switch/bounding_box_switch_y");
    ROSUnit* rosunit_ID_switch_pitch_trigger = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Server, 
                                                                    ROSUnit_msg_type::ROSUnit_Int,
                                                                    "/switch/ID_switch_pitch");
    ROSUnit* rosunit_ID_switch_z_trigger = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Server, 
                                                                    ROSUnit_msg_type::ROSUnit_Int,
                                                                    "/switch/ID_switch_z");


    //**************************SETTING BLOCKS**********************************

    Block* PID_x = new PIDController(block_id::PID_X);
    Block* PID_pitch = new PIDController(block_id::PID_PITCH);
    Block* PID_y = new PIDController(block_id::PID_Y);
    Block* PID_roll = new PIDController(block_id::PID_ROLL);
    Block* PID_z = new PIDController(block_id::PID_Z);
    Block* PID_z_identification = new PIDController(block_id::PID_Z_ID);
    Block* PID_yaw = new PIDController(block_id::PID_YAW);
    Block* PID_yaw_rate = new PIDController(block_id::PID_YAW_RATE);

    Block* MRFT_x = new MRFTController(block_id::MRFT_X);
    Block* MRFT_y = new MRFTController(block_id::MRFT_Y);
    Block* MRFT_z = new MRFTController(block_id::MRFT_Z);
    Block* MRFT_roll = new MRFTController(block_id::MRFT_ROLL);
    Block* MRFT_pitch = new MRFTController(block_id::MRFT_PITCH);
    Block* MRFT_yaw = new MRFTController(block_id::MRFT_YAW);
    Block* MRFT_yaw_rate = new MRFTController(block_id::MRFT_YAW_RATE);
    Block* BB_x = new BoundingBoxController(block_id::BB_X);
    Block* BB_y = new BoundingBoxController(block_id::BB_Y);

    Transform_InertialToBody* inertialToBody_RotMat = new Transform_InertialToBody();

    Saturation* X_Saturation = new Saturation(SATURATION_VALUE_XY);
    Saturation* Y_Saturation = new Saturation(SATURATION_VALUE_XY);
    Saturation* Yaw_Saturation = new Saturation(SATURATION_VALUE_YAW);
    Saturation* YawRate_Saturation = new Saturation(SATURATION_VALUE_YAWRATE);

    //*********************SETTING ACTUATION SYSTEMS************************
    
    Actuator* M1 = new ESCMotor(0, PWM_FREQUENCY);
    Actuator* M2 = new ESCMotor(1, PWM_FREQUENCY);
    Actuator* M3 = new ESCMotor(2, PWM_FREQUENCY);
    Actuator* M4 = new ESCMotor(3, PWM_FREQUENCY);
    Actuator* M5 = new ESCMotor(4, PWM_FREQUENCY);
    Actuator* M6 = new ESCMotor(5, PWM_FREQUENCY);

    std::vector<Actuator*> actuators{M1, M2, M3, M4, M5, M6};

    ActuationSystem* myActuationSystem = new HexaActuationSystem(actuators);
    // ActuationSystem* myActuationSystem = new QuadActuationSystem(actuators);

    // //***********************************SETTING CONNECTIONS***********************************
    // //========                                                                             =============
    // //|      |-------------->X_Control_System-->RM_X-->Saturation-->Roll_Control_System--->|           |
    // //| USER |-------------->Y_Control_System-->RM_Y-->Saturation-->Pitch_Control_System-->| Actuation |
    // //|      |-------------->Z_Control_System--------------------------------------------->|  System   |
    // //|      |-------------->Yaw_Control_System-->Saturation--->YawRate_Control_System---->|           |
    // //========                                                                             =============
    
    //*******************************************************************************************************************
    // X CHANNEL ->  Multirotors From Takeoff to Real-Time Full Identification Using the Modified Relay Feedback Test and Deep Neural Networks //

    InvertedSwitch* bounding_box_switch_x = new InvertedSwitch(std::equal_to<int>(), 1);
    InvertedSwitch* ID_switch_x = new InvertedSwitch(std::equal_to<int>(), 1);
    Sum* sum_ref_x = new Sum(std::minus<float>());
    Sum* sum_ref_dot_x = new Sum(std::minus<float>());
    Sum* sum_ref_dot_dot_x = new Sum(std::minus<float>());
    Demux3D* prov_demux_x = new Demux3D();
    Mux3D* error_mux_x = new Mux3D();

    rosunit_ID_switch_x_trigger->getPorts()[(int)ROSUnit_SetIntSrv::ports_id::OP_0]->connect(ID_switch_x->getPorts()[InvertedSwitch::ports_id::IP_1_TRIGGER]);
    rosunit_bounding_box_switch_x_trigger->getPorts()[(int)ROSUnit_SetIntSrv::ports_id::OP_1]->connect(bounding_box_switch_x->getPorts()[InvertedSwitch::ports_id::IP_1_TRIGGER]);

    rosunit_waypoint_x->getPorts()[(int)ROSUnit_FloatSub::ports_id::OP_0]->connect(sum_ref_x->getPorts()[(int)Sum::ports_id::IP_0_DATA]);
    rosunit_x_provider->getPorts()[(int)ROSUnit_PointSub::ports_id::OP_0]->connect(prov_demux_x->getPorts()[(int)Demux3D::ports_id::IP_0_DATA]);

    prov_demux_x->getPorts()[(int)Demux3D::ports_id::OP_0_DATA]->connect(sum_ref_x->getPorts()[(int)Sum::ports_id::IP_1_DATA]);
    prov_demux_x->getPorts()[(int)Demux3D::ports_id::OP_1_DATA]->connect(sum_ref_dot_x->getPorts()[(int)Sum::ports_id::IP_1_DATA]);
    prov_demux_x->getPorts()[(int)Demux3D::ports_id::OP_2_DATA]->connect(sum_ref_dot_dot_x->getPorts()[(int)Sum::ports_id::IP_1_DATA]);
    sum_ref_x->getPorts()[(int)Sum::ports_id::OP_0_DATA]->connect(error_mux_x->getPorts()[(int)Mux3D::ports_id::IP_0_DATA]);
    sum_ref_dot_x->getPorts()[(int)Sum::ports_id::OP_0_DATA]->connect(error_mux_x->getPorts()[(int)Mux3D::ports_id::IP_1_DATA]);
    sum_ref_dot_dot_x->getPorts()[(int)Sum::ports_id::OP_0_DATA]->connect(error_mux_x->getPorts()[(int)Mux3D::ports_id::IP_2_DATA]);
    
    error_mux_x->getPorts()[(int)Mux3D::ports_id::OP_0_DATA]->connect(PID_x->getPorts()[(int)PIDController::ports_id::IP_0_DATA]);
    error_mux_x->getPorts()[(int)Mux3D::ports_id::OP_0_DATA]->connect(((MRFTController*)MRFT_x)->getPorts()[(int)MRFTController::ports_id::IP_0_DATA]);
    error_mux_x->getPorts()[(int)Mux3D::ports_id::OP_0_DATA]->connect(((BoundingBoxController*)BB_x)->getPorts()[(int)BoundingBoxController::ports_id::IP_0_DATA]);
    

    PID_x->getPorts()[(int)PIDController::ports_id::OP_0_DATA]->connect(ID_switch_x->getPorts()[(int)InvertedSwitch::ports_id::IP_2_DATA]);
    //TODO SEG FAULT HERE
    MRFT_x->getPorts()[(int)MRFTController::ports_id::OP_0_DATA]->connect(ID_switch_x->getPorts()[(int)InvertedSwitch::ports_id::IP_0_DATA_DEFAULT]);
    
    BB_x->getPorts()[(int)BoundingBoxController::ports_id::OP_0_DATA]->connect(bounding_box_switch_x->getPorts()[(int)InvertedSwitch::ports_id::IP_0_DATA_DEFAULT]);
    ID_switch_x->getPorts()[(int)InvertedSwitch::ports_id::OP_0_DATA]->connect(bounding_box_switch_x->getPorts()[(int)InvertedSwitch::ports_id::IP_2_DATA]);

    // Rotation Matrix
    bounding_box_switch_x->getPorts()[(int)InvertedSwitch::ports_id::OP_0_DATA]->connect(inertialToBody_RotMat->getPorts()[(int)Transform_InertialToBody::ports_id::IP_0_X]);

    // Saturation
    inertialToBody_RotMat->getPorts()[(int)Transform_InertialToBody::ports_id::OP_0_DATA]->connect(X_Saturation->getPorts()[(int)Saturation::ports_id::IP_0_DATA]);
    
    // Roll
    InvertedSwitch* ID_switch_roll = new InvertedSwitch(std::equal_to<int>(), 1);
    Sum* sum_ref_roll = new Sum(std::minus<float>());
    Sum* sum_ref_dot_roll = new Sum(std::minus<float>());
    Sum* sum_ref_dot_dot_roll = new Sum(std::minus<float>());
    Demux3D* prov_demux_roll = new Demux3D();
    Mux3D* error_mux_roll = new Mux3D();

    rosunit_ID_switch_roll_trigger->getPorts()[(int)ROSUnit_SetIntSrv::ports_id::OP_2]->connect(ID_switch_roll->getPorts()[InvertedSwitch::ports_id::IP_1_TRIGGER]);

    X_Saturation->getPorts()[(int)Saturation::ports_id::OP_0_DATA]->connect(sum_ref_roll->getPorts()[(int)Sum::ports_id::IP_0_DATA]);
    rosunit_roll_provider->getPorts()[(int)ROSUnit_PointSub::ports_id::OP_3]->connect(prov_demux_roll->getPorts()[(int)Demux3D::ports_id::IP_0_DATA]);

    prov_demux_roll->getPorts()[(int)Demux3D::ports_id::OP_0_DATA]->connect(sum_ref_roll->getPorts()[(int)Sum::ports_id::IP_1_DATA]);
    prov_demux_roll->getPorts()[(int)Demux3D::ports_id::OP_1_DATA]->connect(sum_ref_dot_roll->getPorts()[(int)Sum::ports_id::IP_1_DATA]);
    prov_demux_roll->getPorts()[(int)Demux3D::ports_id::OP_2_DATA]->connect(sum_ref_dot_dot_roll->getPorts()[(int)Sum::ports_id::IP_1_DATA]);
    sum_ref_roll->getPorts()[(int)Sum::ports_id::OP_0_DATA]->connect(error_mux_roll->getPorts()[(int)Mux3D::ports_id::IP_0_DATA]);
    sum_ref_dot_roll->getPorts()[(int)Sum::ports_id::OP_0_DATA]->connect(error_mux_roll->getPorts()[(int)Mux3D::ports_id::IP_1_DATA]);
    sum_ref_dot_dot_roll->getPorts()[(int)Sum::ports_id::OP_0_DATA]->connect(error_mux_roll->getPorts()[(int)Mux3D::ports_id::IP_2_DATA]);

    error_mux_roll->getPorts()[(int)Mux3D::ports_id::OP_0_DATA]->connect(PID_roll->getPorts()[(int)PIDController::ports_id::IP_0_DATA]);
    error_mux_roll->getPorts()[(int)Mux3D::ports_id::OP_0_DATA]->connect(((MRFTController*)MRFT_roll)->getPorts()[(int)MRFTController::ports_id::IP_0_DATA]);
    
    PID_roll->getPorts()[(int)PIDController::ports_id::OP_0_DATA]->connect(ID_switch_roll->getPorts()[(int)InvertedSwitch::ports_id::IP_2_DATA]);
    ((MRFTController*)MRFT_roll)->getPorts()[(int)MRFTController::ports_id::OP_0_DATA]->connect(ID_switch_roll->getPorts()[(int)InvertedSwitch::ports_id::IP_0_DATA_DEFAULT]);

    ID_switch_roll->getPorts()[(int)InvertedSwitch::ports_id::OP_0_DATA]->connect(((Block*)myActuationSystem)->getPorts()[(int)HexaActuationSystem::ports_id::IP_0_DATA_ROLL]);
    
    //*******************************************************************************************************************
    // Y CHANNEL ->  Multirotors From Takeoff to Real-Time Full Identification Using the Modified Relay Feedback Test and Deep Neural Networks //

    InvertedSwitch* bounding_box_switch_y = new InvertedSwitch(std::equal_to<int>(), 1);
    InvertedSwitch* ID_switch_y = new InvertedSwitch(std::equal_to<int>(), 1);
    Sum* sum_ref_y = new Sum(std::minus<float>());
    Sum* sum_ref_dot_y = new Sum(std::minus<float>());
    Sum* sum_ref_dot_dot_y = new Sum(std::minus<float>());
    Demux3D* prov_demux_y = new Demux3D();
    Mux3D* error_mux_y = new Mux3D();

    rosunit_ID_switch_y_trigger->getPorts()[(int)ROSUnit_SetIntSrv::ports_id::OP_3]->connect(ID_switch_y->getPorts()[InvertedSwitch::ports_id::IP_1_TRIGGER]);
    rosunit_bounding_box_switch_y_trigger->getPorts()[(int)ROSUnit_SetIntSrv::ports_id::OP_4]->connect(bounding_box_switch_y->getPorts()[InvertedSwitch::ports_id::IP_1_TRIGGER]);

    rosunit_waypoint_y->getPorts()[(int)ROSUnit_FloatSub::ports_id::OP_1]->connect(sum_ref_y->getPorts()[(int)Sum::ports_id::IP_0_DATA]);
    rosunit_y_provider->getPorts()[(int)ROSUnit_PointSub::ports_id::OP_1]->connect(prov_demux_y->getPorts()[(int)Demux3D::ports_id::IP_0_DATA]);

    prov_demux_y->getPorts()[(int)Demux3D::ports_id::OP_0_DATA]->connect(sum_ref_y->getPorts()[(int)Sum::ports_id::IP_1_DATA]);
    prov_demux_y->getPorts()[(int)Demux3D::ports_id::OP_1_DATA]->connect(sum_ref_dot_y->getPorts()[(int)Sum::ports_id::IP_1_DATA]);
    prov_demux_y->getPorts()[(int)Demux3D::ports_id::OP_2_DATA]->connect(sum_ref_dot_dot_y->getPorts()[(int)Sum::ports_id::IP_1_DATA]);
    sum_ref_y->getPorts()[(int)Sum::ports_id::OP_0_DATA]->connect(error_mux_y->getPorts()[(int)Mux3D::ports_id::IP_0_DATA]);
    sum_ref_dot_y->getPorts()[(int)Sum::ports_id::OP_0_DATA]->connect(error_mux_y->getPorts()[(int)Mux3D::ports_id::IP_1_DATA]);
    sum_ref_dot_dot_y->getPorts()[(int)Sum::ports_id::OP_0_DATA]->connect(error_mux_y->getPorts()[(int)Mux3D::ports_id::IP_2_DATA]);

    error_mux_y->getPorts()[(int)Mux3D::ports_id::OP_0_DATA]->connect(PID_y->getPorts()[(int)PIDController::ports_id::IP_0_DATA]);
    error_mux_y->getPorts()[(int)Mux3D::ports_id::OP_0_DATA]->connect(((MRFTController*)MRFT_y)->getPorts()[(int)MRFTController::ports_id::IP_0_DATA]);
    error_mux_y->getPorts()[(int)Mux3D::ports_id::OP_0_DATA]->connect(((BoundingBoxController*)BB_y)->getPorts()[(int)BoundingBoxController::ports_id::IP_0_DATA]);
    
    PID_y->getPorts()[(int)PIDController::ports_id::OP_0_DATA]->connect(ID_switch_y->getPorts()[(int)InvertedSwitch::ports_id::IP_2_DATA]);
    ((MRFTController*)MRFT_y)->getPorts()[(int)MRFTController::ports_id::OP_0_DATA]->connect(ID_switch_y->getPorts()[(int)InvertedSwitch::ports_id::IP_0_DATA_DEFAULT]);
    ((BoundingBoxController*)BB_y)->getPorts()[(int)BoundingBoxController::ports_id::OP_0_DATA]->connect(bounding_box_switch_y->getPorts()[(int)InvertedSwitch::ports_id::IP_0_DATA_DEFAULT]);
    ID_switch_y->getPorts()[(int)InvertedSwitch::ports_id::OP_0_DATA]->connect(bounding_box_switch_y->getPorts()[(int)InvertedSwitch::ports_id::IP_2_DATA]);

    // Rotation Matrix
    bounding_box_switch_y->getPorts()[(int)InvertedSwitch::ports_id::OP_0_DATA]->connect(inertialToBody_RotMat->getPorts()[(int)Transform_InertialToBody::ports_id::IP_1_Y]);

    // Saturation
    inertialToBody_RotMat->getPorts()[(int)Transform_InertialToBody::ports_id::OP_0_DATA]->connect(Y_Saturation->getPorts()[(int)Saturation::ports_id::IP_0_DATA]);
    // Pitch
    InvertedSwitch* ID_switch_pitch = new InvertedSwitch(std::equal_to<int>(), 1);
    Sum* sum_ref_pitch = new Sum(std::minus<float>());
    Sum* sum_ref_dot_pitch = new Sum(std::minus<float>());
    Sum* sum_ref_dot_dot_pitch = new Sum(std::minus<float>());
    Demux3D* prov_demux_pitch = new Demux3D();
    Mux3D* error_mux_pitch = new Mux3D();

    rosunit_ID_switch_pitch_trigger->getPorts()[(int)ROSUnit_SetIntSrv::ports_id::OP_5]->connect(ID_switch_pitch->getPorts()[InvertedSwitch::ports_id::IP_1_TRIGGER]);

    Y_Saturation->getPorts()[(int)Saturation::ports_id::OP_0_DATA]->connect(sum_ref_pitch->getPorts()[(int)Sum::ports_id::IP_0_DATA]);
    rosunit_pitch_provider->getPorts()[(int)ROSUnit_PointSub::ports_id::OP_4]->connect(prov_demux_pitch->getPorts()[(int)Demux3D::ports_id::IP_0_DATA]);

    prov_demux_pitch->getPorts()[(int)Demux3D::ports_id::OP_0_DATA]->connect(sum_ref_pitch->getPorts()[(int)Sum::ports_id::IP_1_DATA]);
    prov_demux_pitch->getPorts()[(int)Demux3D::ports_id::OP_1_DATA]->connect(sum_ref_dot_pitch->getPorts()[(int)Sum::ports_id::IP_1_DATA]);
    prov_demux_pitch->getPorts()[(int)Demux3D::ports_id::OP_2_DATA]->connect(sum_ref_dot_dot_pitch->getPorts()[(int)Sum::ports_id::IP_1_DATA]);
    sum_ref_pitch->getPorts()[(int)Sum::ports_id::OP_0_DATA]->connect(error_mux_pitch->getPorts()[(int)Mux3D::ports_id::IP_0_DATA]);
    sum_ref_dot_pitch->getPorts()[(int)Sum::ports_id::OP_0_DATA]->connect(error_mux_pitch->getPorts()[(int)Mux3D::ports_id::IP_1_DATA]);
    sum_ref_dot_dot_pitch->getPorts()[(int)Sum::ports_id::OP_0_DATA]->connect(error_mux_pitch->getPorts()[(int)Mux3D::ports_id::IP_2_DATA]);
 
    error_mux_pitch->getPorts()[(int)Mux3D::ports_id::OP_0_DATA]->connect(PID_pitch->getPorts()[(int)PIDController::ports_id::IP_0_DATA]);
    error_mux_pitch->getPorts()[(int)Mux3D::ports_id::OP_0_DATA]->connect(((MRFTController*)MRFT_pitch)->getPorts()[(int)MRFTController::ports_id::IP_0_DATA]);
    
    PID_pitch->getPorts()[(int)PIDController::ports_id::OP_0_DATA]->connect(ID_switch_pitch->getPorts()[(int)InvertedSwitch::ports_id::IP_2_DATA]);
    ((MRFTController*)MRFT_pitch)->getPorts()[(int)MRFTController::ports_id::OP_0_DATA]->connect(ID_switch_pitch->getPorts()[(int)InvertedSwitch::ports_id::IP_0_DATA_DEFAULT]);
    ID_switch_pitch->getPorts()[(int)InvertedSwitch::ports_id::OP_0_DATA]->connect(((Block*)myActuationSystem)->getPorts()[(int)HexaActuationSystem::ports_id::IP_1_DATA_PITCH]);
    
    //*******************************************************************************************************************
    // Z CHANNEL ->  Multirotors From Takeoff to Real-Time Full Identification Using the Modified Relay Feedback Test and Deep Neural Networks //

    InvertedSwitch* ID_switch_z = new InvertedSwitch(std::equal_to<int>(), 1);
    Switch* PID_MRFT_switch_z = new Switch(std::greater_equal<float>(), 0.119842615399054);
    Sum* sum_PID_MRFT_z = new Sum(std::plus<float>());
    Sum* sum_ref_z = new Sum(std::minus<float>());
    Sum* sum_ref_dot_z = new Sum(std::minus<float>());
    Sum* sum_ref_dot_dot_z = new Sum(std::minus<float>());
    Demux3D* prov_demux_z = new Demux3D();
    Mux3D* error_mux_z = new Mux3D();

    rosunit_ID_switch_z_trigger->getPorts()[(int)ROSUnit_SetIntSrv::ports_id::OP_6]->connect(ID_switch_z->getPorts()[InvertedSwitch::ports_id::IP_1_TRIGGER]);

    rosunit_waypoint_z->getPorts()[(int)ROSUnit_FloatSub::ports_id::OP_2]->connect(sum_ref_z->getPorts()[(int)Sum::ports_id::IP_0_DATA]);
    rosunit_z_provider->getPorts()[(int)ROSUnit_PointSub::ports_id::OP_2]->connect(prov_demux_z->getPorts()[(int)Demux3D::ports_id::IP_0_DATA]);

    prov_demux_z->getPorts()[(int)Demux3D::ports_id::OP_0_DATA]->connect(sum_ref_z->getPorts()[(int)Sum::ports_id::IP_1_DATA]);
    prov_demux_z->getPorts()[(int)Demux3D::ports_id::OP_1_DATA]->connect(sum_ref_dot_z->getPorts()[(int)Sum::ports_id::IP_1_DATA]);
    prov_demux_z->getPorts()[(int)Demux3D::ports_id::OP_2_DATA]->connect(sum_ref_dot_dot_z->getPorts()[(int)Sum::ports_id::IP_1_DATA]);
    sum_ref_z->getPorts()[(int)Sum::ports_id::OP_0_DATA]->connect(error_mux_z->getPorts()[(int)Mux3D::ports_id::IP_0_DATA]);
    sum_ref_dot_z->getPorts()[(int)Sum::ports_id::OP_0_DATA]->connect(error_mux_z->getPorts()[(int)Mux3D::ports_id::IP_1_DATA]);
    sum_ref_dot_dot_z->getPorts()[(int)Sum::ports_id::OP_0_DATA]->connect(error_mux_z->getPorts()[(int)Mux3D::ports_id::IP_2_DATA]);

    error_mux_z->getPorts()[(int)Mux3D::ports_id::OP_0_DATA]->connect(PID_z->getPorts()[(int)PIDController::ports_id::IP_0_DATA]);
    error_mux_z->getPorts()[(int)Mux3D::ports_id::OP_0_DATA]->connect(PID_z_identification->getPorts()[(int)PIDController::ports_id::IP_0_DATA]);
    error_mux_z->getPorts()[(int)Mux3D::ports_id::OP_0_DATA]->connect(((MRFTController*)MRFT_z)->getPorts()[(int)MRFTController::ports_id::IP_0_DATA]);
    
    PID_z->getPorts()[(int)PIDController::ports_id::OP_0_DATA]->connect(ID_switch_z->getPorts()[(int)InvertedSwitch::ports_id::IP_2_DATA]);
    PID_z_identification->getPorts()[(int)PIDController::ports_id::OP_0_DATA]->connect(PID_MRFT_switch_z->getPorts()[(int)Switch::ports_id::IP_0_DATA]);
    prov_demux_z->getPorts()[(int)Demux3D::ports_id::OP_0_DATA]->connect(PID_MRFT_switch_z->getPorts()[(int)Switch::ports_id::IP_1_TRIGGER]);
    PID_MRFT_switch_z->getPorts()[(int)Switch::ports_id::OP_0_DATA_DEFAULT]->connect(sum_PID_MRFT_z->getPorts()[(int)Sum::ports_id::IP_0_DATA]);
    ((MRFTController*)MRFT_z)->getPorts()[(int)MRFTController::ports_id::OP_0_DATA]->connect(sum_PID_MRFT_z->getPorts()[(int)Sum::ports_id::IP_1_DATA]);
    sum_PID_MRFT_z->getPorts()[(int)Sum::ports_id::OP_0_DATA]->connect(ID_switch_z->getPorts()[(int)InvertedSwitch::ports_id::IP_0_DATA_DEFAULT]);
    ID_switch_z->getPorts()[(int)InvertedSwitch::ports_id::OP_0_DATA]->connect(((Block*)myActuationSystem)->getPorts()[(int)HexaActuationSystem::ports_id::IP_3_DATA_Z]);
    
    //*******************************************************************************************************************
    // YAW CHANNEL ->  Multirotors From Takeoff to Real-Time Full Identification Using the Modified Relay Feedback Test and Deep Neural Networks //

    CircularProcessVariableReference* sum_ref_yaw = new CircularProcessVariableReference();
    Sum* sum_ref_dot_yaw = new Sum(std::minus<float>());
    Sum* sum_ref_dot_dot_yaw = new Sum(std::minus<float>());
    Demux3D* prov_demux_yaw = new Demux3D();
    Mux3D* error_mux_yaw = new Mux3D();

    rosunit_waypoint_yaw->getPorts()[(int)ROSUnit_FloatSub::ports_id::OP_3]->connect(((Block*)sum_ref_yaw)->getPorts()[(int)Sum::ports_id::IP_0_DATA]);
    rosunit_yaw_provider->getPorts()[(int)ROSUnit_PointSub::ports_id::OP_5]->connect(prov_demux_yaw->getPorts()[(int)Demux3D::ports_id::IP_0_DATA]);
    rosunit_yaw_provider->getPorts()[(int)ROSUnit_PointSub::ports_id::OP_5]->connect(inertialToBody_RotMat->getPorts()[(int)Transform_InertialToBody::ports_id::IP_2_YAW]);

    prov_demux_yaw->getPorts()[(int)Demux3D::ports_id::OP_0_DATA]->connect(((Block*)sum_ref_yaw)->getPorts()[(int)Sum::ports_id::IP_1_DATA]);
    prov_demux_yaw->getPorts()[(int)Demux3D::ports_id::OP_1_DATA]->connect(sum_ref_dot_yaw->getPorts()[(int)Sum::ports_id::IP_1_DATA]);
    prov_demux_yaw->getPorts()[(int)Demux3D::ports_id::OP_2_DATA]->connect(sum_ref_dot_dot_yaw->getPorts()[(int)Sum::ports_id::IP_1_DATA]);
    ((Block*)sum_ref_yaw)->getPorts()[(int)Sum::ports_id::OP_0_DATA]->connect(error_mux_yaw->getPorts()[(int)Mux3D::ports_id::IP_0_DATA]);
    sum_ref_dot_yaw->getPorts()[(int)Sum::ports_id::OP_0_DATA]->connect(error_mux_yaw->getPorts()[(int)Mux3D::ports_id::IP_1_DATA]);
    sum_ref_dot_dot_yaw->getPorts()[(int)Sum::ports_id::OP_0_DATA]->connect(error_mux_yaw->getPorts()[(int)Mux3D::ports_id::IP_2_DATA]);

    error_mux_yaw->getPorts()[(int)Mux3D::ports_id::OP_0_DATA]->connect(PID_yaw->getPorts()[(int)PIDController::ports_id::IP_0_DATA]);
    
    PID_yaw->getPorts()[(int)PIDController::ports_id::OP_0_DATA]->connect(Yaw_Saturation->getPorts()[(int)Saturation::ports_id::IP_0_DATA]);

    // Yaw Rate
    Sum* sum_ref_yaw_rate = new Sum(std::minus<float>());
    Sum* sum_ref_dot_yaw_rate = new Sum(std::minus<float>());
    Sum* sum_ref_dot_dot_yaw_rate = new Sum(std::minus<float>());
    Demux3D* prov_demux_yaw_rate = new Demux3D();
    Mux3D* error_mux_yaw_rate = new Mux3D();

    Yaw_Saturation->getPorts()[(int)Saturation::ports_id::OP_0_DATA]->connect(sum_ref_yaw_rate->getPorts()[(int)Sum::ports_id::IP_0_DATA]);
    rosunit_yaw_rate_provider->getPorts()[(int)ROSUnit_PointSub::ports_id::OP_6]->connect(prov_demux_yaw_rate->getPorts()[(int)Demux3D::ports_id::IP_0_DATA]);

    prov_demux_yaw_rate->getPorts()[(int)Demux3D::ports_id::OP_0_DATA]->connect(sum_ref_yaw_rate->getPorts()[(int)Sum::ports_id::IP_1_DATA]);
    prov_demux_yaw_rate->getPorts()[(int)Demux3D::ports_id::OP_1_DATA]->connect(sum_ref_dot_yaw_rate->getPorts()[(int)Sum::ports_id::IP_1_DATA]);
    prov_demux_yaw_rate->getPorts()[(int)Demux3D::ports_id::OP_2_DATA]->connect(sum_ref_dot_dot_yaw_rate->getPorts()[(int)Sum::ports_id::IP_1_DATA]);
    sum_ref_yaw_rate->getPorts()[(int)Sum::ports_id::OP_0_DATA]->connect(error_mux_yaw_rate->getPorts()[(int)Mux3D::ports_id::IP_0_DATA]);
    sum_ref_dot_yaw_rate->getPorts()[(int)Sum::ports_id::OP_0_DATA]->connect(error_mux_yaw_rate->getPorts()[(int)Mux3D::ports_id::IP_1_DATA]);
    sum_ref_dot_dot_yaw_rate->getPorts()[(int)Sum::ports_id::OP_0_DATA]->connect(error_mux_yaw_rate->getPorts()[(int)Mux3D::ports_id::IP_2_DATA]);

    error_mux_yaw_rate->getPorts()[(int)Mux3D::ports_id::OP_0_DATA]->connect(PID_yaw_rate->getPorts()[(int)PIDController::ports_id::IP_0_DATA]);
    
    PID_yaw_rate->getPorts()[(int)PIDController::ports_id::OP_0_DATA]->connect(((Block*)myActuationSystem)->getPorts()[(int)HexaActuationSystem::ports_id::IP_2_DATA_YAW]);
    //*******************************************************************************************************************
    
    // ROS CONTROL OUTPUTS
    X_Saturation->getPorts()[(int)Saturation::ports_id::OP_0_DATA]->connect(((Block*)myROSBroadcastData)->getPorts()[(int)ROSUnit_BroadcastData::ports_id::IP_0_X_OUTPUT]);
    Y_Saturation->getPorts()[(int)Saturation::ports_id::OP_0_DATA]->connect(((Block*)myROSBroadcastData)->getPorts()[(int)ROSUnit_BroadcastData::ports_id::IP_1_Y_OUTPUT]);
    ID_switch_z->getPorts()[(int)InvertedSwitch::ports_id::OP_0_DATA]->connect(((Block*)myROSBroadcastData)->getPorts()[(int)ROSUnit_BroadcastData::ports_id::IP_2_Z_OUTPUT]);
    ID_switch_roll->getPorts()[(int)InvertedSwitch::ports_id::OP_0_DATA]->connect(((Block*)myROSBroadcastData)->getPorts()[(int)ROSUnit_BroadcastData::ports_id::IP_3_ROLL_OUTPUT]);
    ID_switch_pitch->getPorts()[(int)InvertedSwitch::ports_id::OP_0_DATA]->connect(((Block*)myROSBroadcastData)->getPorts()[(int)ROSUnit_BroadcastData::ports_id::IP_4_PITCH_OUTPUT]);
    Yaw_Saturation->getPorts()[(int)Saturation::ports_id::OP_0_DATA]->connect(((Block*)myROSBroadcastData)->getPorts()[(int)ROSUnit_BroadcastData::ports_id::IP_5_YAW_OUTPUT]);
    PID_yaw_rate->getPorts()[(int)PIDController::ports_id::OP_0_DATA]->connect(((Block*)myROSBroadcastData)->getPorts()[(int)ROSUnit_BroadcastData::ports_id::IP_6_YAWRATE_OUTPUT]);   

    //***********************SETTING FLIGHT SCENARIO INPUTS****************************
    myROSUpdateController->getPorts()[(int)ROSUnit_UpdateController::ports_id::OP_0_PID]->connect(PID_x->getPorts()[(int)PIDController::ports_id::IP_1_UPDATE]);
    myROSUpdateController->getPorts()[(int)ROSUnit_UpdateController::ports_id::OP_0_PID]->connect(PID_y->getPorts()[(int)PIDController::ports_id::IP_1_UPDATE]);
    myROSUpdateController->getPorts()[(int)ROSUnit_UpdateController::ports_id::OP_0_PID]->connect(PID_roll->getPorts()[(int)PIDController::ports_id::IP_1_UPDATE]);
    myROSUpdateController->getPorts()[(int)ROSUnit_UpdateController::ports_id::OP_0_PID]->connect(PID_pitch->getPorts()[(int)PIDController::ports_id::IP_1_UPDATE]);
    myROSUpdateController->getPorts()[(int)ROSUnit_UpdateController::ports_id::OP_0_PID]->connect(PID_yaw->getPorts()[(int)PIDController::ports_id::IP_1_UPDATE]);
    myROSUpdateController->getPorts()[(int)ROSUnit_UpdateController::ports_id::OP_0_PID]->connect(PID_yaw_rate->getPorts()[(int)PIDController::ports_id::IP_1_UPDATE]);

    myROSUpdateController->getPorts()[(int)ROSUnit_UpdateController::ports_id::OP_1_MRFT]->connect(((MRFTController*)MRFT_x)->getPorts()[(int)MRFTController::ports_id::IP_1_UPDATE]);
    myROSUpdateController->getPorts()[(int)ROSUnit_UpdateController::ports_id::OP_1_MRFT]->connect(((MRFTController*)MRFT_y)->getPorts()[(int)MRFTController::ports_id::IP_1_UPDATE]);
    myROSUpdateController->getPorts()[(int)ROSUnit_UpdateController::ports_id::OP_1_MRFT]->connect(((MRFTController*)MRFT_z)->getPorts()[(int)MRFTController::ports_id::IP_1_UPDATE]);
    myROSUpdateController->getPorts()[(int)ROSUnit_UpdateController::ports_id::OP_1_MRFT]->connect(((MRFTController*)MRFT_roll)->getPorts()[(int)MRFTController::ports_id::IP_1_UPDATE]);
    myROSUpdateController->getPorts()[(int)ROSUnit_UpdateController::ports_id::OP_1_MRFT]->connect(((MRFTController*)MRFT_pitch)->getPorts()[(int)MRFTController::ports_id::IP_1_UPDATE]);
    myROSUpdateController->getPorts()[(int)ROSUnit_UpdateController::ports_id::OP_1_MRFT]->connect(((MRFTController*)MRFT_yaw)->getPorts()[(int)MRFTController::ports_id::IP_1_UPDATE]);
    myROSUpdateController->getPorts()[(int)ROSUnit_UpdateController::ports_id::OP_1_MRFT]->connect(((MRFTController*)MRFT_yaw_rate)->getPorts()[(int)MRFTController::ports_id::IP_1_UPDATE]);

    myROSUpdateController->getPorts()[(int)ROSUnit_UpdateController::ports_id::OP_2_BB]->connect(((BoundingBoxController*)BB_x)->getPorts()[(int)BoundingBoxController::ports_id::IP_1_UPDATE]);
    myROSUpdateController->getPorts()[(int)ROSUnit_UpdateController::ports_id::OP_2_BB]->connect(((BoundingBoxController*)BB_y)->getPorts()[(int)BoundingBoxController::ports_id::IP_1_UPDATE]);

    myROSResetController->getPorts()[(int)ROSUnit_ResetController::ports_id::OP_0_DATA]->connect(PID_x->getPorts()[(int)PIDController::ports_id::IP_2_RESET]);
    myROSResetController->getPorts()[(int)ROSUnit_ResetController::ports_id::OP_0_DATA]->connect(PID_y->getPorts()[(int)PIDController::ports_id::IP_2_RESET]);
    myROSResetController->getPorts()[(int)ROSUnit_ResetController::ports_id::OP_0_DATA]->connect(PID_z->getPorts()[(int)PIDController::ports_id::IP_2_RESET]);
    myROSResetController->getPorts()[(int)ROSUnit_ResetController::ports_id::OP_0_DATA]->connect(PID_z_identification->getPorts()[(int)PIDController::ports_id::IP_2_RESET]);

    myROSResetController->getPorts()[(int)ROSUnit_ResetController::ports_id::OP_0_DATA]->connect(PID_roll->getPorts()[(int)PIDController::ports_id::IP_2_RESET]);
    myROSResetController->getPorts()[(int)ROSUnit_ResetController::ports_id::OP_0_DATA]->connect(PID_pitch->getPorts()[(int)PIDController::ports_id::IP_2_RESET]);
    myROSResetController->getPorts()[(int)ROSUnit_ResetController::ports_id::OP_0_DATA]->connect(PID_yaw->getPorts()[(int)PIDController::ports_id::IP_2_RESET]);
    myROSResetController->getPorts()[(int)ROSUnit_ResetController::ports_id::OP_0_DATA]->connect(PID_yaw_rate->getPorts()[(int)PIDController::ports_id::IP_2_RESET]);

    myROSResetController->getPorts()[(int)ROSUnit_ResetController::ports_id::OP_0_DATA]->connect(((MRFTController*)MRFT_x)->getPorts()[(int)MRFTController::ports_id::IP_2_RESET]);
    myROSResetController->getPorts()[(int)ROSUnit_ResetController::ports_id::OP_0_DATA]->connect(((MRFTController*)MRFT_y)->getPorts()[(int)MRFTController::ports_id::IP_2_RESET]);
    myROSResetController->getPorts()[(int)ROSUnit_ResetController::ports_id::OP_0_DATA]->connect(((MRFTController*)MRFT_z)->getPorts()[(int)MRFTController::ports_id::IP_2_RESET]);
    myROSResetController->getPorts()[(int)ROSUnit_ResetController::ports_id::OP_0_DATA]->connect(((MRFTController*)MRFT_roll)->getPorts()[(int)MRFTController::ports_id::IP_2_RESET]);
    myROSResetController->getPorts()[(int)ROSUnit_ResetController::ports_id::OP_0_DATA]->connect(((MRFTController*)MRFT_pitch)->getPorts()[(int)MRFTController::ports_id::IP_2_RESET]);
    myROSResetController->getPorts()[(int)ROSUnit_ResetController::ports_id::OP_0_DATA]->connect(((MRFTController*)MRFT_yaw)->getPorts()[(int)MRFTController::ports_id::IP_2_RESET]);
    myROSResetController->getPorts()[(int)ROSUnit_ResetController::ports_id::OP_0_DATA]->connect(((MRFTController*)MRFT_yaw_rate)->getPorts()[(int)MRFTController::ports_id::IP_2_RESET]);

    myROSArm->getPorts()[(int)ROSUnit_Arm::ports_id::OP_0_DATA]->connect(((Block*)myActuationSystem)->getPorts()[(int)HexaActuationSystem::ports_id::IP_4_ARM]);
    
    //********************SETTING FLIGHT SCENARIO OUTPUTS***************************

    ((Block*)myActuationSystem)->getPorts()[(int)HexaActuationSystem::ports_id::OP_0_CMD]->connect(((Block*)myROSBroadcastData)->getPorts()[(int)ROSUnit_BroadcastData::ports_id::IP_14_MOTORS]);
    ((Block*)myActuationSystem)->getPorts()[(int)HexaActuationSystem::ports_id::OP_1_ARM]->connect(((Block*)myROSBroadcastData)->getPorts()[(int)ROSUnit_BroadcastData::ports_id::IP_15_ARMED]);
    

    set_realtime_priority();

    Timer tempo;
    while(ros::ok()){
        tempo.tick();

        ros::spinOnce();

        int gone = tempo.tockMicroSeconds();
        if(gone > 5000) {
            std::cout  << "FC over 5000: " << gone << "\n";
        }
        rate.sleep();

    }

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
