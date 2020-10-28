#include "ROSUnit_UpdateController.hpp"

ROSUnit_UpdateController* ROSUnit_UpdateController::_instance_ptr = NULL;
ControllerMessage ROSUnit_UpdateController::_update_controller_msg;
control_system ROSUnit_UpdateController::_id;
Port* ROSUnit_UpdateController::_output_port_0 = new OutputPort(ports_id::OP_0_PID, nullptr);
Port* ROSUnit_UpdateController::_output_port_1 = new OutputPort(ports_id::OP_1_MRFT, nullptr);
Port* ROSUnit_UpdateController::_output_port_2 = new OutputPort(ports_id::OP_2_BB, nullptr);

ROSUnit_UpdateController::ROSUnit_UpdateController(ros::NodeHandle& t_main_handler) : ROSUnit(t_main_handler) {
    _srv_update_controller_pid = t_main_handler.advertiseService("update_controller/pid", callbackUpdateControllerPID);
    _srv_update_controller_mrft = t_main_handler.advertiseService("update_controller/mrft", callbackUpdateControllerMRFT);
    _srv_update_controller_sm = t_main_handler.advertiseService("update_controller/sm", callbackUpdateControllerSM);

    _instance_ptr = this;
    _ports = {_output_port_0, _output_port_1, _output_port_2};
}   

ROSUnit_UpdateController::~ROSUnit_UpdateController() {

}

bool ROSUnit_UpdateController::callbackUpdateControllerPID(flight_controller::Update_Controller_PID::Request &req, 
                                                           flight_controller::Update_Controller_PID::Response &res){

    block_id _id = static_cast<block_id>((int)req.controller_parameters.id);
    
    PID_parameters pid_data;
    pid_data.kp = req.controller_parameters.pid_kp;
    pid_data.ki = req.controller_parameters.pid_ki;
    pid_data.kd = req.controller_parameters.pid_kd;
    pid_data.kdd = req.controller_parameters.pid_kdd;
    pid_data.anti_windup = req.controller_parameters.pid_anti_windup;
    pid_data.en_pv_derivation = req.controller_parameters.pid_en_pv_derivation;
    pid_data.dt = req.controller_parameters.pid_dt;
    pid_data.id = _id;
    
    _update_controller_msg.setPIDParam(pid_data);
    _instance_ptr->_output_port_0->receiveMsgData(&_update_controller_msg);

    return true;
}

bool ROSUnit_UpdateController::callbackUpdateControllerMRFT(flight_controller::Update_Controller_MRFT::Request &req, 
                                                                    flight_controller::Update_Controller_MRFT::Response &res){
    
    block_id _id = static_cast<block_id>((int)req.controller_parameters.id);
    
    MRFT_parameters mrft_data;
    mrft_data.beta = req.controller_parameters.mrft_beta;
    mrft_data.relay_amp = req.controller_parameters.mrft_relay_amp;
    mrft_data.bias = req.controller_parameters.mrft_bias;
    mrft_data.id = _id;
    
    _update_controller_msg.setMRFTParam(mrft_data);

    _instance_ptr->_output_port_1->receiveMsgData(&_update_controller_msg);


    return true;
}

bool ROSUnit_UpdateController::callbackUpdateControllerSM(flight_controller::Update_Controller_SM::Request &req, 
                                                                flight_controller::Update_Controller_SM::Response &res){
    
    block_id _id = static_cast<block_id>((int)req.controller_parameters.id);
    
    BB_parameters bb_param;
    bb_param.alpha1 = req.controller_parameters.bb_alpha1;
    bb_param.alpha2 = req.controller_parameters.bb_alpha2;
    bb_param.h1 = req.controller_parameters.bb_h1;
    bb_param.h2 = req.controller_parameters.bb_h2;
    bb_param.id = _id;

    _update_controller_msg.setSMParam(bb_param);

    _instance_ptr->_output_port_2->receiveMsgData(&_update_controller_msg);

    return true;
}

