#include "ROSUnit_ResetController.hpp"
ROSUnit_ResetController* ROSUnit_ResetController::_instance_ptr = NULL;
IntegerMsg ROSUnit_ResetController::_reset_msg;
Port* ROSUnit_ResetController::_output_port = new OutputPort(ports_id::OP_0_DATA, NULL);

ROSUnit_ResetController::ROSUnit_ResetController(ros::NodeHandle& t_main_handler) : ROSUnit(t_main_handler) {
    _srv_reset_controller = t_main_handler.advertiseService("reset_controller", callbackResetController);
    _instance_ptr = this;
    _ports = {_output_port};
}   

ROSUnit_ResetController::~ROSUnit_ResetController() {

}

void ROSUnit_ResetController::process(DataMessage* t_msg, Port* t_port){

}

DataMessage* ROSUnit_ResetController::runTask(DataMessage* t_msg){
}


void ROSUnit_ResetController::receiveMsgData(DataMessage* t_msg){

}

bool ROSUnit_ResetController::callbackResetController(flight_controller::Reset_Controller::Request &req, flight_controller::Reset_Controller::Response &res){

    int data;
    data = req.id;
    _reset_msg.data = data;

    _output_port->receiveMsgData(&_reset_msg);


    return true;
}