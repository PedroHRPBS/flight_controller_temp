#include "PIDplusMRFTController.hpp"

PIDplusMRFTController::PIDplusMRFTController(block_id t_id, PIDController* pid_ctr, MRFTController* mrft_ctr) {
    _controller_type = controller_type::pid_mrft;
    _id = t_id;
    _pid_controller = pid_ctr;
    _mrft_controller = mrft_ctr;
}

PIDplusMRFTController::~PIDplusMRFTController() {

}

// void PIDplusMRFTController::switchIn(DataMessage* t_msg){
// 	Logger::getAssignedLogger()->log("SWITCH IN PID+MRFT CONTROLLER", LoggerLevel::Warning);
// }

// DataMessage* PIDplusMRFTController::switchOut(){
//     Logger::getAssignedLogger()->log("SWITCH OUT PID+MRFT CONTROLLER",LoggerLevel::Warning);
//     DataMessage* msg;
//     return msg;
// }

DataMessage* PIDplusMRFTController::runTask(DataMessage* t_msg){

    FloatMsg* mrft_output_msg = (FloatMsg*)(_mrft_controller->runTask(t_msg));

    if(!_PID_enabled || _current_pv >= z_min){
        _command_msg.data = _last_PID + mrft_output_msg->data;
        _PID_enabled = false;
        
    }else if(_PID_enabled){
        FloatMsg* pid_output_msg = (FloatMsg*)(_pid_controller->runTask(t_msg));
        _last_PID = pid_output_msg->data;
        _command_msg.data = _last_PID;
    }

	return (DataMessage*) &_command_msg;
}