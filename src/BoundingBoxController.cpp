#include "BoundingBoxController.hpp"

BoundingBoxController::BoundingBoxController(block_id t_id) {  

	_input_port_0 = new InputPort(ports_id::IP_0_DATA, this);
	_input_port_1 = new InputPort(ports_id::IP_1_UPDATE, this);
	_input_port_2 = new InputPort(ports_id::IP_2_RESET, this);
	_output_port = new OutputPort(ports_id::OP_0_DATA, this);
	_ports = {_input_port_0, _input_port_1, _input_port_2, _output_port};

    _controller_type = controller_type::bounding_box;
	_id = t_id;

}

BoundingBoxController::~BoundingBoxController() {

}

void BoundingBoxController::switchIn(DataMessage* t_msg){
	Logger::getAssignedLogger()->log("SWITCH IN BOUNDING_BOX CONTROLLER", LoggerLevel::Warning);
}

DataMessage* BoundingBoxController::switchOut(){
	Logger::getAssignedLogger()->log("SWITCH OUT BOUNDING_BOX CONTROLLER", LoggerLevel::Warning);
    
	_switchout_msg.setSwitchOutMsg(0.0);

    return (DataMessage*)&_switchout_msg;
} 

void BoundingBoxController::process(DataMessage* t_msg, Port* t_port) {

	if(t_port->getID() == ports_id::IP_0_DATA){
        this->runTask(t_msg);
    } else if(t_port->getID() == ports_id::IP_1_UPDATE){
        ControllerMessage* sm_msg = (ControllerMessage*)t_msg;
		SM_parameters params = sm_msg->getSMParam();

		if(params.id == this->_id){		
			this->initialize(&params);	
		}
    } else if(t_port->getID() == ports_id::IP_2_RESET){
        IntegerMsg* integer_msg = (IntegerMsg*)t_msg;

		if(static_cast<block_id>(integer_msg->data) == this->_id){
			Logger::getAssignedLogger()->log("RESET CONTROLLER: %.0f", (int)this->_id, LoggerLevel::Warning);
			this->reset();
		}
	}
}

// void BoundingBoxController::receiveMsgData(DataMessage* t_msg){

// 	if(t_msg->getType() == msg_type::UPDATECONTROLLER){
// 		ControllerMessage* sm_msg = (ControllerMessage*)t_msg;
// 		SM_parameters params = sm_msg->getSMParam();

// 		if(params.id == this->_id){		
// 			this->initialize(&params);	
// 		}

// 	}else if(t_msg->getType() == msg_type::INTEGER){
// 		IntegerMsg* integer_msg = (IntegerMsg*)t_msg;

// 		if(static_cast<block_id>(integer_msg->data) == this->_id){
// 			Logger::getAssignedLogger()->log("RESET CONTROLLER: %.0f", (int)this->_id, LoggerLevel::Warning);
// 			this->reset();
// 		}
// 	}
// }

std::vector<Port*> BoundingBoxController::getPorts(){ //TODO move to Block
    return _ports;
}

void BoundingBoxController::reset(){
}

void BoundingBoxController::initialize(SM_parameters* t_params){
	_id = t_params->id;
	_alpha1 = t_params->alpha1;
	_alpha2 = t_params->alpha2;
	_h1 = t_params->h1;
	_h2 = t_params->h2;

	Logger::getAssignedLogger()->log("BOUNDING_BOX SETTINGS: ID_%.0f", static_cast<int>(_id), LoggerLevel::Info);
	Logger::getAssignedLogger()->log("Alpha1: %.2f", _alpha1, LoggerLevel::Info);
	Logger::getAssignedLogger()->log("Alpha2: %.2f", _alpha2, LoggerLevel::Info);
	Logger::getAssignedLogger()->log("H1: %.2f", _h1, LoggerLevel::Info);
	Logger::getAssignedLogger()->log("H2: %.2f", _h2, LoggerLevel::Info);
}

DataMessage* BoundingBoxController::runTask(DataMessage* t_msg){ //TODO: WUT? ~ WAHBAH
        
	Vector3DMessage* controller_msg = (Vector3DMessage*)t_msg;

    Vector3D<float> data = controller_msg->getData();
	
	// data.x is Error
	// data.y is PV_First
	// data.z is PV_Second

    float command;
	
	command = bounding_box_algorithm(data.x);
	
    _command_msg.data = command;

    this->_output_port->receiveMsgData(&_command_msg);

	return (DataMessage*) &_command_msg;
}

float BoundingBoxController::bounding_box_algorithm(float t_error){

    if(-t_error > _h1){
		_command = -_alpha1;
	}else if(-t_error < -_h1){
		_command = _alpha1;
	}

	if(-t_error > _h2){
		_command = -_alpha2;
	}else if(-t_error < -_h2){
		_command = _alpha2;
	}

	return _command;
}