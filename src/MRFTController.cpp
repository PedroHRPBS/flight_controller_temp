#include "MRFTController.hpp"

MRFTController::MRFTController(block_id t_id) {  
    _controller_type = controller_type::mrft;
	_id = t_id;
	_input_port_0 = new InputPort(ports_id::IP_0_DATA, this);
	_input_port_1 = new InputPort(ports_id::IP_1_UPDATE, this);
	_output_port = new OutputPort(ports_id::OP_0_DATA, this);
    _ports = {_input_port_0, _input_port_1, _output_port};
}

MRFTController::~MRFTController() {

}

// void MRFTController::switchIn(DataMessage* t_msg){
//     if(t_msg->getType() == msg_type::SWITCHOUT){
// 		SwitchOutMsg* switch_out_msg = (SwitchOutMsg*)t_msg;
// 		// parameters.bias = switch_out_msg->getSwitchOutMsg();
// 	}
// 	Logger::getAssignedLogger()->log("SWITCH IN MRFT CONTROLLER - Bias: %f", parameters.bias, LoggerLevel::Warning);
// }

// DataMessage* MRFTController::switchOut(){
// 	Logger::getAssignedLogger()->log("SWITCH OUT MRFT CONTROLLER",LoggerLevel::Warning);
//     DataMessage* msg;
//     return msg;
// } 

void MRFTController::process(DataMessage* t_msg, Port* t_port) {
    if(t_port->getID() == ports_id::IP_0_DATA){
        this->runTask(t_msg);
    } else if(t_port->getID() == ports_id::IP_1_UPDATE){
        ControllerMessage* mrft_msg = (ControllerMessage*)t_msg;
		MRFT_parameters params = mrft_msg->getMRFTParam();

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


void MRFTController::reset(){
	first_run = true;
	last_output = 0;
	e_max = 0;
	e_min = 0;
	has_reached_min = 0;
	has_reached_max = 0;
	peak_conf_counter = 0;
}

DataMessage* MRFTController::runTask(DataMessage* t_msg){
        
	Vector3DMessage* controller_msg = (Vector3DMessage*)t_msg;

    Vector3D<float> data = controller_msg->getData();
	
	// data.x is Error
	// data.y is PV_First
	// data.z is PV_Second

    float command;	
	command = mrft_anti_false_switching(data.x, parameters.beta, parameters.relay_amp)+parameters.bias;

    _command_msg.data = command;

    this->_output_port->receiveMsgData(&_command_msg);

	return (DataMessage*) &_command_msg;
}


void MRFTController::initialize(MRFT_parameters* para){
	
	parameters.beta = para->beta;
	parameters.relay_amp = para->relay_amp;
	parameters.bias = para->bias;
	parameters.id = para->id;
	if(para->dt > 0){
		_dt = para->dt;
	}

	Logger::getAssignedLogger()->log("MRFT SETTINGS: ID_%.0f", static_cast<int>(parameters.id), LoggerLevel::Info);
	Logger::getAssignedLogger()->log("Beta: %.2f", parameters.beta, LoggerLevel::Info);
	Logger::getAssignedLogger()->log("Relay_amp: %.2f", parameters.relay_amp, LoggerLevel::Info);
	Logger::getAssignedLogger()->log("Bias: %.6f", parameters.bias, LoggerLevel::Info);
	Logger::getAssignedLogger()->log("dt: %.6f", _dt, LoggerLevel::Info);

}

float MRFTController::mrft_anti_false_switching(float err, float beta, float h){
	// MRFT algorithm with false switching prevention mechanism
	// June 2020
	// Coded By M. Chehadeh, Khalifa University
	// Translated to C++ By Pedro Silva, Khalifa University

	// mode_of_operation=0 take last e_max or e_min
	// mode_of_operation=1 take current e_max or e_min and mirror it
	int mode_of_operation = 1;
	

	float output=0;
	float e_max_o=0;
	float e_min_o=0;
	float sw_max_o=0;
	float sw_min_o=0;

	if(first_run){
		first_run = false;
		_timer.tick();
		peak_conf_counter=0;
		e_max=0;
		e_min=0;
		if(err>0){
			has_reached_max=true;
			has_reached_min=false;
			last_output=h;
		}else{
			has_reached_max=false;
			has_reached_min=true;
			last_output=-h;
		}			
		output = last_output;
		return output;
	}		 

	output = last_output;

	if (_timer.tockMilliSeconds() <= no_switch_delay_in_ms){
		e_min_o = e_min;
		e_max_o = e_max;
		return output;
	}
		
	if (last_output<0){

		if (err<e_min){
			e_min=err;
			peak_conf_counter=0;
		}else{
			peak_conf_counter=peak_conf_counter+1;
			if (peak_conf_counter>=num_of_peak_conf_samples){
				has_reached_min=true;
				peak_conf_counter=0;
			}
			e_max=err;
		}
		float sw_min;
		if (mode_of_operation==0){
			sw_min = ((e_max-e_min)/2)+e_min+beta*((e_max-e_min)/2);
		}else if (mode_of_operation==1){
			float e_max_star=-e_min;
			sw_min = ((e_max_star-e_min)/2)+e_min+beta*((e_max_star-e_min)/2);
		}
		sw_min_o = sw_min;
		if (has_reached_min){
			if (err>sw_min){
				output=h;
				_timer.tick();
				has_reached_min=false;
			}else{
				output=last_output;
			}
		}
	}else{
		if (err>e_max){
			e_max=err;
			peak_conf_counter=0;
		}else{
			peak_conf_counter=peak_conf_counter+1;
			if (peak_conf_counter>=num_of_peak_conf_samples){
				has_reached_max=true;
				peak_conf_counter=0;
			}
			e_min=err;
		}
		float sw_max;
		if (mode_of_operation==0){
			sw_max=e_max-((e_max-e_min)/2)-(beta*((e_max-e_min)/2));
		}else if (mode_of_operation==1){
			float e_min_star=-e_max;
			sw_max=e_max-((e_max-e_min_star)/2)-(beta*((e_max-e_min_star)/2));
		}    
		sw_max_o=sw_max;
		if (has_reached_max){
			if (err<sw_max){
				output=-h;
				_timer.tick();
				has_reached_max=false;
			}else{
				output=last_output;
			}
		}
	}

	e_min_o=e_min;
	e_max_o=e_max;
	last_output=output;

	return output;
}	
