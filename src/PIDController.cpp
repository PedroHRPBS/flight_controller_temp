#include "PIDController.hpp"

PIDController::PIDController(block_id t_id){
    _controller_type = controller_type::pid;
	_id = t_id;
	_input_port_0 = new InputPort(ports_id::IP_0_DATA, this);
	_input_port_1 = new InputPort(ports_id::IP_1_UPDATE, this);
	_input_port_2 = new InputPort(ports_id::IP_2_RESET, this);
	_output_port = new OutputPort(ports_id::OP_0_DATA, this);
    _ports = {_input_port_0, _input_port_1, _input_port_2, _output_port};

}

PIDController::~PIDController() {

}

void PIDController::process(DataMessage* t_msg, Port* t_port) {
    if(t_port->getID() == ports_id::IP_0_DATA){
        this->runTask(t_msg);
    } else if(t_port->getID() == ports_id::IP_1_UPDATE){
        ControllerMessage* pid_msg = (ControllerMessage*)t_msg;
		PID_parameters params = pid_msg->getPIDParam();
		if(params.id == this->_id){		
			this->update_params(&params);	
		}
    } else if(t_port->getID() == ports_id::IP_2_RESET){
        IntegerMsg* integer_msg = (IntegerMsg*)t_msg;

		if(static_cast<block_id>(integer_msg->data) == this->_id){
			Logger::getAssignedLogger()->log("RESET CONTROLLER: %.0f", (int)this->_id, LoggerLevel::Warning);
			this->reset();
		}
	}
}

DataMessage* PIDController::runTask(DataMessage* t_msg){
        
	Vector3DMessage* controller_msg = (Vector3DMessage*)t_msg;

    Vector3D<float> data = controller_msg->getData();
	float command = pid_direct(data.x, data.y, data.z);

	_command_msg.data = command;

    this->_output_port->receiveMsgData(&_command_msg);

	return (DataMessage*) &_command_msg;
}

void PIDController::reset(){
	accum_u = 0; //This is important as it resets NaN condition
	accum_I = 0;
}

void PIDController::update_params(PID_parameters* para){
    
	if(para->kp >= 0.0){
		_parameters.kp = para->kp;
	}
	if(para->ki >= 0.0){
		_parameters.ki = para->ki;
	}
	if(para->kd >= 0.0){
		_parameters.kd = para->kd;
	}
	if(para->kdd >= 0.0){
		_parameters.kdd = para->kdd;
	}
	if(para->en_pv_derivation >= 0.0){
		_parameters.en_pv_derivation = para->en_pv_derivation;
	}
	if(para->anti_windup >= 0.0){
		_parameters.anti_windup = para->anti_windup;
	}

	_parameters.id = para->id;
	_parameters.dt = para->dt;

	set_internal_sw(_parameters);
	
	Logger::getAssignedLogger()->log("PID SETTINGS: ID_%.0f", static_cast<int>(_parameters.id), LoggerLevel::Info);
	Logger::getAssignedLogger()->log("Kp Term: %.3f", _parameters.kp, LoggerLevel::Info);
	Logger::getAssignedLogger()->log("Ki Term: %.3f", _parameters.ki, LoggerLevel::Info);
	Logger::getAssignedLogger()->log("Kd Term: %.3f", _parameters.kd, LoggerLevel::Info);
	Logger::getAssignedLogger()->log("Kdd Term: %.3f", _parameters.kdd, LoggerLevel::Info);
	Logger::getAssignedLogger()->log("Anti Windup Term: %.3f", _parameters.anti_windup, LoggerLevel::Info);
	Logger::getAssignedLogger()->log("en_pv_derivation Term: %.0f", static_cast<int>(_parameters.en_pv_derivation), LoggerLevel::Info);
	Logger::getAssignedLogger()->log("dt: %.6f", _dt, LoggerLevel::Info);
}


// Start of Chehadeh's Code
void PIDController::set_internal_sw(PID_parameters pid_para_x){ //This checks input _parameters. If Kd or Ki<0 it means we do not use them
	i_term = !(pid_para_x.ki <= 0);
	d_term = !(pid_para_x.kd <= 0);
	dd_term= !(pid_para_x.kdd <= 0);
	en_anti_windup = !(pid_para_x.anti_windup <= 0); //Same check for Anti-Windup
	en_pv_derivation = pid_para_x.en_pv_derivation;
	if(pid_para_x.dt > 0){
		_dt = pid_para_x.dt;
	}
}

void PIDController::initialize(PID_parameters para){ //Refer to example 1 on how to initialize
	
	_parameters = para; 
	set_internal_sw(_parameters);
	accum_u = 0; //This is important as it resets NaN condition
	accum_I = 0;
	prev_err = 0;
	prev2_err = 0;
	prev_pv_rate = 0;
	
	Logger::getAssignedLogger()->log("PID SETTINGS: ID_%.0f", static_cast<int>(_parameters.id), LoggerLevel::Info);
	Logger::getAssignedLogger()->log("Kp Term: %.3f", _parameters.kp, LoggerLevel::Info);
	Logger::getAssignedLogger()->log("Ki Term: %.3f", _parameters.ki, LoggerLevel::Info);
	Logger::getAssignedLogger()->log("Kd Term: %.3f", _parameters.kd, LoggerLevel::Info);
	Logger::getAssignedLogger()->log("Kdd Term: %.3f", _parameters.kdd, LoggerLevel::Info);
	Logger::getAssignedLogger()->log("Anti Windup Term: %.3f", _parameters.anti_windup, LoggerLevel::Info);
	Logger::getAssignedLogger()->log("en_pv_derivation Term: %.0f", static_cast<int>(_parameters.en_pv_derivation), LoggerLevel::Info);
	Logger::getAssignedLogger()->log("dt: %.6f", _dt, LoggerLevel::Info);

}

float PIDController::pid_direct(float err, float pv_first, float pv_second) { //Arbitrary large default value for pv_rate
	float u = 0;
	// ************************** P-term ***************************
	u = err *_parameters.kp;
	// ************************** I-term ***************************
	if (i_term)//&& os::is_flying) 
	{
		if (en_anti_windup) { //$$$$$$$$$$$$$$$$$$$$ TODO: Optimize! $$$$$$$$$$$$$$$$$$$$$
			if (fabs(accum_I) < _parameters.anti_windup) {
				accum_I += _parameters.ki*err*_dt;
			}
			else {
				//float buff_I = accum_I + _parameters.ki*err*_dt;
				//if (abs(buff_I) < _parameters.anti_windup) {
				//	accum_I = buff_I;
				//}
				if (((accum_I > 0) && (err < 0))||((accum_I < 0) && (err > 0))) {
					accum_I += _parameters.ki*err*_dt;
				}
			}
		}
		else {
			accum_I += _parameters.ki*err*_dt;
		}
	}
	u += accum_I;
	// ************************** D-term ***************************
	if (d_term) {
		if (en_pv_derivation) {
			u += _parameters.kd*(pv_first);
		}
		else {
			u += _parameters.kd*(err - prev_err) / _dt;
		}
	}
	// ************************* DD-term ***************************
	if (dd_term) {
		u+= _parameters.kdd*(-pv_second);
	}
	prev_err = err;
	return u;
}

float PIDController::pid_inc(float err, float pv_first,float pv_second) { //Arbitrary large default value for pv_rate
	static int ctr = 0;
	ctr++;
	// ************************** P-term ***************************
	accum_u += (err - prev_err)*_parameters.kp;
	// ************************** I-term ***************************
	if (i_term)//&& os::is_flying) 
	{
		if (en_anti_windup) { //$$$$$$$$$$$$$$$$$$$$ TODO: Optimize! $$$$$$$$$$$$$$$$$$$$$
			if (fabs(accum_u)<_parameters.anti_windup)
				accum_u += _parameters.ki*err*_dt;//os::get_dt()
		}
		else {
			accum_u += _parameters.ki*err*_dt;
		}
	}
	// ************************** D-term ***************************
	if (d_term) {
		if (en_pv_derivation) {
			accum_u += _parameters.kd*(-pv_first + prev_pv_rate);
		}
		else {
			accum_u += _parameters.kd*(-err + 2 * prev_err - prev2_err) / _dt;
		}
	}
	prev_pv_rate = pv_first;
	prev2_err = prev_err;
	prev_err = err;
	return accum_u;
	}

void PIDController::set_I_term(float forced_I) {
// #ifdef PID_INC
// 	accum_u = forced_I;
// #else
	accum_I = forced_I;
// #endif // PID_INC

}