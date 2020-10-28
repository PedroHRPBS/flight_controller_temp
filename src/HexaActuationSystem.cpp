#include "HexaActuationSystem.hpp"
pthread_mutex_t HexaActuationSystem::lock;

HexaActuationSystem::HexaActuationSystem(std::vector<Actuator*> t_actuators) : ActuationSystem(t_actuators){
    _actuators = t_actuators;
    _input_port_0 = new InputPort(ports_id::IP_0_DATA_ROLL, this);
	_input_port_1 = new InputPort(ports_id::IP_1_DATA_PITCH, this);
	_input_port_2 = new InputPort(ports_id::IP_2_DATA_YAW, this);
	_input_port_3 = new InputPort(ports_id::IP_3_DATA_Z, this);
	_input_port_4 = new InputPort(ports_id::IP_4_ARM, this);
	_output_port_0 = new OutputPort(ports_id::OP_0_CMD, this);
    _output_port_1 = new OutputPort(ports_id::OP_1_ARM, this);
    _ports = {_input_port_0, _input_port_1, _input_port_2, _input_port_3, _input_port_4, _output_port_0, _output_port_1};
}

HexaActuationSystem::~HexaActuationSystem() {

}

void HexaActuationSystem::process(DataMessage* t_msg, Port* t_port) {
    if(t_port->getID() == ports_id::IP_0_DATA_ROLL){
        FloatMsg* float_msg = (FloatMsg*)t_msg;
        _u[0] = float_msg->data;
    } else if(t_port->getID() == ports_id::IP_1_DATA_PITCH){
        FloatMsg* float_msg = (FloatMsg*)t_msg;
        _u[1] = float_msg->data;
        this->runTask(t_msg);
    } else if(t_port->getID() == ports_id::IP_2_DATA_YAW){
        FloatMsg* float_msg = (FloatMsg*)t_msg;
        _u[2] = float_msg->data;
    } else if(t_port->getID() == ports_id::IP_3_DATA_Z){
        FloatMsg* float_msg = (FloatMsg*)t_msg;
        _u[3] = float_msg->data;
    } else if(t_port->getID() == ports_id::IP_4_ARM){
        BooleanMsg* bool_msg = (BooleanMsg*)t_msg;
        _armed = bool_msg->data;
    } 
}

DataMessage* HexaActuationSystem::runTask(DataMessage* t_msg) {
    this->command();
    return t_msg;
} 


void HexaActuationSystem::command(){

    //TODO split into more methods
    for(int i = 0; i < 6; i++){
        _commands[i] = 0.0;
    }

    //Update pulse values
    for(int i = 0; i < 6; i++){
        for(int j = 0; j < 4; j++){
            _commands[i] += _geometry[i][j] * _u[j];
        }
    }

    //_u (PID outputs) should be between 0 and 1. Thus, we have to adjust for the range _escMin_armed to _escMax on _commands.
    //Normalize and Constrain

    for(int i = 0; i < 6; i++){
        _commands[i] = (_commands[i] * (_escMax-_escMin_armed)) + _escMin_armed;
    }

    //Get minimum command
    float min_command = _commands[0];

    for(int i = 1; i < 6; i++){
        if(_commands[i] < min_command){
            min_command = _commands[i];
        }
    }

    float bias = 0;

    //Anti saturation
    if(min_command < _escMin_armed){
        bias = _escMin_armed - min_command;
        
        for(int i = 0; i < 6; i++){
            _commands[i] = _commands[i] + bias;
        }
    }

    for(int i = 0; i < 6; i++){
        if(_armed){
            _commands[i] = this->constrain(_commands[i], _escMin_armed, _escMax);
        }else{
            _commands[i] = _escMin;
        }  
    }

    //Actuate
    for(int i = 0; i < 6; i++){
        _actuators[i]->applyCommand(_commands[i]);
    }



    VectorDoubleMsg commands_msg;
    commands_msg.data = _commands;
    this->_output_port_0->receiveMsgData((DataMessage*)&commands_msg);

    BooleanMsg armed_msg;
    armed_msg.data = _armed;
    this->_output_port_1->receiveMsgData((DataMessage*)&armed_msg);

}

int HexaActuationSystem::constrain(float value, int min_value, int max_value) {
    
    if (value > max_value) {
        value = max_value;
    } else if (value < min_value) {
        value = min_value;
    }

    return int(value);
}

void HexaActuationSystem::receiveMsgData(DataMessage* t_msg){

}

void HexaActuationSystem::receiveMsgData(DataMessage* t_msg, int t_channel){

    // if(t_msg->getType() == msg_type::FLOAT){
    //     FloatMsg* float_msg = (FloatMsg*)t_msg;

    //     if(_armed){
    //         _u[t_channel] = float_msg->data;
    //         if(t_channel == (int)receiving_channels::ch_pitch){ //This sends the commands to the motors on the fastest loop, avoiding thread issues.
    //             this->command();
    //         }
    //     }else{
    //         _u[0] = 0.0;
    //         _u[1] = 0.0;
    //         _u[2] = 0.0;
    //         _u[3] = 0.0;
    //         this->command();
    //     }     
    // }
}
