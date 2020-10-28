#include "InvertedSwitch.hpp"
#include "common_srv/FloatMsg.hpp"

InvertedSwitch::InvertedSwitch(std::function<bool(float,float)> t_operation, float t_trigger_value) {
    this->_operation = t_operation;
    this->_trigger_value = t_trigger_value;

    this->_input_port_0 = new InputPort(ports_id::IP_0_DATA_DEFAULT, this);
    this->_trigger_port = new InputPort(ports_id::IP_1_TRIGGER, this);
    this->_input_port_1 = new InputPort(ports_id::IP_2_DATA, this);
    this->_output_port = new OutputPort(ports_id::OP_0_DATA, this);
    this->_active_input_port = _input_port_0;

    _ports = {_input_port_0, _trigger_port, _input_port_1, _output_port};
}

InvertedSwitch::~InvertedSwitch() {

}

void InvertedSwitch::triggerCallback(float t_current_value){
    if (this->_operation(t_current_value, _trigger_value)){
        _active_input_port = _input_port_1;
    }else{
        _active_input_port = _input_port_0;
    }
}

void InvertedSwitch::process(DataMessage* t_msg, Port* t_port) {
    
    if(t_port->getID() == _active_input_port->getID()){
        _output_port->receiveMsgData(t_msg);

    }else if(t_port->getID() == ports_id::IP_1_TRIGGER){
        this->triggerCallback(((FloatMsg*)t_msg)->data);
    }
}