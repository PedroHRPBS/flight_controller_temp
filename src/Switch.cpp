#include "Switch.hpp"
#include "common_srv/FloatMsg.hpp"

Switch::Switch(std::function<bool(float,float)> t_operation, float t_trigger_value) {
    this->_operation = t_operation;
    this->_trigger_value = t_trigger_value;

    this->_input_port = new InputPort(ports_id::IP_0_DATA, this);
    this->_trigger_port = new InputPort(ports_id::IP_1_TRIGGER, this);
    this->_output_port_0 = new OutputPort(ports_id::OP_0_DATA_DEFAULT, this);
    this->_output_port_1 = new OutputPort(ports_id::OP_1_DATA, this);
    this->_active_output_port = _output_port_0;
    _ports = {_input_port, _trigger_port, _output_port_0, _output_port_1};
}

Switch::~Switch() {

}

void Switch::triggerCallback(float t_current_value){

    if (this->_operation(t_current_value, _trigger_value)){
        _active_output_port = _output_port_1;
    }else{
        _active_output_port = _output_port_0;
    }
}

DataMessage* Switch::runTask(DataMessage* t_msg){
    _active_output_port->receiveMsgData(t_msg);
    return t_msg;
}

void Switch::process(DataMessage* t_msg, Port* t_port) {
    
    if(t_port->getID() == ports_id::IP_0_DATA){
        this->runTask(t_msg);

    }else if(t_port->getID() == ports_id::IP_1_TRIGGER){
        float data = 0.0;

        FloatMsg* float_msg = (FloatMsg*)t_msg;
        data = float_msg->data;

        // std::cout << data << std::endl;
        // std::cout << _active_output_port->getID() << std::endl;

        this->triggerCallback(data) ;
    }
}
