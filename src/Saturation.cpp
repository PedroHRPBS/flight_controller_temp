#include "Saturation.hpp"

Saturation::Saturation(float t_clip_value) {
    _clip_value = t_clip_value;
    this->_input_port = new InputPort(ports_id::IP_0_DATA, this);
    this->_output_port = new OutputPort(ports_id::OP_0_DATA, this);
    _ports = {_input_port, _output_port};
}

Saturation::~Saturation() {

}

void Saturation::process(DataMessage* t_msg, Port* t_port) {
    if(t_port->getID() == ports_id::IP_0_DATA){
        FloatMsg* float_msg = (FloatMsg*)t_msg;

        if(float_msg->data > _clip_value){
            float_msg->data = _clip_value;
        }
        else if(float_msg->data < -_clip_value){
            float_msg->data = -_clip_value;
        }
        
        this->_output_port->receiveMsgData(float_msg);
    }
}
