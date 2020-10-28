#include "Differentiator.hpp"

#undef Differentiator_debug

Differentiator::Differentiator(float t_dt) {
    _dt = t_dt;
    this->_input_port = new InputPort(ports_id::IP_0_DATA, this);
    this->_output_port = new OutputPort(ports_id::OP_0_DATA, this);
    _ports = {_input_port, _output_port};
}

Differentiator::~Differentiator() {

}

void Differentiator::process(DataMessage* t_msg, Port* t_port){
    if(t_port->getID() == ports_id::IP_0_DATA){
        FloatMsg* float_msg = (FloatMsg*)t_msg;

        diff = (float_msg->data - _prev) / _dt;
        _prev = float_msg->data;

        FloatMsg output_msg;
        output_msg.data = diff;
        this->_output_port->receiveMsgData((DataMessage*) &output_msg);
    }
}