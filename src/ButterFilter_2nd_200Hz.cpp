#include "ButterFilter_2nd_200Hz.hpp"


ButterFilter_2nd_200Hz::ButterFilter_2nd_200Hz() {
    this->_input_port_0 = new InputPort(ports_id::IP_0_DATA, this);
    this->_output_port = new OutputPort(ports_id::OP_0_DATA, this);
    _ports = {_input_port_0,_output_port};
}

ButterFilter_2nd_200Hz::~ButterFilter_2nd_200Hz() {

}


DataMessage* ButterFilter_2nd_200Hz::runTask(DataMessage* t_msg){

    float x=_ip_0;
	float y = -coeff_200Hz_2nd_butter[0] * prev_y - coeff_200Hz_2nd_butter[1] * prev2_y + coeff_200Hz_2nd_butter[2] * x + coeff_200Hz_2nd_butter[3] * prev_x + coeff_200Hz_2nd_butter[4] * prev2_x;
	prev2_y = prev_y;
	prev_y = y;
	prev2_x = prev_x;
	prev_x = x;

    FloatMsg* x_msg = new FloatMsg();
    x_msg->data = y;
    this->_output_port->receiveMsgData(x_msg);

    return t_msg;
}

void ButterFilter_2nd_200Hz::process(DataMessage* t_msg, Port* t_port) {
    
    if(t_port->getID() == ports_id::IP_0_DATA)
	{
        FloatMsg* float_msg = (FloatMsg*)t_msg;
        _ip_0 = float_msg->data;
        this->runTask(t_msg);
    }
}

std::vector<Port*> ButterFilter_2nd_200Hz::getPorts(){
    return _ports;
}