#include "CircularProcessVariableReference.hpp"
#include <math.h>

CircularProcessVariableReference::CircularProcessVariableReference() {

    this->_input_port_0 = new InputPort(ports_id::IP_0_DATA, this);
    this->_input_port_1 = new InputPort(ports_id::IP_1_DATA, this);
    this->_output_port = new OutputPort(ports_id::OP_0_DATA, this);
    _ports = {_input_port_0, _input_port_1,_output_port};
}

CircularProcessVariableReference::~CircularProcessVariableReference() {

}


DataMessage* CircularProcessVariableReference::runTask(DataMessage* t_msg){

    FloatMsg* yaw_msg = (FloatMsg*)t_msg;

    FloatMsg* error_msg = new FloatMsg();
    
    float pv = yaw_msg->data;

    //Adjust the reference to be between +PI and -PI
    if(fabs(_reference_value - pv) > M_PI){
        float a = fabs(_reference_value - pv);
        float b = 2 * M_PI - a;
        float sign_a;
        if(pv-_reference_value > 0){
            sign_a = 1;
        }else{
            sign_a = -1;
        }
        float ref_a = pv + b * sign_a;
        error_msg->data = ref_a - pv;
    }else{
        error_msg->data = _reference_value - pv;
    }
    
    this->_output_port->receiveMsgData(error_msg);
    return t_msg;

}

void CircularProcessVariableReference::process(DataMessage* t_msg, Port* t_port) {
    
    if(t_port->getID() == ports_id::IP_0_DATA){
        FloatMsg* float_msg = (FloatMsg*)t_msg;
        _ip_0 = float_msg->data;
        this->runTask(t_msg);
    }else if(t_port->getID() == ports_id::IP_1_DATA){
        FloatMsg* float_msg = (FloatMsg*)t_msg;
        _ip_1 = float_msg->data;
        this->runTask(t_msg);

    }
}

std::vector<Port*> CircularProcessVariableReference::getPorts(){
    return _ports;
}