#include "WrapAroundFunction.hpp"

WrapAroundFunction::WrapAroundFunction(double t_min_val,double t_max_val){
    _min_val = t_min_val;
    _max_val = t_max_val;
    _span = _max_val-_min_val;

    this->_input_port = new InputPort(ports_id::IP_0_DATA, this);
    this->_output_port = new OutputPort(ports_id::OP_0_DATA, this);
    _ports = {_input_port, _output_port};
}

double WrapAroundFunction::wrapAround(double input){ 
    if (input>_max_val){
        return input-_span;
    }
    else if (input<_min_val){
        return input+_span;
    }
    return input;
}

void WrapAroundFunction::process(DataMessage* t_msg, Port* t_port) {
    if(t_port->getID() == ports_id::IP_0_DATA){    
        _input = ((FloatMsg*)t_msg)->data;
        _output = _input;

        if (_input>_max_val){
            _output = _input-_span;
        }
        else if (_input<_min_val){
            _output = _input+_span;
        }

        FloatMsg output_msg;
        output_msg.data = _output;
        this->_output_port->receiveMsgData((DataMessage*)&output_msg);

    }
}

DataMessage* WrapAroundFunction::runTask(DataMessage* t_msg){ //TODO handle cases for abs(input)>2span
    return t_msg;
}


std::vector<Port*> WrapAroundFunction::getPorts(){
    return _ports;
}
