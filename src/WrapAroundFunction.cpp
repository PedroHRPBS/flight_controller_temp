#include "WrapAroundFunction.hpp"

// void WrapAroundFunction::receiveMsgData(DataMessage* rec_msg){
//     //this->receiveMsgData(rec_msg,msg_broadcast_channel);
// }
// void WrapAroundFunction::receiveMsgData(DataMessage* rec_msg, int ch){
//     if (rec_msg->getType()==msg_type::VECTOR3D){
//         Vector3D<double> msg_data=((Vector3DMessage*)rec_msg)->getData();
//         msg_data.z=wrapAround(msg_data.z);
//         Vector3DMessage emit_msg;
//         emit_msg.setVector3DMessage(msg_data);
//         this->emitMsgUnicastDefault(&emit_msg,
//                                     ch);
//     }
// }

double WrapAroundFunction::wrapAround(double input){ //TODO handle cases for abs(input)>2span
    if (input>max_val){
        return input-span;
    }
    else if (input<min_val){
        return input+span;
    }
    return input;
}

void WrapAroundFunction::assignParametersRange(double t_min_val,double t_max_val){
    min_val = t_min_val;
    max_val =t_max_val;
    span=max_val-min_val;
}

WrapAroundFunction::WrapAroundFunction(){}

WrapAroundFunction::WrapAroundFunction(double t_min_val,double t_max_val){
    min_val = t_min_val;
    max_val = t_max_val;
    span = max_val-min_val;
    this->_input_port = new InputPort(ports_id::IP_0_DATA, this);
    this->_output_port = new OutputPort(ports_id::OP_0_DATA, this);
    _ports = {_input_port, _output_port};
}

void WrapAroundFunction::process(DataMessage* t_msg, Port* t_port) {
    if(t_port->getID() == ports_id::IP_0_DATA){    
        _input = ((FloatMsg*)t_msg)->data;
        _output = _input;

        if (_input>max_val){
            _output = _input-span;
        }
        else if (_input<min_val){
            _output = _input+span;
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
