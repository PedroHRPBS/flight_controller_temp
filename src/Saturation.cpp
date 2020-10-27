#include "Saturation.hpp"

Saturation::Saturation(float t_clip_value) {
    _clip_value = t_clip_value;
    this->_input_port = new InputPort(ports_id::IP_0_DATA, this);
    this->_output_port = new OutputPort(ports_id::OP_0_DATA, this);
    _ports = {_input_port, _output_port};
}

Saturation::~Saturation() {

}

// void Saturation::receiveMsgData(DataMessage* t_msg, int t_channel){

//     if(t_msg->getType() == msg_type::FLOAT){

//         FloatMsg* float_msg = (FloatMsg*)t_msg;
//         FloatMsg output;
//         output.data = float_msg->data;

//         if(output.data > _clip_value){
//             output.data = _clip_value;
//         }else if(output.data < -_clip_value){
//             output.data = -_clip_value;
//         }

//         this->emitMsgUnicast((DataMessage*) &output,
//                                     -1,
//                                     t_channel);   
//     }
// }

DataMessage* Saturation::runTask(DataMessage* t_msg){
    
    FloatMsg* float_msg = new FloatMsg();
    float_msg->data = _input;

    if(float_msg->data > _clip_value){
        float_msg->data = _clip_value;
    }
    else if(float_msg->data < -_clip_value){
        float_msg->data = -_clip_value;
    }
    
    this->_output_port->receiveMsgData(float_msg);

    return t_msg; //TODO no need for t_msg
}

void Saturation::process(DataMessage* t_msg, Port* t_port) {
    if(t_port->getID() == ports_id::IP_0_DATA){
        FloatMsg* float_msg = (FloatMsg*)t_msg;
        float data = float_msg->data;
        _input = data;
    }
}

std::vector<Port*> Saturation::getPorts(){
    return _ports;
}

