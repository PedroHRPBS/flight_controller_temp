#include "Differentiator.hpp"

#undef Differentiator_debug

// void Differentiator::receiveMsgData(DataMessage* t_msg, int t_channel){
//     if(t_msg->getType() == msg_type::VECTOR3D){
//         Vector3DMessage* vector3d_data = (Vector3DMessage*)t_msg;

//         Vector3DMessage output_msg;

//         diff_values.x = (vector3d_data->getData().x - _old_vector3d_data.x) / _dt;
//         diff_values.y = (vector3d_data->getData().y - _old_vector3d_data.y) / _dt;
//         diff_values.z = (vector3d_data->getData().z - _old_vector3d_data.z) / _dt;

//         //FILTERING BEFORE SENDING
//         diff_values.x = low_pass_filter_x.perform(diff_values.x);
//         diff_values.y = low_pass_filter_y.perform(diff_values.y);
//         diff_values.z = low_pass_filter_z.perform(diff_values.z);

//         output_msg.setVector3DMessage(diff_values);
//         this->emitMsgUnicastDefault((DataMessage*) &output_msg);
        
//         _old_vector3d_data = vector3d_data->getData();
//     }
// }

Differentiator::Differentiator(float t_dt) {
    _dt = t_dt;
    this->_input_port = new InputPort(ports_id::IP_0_DATA, this);
    this->_output_port = new OutputPort(ports_id::OP_0_DATA, this);
    _ports = {_input_port, _output_port};
}

Differentiator::~Differentiator() {

}

DataMessage* Differentiator::runTask(DataMessage* t_msg){
    
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

std::vector<Port*> Differentiator::getPorts(){
    return _ports;
}