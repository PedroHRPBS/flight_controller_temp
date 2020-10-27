#include "PVConcatenator.hpp"

#define PVConc_debug

PVConcatenator::PVConcatenator(concatenation_axes t_selected_concatenation_axes, act_on t_act_on) {
    _selected_concatenation_axes = t_selected_concatenation_axes;
    _act_on = t_act_on;
    pv = 0.0;
    pv_dot = 0.0;
    pv_dot_dot = 0.0;
    this->_input_port_0 = new InputPort(ports_id::IP_0_PV, this);
    this->_input_port_1 = new InputPort(ports_id::IP_1_PV_DOT, this);
    this->_input_port_2 = new InputPort(ports_id::IP_2_PV_DOT_DOT, this);
    this->_output_port = new OutputPort(ports_id::OP_0_DATA, this);
    _ports = {_input_port_0, _input_port_1, _input_port_2, _output_port};
}

PVConcatenator::~PVConcatenator() {

}

void PVConcatenator::process(DataMessage* t_msg, Port* t_port) {
    #ifdef PVConc_debug
    //std::cout << "PVConcatenator::receiveMsgData(DataMessage* t_msg, int t_channel)" << std::endl;
    #endif
    #ifdef PVConc_debug
        //std::cout << "t_msg->getType() == msg_type::VECTOR3D" << std::endl;
    #endif
    Vector3DMessage* v3d_msg = (Vector3DMessage*)t_msg;
    if(t_port->getID() == ports_id::IP_0_PV){
        if (_selected_concatenation_axes==conc_x_axis){
            pv=v3d_msg->getData().x;
        }else if (_selected_concatenation_axes==conc_y_axis){
            pv=v3d_msg->getData().y;
        }else if (_selected_concatenation_axes==conc_z_axis){
            pv=v3d_msg->getData().z;
        }
        if(_act_on == act_on::pv){
            pv_vector.x = pv;
            pv_vector.y = pv_dot;
            pv_vector.z = pv_dot_dot;
            Vector3DMessage pv_vector_msg;
            pv_vector_msg.setVector3DMessage(pv_vector);
            this->_output_port->receiveMsgData((DataMessage*)&pv_vector_msg);
        }
        this->runTask(t_msg);
    }else if(t_port->getID() == ports_id::IP_1_PV_DOT){
        if (_selected_concatenation_axes==conc_x_axis){
            pv_dot=v3d_msg->getData().x;
        }else if (_selected_concatenation_axes==conc_y_axis){
            pv_dot=v3d_msg->getData().y;
        }else if (_selected_concatenation_axes==conc_z_axis){
            pv_dot=v3d_msg->getData().z;
        }
        if(_act_on == act_on::pv_dot){
            pv_vector.x = pv;
            pv_vector.y = pv_dot;
            pv_vector.z = pv_dot_dot;
            Vector3DMessage pv_vector_msg;
            pv_vector_msg.setVector3DMessage(pv_vector);
            this->_output_port->receiveMsgData((DataMessage*)&pv_vector_msg);
        }
    }else if(t_port->getID() == ports_id::IP_2_PV_DOT_DOT){
        if (_selected_concatenation_axes==conc_x_axis){
            pv_dot_dot=v3d_msg->getData().x;
        }else if (_selected_concatenation_axes==conc_y_axis){
            pv_dot_dot=v3d_msg->getData().y;
        }else if (_selected_concatenation_axes==conc_z_axis){
            pv_dot_dot=v3d_msg->getData().z;
        }
    }
}

DataMessage* PVConcatenator::runTask(DataMessage*) {

}

// void PVConcatenator::receiveMsgData(DataMessage* t_msg){
//     std::cout << "PVConcatenator::receiveMsgData(DataMessage* t_msg)" << std::endl;
// }

// void PVConcatenator::receiveMsgData(DataMessage* t_msg, int t_channel){

// }
