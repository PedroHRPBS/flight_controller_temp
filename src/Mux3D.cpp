#include "Mux3D.hpp"
#include "common_srv/FloatMsg.hpp"

Mux3D::Mux3D() {
    this->_input_port_0 = new InputPort(ports_id::IP_0_DATA, this);
    this->_input_port_1 = new InputPort(ports_id::IP_1_DATA, this);
    this->_input_port_2 = new InputPort(ports_id::IP_2_DATA, this);
    this->_output_port = new OutputPort(ports_id::OP_0_DATA, this);
    _ports = {_input_port_0, _input_port_1, _input_port_2, _output_port};
}

Mux3D::~Mux3D() {

}


DataMessage* Mux3D::runTask(DataMessage* t_msg){

    Vector3D<float> mux_data;
    mux_data.x = _ip_0;
    mux_data.y = _ip_1;
    mux_data.z = _ip_2;

    Vector3DMessage* mux3d_msg = new Vector3DMessage();
    mux3d_msg->setVector3DMessage(mux_data);

    this->_output_port->receiveMsgData(mux3d_msg);

    return t_msg;
}

void Mux3D::process(DataMessage* t_msg, Port* t_port) {
    
    if(t_port->getID() == ports_id::IP_0_DATA){
        FloatMsg* float_msg = (FloatMsg*)t_msg;
        _ip_0 = float_msg->data;
        this->runTask(t_msg);
    }else if(t_port->getID() == ports_id::IP_1_DATA){
        FloatMsg* float_msg = (FloatMsg*)t_msg;
        _ip_1 = float_msg->data;
    }else if(t_port->getID() == ports_id::IP_2_DATA){
        FloatMsg* float_msg = (FloatMsg*)t_msg;
        _ip_2 = float_msg->data;
    }
}