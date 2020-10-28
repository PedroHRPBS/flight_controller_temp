#include "Demux3D.hpp"
#include "common_srv/FloatMsg.hpp"

Demux3D::Demux3D() {
    this->_input_port = new InputPort(ports_id::IP_0_DATA, this);
    this->_output_port_0 = new OutputPort(ports_id::OP_0_DATA, this);
    this->_output_port_1 = new OutputPort(ports_id::OP_1_DATA, this);
    this->_output_port_2 = new OutputPort(ports_id::OP_2_DATA, this);
    _ports = {_input_port, _output_port_0, _output_port_1, _output_port_2};
}

Demux3D::~Demux3D() {

}


DataMessage* Demux3D::runTask(DataMessage* t_msg){

    FloatMsg* _op_0_msg = new FloatMsg();
    _op_0_msg->data = _ip.x;
    this->_output_port_0->receiveMsgData(_op_0_msg);

    FloatMsg* _op_1_msg = new FloatMsg();
    _op_1_msg->data = _ip.y;
    this->_output_port_1->receiveMsgData(_op_1_msg);

    FloatMsg* _op_2_msg = new FloatMsg();
    _op_2_msg->data = _ip.z;
    this->_output_port_2->receiveMsgData(_op_2_msg);

    return t_msg;
}

void Demux3D::process(DataMessage* t_msg, Port* t_port) {
    if(t_port->getID() == ports_id::IP_0_DATA){
        Vector3DMessage* v3d_msg = (Vector3DMessage*)t_msg;
        _ip = v3d_msg->getData();
        this->runTask(t_msg);
    }
}