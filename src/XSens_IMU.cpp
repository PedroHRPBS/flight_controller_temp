#include "XSens_IMU.hpp"
#include <fstream>
#include <vector>
#include <iostream>


XSens_IMU::XSens_IMU() {

    _input_port = new InputPort(ports_id::IP_0_XSENS, this);;
    _output_port_0 = new OutputPort(ports_id::OP_0_ROLL, this);
    _output_port_1 = new OutputPort(ports_id::OP_1_PITCH, this);
    _ports = {_input_port, _output_port_0, _output_port_1};

}

XSens_IMU::~XSens_IMU() {

}

void XSens_IMU::process(DataMessage* t_msg, Port* t_port) {
    
    if(t_port->getID() == ports_id::IP_0_XSENS){
        XSensMessage* xsens_msg = (XSensMessage*)t_msg;

        Vector3D<float> orientation_euler = xsens_msg->getOrientationEuler(); 
        Vector3D<float> body_rate = xsens_msg->getAngularVelocity(); 

        Vector3D<float> roll_pv;
        roll_pv.x = orientation_euler.x;
        roll_pv.y = body_rate.x;
        roll_pv.z = 0.0;
        _roll_pv_msg.setVector3DMessage(roll_pv);
        this->_output_port_0->receiveMsgData(&_roll_pv_msg);
        //this->emitMsgUnicast((DataMessage*) &_roll_pv_msg, (int)control_system::roll, (int)control_system::roll);

        Vector3D<float> pitch_pv;
        pitch_pv.x = orientation_euler.y;
        pitch_pv.y = body_rate.y;
        pitch_pv.z = 0.0;
        _pitch_pv_msg.setVector3DMessage(pitch_pv);
        this->_output_port_1->receiveMsgData(&_pitch_pv_msg);
        //this->emitMsgUnicast((DataMessage*) &_pitch_pv_msg, (int)control_system::pitch, (int)control_system::pitch);
    }

}