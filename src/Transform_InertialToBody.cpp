#include "Transform_InertialToBody.hpp"

std::atomic<float>  Transform_InertialToBody::_inertial_command_x;
std::atomic<float>  Transform_InertialToBody::_inertial_command_y;
std::atomic<float>  Transform_InertialToBody::_inertial_command_z;

Transform_InertialToBody::Transform_InertialToBody() {
    this->_input_port_0 = new InputPort(ports_id::IP_0_X, this);
    this->_input_port_1 = new InputPort(ports_id::IP_1_Y, this);
    this->_input_port_2 = new InputPort(ports_id::IP_2_YAW, this);
    this->_output_port = new OutputPort(ports_id::OP_0_DATA, this);
    _ports = {_input_port_0, _input_port_1, _input_port_2, _output_port};
}

Transform_InertialToBody::~Transform_InertialToBody() {

}

void Transform_InertialToBody::receiveMsgData(DataMessage* t_msg){

    
}

DataMessage* Transform_InertialToBody::runTask(DataMessage*){

}

void Transform_InertialToBody::process(DataMessage* t_msg, Port* t_port){
    if(t_port->getID() == ports_id::IP_0_X){

        FloatMsg* float_msg = (FloatMsg*)t_msg;
        _inertial_command_x = float_msg->data;

        Vector3D<float> inertial_command;
        inertial_command.x=_inertial_command_x;
        inertial_command.y=_inertial_command_y;
        inertial_command.z=_inertial_command_z;
        Vector3D<float> body_command = _rotation_matrix.TransformVector(inertial_command);

        FloatMsg output;
        output.data = body_command.x;
        this->_output_port->receiveMsgData(&output);

    }else if(t_port->getID() == ports_id::IP_1_Y){
        
        FloatMsg* float_msg = (FloatMsg*)t_msg;
        _inertial_command_y = float_msg->data;

        Vector3D<float> inertial_command;
        inertial_command.x=_inertial_command_x;
        inertial_command.y=_inertial_command_y;
        inertial_command.z=_inertial_command_z;
        Vector3D<float> body_command = _rotation_matrix.TransformVector(inertial_command);

        FloatMsg output;
        output.data = body_command.y;
        this->_output_port->receiveMsgData(&output);

    }else if(t_port->getID() == ports_id::IP_2_YAW){
        
        FloatMsg* yaw_msg = (FloatMsg*)t_msg;
        float yaw = yaw_msg->data;

        Vector3D<float> yaw_rotation;
        yaw_rotation.x = 0.0;
        yaw_rotation.y = 0.0;
        yaw_rotation.z = -yaw;

        _rotation_matrix.Update(yaw_rotation);
    }
}

std::vector<Port*> Transform_InertialToBody::getPorts(){
    return _ports;
}

void Transform_InertialToBody::receiveMsgData(DataMessage* t_msg, int t_channel){ 

    // if(t_msg->getType() == msg_type::VECTOR3D){
    //     Vector3DMessage* yaw_msg = (Vector3DMessage*)t_msg;
    //     Vector3D<float> yawpv = yaw_msg->getData();

    //     Vector3D<float> yaw_rotation;
    //     yaw_rotation.x = 0.0;
    //     yaw_rotation.y = 0.0;
    //     yaw_rotation.z = -yawpv.x;

    //     _rotation_matrix.Update(yaw_rotation);
    
    // }else if(t_msg->getType() == msg_type::FLOAT){

    //     FloatMsg* float_msg = (FloatMsg*)t_msg;

    //     FloatMsg output;
    //     Vector3D<float> body_command;
    //     Vector3D<float> inertial_command;
    //     if(_source == control_system::x){
    //         _inertial_command_x = float_msg->data;

    //         inertial_command.x=_inertial_command_x;
    //         inertial_command.y=_inertial_command_y;
    //         inertial_command.z=_inertial_command_z;
    //         body_command = _rotation_matrix.TransformVector(inertial_command);

    //         output.data = body_command.x;
    //         this->emitMsgUnicast((DataMessage*) &output,
    //                                     -1,
    //                                     ControlSystem::receiving_channels::ch_reference);

    //     } else if (_source == control_system::y){
    //         _inertial_command_y = float_msg->data; 

    //         inertial_command.x=_inertial_command_x;
    //         inertial_command.y=_inertial_command_y;
    //         inertial_command.z=_inertial_command_z;
    //         body_command = _rotation_matrix.TransformVector(inertial_command);

    //         output.data = body_command.y;
    //         this->emitMsgUnicast((DataMessage*) &output,
    //                                     -1,
    //                                     ControlSystem::receiving_channels::ch_reference);
    //     }   
    // }
}
