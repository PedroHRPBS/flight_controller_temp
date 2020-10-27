#pragma once
#include "common_srv/MsgEmitter.hpp"
#include "common_srv/MsgReceiver.hpp"
#include "cmath"
#include "RotationMatrix3by3.hpp"
#include <atomic>
#include "common_srv/Vector3DMessage.hpp"
#include "common_srv/FloatMsg.hpp"
#include "common_srv/Block.hpp"
#include "common_srv/InputPort.hpp"
#include "common_srv/OutputPort.hpp"

class Transform_InertialToBody : public MsgEmitter, public Block {

private:
    static std::atomic<float>  _inertial_command_x;
    static std::atomic<float>  _inertial_command_y;
    static std::atomic<float>  _inertial_command_z;
    bool _opti_x_received = false, _opti_y_received = false, _current_yaw_received = false;
    RotationMatrix3by3 _rotation_matrix;

    Port* _input_port_0;
    Port* _input_port_1;
    Port* _input_port_2;
    Port* _output_port;
    std::vector<Port*> _ports;

public:

    enum ports_id {IP_0_X, IP_1_Y, IP_2_YAW, OP_0_DATA};
    DataMessage* runTask(DataMessage*);
    void process(DataMessage* t_msg, Port* t_port);
    std::vector<Port*> getPorts();

    void receiveMsgData(DataMessage*);
    void receiveMsgData(DataMessage*, int);
    Transform_InertialToBody();
    ~Transform_InertialToBody();

    //TODO Refactor below
    block_id getID() {}
    block_type getType() {}
    void switchIn(DataMessage*) {}
    DataMessage* switchOut() {}
};