#pragma once
#include "common_srv/Vector3DMessage.hpp"
#include "common_srv/Block.hpp"
#include "InputPort.hpp"
#include "OutputPort.hpp"
#include "XSensMessage.hpp"
#include "common_types.hpp"

class XSens_IMU : public Block{

private:

    Port* _input_port;
    Port* _output_port_0;
    Port* _output_port_1;
    std::vector<Port*> _ports;

    Vector3D<float> _bodyrate;
    Vector3D<float> last_euler_angles;
    Vector3DMessage _roll_pv_msg;
    Vector3DMessage _pitch_pv_msg;

public:

    enum ports_id {IP_0_XSENS, OP_0_ROLL, OP_1_PITCH};

    void process(DataMessage* t_msg, Port* t_port);
    XSens_IMU();
    ~XSens_IMU();

    block_id getID() {};
    block_type getType() {};
    void switchIn(DataMessage*) {};
    DataMessage* switchOut() {};
    DataMessage* runTask(DataMessage*) {};
    void receiveMsgData(DataMessage* t_msg) {};

};