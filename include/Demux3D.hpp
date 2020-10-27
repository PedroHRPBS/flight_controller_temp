#pragma once
#include <string.h>
#include <iostream>
#include <functional>
#include <common_srv/InputPort.hpp>
#include <common_srv/OutputPort.hpp>
#include "common_srv/Block.hpp"
#include "common_srv/Vector3DMessage.hpp"
#include "common_srv/FloatMsg.hpp"

class Demux3D : public Block{

private:
    Port* _input_port;
    Port* _output_port_0;
    Port* _output_port_1;
    Port* _output_port_2;
    std::vector<Port*> _ports;
    Vector3D<float> _ip;

public:
    enum ports_id {IP_0_DATA, OP_0_DATA, OP_1_DATA, OP_2_DATA};
    DataMessage* runTask(DataMessage*);
    void process(DataMessage* t_msg, Port* t_port);
    Demux3D();
    ~Demux3D();
    std::vector<Port*> getPorts();

    //TODO Refactor below
    block_id getID() {}
    block_type getType() {}
    void switchIn(DataMessage*) {}
    DataMessage* switchOut() {}
    void receiveMsgData(DataMessage* t_msg) {}
};