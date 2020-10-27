#pragma once
#include <string.h>
#include <iostream>
#include <functional>
#include <InputPort.hpp>
#include <OutputPort.hpp>
#include "common_srv/Block.hpp"
#include "common_srv/Vector3DMessage.hpp"
#include "common_srv/FloatMsg.hpp"

class Mux3D : public Block {

private:
    Port* _input_port_0;
    Port* _input_port_1;
    Port* _input_port_2;
    Port* _output_port;
    std::vector<Port*> _ports;
    float _ip_0 = 0, _ip_1 = 0, _ip_2 = 0;

public:
    enum ports_id {IP_0_DATA, IP_1_DATA, IP_2_DATA, OP_0_DATA};
    DataMessage* runTask(DataMessage*);
    void process(DataMessage* t_msg, Port* t_port);
    std::vector<Port*> getPorts();
    Mux3D();
    ~Mux3D();

    //TODO Refactor below
    block_id getID() {}
    block_type getType() {}
    void switchIn(DataMessage*) {}
    DataMessage* switchOut() {}
    void receiveMsgData(DataMessage* t_msg) {}
};