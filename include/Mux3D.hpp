#pragma once
#include <string.h>
#include <iostream>
#include <functional>
#include <common_srv/InputPort.hpp>
#include <common_srv/OutputPort.hpp>
#include "common_srv/Block.hpp"
#include "common_srv/Vector3DMessage.hpp"
#include "common_srv/FloatMsg.hpp"

class Mux3D : public Block {

private:
    Port* _input_port_0;
    Port* _input_port_1;
    Port* _input_port_2;
    Port* _output_port;
    float _ip_0 = 0, _ip_1 = 0, _ip_2 = 0;

public:
    enum ports_id {IP_0_DATA, IP_1_DATA, IP_2_DATA, OP_0_DATA};
    DataMessage* runTask(DataMessage*);
    void process(DataMessage* t_msg, Port* t_port);
    Mux3D();
    ~Mux3D();
};