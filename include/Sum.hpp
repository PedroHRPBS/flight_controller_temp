#pragma once
#include <string.h>
#include <iostream>
#include <functional>
#include <common_srv/InputPort.hpp>
#include <common_srv/OutputPort.hpp>
#include "common_srv/Block.hpp"
#include "common_srv/FloatMsg.hpp"
#include "common_srv/Vector3DMessage.hpp"


class Sum : public Block{

private:
    std::function<float(float,float)> _operation;
    Port* _input_port_0;
    Port* _input_port_1;
    Port* _output_port;
    float _v1=0.0, _v2=0.0;

public:
    enum ports_id {IP_0_DATA, IP_1_DATA, OP_0_DATA};
    DataMessage* runTask(DataMessage*);
    void process(DataMessage* t_msg, Port* t_port);
    Sum(std::function<float(float,float)> t_operation);
    ~Sum();

};