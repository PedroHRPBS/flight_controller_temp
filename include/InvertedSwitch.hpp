#pragma once
#include <string.h>
#include <iostream>
#include <functional>
#include <common_srv/InputPort.hpp>
#include <common_srv/OutputPort.hpp>
#include "common_srv/Block.hpp"
#include "common_srv/Vector3DMessage.hpp"

class InvertedSwitch : public Block{

private:
    std::function<bool(float,float)> _operation;
    float _trigger_value;
    Port* _input_port_0;
    Port* _input_port_1;
    Port* _trigger_port;
    Port* _output_port;
    Port* _active_input_port;

public:
    enum ports_id {IP_0_DATA_DEFAULT, IP_1_TRIGGER, IP_2_DATA, OP_0_DATA};
    void triggerCallback(float t_current_value);
    void process(DataMessage* t_msg, Port* t_port);
    InvertedSwitch(std::function<bool(float,float)> t_operation, float t_trigger_value);
    ~InvertedSwitch();
};