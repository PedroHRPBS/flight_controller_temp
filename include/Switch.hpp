#pragma once
#include <string.h>
#include <iostream>
#include <functional>
#include <common_srv/InputPort.hpp>
#include <common_srv/OutputPort.hpp>
#include "common_srv/Block.hpp"
#include "common_srv/FloatMsg.hpp"

class Switch : public Block{

private:
    std::function<bool(float,float)> _operation;
    float _trigger_value;
    Port* _input_port;
    Port* _trigger_port;
    Port* _output_port_0;
    Port* _output_port_1;
    Port* _active_output_port;
    std::vector<Port*> _ports;


public:
    enum ports_id {IP_0_DATA, IP_1_TRIGGER, OP_0_DATA_DEFAULT, OP_1_DATA};
    void triggerCallback(float t_current_value);
    DataMessage* runTask(DataMessage*);
    void process(DataMessage* t_msg, Port* t_port);
    Switch(std::function<bool(float,float)> t_operation, float t_trigger_value);
    std::vector<Port*> getPorts();
    ~Switch();

    //TODO Refactor below
    block_id getID() {}
    block_type getType() {}
    void switchIn(DataMessage*) {}
    DataMessage* switchOut() {}
    void receiveMsgData(DataMessage* t_msg) {}
};