#pragma once
#include <common_srv/InputPort.hpp>
#include <common_srv/OutputPort.hpp>
#include "common_srv/Block.hpp"
#include "common_srv/MsgEmitter.hpp"
#include "common_srv/MsgReceiver.hpp"
#include "cmath"
#include "common_srv/FloatMsg.hpp"
#include "ControlSystem.hpp"

class Saturation : public Block, public MsgEmitter  {

private:
    float _clip_value;
    Port* _input_port;
    Port* _output_port;
    std::vector<Port*> _ports;
    float _input = 0.0;

public:
//    void receiveMsgData(DataMessage*, int);
    void clip(float);
    
    enum ports_id {IP_0_DATA, OP_0_DATA};
    DataMessage* runTask(DataMessage*);
    void process(DataMessage* t_msg, Port* t_port);
    Saturation(float);
    std::vector<Port*> getPorts();
    ~Saturation();

    //TODO Refactor below
    block_id getID() {}
    block_type getType() {}
    void switchIn(DataMessage*) {}
    DataMessage* switchOut() {}
    void receiveMsgData(DataMessage* t_msg) {}

};