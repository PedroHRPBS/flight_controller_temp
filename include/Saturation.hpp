#pragma once
#include <common_srv/InputPort.hpp>
#include <common_srv/OutputPort.hpp>
#include "common_srv/Block.hpp"
#include "common_srv/MsgEmitter.hpp"
#include "common_srv/MsgReceiver.hpp"
#include "cmath"
#include "common_srv/FloatMsg.hpp"

class Saturation : public Block {

private:
    float _clip_value;
    Port* _input_port;
    Port* _output_port;
    float _input = 0.0;

public:
    void clip(float);
    
    enum ports_id {IP_0_DATA, OP_0_DATA};
    DataMessage* runTask(DataMessage*);
    void process(DataMessage* t_msg, Port* t_port);
    Saturation(float);
    ~Saturation();

};