#pragma once
#include <common_srv/InputPort.hpp>
#include <common_srv/OutputPort.hpp>
#include "common_srv/Block.hpp"
// #include "common_srv/MsgEmitter.hpp"
// #include "common_srv/MsgReceiver.hpp"
#include "cmath"
#include "common_srv/Vector3DMessage.hpp"
#include "common_srv/Timer.hpp"
#include "common_srv/FloatMsg.hpp"
#include "ButterFilter_120hz.hpp"

class Differentiator : public Block, public MsgEmitter {

private:
    float _prev = 0.0;
    Timer timer;
    float _dt;
    float diff;
    Port* _input_port;
    Port* _output_port;
    std::vector<Port*> _ports;
   
public:
    
//    void receiveMsgData(DataMessage*, int);
    Differentiator(float);
    enum ports_id {IP_0_DATA, OP_0_DATA};
    DataMessage* runTask(DataMessage*);
    void process(DataMessage* t_msg, Port* t_port);
    std::vector<Port*> getPorts();
    ~Differentiator();

    block_id getID() {}
    block_type getType() {}
    void switchIn(DataMessage*) {}
    DataMessage* switchOut() {}
    void receiveMsgData(DataMessage* t_msg) {}
};