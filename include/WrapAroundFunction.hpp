#pragma once
#include <common_srv/InputPort.hpp>
#include <common_srv/OutputPort.hpp>
#include "common_srv/Block.hpp"
#include "common_srv/MsgEmitter.hpp"
#include "common_srv/MsgReceiver.hpp"
#include "common_srv/Vector3D.hpp"
#include "common_srv/Vector3DMessage.hpp"
#include "common_srv/FloatMsg.hpp"

class WrapAroundFunction : public Block, public MsgEmitter 
{
    double _min_val,_max_val,_span;
    double _input;
    double _output;
    Port* _input_port;
    Port* _output_port;
    std::vector<Port*> _ports;

public:
    
    double wrapAround(double input);
    enum ports_id {IP_0_DATA, OP_0_DATA};
    WrapAroundFunction(double t_min_val,double t_max_val);

    void process(DataMessage* t_msg, Port* t_port);
    DataMessage* runTask(DataMessage*);
    std::vector<Port*> getPorts();

    block_id getID() {}
    block_type getType() {}
    void switchIn(DataMessage*) {}
    DataMessage* switchOut() {}
    void receiveMsgData(DataMessage* t_msg) {}
};