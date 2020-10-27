#pragma once
#include <InputPort.hpp>
#include <OutputPort.hpp>
#include "common_srv/Block.hpp"
#include "common_srv/MsgEmitter.hpp"
#include "common_srv/MsgReceiver.hpp"
#include "common_srv/Vector3D.hpp"
#include "common_srv/Vector3DMessage.hpp"

class WrapAroundFunction : public Block, public MsgEmitter 
{
    double min_val,max_val,span;
    double _input;
    Vector3D<double> _output;
    Port* _input_port;
    Port* _output_port;
    std::vector<Port*> _ports;

public:
    
    void assignParametersRange(double t_min_val,double t_max_val);
    double wrapAround(double input);
    // void receiveMsgData(DataMessage*);
    // void receiveMsgData(DataMessage* rec_msg, int ch);
    
    enum ports_id {IP_DATA, OP_DATA};
    WrapAroundFunction();
    WrapAroundFunction(double t_min_val, double t_max_val);   

    void process(DataMessage* t_msg, Port* t_port);
    DataMessage* runTask(DataMessage*);
    std::vector<Port*> getPorts();

    block_id getID() {}
    block_type getType() {}
    void switchIn(DataMessage*) {}
    DataMessage* switchOut() {}
    void receiveMsgData(DataMessage* t_msg) {}
};