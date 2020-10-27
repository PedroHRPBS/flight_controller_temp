#pragma once
#include "common_srv/Port.hpp"
#include "InputPort.hpp"

class OutputPort : public Port{

private:
    DataMessage* _data;
    int _id;
    Block* _block;

public:
    
    void receiveMsgData(DataMessage* t_msg);
    void receiveMsgData(DataMessage* t_msg, int channel_id);
    void connect(InputPort*);
    OutputPort(int t_id, Block* t_block);
    ~OutputPort();
};