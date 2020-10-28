#pragma once
#include "common_types.hpp"
#include <functional>
#include <InputPort.hpp>
#include <OutputPort.hpp>
#include "common_srv/FloatMsg.hpp"
#include "common_srv/Block.hpp"

class ButterFilter_120hz: public Block {

private:
    Port* _input_port_0;
    Port* _output_port;
    std::vector<Port*> _ports;
    float _ip_0 = 0;
	float prev_y=0, prev2_y=0, prev_x=0, prev2_x=0;
    const float coeff_120Hz_2nd_butter_5hz[5] = { -1.279632424997809,0.477592250072517,0.049489956268677,0.098979912537354,0.049489956268677 };

public:
    enum ports_id {IP_0_DATA,OP_0_DATA};
    DataMessage* runTask(DataMessage*);
    void process(DataMessage* t_msg, Port* t_port);
    std::vector<Port*> getPorts();
    ButterFilter_120hz();
    ~ButterFilter_120hz();

    block_id getID() {}
    block_type getType() {}
    void switchIn(DataMessage*) {}
    DataMessage* switchOut() {}
    void receiveMsgData(DataMessage* t_msg) {}

};
