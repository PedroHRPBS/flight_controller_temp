#pragma once
#include "common_types.hpp"
#include <functional>
#include <InputPort.hpp>
#include <OutputPort.hpp>
#include "common_srv/Block.hpp"
#include "common_srv/FloatMsg.hpp"

class ButterFilter_2nd_200Hz: public Block{


private:
    Port* _input_port_0;
    Port* _output_port;
    std::vector<Port*> _ports;
    float _ip_0 = 0;
    float prev_y=0, prev2_y=0, prev_x=0, prev2_x=0;

public:
    enum ports_id {IP_0_DATA,OP_0_DATA};
    DataMessage* runTask(DataMessage*);
    void process(DataMessage* t_msg, Port* t_port);
    std::vector<Port*> getPorts();
    ButterFilter_2nd_200Hz();
    ~ButterFilter_2nd_200Hz();
};
