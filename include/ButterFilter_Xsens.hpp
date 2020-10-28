#pragma once
#include <functional>
#include <InputPort.hpp>
#include <OutputPort.hpp>
#include "common_srv/Block.hpp"
#include "common_srv/FloatMsg.hpp"

//THIS IS A FILTER WITH FS = 200Hz and FC = 50Hz
class ButterFilter_Xsens: public Block{

private:
	float prev_y=0, prev2_y=0, prev_x=0, prev2_x=0;
    const float coeff_200Hz_2nd_butter_50hz[5] = {-1.561018075800718, 0.641351538057563, 0.020083365564211, 0.040166731128423, 0.020083365564211};
	Port* _input_port_0;
    Port* _output_port;
    std::vector<Port*> _ports;
    float _ip_0 = 0;

public:
    enum ports_id {IP_0_DATA,OP_0_DATA};
    DataMessage* runTask(DataMessage*);
    void process(DataMessage* t_msg, Port* t_port);
    std::vector<Port*> getPorts();
    ButterFilter_Xsens();
    ~ButterFilter_Xsens();

};