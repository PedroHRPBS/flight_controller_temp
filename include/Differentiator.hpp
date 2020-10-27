#pragma once
#include <InputPort.hpp>
#include <OutputPort.hpp>
#include "common_srv/Block.hpp"
// #include "common_srv/MsgEmitter.hpp"
// #include "common_srv/MsgReceiver.hpp"
#include "cmath"
#include "common_srv/Vector3DMessage.hpp"
#include "common_srv/Timer.hpp"
#include "common_srv/FloatMsg.hpp"
#include "PVConcatenator.hpp"
#include "ButterFilter_120hz.hpp"

class Differentiator : public Block, public MsgEmitter {

private:
    float _old_float_data;
    Vector3D<float> _old_vector3d_data, _curr_vector3d_data;
    Timer timer;
    float _dt;
    Vector3D<float> diff_values;
    ButterFilter_120hz low_pass_filter_x, low_pass_filter_y, low_pass_filter_z;
    Port* _input_port;
    Port* _output_port;
    std::vector<Port*> _ports;
   
public:
    
//    void receiveMsgData(DataMessage*, int);
    Differentiator(float);
    enum ports_id {IP_DATA, OP_DATA};
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