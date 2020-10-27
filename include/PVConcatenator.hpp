#pragma once
#include "common_srv/Vector3DMessage.hpp"
#include "common_types.hpp"
#include "common_srv/Block.hpp"
#include "common_srv/InputPort.hpp"
#include "common_srv/OutputPort.hpp"
#include <atomic>

class PVConcatenator : public Block{


public:
    enum ports_id {IP_0_PV, IP_1_PV_DOT, IP_2_PV_DOT_DOT, OP_0_DATA};
    DataMessage* runTask(DataMessage*);
    void process(DataMessage* t_msg, Port* t_port);
    
    enum concatenation_axes {conc_x_axis,conc_y_axis,conc_z_axis};
    // enum receiving_channels {ch_broadcast,ch_pv,ch_pv_dot,ch_pv_dot_dot};
    void setConcatenationAxes(concatenation_axes); //TODO implement

    PVConcatenator(concatenation_axes, act_on);
    ~PVConcatenator();

    block_id getID() {}
    block_type getType() {}
    void switchIn(DataMessage*) {}
    DataMessage* switchOut() {}
    void receiveMsgData(DataMessage* t_msg) {}

private:
    
    Port* _input_port_0;
    Port* _input_port_1;
    Port* _input_port_2;
    Port* _output_port;
    std::vector<Port*> _ports;

    act_on _act_on;
    Vector3D<double> pv_vector;
    std::atomic<double> pv;
    std::atomic<double> pv_dot;
    std::atomic<double> pv_dot_dot;
    concatenation_axes _selected_concatenation_axes;
};