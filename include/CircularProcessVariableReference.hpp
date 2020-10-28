#pragma once
#include "Reference.hpp"
#include "common_srv/FloatMsg.hpp"
#include "common_srv/Vector3D.hpp"
#include "common_srv/Vector3DMessage.hpp"
#include <functional>
#include "common_srv/InputPort.hpp"
#include "common_srv/OutputPort.hpp"
#include "common_srv/Block.hpp"
#include "Sum.hpp"
#include "Mux3D.hpp"
#include "Demux3D.hpp"
#include <atomic>

class CircularProcessVariableReference :public Block{

private:
    Port* _input_port_0;
    Port* _input_port_1;
    Port* _output_port;
    float _ip_0 = 0, _ip_1 = 0;

public:
    enum ports_id {IP_0_DATA, IP_1_DATA,OP_0_DATA};
    DataMessage* runTask(DataMessage*);
    void process(DataMessage* t_msg, Port* t_port);
    std::vector<Port*> getPorts();
    Sum* sum_ref_yaw = new Sum(std::minus<float>());
    Demux3D* prov_demux_yaw = new Demux3D();
    Mux3D* error_mux_yaw = new Mux3D();
    CircularProcessVariableReference();
    ~CircularProcessVariableReference();
    float _reference_value=1;
};