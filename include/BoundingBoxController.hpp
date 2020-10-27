#pragma once
#include <math.h>
#include "common_srv/FloatMsg.hpp"
#include "Controller.hpp"
#include "common_srv/Timer.hpp"
#include "common_srv/Vector3DMessage.hpp"
#include "SwitchOutMsg.hpp"
#include "logger.hpp"
#include "common_srv/IntegerMsg.hpp"
#include "BB_values.hpp"
#include "common_srv/InputPort.hpp"
#include "common_srv/OutputPort.hpp"

class BoundingBoxController : public Controller{

private:
    Port* _input_port_0;
    Port* _input_port_1;
    Port* _input_port_2;
    Port* _output_port;
    std::vector<Port*> _ports;

	Timer _timer;
	block_id _id;
    controller_type _controller_type;
	float _dt;
	FloatMsg _command_msg;
    double _alpha1, _alpha2, _h1, _h2;
    SwitchOutMsg _switchout_msg;
	double _command = 0;


public:

    enum ports_id {IP_0_DATA, IP_1_UPDATE, IP_2_RESET, OP_0_DATA};

	void switchIn(DataMessage*);
    DataMessage* switchOut();
	void process(DataMessage* t_msg, Port* t_port);
    std::vector<Port*> getPorts();
    void reset();
    void initialize(BB_parameters*);
	DataMessage* runTask(DataMessage*);
    controller_type getControllerType(){ return _controller_type; }
    block_id getID(){ return _id; }
    float bounding_box_algorithm(float);

    BoundingBoxController(block_id t_id);
    ~BoundingBoxController();

    virtual block_type getType() = 0;
    virtual void receiveMsgData(DataMessage* t_msg) = 0;

};
