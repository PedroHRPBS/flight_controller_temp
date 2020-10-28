#pragma once
#include <math.h>
#include "ControllerMessage.hpp"
#include "common_srv/FloatMsg.hpp"
#include "MRFT_values.hpp"
#include "Controller.hpp"
#include "common_srv/Timer.hpp"
#include "common_srv/Vector3DMessage.hpp"
#include "SwitchOutMsg.hpp"
#include "logger.hpp"
#include "common_srv/IntegerMsg.hpp"
#include "common_srv/InputPort.hpp"
#include "common_srv/OutputPort.hpp"

class MRFTController : public Controller{

private:
	Timer _timer;
	block_id _id;
    controller_type _controller_type;
	float _dt;
	FloatMsg _command_msg;
	const int no_switch_delay_in_ms = 20;
	const int num_of_peak_conf_samples = 5;
	Port* _input_port_0;
	Port* _input_port_1;
	Port* _input_port_2;
	Port* _output_port;
   	// NEW ALGO
	bool first_run = true;
	float last_output;
	float e_max;
	float e_min;
	float has_reached_min;
	float has_reached_max;
	float peak_conf_counter;
	//
public:
	enum ports_id {IP_0_DATA, IP_1_UPDATE, IP_2_RESET, OP_0_DATA};
	void process(DataMessage* t_msg, Port* t_port);
    MRFT_parameters parameters;
	void initialize(MRFT_parameters*);
	//---------------
	float mrft_anti_false_switching(float err, float beta, float h);
	//---------------
    void reset();
	DataMessage* runTask(DataMessage*);
    controller_type getControllerType(){ return _controller_type; }
    block_id getID(){ return _id; }

    MRFTController(block_id t_id);
    ~MRFTController();
};
