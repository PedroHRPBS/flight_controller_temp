#pragma once
#include "common_srv/Block.hpp"
#include <math.h>
#include "ControllerMessage.hpp"
#include "PID_values.hpp"
#include "common_srv/FloatMsg.hpp"
#include "Controller.hpp"
#include "common_srv/Vector3DMessage.hpp"
#include "common_srv/FloatMsg.hpp"
#include "ButterFilter_2nd_200Hz.hpp"
#include "SwitchOutMsg.hpp"
#include "logger.hpp"
#include "common_srv/IntegerMsg.hpp"
#include "InputPort.hpp"
#include "OutputPort.hpp"

class PIDController : public Controller{

    private:
        controller_type _controller_type;
        SwitchOutMsg m_switchout_msg;
        block_id _id;
        FloatMsg _command_msg;
        ButterFilter_2nd_200Hz _filter;
        float _filter_y;
        Port* _input_port_0;
        Port* _input_port_1;
        Port* _input_port_2;
        Port* _output_port;
        std::vector<Port*> _ports;
        //Chehadeh's code
        PID_parameters _parameters;
        bool i_term, d_term, dd_term; //Comparing against booleans is faster
        float _dt;
	    bool en_pv_derivation = true, en_anti_windup = false;
        void set_internal_sw(PID_parameters pid_para_x);
        //---------------
        
    public:
        enum ports_id {IP_0_DATA, IP_1_UPDATE, IP_2_RESET, OP_0_DATA};
        void process(DataMessage* t_msg, Port* t_port);
        std::vector<Port*> getPorts();
        //Chehadeh's code
        float prev_err = 0, prev2_err = 0, prev_pv_rate = 0, accum_u = 0, accum_I = 0;
        void initialize(PID_parameters);
        float pid_inc(float err, float pv_first, float pv_second=-1);
        float pid_direct(float err, float pv_first, float pv_second=-1);
        void set_I_term(float);
        //---------------
        void update_params(PID_parameters*);
        enum receiving_channels {ch_update, ch_reset};
        void switchIn(DataMessage*);
        DataMessage* switchOut();
        void receiveMsgData(DataMessage* t_msg); 
        void reset();
        DataMessage* runTask(DataMessage*);
        controller_type getControllerType(){ return _controller_type; }
        block_id getID(){ return _id; }
       
       

        PIDController(block_id t_id);
        ~PIDController();
};
