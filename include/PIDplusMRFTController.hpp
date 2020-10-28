#pragma once
#include "Controller.hpp"
#include "PIDController.hpp"
#include "MRFTController.hpp"
#include "common_srv/Vector3DMessage.hpp"
#include "common_srv/FloatMsg.hpp"

class PIDplusMRFTController : public Controller{

private:
    PIDController* _pid_controller;
    MRFTController* _mrft_controller;
    float _current_pv, _current_pv_dot, _current_pv_dot_dot;
    float z_min = 0.119842615399054, z_acc_max = 0.5; //z_max = 0.8037149538, z_dot_max = 0.4516856909; //z_max = 0.351764798554836, z_dot_max = 0.496205147596467;
    FloatMsg _command_msg;
    controller_type _controller_type;   
    block_id _id;
    bool _PID_enabled = true;
    float _last_PID;
public:
    // void switchIn(DataMessage*);
    // DataMessage* switchOut();
    DataMessage* runTask(DataMessage*);
    controller_type getControllerType(){ return _controller_type; }
    block_id getID(){ return _id; }

    PIDplusMRFTController(block_id t_id, PIDController* pid_ctr, MRFTController* mrft_ctr);
    ~PIDplusMRFTController();
};