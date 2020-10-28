#pragma once
#include "common_srv/ROSUnit.hpp"
#include <geometry_msgs/PoseStamped.h>
#include "common_srv/Vector3D.hpp"
#include "PID_values.hpp"
#include <flight_controller/PID_param.h>
#include <flight_controller/MRFT_param.h>
#include <flight_controller/SM_param.h>
#include <flight_controller/Update_Controller_PID.h>
#include <flight_controller/Update_Controller_MRFT.h>
#include <flight_controller/Update_Controller_SM.h>
#include "common_types.hpp"
#include "ControllerMessage.hpp"
#include "common_srv/OutputPort.hpp"

class ROSUnit_UpdateController :  public ROSUnit{

    private:
        static control_system _id;
        static ROSUnit_UpdateController* _instance_ptr;
        static ControllerMessage _update_controller_msg; 
        ros::ServiceServer _srv_update_controller_pid;
        ros::ServiceServer _srv_update_controller_mrft;
        ros::ServiceServer _srv_update_controller_sm;

        static bool callbackUpdateControllerPID(flight_controller::Update_Controller_PID::Request  &req, 
                                             flight_controller::Update_Controller_PID::Response &res);
        static bool callbackUpdateControllerMRFT(flight_controller::Update_Controller_MRFT::Request  &req, 
                                             flight_controller::Update_Controller_MRFT::Response &res);
        static bool callbackUpdateControllerSM(flight_controller::Update_Controller_SM::Request  &req, 
                                             flight_controller::Update_Controller_SM::Response &res);

                                             
        static Port* _output_port_0;
        static Port* _output_port_1;
        static Port* _output_port_2;
        
    public:
        enum ports_id {OP_0_PID, OP_1_MRFT, OP_2_BB};
        void process(DataMessage* t_msg, Port* t_port) {};
        enum unicast_addresses {broadcast, pid, mrft, sm};
        ROSUnit_UpdateController(ros::NodeHandle&);
        ~ROSUnit_UpdateController();
};