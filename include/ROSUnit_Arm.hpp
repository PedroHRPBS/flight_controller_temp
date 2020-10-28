#pragma once
#include "common_srv/ROSUnit.hpp"
#include <std_msgs/Bool.h>
#include "common_srv/BooleanMsg.hpp"
#include "common_srv/Vector3D.hpp"
#include "common_srv/BooleanMsg.hpp"
#include <flight_controller/Arm.h>

class ROSUnit_Arm :  public ROSUnit{

    private:
        static ROSUnit_Arm* _instance_ptr;
        static BooleanMsg _bool_msg; 
        ros::ServiceServer _srv_armed;
        static bool callbackArm(flight_controller::Arm::Request  &req, flight_controller::Arm::Response &res);
        void receiveMsgData(DataMessage* t_msg);  
        static Port* _output_port;
    
    public:
        enum ports_id {OP_0_DATA};
        void process(DataMessage* t_msg, Port* t_port);
        std::vector<Port*> getPorts();
        DataMessage* runTask(DataMessage*);

        ROSUnit_Arm(ros::NodeHandle&);
        ~ROSUnit_Arm();
};