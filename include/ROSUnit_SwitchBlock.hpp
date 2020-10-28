#pragma once
#include "common_srv/ROSUnit.hpp"
#include <std_msgs/Bool.h>
#include "common_srv/BooleanMsg.hpp"
#include "common_srv/Vector3D.hpp"
#include "common_srv/FloatMsg.hpp"
#include "SwitchBlockMsg.hpp"
#include <flight_controller/SwitchBlock.h>

class ROSUnit_SwitchBlock :  public ROSUnit{

    private:
        static ROSUnit_SwitchBlock* _instance_ptr;
        static SwitchBlockMsg _switch_msg; 
        static FloatMsg _float_msg; 

        ros::ServiceServer _srv_switch;
        static bool callbackSwitchBlocks(flight_controller::SwitchBlock::Request  &req,
                                         flight_controller::SwitchBlock::Response &res);
    
    public:
        ROSUnit_SwitchBlock(ros::NodeHandle&);
        ~ROSUnit_SwitchBlock();
};