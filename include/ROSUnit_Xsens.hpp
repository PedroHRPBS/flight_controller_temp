#pragma once
#include "ros/ros.h"
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/Imu.h>
#include "common_srv/ROSUnit.hpp"
#include "common_srv/Vector3D.hpp"
#include "Quaternion.hpp"
#include "common_srv/Vector3DMessage.hpp"
#include "PVConcatenator.hpp"
#include "Global2Inertial.hpp"
#include "common_srv/Timer.hpp"
#include "ButterFilter_Xsens.hpp"
#include "OutputPort.hpp"

class ROSUnit_Xsens : public ROSUnit{

    private:
        static ROSUnit_Xsens* _instance_ptr;
        ros::Subscriber _sub_attitude;
        ros::Subscriber _sub_acceleration;
        ros::Subscriber _sub_velocity;
        ros::Subscriber _sub_body_rate;
        static void callbackXsensFreeAcceleration(const geometry_msgs::Vector3Stamped& msg_free_acceleration);
        static void callbackXsensAttitude(const geometry_msgs::QuaternionStamped& msg_attitude);
        static void callbackXsensVelocity(const geometry_msgs::TwistStamped& msg_velocity);
        static void callbackXsensBodyRate(const geometry_msgs::Vector3Stamped& msg_bodyrate);
        static ButterFilter_Xsens filter_gyro_x;
        static ButterFilter_Xsens filter_gyro_y;
        static ButterFilter_Xsens filter_gyro_z;
        static Port* _output_port_0;
        static Port* _output_port_1;
        static Port* _output_port_2;
        static Port* _output_port_3;
        static Port* _output_port_4;
        std::vector<Port*> _ports;
        static Timer t_pedro;
        

    public:
        enum ports_id {OP_0_ROLL, OP_1_PITCH, OP_2_ROLL_RATE, OP_3_PITCH_RATE, OP_4_YAW_RATE};
        void process(DataMessage* t_msg, Port* t_port);
        std::vector<Port*> getPorts();
        DataMessage* runTask(DataMessage*);
        
        enum unicast_addresses {broadcast,unicast_XSens_translation,unicast_XSens_orientation,unicast_XSens_attitude_rate,unicast_XSens_yaw_rate,unicast_XSens_translation_rate, unicast_XSens_acceleration};
        void receiveMsgData(DataMessage*);
        ROSUnit_Xsens(ros::NodeHandle&);
        ~ROSUnit_Xsens();

};