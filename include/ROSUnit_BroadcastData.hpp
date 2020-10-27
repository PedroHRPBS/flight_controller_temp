#pragma once
#include "common_srv/ROSUnit.hpp"
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Float64MultiArray.h>
#include <flight_controller/Info.h>
#include <vector>
#include "common_srv/Vector3DMessage.hpp"
#include "common_srv/FloatMsg.hpp"
#include "common_srv/IntegerMsg.hpp"
#include "common_srv/BooleanMsg.hpp"
#include "common_srv/VectorDoubleMsg.hpp"
#include <InputPort.hpp>

class ROSUnit_BroadcastData : public ROSUnit{

private:
    ros::Publisher _pos_prov_pub;
    ros::Publisher _ori_prov_pub;
    ros::Publisher _cs_prov_pub;
    ros::Publisher _csr_prov_pub;
    ros::Publisher _act_prov_pub;
    ros::Publisher _info_prov_pub;
    ros::Publisher _error_prov_pub;
    bool roll_received = false, pitch_received = false, yaw_received = false,
         x_received = false, y_received = false, z_received = false,
         armed_received = false;
    bool _armed = false;
    float _voltage = -1.0;
    int _number_of_waypoints = 0;
    int _error_accumulator = 0;
    static ROSUnit_BroadcastData* _instance_ptr;
    void receiveMsgData(DataMessage* t_msg); 
    void receiveMsgData(DataMessage* t_msg, int);
    int _seq_pos = 0, _seq_ori = 0, _seq_xpv = 0, _seq_ypv = 0, _seq_zpv = 0;
    int _seq_rollpv = 0, _seq_pitchpv = 0, _seq_yawpv = 0, _seq_cs = 0, _seq_act = 0;
    int _seq_yawratepv = 0, _seq_info = 0;
    std::vector<double> _cs_outputs{ 0, 0, 0, 0, 0, 0, 0 }; 
    std::vector<double> _cs_references{ 0, 0, 0, 0, 0, 0, 0 };
    std::vector<double> _act_outputs{ 0, 0, 0, 0, 0, 0 }; 
    Vector3D<double> _position;
    Vector3D<double> _att;
    double _head;
    
    Port* _input_port_0;
    Port* _input_port_1;
    Port* _input_port_2;
    Port* _input_port_3;
    Port* _input_port_4;
    Port* _input_port_5;
    Port* _input_port_6;
    Port* _input_port_7;
    Port* _input_port_8;
    Port* _input_port_9;
    Port* _input_port_10;
    Port* _input_port_11;
    Port* _input_port_12;
    Port* _input_port_13;
    Port* _input_port_14;
    Port* _input_port_15;
    Port* _input_port_16;
    Port* _input_port_17;
    Port* _input_port_18;
    Port* _input_port_19;

    std::vector<Port*> _ports;

public:

    enum ports_id {IP_0_X_OUTPUT, IP_1_Y_OUTPUT, IP_2_Z_OUTPUT, IP_3_ROLL_OUTPUT, IP_4_PITCH_OUTPUT, IP_5_YAW_OUTPUT, IP_6_YAWRATE_OUTPUT,
                    IP_7_X_REF, IP_8_Y_REF, IP_9_Z_REF, IP_10_ROLL_REF, IP_11_PITCH_REF, IP_12_YAW_REF, IP_13_YAWRATE_REF,
                    IP_14_MOTORS, IP_15_ARMED};
    void process(DataMessage* t_msg, Port* t_port);
    std::vector<Port*> getPorts();

    enum ros_broadcast_channels {broadcast, x, y, z, roll, pitch, yaw, yaw_rate, actuation, armed, control_outputs, references, waypoints, error};
    ROSUnit_BroadcastData(ros::NodeHandle&);
    ~ROSUnit_BroadcastData();

    
};