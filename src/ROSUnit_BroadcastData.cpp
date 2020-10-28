#include "ROSUnit_BroadcastData.hpp"
ROSUnit_BroadcastData* ROSUnit_BroadcastData::_instance_ptr = NULL;

ROSUnit_BroadcastData::ROSUnit_BroadcastData(ros::NodeHandle& t_main_handler) : ROSUnit(t_main_handler){

    // _pos_prov_pub = t_main_handler.advertise<geometry_msgs::Point>("uav_control/uav_position", 2);
    // _ori_prov_pub = t_main_handler.advertise<geometry_msgs::Point>("uav_control/uav_orientation", 2);
    _cs_prov_pub = t_main_handler.advertise<std_msgs::Float64MultiArray>("control_system_output", 2);
    _csr_prov_pub = t_main_handler.advertise<std_msgs::Float64MultiArray>("control_system_reference", 2);
    _act_prov_pub = t_main_handler.advertise<std_msgs::Float64MultiArray>("actuation_output", 2);
    _info_prov_pub = t_main_handler.advertise<flight_controller::Info>("info", 2);
    _error_prov_pub = t_main_handler.advertise<geometry_msgs::PointStamped>("error", 2);

    _att.x = 0;
    _head = 0;

    _instance_ptr = this;

    _input_port_0 = new InputPort(ports_id::IP_0_X_OUTPUT, this);
    _input_port_1 = new InputPort(ports_id::IP_1_Y_OUTPUT, this);
    _input_port_2 = new InputPort(ports_id::IP_2_Z_OUTPUT, this);
    _input_port_3 = new InputPort(ports_id::IP_3_ROLL_OUTPUT, this);
    _input_port_4 = new InputPort(ports_id::IP_4_PITCH_OUTPUT, this);
    _input_port_5 = new InputPort(ports_id::IP_5_YAW_OUTPUT, this);
    _input_port_6 = new InputPort(ports_id::IP_6_YAWRATE_OUTPUT, this);

    _input_port_7 = new InputPort(ports_id::IP_7_X_REF, this);
    _input_port_8 = new InputPort(ports_id::IP_8_Y_REF, this);
    _input_port_9 = new InputPort(ports_id::IP_9_Z_REF, this);
    _input_port_10 = new InputPort(ports_id::IP_10_ROLL_REF, this);
    _input_port_11 = new InputPort(ports_id::IP_11_PITCH_REF, this);
    _input_port_12 = new InputPort(ports_id::IP_12_YAW_REF, this);
    _input_port_13 = new InputPort(ports_id::IP_13_YAWRATE_REF, this);

    _input_port_14 = new InputPort(ports_id::IP_14_MOTORS, this);
    _input_port_15 = new InputPort(ports_id::IP_15_ARMED, this);


    _ports = {_input_port_0, _input_port_1, _input_port_2, _input_port_3, _input_port_4, _input_port_5, _input_port_6,
              _input_port_7, _input_port_8, _input_port_9, _input_port_10, _input_port_11, _input_port_12, _input_port_13,
              _input_port_14, _input_port_15};
}

ROSUnit_BroadcastData::~ROSUnit_BroadcastData() {

}

void ROSUnit_BroadcastData::process(DataMessage* t_msg, Port* t_port) {
    
    if(t_port->getID() == ports_id::IP_0_X_OUTPUT){
        FloatMsg* float_msg = (FloatMsg*)t_msg;
        _cs_outputs[0] = (double)float_msg->data;
    } else if(t_port->getID() == ports_id::IP_1_Y_OUTPUT){
        FloatMsg* float_msg = (FloatMsg*)t_msg;
        _cs_outputs[1] = (double)float_msg->data;
    } else if(t_port->getID() == ports_id::IP_2_Z_OUTPUT){
        FloatMsg* float_msg = (FloatMsg*)t_msg;
        _cs_outputs[2] = (double)float_msg->data;
    } else if(t_port->getID() == ports_id::IP_3_ROLL_OUTPUT){
        FloatMsg* float_msg = (FloatMsg*)t_msg;
        _cs_outputs[3] = (double)float_msg->data;
    } else if(t_port->getID() == ports_id::IP_4_PITCH_OUTPUT){
        FloatMsg* float_msg = (FloatMsg*)t_msg;
        _cs_outputs[4] = (double)float_msg->data;
        std_msgs::Float64MultiArray msg;
        msg.data = _cs_outputs;
        _cs_prov_pub.publish(msg);
    } else if(t_port->getID() == ports_id::IP_5_YAW_OUTPUT){
        FloatMsg* float_msg = (FloatMsg*)t_msg;
        _cs_outputs[5] = (double)float_msg->data;
    } else if(t_port->getID() == ports_id::IP_6_YAWRATE_OUTPUT){
        FloatMsg* float_msg = (FloatMsg*)t_msg;
        _cs_outputs[6] = (double)float_msg->data;
    } else if(t_port->getID() == ports_id::IP_7_X_REF){
        FloatMsg* float_msg = (FloatMsg*)t_msg;
        _cs_references[0] = (double)float_msg->data;
    } else if(t_port->getID() == ports_id::IP_8_Y_REF){
        FloatMsg* float_msg = (FloatMsg*)t_msg;
        _cs_references[1] = (double)float_msg->data;
    } else if(t_port->getID() == ports_id::IP_9_Z_REF){
        FloatMsg* float_msg = (FloatMsg*)t_msg;
        _cs_references[2] = (double)float_msg->data;
    } else if(t_port->getID() == ports_id::IP_10_ROLL_REF){
        FloatMsg* float_msg = (FloatMsg*)t_msg;
        _cs_references[3] = (double)float_msg->data;
    } else if(t_port->getID() == ports_id::IP_11_PITCH_REF){
        FloatMsg* float_msg = (FloatMsg*)t_msg;
        _cs_references[4] = (double)float_msg->data;
        std_msgs::Float64MultiArray msg;
        msg.data = _cs_references;
        _csr_prov_pub.publish(msg);
    } else if(t_port->getID() == ports_id::IP_12_YAW_REF){
        FloatMsg* float_msg = (FloatMsg*)t_msg;
        _cs_references[5] = (double)float_msg->data;
    } else if(t_port->getID() == ports_id::IP_13_YAWRATE_REF){
        FloatMsg* float_msg = (FloatMsg*)t_msg;
        _cs_references[6] = (double)float_msg->data;
    } else if(t_port->getID() == ports_id::IP_14_MOTORS){
        VectorDoubleMsg* vector_double_msg = (VectorDoubleMsg*)t_msg;
        _act_outputs = vector_double_msg->data;
        std_msgs::Float64MultiArray msg;
        msg.data = _act_outputs;
        _act_prov_pub.publish(msg);
    } else if(t_port->getID() == ports_id::IP_15_ARMED){
        BooleanMsg* armed_msg = (BooleanMsg*)t_msg;
        _armed = armed_msg->data;
        flight_controller::Info msg;
        msg.header.seq = ++_seq_info;
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = "";
        msg.armed = _armed;
        msg.battery_voltage = _voltage;
        _info_prov_pub.publish(msg);
    }
}

void ROSUnit_BroadcastData::receiveMsgData(DataMessage* t_msg){
    if(t_msg->getType() == msg_type::FLOAT){
        FloatMsg* voltage_msg = (FloatMsg*)t_msg;
        _voltage = voltage_msg->data;
    }
}

void ROSUnit_BroadcastData::receiveMsgData(DataMessage* t_msg, int t_channel){

    if(t_msg->getType() == msg_type::VECTOR3D){
        Vector3DMessage* vector3d_msg = (Vector3DMessage*)t_msg;
        if(t_channel == (int)ros_broadcast_channels::x){
            Vector3D<float> xpv = vector3d_msg->getData();
            _position.x = xpv.x;
            x_received = true;

        }else if(t_channel == (int)ros_broadcast_channels::y){
            Vector3D<float> ypv = vector3d_msg->getData();
            _position.y = ypv.x;
            y_received = true;

        }else if(t_channel == (int)ros_broadcast_channels::z){
            Vector3D<float> zpv = vector3d_msg->getData();
            _position.z = zpv.x;
            z_received = true;

        }else if(t_channel == (int)ros_broadcast_channels::yaw){
            Vector3D<float> yawpv = vector3d_msg->getData();
            _head = yawpv.x;
            yaw_received = true;

        }else if(t_channel == (int)ros_broadcast_channels::roll){
            Vector3D<float> rollpv = vector3d_msg->getData();
            _att.x = rollpv.x;
            roll_received = true;

        }else if(t_channel == (int)ros_broadcast_channels::pitch){
            Vector3D<float> pitchpv = vector3d_msg->getData();
            _att.y = pitchpv.x;
            pitch_received = true;
            
        }
    }else if(t_msg->getType() == msg_type::VECTORDOUBLE){
        // VectorDoubleMsg* vector_double_msg = (VectorDoubleMsg*)t_msg;
        
        // if(t_channel == (int)ros_broadcast_channels::actuation){
        //     _act_outputs = vector_double_msg->data;
            
        //     std_msgs::Float64MultiArray msg;
        //     msg.data = _act_outputs;
        //     _act_prov_pub.publish(msg);

        // }else if(t_channel == (int)ros_broadcast_channels::references){
        //      int i = (int)(vector_double_msg->data[0]);
        //     _cs_references[i] = vector_double_msg->data[1];

        //     std_msgs::Float64MultiArray msg;
        //     msg.data = _cs_references;
        //     if(i == 4){
        //         _csr_prov_pub.publish(msg);
        //     }

        // }else if(t_channel == (int)ros_broadcast_channels::control_outputs){
        //     int i = (int)(vector_double_msg->data[0]);
        //     _cs_outputs[i] = vector_double_msg->data[1];

        //     std_msgs::Float64MultiArray msg;
        //     msg.data = _cs_outputs;
        //     if(i == 4){
        //         _cs_prov_pub.publish(msg);
        //     }
        // }
    }else if(t_msg->getType() == msg_type::INTEGER){
        IntegerMsg* integer_msg = (IntegerMsg*)t_msg;
        if(t_channel == ros_broadcast_channels::error){
            _error_accumulator += integer_msg->data;
            geometry_msgs::PointStamped msg;
            msg.header.seq = ++_seq_xpv;
            msg.header.stamp = ros::Time::now();
            msg.header.frame_id = "";
            msg.point.x = _error_accumulator;
            msg.point.y = 0.;
            msg.point.z = 0.;
            _error_prov_pub.publish(msg);
        }
    }else if(t_msg->getType() == msg_type::BOOLEAN){
        // BooleanMsg* armed_msg = (BooleanMsg*)t_msg;
        // _armed = armed_msg->data;
        // flight_controller::Info msg;
        // msg.header.seq = ++_seq_info;
        // msg.header.stamp = ros::Time::now();
        // msg.header.frame_id = "";
        // msg.armed = _armed;
        // msg.battery_voltage = _voltage;
        // _info_prov_pub.publish(msg);

    }

    if(x_received && y_received && z_received){
        geometry_msgs::Point msg;
        msg.x = _position.x;
        msg.y = _position.y;
        msg.z = _position.z;
        _pos_prov_pub.publish(msg);
        x_received = false;
        y_received = false;
        z_received = false;
    }

    if(roll_received && pitch_received && yaw_received){
        geometry_msgs::Point msg;
        msg.x = _att.x;
        msg.y = _att.y;
        msg.z = _head;
        _ori_prov_pub.publish(msg);
        roll_received = false;
        pitch_received = false;
        yaw_received = false;
    }

}