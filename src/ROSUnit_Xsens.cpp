#include "ROSUnit_Xsens.hpp"
#include <iostream>
#include <fstream>
ROSUnit_Xsens* ROSUnit_Xsens::_instance_ptr = NULL;
Timer ROSUnit_Xsens::t_pedro;
ButterFilter_Xsens ROSUnit_Xsens::filter_gyro_x;
ButterFilter_Xsens ROSUnit_Xsens::filter_gyro_y;
ButterFilter_Xsens ROSUnit_Xsens::filter_gyro_z;
// std::ofstream write_data("/home/pi/gyro_data.txt"); 
// ros::WallTime start_, end_;

ROSUnit_Xsens::ROSUnit_Xsens(ros::NodeHandle& t_main_handler) : ROSUnit(t_main_handler){
    _sub_attitude = t_main_handler.subscribe("filter/quaternion", 2, callbackXsensAttitude, ros::TransportHints().tcpNoDelay());
    _sub_body_rate = t_main_handler.subscribe("imu/angular_velocity", 2, callbackXsensBodyRate, ros::TransportHints().tcpNoDelay());
    _sub_acceleration = t_main_handler.subscribe("filter/free_acceleration", 2, callbackXsensFreeAcceleration, ros::TransportHints().tcpNoDelay());
    // _sub_velocity = t_main_handler.subscribe("filter/twist", 2, callbackXsensVelocity);
    _instance_ptr = this;
    // write_data << "GyroX, GyroY, GyroZ, F_GyroX, F_GyroY, F_GyroZ, ROS_time, PI_time \n";
    // t_pedro.tick();
    // start_ = ros::WallTime::now();
}

ROSUnit_Xsens::~ROSUnit_Xsens() {

}

void ROSUnit_Xsens::callbackXsensBodyRate(const geometry_msgs::Vector3Stamped& msg_bodyrate){
    
    Vector3DMessage pv_dot_msg;
    Vector3D<double> angular_vel;
    angular_vel.x = -1 * msg_bodyrate.vector.y;
    angular_vel.y = msg_bodyrate.vector.x;
    angular_vel.z = msg_bodyrate.vector.z;
    
    // write_data << angular_vel.x << ", " << angular_vel.y << ", " << angular_vel.z << ", ";
    
    //FILTERING
    // Vector3D<double> filter_vel;
    angular_vel.x = filter_gyro_x.perform(angular_vel.x);
    angular_vel.y = filter_gyro_y.perform(angular_vel.y);
    angular_vel.z = filter_gyro_z.perform(angular_vel.z);
    // end_ = ros::WallTime::now();

    // write_data << filter_vel.x << ", " << filter_vel.y << ", " << filter_vel.z << ", " << (end_ - start_).toNSec() * 1e-6 << ", " << t_pedro.tockMicroSeconds() << "\n";


    pv_dot_msg.setVector3DMessage(angular_vel);

	//_instance_ptr->emitMsgUnicast((DataMessage*) &pv_dot_msg,(int)ROSUnit_Xsens::unicast_addresses::unicast_XSens_yaw_rate, (int)PVConcatenator::receiving_channels::ch_pv);
	//_instance_ptr->emitMsgUnicast((DataMessage*) &pv_dot_msg,(int)ROSUnit_Xsens::unicast_addresses::unicast_XSens_attitude_rate, (int)PVConcatenator::receiving_channels::ch_pv_dot);
			
}


void ROSUnit_Xsens::callbackXsensFreeAcceleration(const geometry_msgs::Vector3Stamped& msg_free_acceleration){

    Vector3DMessage pv_dot_dot_msg;
    Vector3D<double> free_acceleration;
    free_acceleration.x = -1 * msg_free_acceleration.vector.y;
    free_acceleration.y = msg_free_acceleration.vector.x;
    free_acceleration.z = msg_free_acceleration.vector.z;

    //FILTERING
    // Vector3D<double> filter_vel;
    // free_acceleration.x = filter_gyro_x.perform(free_acceleration.x);
    // free_acceleration.y = filter_gyro_y.perform(free_acceleration.y);
    // free_acceleration.z = filter_gyro_z.perform(free_acceleration.z);
    // end_ = ros::WallTime::now();

    pv_dot_dot_msg.setVector3DMessage(free_acceleration);

	//_instance_ptr->emitMsgUnicast((DataMessage*) &pv_dot_dot_msg,(int)ROSUnit_Xsens::unicast_addresses::unicast_XSens_acceleration, (int)PVConcatenator::receiving_channels::ch_pv_dot_dot);
   
}

void ROSUnit_Xsens::callbackXsensAttitude( const geometry_msgs::QuaternionStamped& msg_attitude){

    Vector3DMessage pv_msg;
    Quaternion att_data;
    att_data.x = msg_attitude.quaternion.x;
    att_data.y = msg_attitude.quaternion.y;
    att_data.z = msg_attitude.quaternion.z;
    att_data.w = msg_attitude.quaternion.w;

//Convert Quaternion to euler
//TODO move to outside ROSUnit
    Vector3D<float> _euler;

    double sinr_cosp = +2.0 * ( att_data.w * att_data.x + att_data.y * att_data.z);
    double cosr_cosp = +1.0 - 2.0 * (att_data.x * att_data.x + att_data.y * att_data.y);
    _euler.x = atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = +2.0 * ( att_data.w * att_data.y - att_data.z * att_data.x);
    if (fabs(sinp) >= 1)
        _euler.y = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        _euler.y = asin(sinp);

    // yaw (z-axis rotation)
    double siny_cosp = +2.0 * ( att_data.w * att_data.z + att_data.x * att_data.y);
    double cosy_cosp = +1.0 - 2.0 * (att_data.y * att_data.y + att_data.z * att_data.z );  
    _euler.z= atan2(siny_cosp, cosy_cosp);


    Vector3D<double> orientation_euler;
    orientation_euler.x = -1 * _euler.y;
    orientation_euler.y = _euler.x; //Arranging the frames to match with the drone's
    orientation_euler.z = _euler.z;


    pv_msg.setVector3DMessage(orientation_euler);
	//_instance_ptr->emitMsgUnicast((DataMessage*) &pv_msg,(int)ROSUnit_Xsens::unicast_addresses::unicast_XSens_orientation, (int)PVConcatenator::receiving_channels::ch_pv);
		
}
void ROSUnit_Xsens::callbackXsensVelocity(const geometry_msgs::TwistStamped& msg_velocity){
 
    Vector3D<double> velocity;
	Vector3DMessage velocity_msg;
    velocity.x = msg_velocity.twist.linear.x;
    velocity.y = msg_velocity.twist.linear.y;
    velocity.z = msg_velocity.twist.linear.z;
	velocity_msg.setVector3DMessage(velocity);


	_instance_ptr->emitMsgUnicast(&velocity_msg,(int)ROSUnit_Xsens::unicast_addresses::unicast_XSens_translation_rate,(int)Global2Inertial::receiving_channels::ch_XSens_vel);
		
}

void ROSUnit_Xsens::receiveMsgData(DataMessage*){

}