//GLOBAL2INERTIAL V1.0.1
// 18 June 2020
// Pedro Henrique Silva

#include "Global2Inertial.hpp"
#define DEBUG_HR_LR_DECOUPLED

Global2Inertial::Global2Inertial() {

    _input_port_0 = new InputPort(ports_id::IP_0_OPTI_MSG, this);
    _input_port_1 = new InputPort(ports_id::IP_1_FLOAT_DATA, this);
    _input_port_2 = new InputPort(ports_id::IP_2_RTK_POS, this);
    _input_port_3 = new InputPort(ports_id::IP_3_RTK_XSESN_POS, this);
    _input_port_4 = new InputPort(ports_id::IP_4_RTK_XSESN_VEL, this);
    _input_port_5 = new InputPort(ports_id::IP_5_RTK_XSESN_ORI, this);
    _input_port_6 = new InputPort(ports_id::IP_6_CAM_Z, this);
    _input_port_7 = new InputPort(ports_id::IP_7_CAM_EN, this);
    _output_port_0 = new OutputPort(ports_id::OP_0_OPTIPOS, this);
    _output_port_1 = new OutputPort(ports_id::OP_1_OPTIHEADING, this);
    _output_port_2 = new OutputPort(ports_id::OP_2_RTK_POS_WP, this);
    _output_port_3 = new OutputPort(ports_id::OP_3_XSENS_POS, this);
    _output_port_4 = new OutputPort(ports_id::OP_4_XSENS_VEL, this);
    _output_port_5 = new OutputPort(ports_id::OP_5_XSENS_ORI, this);
    _ports = {_input_port_0, _input_port_1, _input_port_2, _input_port_3, _input_port_4, _input_port_5, _input_port_6, _input_port_7, 
             _output_port_0, _output_port_1, _output_port_2, _output_port_3, _output_port_4,  _output_port_5};

    //TODO: Ensure altitude is calibrated
    calib_point1.x = 0.0;
    calib_point1.y = 0.0;
    calib_point1.z = 0.0; //No need to correct height here. Flight Scenario is responsible for that.

    calib_point2.x = 0.0; //defines x(+) axis
    calib_point2.y = -0.80;
    calib_point2.z = 0.0;

    calib_point3.x = 24.44814808; //correcting x offset in y
    calib_point3.y = 54.39666318;
    calib_point3.z = 0.0;

    calib_point3_true_SI.x = 0; //Take this using hand measurement. this would scale y with no effect on x. in meters
    calib_point3_true_SI.y = 6;
    calib_point3_true_SI.z = 0.0;

    calib_point4.x = 24.44814808; //This is a validation point
    calib_point4.y = 54.39666318;
    calib_point4.z = 0.0;

    calib_point4_true_SI.x = 24; //Take this using hand measurement. For validation
    calib_point4_true_SI.y = 6 ;
    calib_point4_true_SI.z = 0.0;

    // calib_point1.x=0;
    // calib_point1.y=0;
    // calib_point1.z=0;
    // calib_point2.x=0;
    // calib_point2.y=0;
    // calib_point2.z=0;
    //calibrated_reference_inertial_heading=180.*(M_PI/180.);
    calibrated_reference_inertial_heading=-90.*(M_PI/180.);
    Vector3D<double> calib_points_diff = calib_point2 - calib_point1;
    calibrated_global_to_inertial_angle = atan2(calib_points_diff.y, calib_points_diff.x);

    Vector3D<double> results = changeLLAtoMeters(calib_point1, calib_point3); // Compute homogenueity calibration terms
    Vector3D<double> results_elev=offsetElevation(results,-calib_point1.z);
    Vector3D<double> calib_point3_meas_SI=rotatePoint(results_elev);

    x_off_calib_slope=-1.0*(calib_point3_meas_SI.x/calib_point3_meas_SI.y);
    y_scale_coeff=calib_point3_true_SI.y/calib_point3_meas_SI.y;

    //Validate calibration
    results = changeLLAtoMeters(calib_point1, calib_point4); //
    results_elev=offsetElevation(results,-calib_point1.z);
    Vector3D<double> calib_point4_meas_SI=rotatePoint(results_elev);
    double error_calib_homo=sqrt(pow((calib_point4_meas_SI.x-calib_point4_true_SI.x), 2)+pow((calib_point4_meas_SI.y-calib_point4_true_SI.y), 2)+pow((calib_point4_meas_SI.z-calib_point4_true_SI.z), 2));
    std::cout << "*** Homogenuity calibration results: ***\n";
    std::cout << "error_calib_homo: "<<error_calib_homo <<"\n";
    std::cout << "Is within tolerance?: "<< (error_calib_homo<error_calib_homo_tol) <<"\n";
    std::cout << "****************************************\n";
    antenna_pose.x=0.;
    antenna_pose.y=0.1;
    antenna_pose.z=0.1;
}
void Global2Inertial::process(DataMessage* t_msg, Port* t_port)
{
    if(t_port->getID() == ports_id::IP_0_OPTI_MSG){ 
        
        OptitrackMessage* opti_msg = ((OptitrackMessage*)t_msg);
        Vector3D<double> pos_point = opti_msg->getPosition();
        Quaternion _bodyAtt = opti_msg->getAttitudeHeading();
        Vector3D<double> att_vec = getEulerfromQuaternion(_bodyAtt);
        Vector3D<double> translate_pos = this->translatePoint(pos_point);
        Vector3D<double> result_pos = this->rotatePoint(translate_pos);
      
        Vector3DMessage results_msg;
        results_msg.setVector3DMessage(result_pos);

        Vector3DMessage yaw_msg;
        att_vec.z -= calibrated_reference_inertial_heading;
        yaw_msg.setVector3DMessage(att_vec);
        
        this->_output_port_0->receiveMsgData(&results_msg);
        this->_output_port_1->receiveMsgData(&yaw_msg);
    
    }
    else if (t_port->getID() == ports_id::IP_1_FLOAT_DATA){
        calib_point1.z = ((FloatMsg*)t_msg)->data;
        std::cout << "NEW HEIGHT OFFSET = " << calib_point1.z << std::endl;
    }
    else if (t_port->getID() == ports_id::IP_2_RTK_POS){
        Vector3D<double> results = changeLLAtoMeters(calib_point1, ((Vector3DMessage*)t_msg)->getData()); 
        Vector3D<double> results_elev=offsetElevation(results,-calib_point1.z);
        Vector3D<double> results_rot=rotatePoint(results_elev);
        Vector3DMessage res_msg;
        res_msg.setVector3DMessage(results_rot);
        #ifndef DEBUG_HR_LR_DECOUPLED
        emitMsgUnicast(&res_msg,Global2Inertial::unicast_addresses::uni_RTK_pos);
        #else
        this->_output_port_2->receiveMsgData(&res_msg);
        #endif
    }
    else if (t_port->getID() == ports_id::IP_3_RTK_XSESN_POS){
        Vector3D<double> results = changeLLAtoMeters(calib_point1,((Vector3DMessage*)t_msg)->getData());
        Vector3D<double> results_rot=rotatePoint(results);
        Vector3DMessage res_msg;
        res_msg.setVector3DMessage(results_rot);
        #ifdef RTK
        emitMsgUnicast(&res_msg,Global2Inertial::unicast_addresses::uni_XSens_pos);
        #else
        this->_output_port_3->receiveMsgData(&res_msg);
        #endif
    }
    else if (t_port->getID() == ports_id::IP_4_RTK_XSESN_VEL){
        Vector3D<double> results = transformVelocity(((Vector3DMessage*)t_msg)->getData());
        Vector3DMessage res_msg;
        res_msg.setVector3DMessage(results);
        this->_output_port_4->receiveMsgData(&res_msg);
    }
    else if (t_port->getID() == ports_id::IP_5_RTK_XSESN_ORI){
        Vector3D<double> results = ((Vector3DMessage*)t_msg)->getData();
        results.z = results.z - calibrated_reference_inertial_heading - calibrated_global_to_inertial_angle;

        Vector3DMessage res_msg;
        res_msg.setVector3DMessage(results);
        this->_output_port_5->receiveMsgData(&res_msg);
    }
}

Vector3D<double> Global2Inertial::translatePoint(Vector3D<double> t_input_point){
        

    Vector3D<double> origin = calib_point1;
    Vector3D<double> calibrated_input_point;
    
    calibrated_input_point = t_input_point - origin;
    
    return calibrated_input_point;
}

Vector3D<double> Global2Inertial::offsetElevation(Vector3D<double> t_input,double elev_offset){
    Vector3D<double> t_res;
    t_res=t_input;
    t_res.z=t_res.z+elev_offset;
    return t_res;
}

Vector3D<double> Global2Inertial::rotatePoint(Vector3D<double> t_input_point){
        
    Vector3D<double> euler_calib;
    euler_calib.z = calibrated_global_to_inertial_angle;
    Vector3D<double> calibrated_input_point;
    RotationMatrix3by3 G_I_rot_matrix;

    G_I_rot_matrix.Update(euler_calib);

    //calibrated_input_point = G_I_rot_matrix.TransformVector(calibrated_input_point);
 
    G_I_rot_matrix.Transpose();
    calibrated_input_point = G_I_rot_matrix.TransformVector(t_input_point);

    return calibrated_input_point;
}

Vector3D<double> Global2Inertial::transformVelocity(Vector3D<double> t_input_point){
    RotationMatrix3by3 G_I_rot_matrix;
    Vector3D<double> euler_calib;
    euler_calib.z = calibrated_global_to_inertial_angle;
    G_I_rot_matrix.Update(euler_calib);
    G_I_rot_matrix.Transpose();
    return G_I_rot_matrix.TransformVector(t_input_point);
}

Vector3D<double> Global2Inertial::getEulerfromQuaternion(Quaternion q){

    // roll (x-axis rotation)
    double sinr_cosp = +2.0 * (q.w * q.x + q.y * q.z);
    double cosr_cosp = +1.0 - 2.0 * (q.x * q.x + q.y * q.y);
    Vector3D<float> _euler;
    _euler.x = atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = +2.0 * (q.w * q.y - q.z * q.x);
    if (fabs(sinp) >= 1)
        _euler.y = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        _euler.y = asin(sinp);

    // yaw (z-axis rotation)
    double siny_cosp = +2.0 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = +1.0 - 2.0 * (q.y * q.y + q.z * q.z);  
    _euler.z = atan2(siny_cosp, cosy_cosp);

    return _euler;
}

Vector3D<double> Global2Inertial::changeLLAtoMeters(Vector3D<double> t_origin,Vector3D<double> t_input2){
    //StackOverflow: "How to convert latitude or longitude to meters". Answer by JJones
    Vector3D<double> res;
    double latMid,m_per_deg_lat,m_per_deg_long,deltaLat,deltaLong,dist_m;
    latMid=(t_origin.x+t_input2.x)/2.;
    m_per_deg_lat=111132.954-559.822 * cos(2.*latMid)+1.175*cos(4.0*latMid);
    m_per_deg_long=(M_PI/180.)*6367449.0*cos(latMid);
    deltaLat=t_input2.x-t_origin.x;
    deltaLong=t_input2.y-t_origin.y;
    res.y=deltaLat*m_per_deg_lat;
    res.x=deltaLong*m_per_deg_long;
    res.z=t_input2.z;
    
    return res;
}

Vector3D<double> Global2Inertial::correctNonHomogeneousSpace(Vector3D<double> t_uncorr_pt){
    Vector3D<double> res;
    res.x=t_uncorr_pt.x+x_off_calib_slope*t_uncorr_pt.y;
    res.y=t_uncorr_pt.y*y_scale_coeff;
    res.z=t_uncorr_pt.z;
    return res;
}