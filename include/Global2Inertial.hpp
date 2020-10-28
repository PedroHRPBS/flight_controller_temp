// Version: 1.1
// Author: Mohamad Chehadeh
// Date: 26 Jan 2020
// Revision Note: Moved from Outdoor Navigation to UAV Control
#pragma once

#include "common_srv/Block.hpp"
#include "common_srv/Vector3D.hpp"
#include "common_srv/InputPort.hpp"
#include "common_srv/OutputPort.hpp"
#include "RotationMatrix3by3.hpp"
#include <math.h>
#include "common_srv/FloatMsg.hpp"
#include "common_srv/IntegerMsg.hpp"
#include "common_srv/Vector3DMessage.hpp"
#include "OptitrackMessage.hpp"

class Global2Inertial : public Block {
private:

    Port* _input_port_0;
    Port* _input_port_1;
    Port* _input_port_2;
    Port* _input_port_3;
    Port* _input_port_4;
    Port* _input_port_5;
    Port* _input_port_6;
    Port* _input_port_7;
    Port* _output_port_0;
    Port* _output_port_1;
    Port* _output_port_2;
    Port* _output_port_3;
    Port* _output_port_4;
    Port* _output_port_5;

    Vector3D<double> calib_point1, calib_point2,calib_point3,calib_point4,calib_point3_true_SI,calib_point4_true_SI;
    const double Earth_R=6371000.;
    const double error_calib_homo_tol=0.2;
    double calibrated_global_to_inertial_angle;
    double calibrated_reference_inertial_heading;//TODO: This needs to be moved to XSens node
    double x_off_calib_slope,y_scale_coeff;
    int _camera_enabled = 0;
    double _camera_z, _camera_bias;

    Vector3D<double> antenna_pose;
    Vector3D<double> last_known_orientation;
    Vector3D<double> changeLLAtoMeters(Vector3D<double>,Vector3D<double>);
    Vector3D<double> translatePoint(Vector3D<double>);
    Vector3D<double> rotatePoint(Vector3D<double>);
    Vector3D<double> transformVelocity(Vector3D<double>);
    Vector3D<double> getEulerfromQuaternion(Quaternion);
    Vector3D<double> offsetElevation(Vector3D<double>,double);
    Vector3D<double> correctNonHomogeneousSpace(Vector3D<double> t_uncorr_pt);
public:
    enum ports_id {IP_0_OPTI_MSG, IP_1_FLOAT_DATA, IP_2_RTK_POS, IP_3_RTK_XSESN_POS,
                    IP_4_RTK_XSESN_VEL, IP_5_RTK_XSESN_ORI, IP_6_CAM_Z, IP_7_CAM_EN,
                    OP_0_OPTIPOS, OP_1_OPTIHEADING, OP_2_RTK_POS_WP, OP_3_XSENS_POS,
                    OP_4_XSENS_VEL, OP_5_XSENS_ORI};

    enum unicast_addresses {broadcast,uni_RTK_pos_pv,uni_RTK_pos_wp,uni_XSens_pos,uni_Optitrack_pos,uni_Optitrack_heading,uni_XSens_vel,uni_XSens_ori};
    enum receiving_channels {ch_broadcast,ch_RTK_pos,ch_XSens_pos,ch_Optitrack_pos,ch_XSens_vel, ch_XSens_ori, ch_Camera};
    Global2Inertial();

    void process(DataMessage* t_msg, Port* t_port);

};
