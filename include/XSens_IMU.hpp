#pragma once
#include "Roll_PVProvider.hpp"
#include "Pitch_PVProvider.hpp"
#include "AttitudeMsg.hpp"
#include "BodyRateMsg.hpp"
#include "QuaternionMessage.hpp"
#include "Vector3DMessage.hpp"
#include "thread_terminal_unit.hpp"
#include "MsgEmitter.hpp"
class XSens_IMU : public Roll_PVProvider, public Pitch_PVProvider, public msg_receiver{

private:
    Vector3D<float> _bodyrate;
    Vector3D<float> last_euler_angles;

public:

    void receive_msg_data(DataMessage* t_msg);
    AttitudeMsg getAttitude();
    Vector3D<float> getBodyRate();
    Vector3D<float> getEulerfromQuaternion(Quaternion);
    XSens_IMU();
    ~XSens_IMU();
};