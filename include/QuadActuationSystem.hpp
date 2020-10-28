#pragma once
#include "ActuationSystem.hpp"
#include "common_srv/BooleanMsg.hpp"
#include <pthread.h>
#include "ROSUnit_BroadcastData.hpp"
#include "common_srv/VectorDoubleMsg.hpp"
#include <vector>
#include "common_srv/InputPort.hpp"
#include "common_srv/OutputPort.hpp"

// GEOMETRY
//      CW(1) (3)CCW                x
//          \ /                     ↑
//           X                      |
//          / \              y <----+ 
//     CCW(4) (2)CW               z up
//
// For Positive Roll, all motors with negative X should be increased
// For Positive Pitch, all motors with negative Y should be increased
// For Positive Yaw, all motors with CW should be increased
// Mx = [x, y, direction, thottle]
// POSITIVE PITCH result in moving in the direction of POSITIVE Y
// POSITIVE ROLL result in moving in the direction of POSITIVE X

const int NUM_MOTORS = 4;

class QuadActuationSystem : public ActuationSystem {

private:    
    std::vector<Actuator*> _actuators;
    const int _escMin = 1000;
    const int _escMin_armed = 1150;
    const int _escMax = 2000;
    bool _armed = false;
    float _u[4]; //[roll, pitch, yaw, throttle]
    std::vector<double> _commands {0,0,0,0,0,0}; //Here we keep commands as size 6 just to make the code generic for both configurations.
    float _geometry[4][4] = {{-0.707107, -0.707107,  1, 1},
                             { 0.707107,  0.707107,  1, 1},
                             {-0.707107,  0.707107, -1, 1},
                             { 0.707107, -0.707107, -1, 1}};
    static pthread_mutex_t lock;

    Port* _input_port_0;
    Port* _input_port_1;
    Port* _input_port_2;
    Port* _input_port_3;
    Port* _output_port_0;
    Port* _output_port_1;

public:
    enum ports_id {IP_0_DATA_ROLL, IP_1_DATA_PITCH, IP_2_DATA_YAW, IP_3_DATA_Z, OP_0_CMD, OP_1_ARM};
    void process(DataMessage* t_msg, Port* t_port);
    DataMessage* runTask(DataMessage*); 
    std::vector<Port*> getPorts();
    enum unicast_addresses {broadcast, unicast_ActuationSystem_commands, unicast_ActuationSystem_armed};
    enum receiving_channels {ch_roll=0, ch_pitch=1, ch_yaw=2, ch_throttle=3};
    void receiveMsgData(DataMessage* t_msg);
    void receiveMsgData(DataMessage* t_msg, int t_channel);
    void command();
    int constrain(float value, int min_value, int max_value);

    QuadActuationSystem(std::vector<Actuator*>);
    ~QuadActuationSystem();
};