#pragma once
#include "common_srv/MsgEmitter.hpp"
#include "common_srv/MsgReceiver.hpp"
#include "common_srv/Vector3D.hpp"
#include "common_srv/Vector3DMessage.hpp"
#include "common_srv/FloatMsg.hpp"
#include "RestrictedNormRefSettingsMsg.hpp"
#include "common_srv/PosesMsg.hpp"
#include <vector>
#include "Waypoint.hpp"
#include "common_srv/PosesMsg.hpp"
#include "common_srv/InputPort.hpp"
#include "common_srv/OutputPort.hpp"

class RestrictedNormWaypointRefGenerator : public MsgEmitter, public Block{

    private:
    static pthread_mutex_t lock;
    std::vector<Waypoint> Waypoints;
    double max_norm = 0.2;
    bool enabled=false;
    int old_size = 0;
    void updateControlSystemsReferences(Vector3D<double> position,double yaw);
    Port* _input_port_0;
    Port* _input_port_1;
    Port* _input_port_2;
    Port* _output_port_0;
    Port* _output_port_1;
    Port* _output_port_2;
    Port* _output_port_3;
    Port* _output_port_4;

    
    public:
    enum ports_id {IP_0_WAYPOINT, IP_1_SETTINGS, IP_2_DATA, OP_0_X, OP_1_Y, OP_2_Z, OP_3_YAW, OP_4_COUNTER};
    void process(DataMessage* t_msg, Port* t_port);
    enum unicast_addresses {x, y, z, yaw};
    RestrictedNormWaypointRefGenerator();

};