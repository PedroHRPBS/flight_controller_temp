#pragma once

#include "common_srv/MsgEmitter.hpp"
#include "common_srv/MsgReceiver.hpp"
#include "Actuator.hpp"
#include <vector>
#include "common_srv/Block.hpp"

class ActuationSystem : public Block{

public:
    
    ActuationSystem(std::vector<Actuator*>) {};
    void process(DataMessage* t_msg, Port* t_port) {}
};