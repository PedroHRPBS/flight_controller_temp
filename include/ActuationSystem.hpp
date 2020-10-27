#pragma once

#include "common_srv/MsgEmitter.hpp"
#include "common_srv/MsgReceiver.hpp"
#include "Actuator.hpp"
#include <vector>
#include "common_srv/Block.hpp"

class ActuationSystem : public Block{

public:

    virtual void receiveMsgData(DataMessage* t_msg) = 0;
    
    ActuationSystem(std::vector<Actuator*>) {};

    //TODO Refactor below
    block_id getID() {}
    block_type getType() {}
    void switchIn(DataMessage*) {}
    DataMessage* switchOut() {}
    DataMessage* runTask(DataMessage*) {}
    void process(DataMessage* t_msg, Port* t_port) {}
};