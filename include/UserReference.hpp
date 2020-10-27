#pragma once
#include "common_types.hpp"
#include "common_srv/MsgEmitter.hpp"
#include "common_srv/MsgReceiver.hpp"

class UserReference : public MsgEmitter, public MsgReceiver{

private:


public:
    enum unicast_addresses {broadcast, x, y, z, yaw};
    virtual void receiveMsgData(DataMessage*) = 0;

    UserReference();
    ~UserReference();
};