#pragma once
#include "DataMessage.hpp"

class UserReferenceMessage : public DataMessage{

private:
    float _x, _y, _z, _yaw;
    msg_type _type;

public:

    float getX();
    float getY();
    float getZ();
    float getYaw();
    msg_type getType();
    const int getSize();
    void setUserMessage(float, float, float, float);
    
    UserReferenceMessage(float,float,float,float);
    UserReferenceMessage();
    ~UserReferenceMessage();
};