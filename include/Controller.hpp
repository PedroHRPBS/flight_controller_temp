#pragma once
#include "common_srv/Block.hpp"
#include "ControllerMessage.hpp"
#include "common_srv/FloatMsg.hpp"
#include "common_types.hpp"

class Controller : public Block{

private:
    block_type _type;

public:

    block_type getType();
    virtual void process(DataMessage* t_msg, Port* t_port) = 0;
    
    Controller();
    ~Controller();
};