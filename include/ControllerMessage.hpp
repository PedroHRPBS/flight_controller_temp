#pragma once
#include "common_srv/DataMessage.hpp"
#include <vector>
#include "PID_values.hpp"
#include "MRFT_values.hpp"
#include "BB_values.hpp"
#include "common_types.hpp"

class ControllerMessage : public DataMessage{

private:
    msg_type _type;
    PID_parameters _pid_param;
    MRFT_parameters _mrft_param;
    BB_parameters _bb_param;

public:
   
    const int getSize();
    msg_type getType();
    controller_msg_type getControllerMsgType();
    void setPIDParam(PID_parameters);
    void set_dt(float);
    void setMRFTParam(MRFT_parameters);
    void setSMParam(BB_parameters);
    MRFT_parameters getMRFTParam(){ return _mrft_param; }
    PID_parameters getPIDParam(){ return _pid_param; }
    BB_parameters getSMParam(){ return _bb_param; }

    ControllerMessage();
    ~ControllerMessage();
};