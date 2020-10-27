#include "OutputPort.hpp"

OutputPort::OutputPort(int t_id, Block* t_block) : Port(t_id, t_block){
    this->_id = t_id;
    this->_block = t_block;
}

OutputPort::~OutputPort() {

}

void OutputPort::receiveMsgData(DataMessage* t_msg){
    this->emitMsgUnicastDefault(t_msg);
}

void OutputPort::receiveMsgData(DataMessage* t_msg, int channel_id){
    this->emitMsgUnicastDefault(t_msg);
}

void OutputPort::connect(InputPort* t_input_port){
    this->addCallbackMsgReceiver((MsgReceiver*) t_input_port);
}
