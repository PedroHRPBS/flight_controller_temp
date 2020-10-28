#pragma once
#include "common_types.hpp"
#include "common_srv/Block.hpp"

class Reference : public Block {

    private:
        block_type _type;

    public:
        block_type getType();
        virtual block_id getID() = 0;
        virtual DataMessage* runTask(DataMessage*) = 0;
        virtual reference_type getReferenceType() = 0;
        virtual void setReferenceValue(float) = 0;
        void process(DataMessage* t_msg, Port* t_port) {}

        Reference();
        ~Reference();
};