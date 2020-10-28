#include "Reference.hpp"

Reference::Reference(){
    _type = block_type::reference;
}

Reference::~Reference() {

}

block_type Reference::getType() {
    return _type;
}