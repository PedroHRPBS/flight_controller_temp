#pragma once
#include <stdint.h>
#include "common_srv/common_types.hpp"


class MRFT_bag {

public:

	float amplitude;
    uint32_t duration;
	   
};

class MRFT_parameters {

public:

	float beta=0;
	float relay_amp=0;
	float bias=0;
	block_id id = block_id::NULL_ID;
	   
};

class MRFT_data {
	
public:
	float err;
	bool mrft_bag_ready_para;
	MRFT_bag mrft_period;
};
