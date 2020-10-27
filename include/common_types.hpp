#pragma once
#include <stdint.h>
#include "common_srv/Timer.hpp"

const struct tPacketProp{
	uint8_t pad_len = 3;
	uint8_t hdr_len = 2;
	uint8_t crc_len = 2;
	uint8_t pad_data[3] = { 0xFF, 0x00, 0xAA };
	uint8_t pad_EOH = 0xFF;
	uint8_t pad_EOP = 0xFF;
}PacketProp;
static Timer t_xsens_loop;
enum class msg_type_optitrack {position, attitude};
enum class msg_type_flight_scenario {USER, SET_PID};
enum class control_system {roll=3, pitch=4, yaw=5, x=0, y=1, z=2, pitch_rate = 7, yaw_rate = 6, null_type};
// enum class block_id {PID_X=0, PID_Y=1, PID_Z=2, PID_ROLL=3, PID_PITCH=4, 
// 					PID_YAW=5, PID_YAW_RATE=6, REF_Y=7, REF_Z=8, REF_ROLL=9, REF_PITCH=10, 
// 					REF_YAW=11, PID_PITCH_RATE = 12, REF_X = 13,
// 					MRFT_X=14, MRFT_Y=15, MRFT_Z=16, MRFT_ROLL=17, MRFT_PITCH=18, 
// 					MRFT_YAW=19, MRFT_YAW_RATE=20, REF_YAW_RATE=21, BB_X=22, BB_Y=23,
// 					PID_MRFT_Z = 24, PID_Z_ID = 25, NULL_ID=999};
// enum class block_type {controller, provider, reference};
enum class switcher_type {controller, provider, reference, null_type};
enum class controller_type {pid, mrft, bounding_box, pid_mrft};
enum class reference_type {process_variable_ref, restricted_process_variable_ref};
enum class internal_switcher_type {position_provider, attitude_provider, reference, controller};
enum class controller_msg_type {data, change_settings, command};
enum class control_system_msg_type {switch_in_out, add_block, change_PID_settings, to_system, PROVIDER, SETREFERENCE, UPDATECONTROLLER};
enum class block_frequency {hz100 = 100, hz200 = 200, hhz200 = 201, hz1000 = 1000, hz120 = 120, hz400 = 400};
enum class act_on {pv, pv_dot, pv_dot_dot};
const float coeff_200Hz_2nd_butter[5] = { -1.97779f, 0.97803f, 6.1006e-05f, 1.2201e-04f, 6.1006e-05f };