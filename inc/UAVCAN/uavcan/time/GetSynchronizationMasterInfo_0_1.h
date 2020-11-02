/*
 *
 * BSD 3-Clause License
 * 
 * Copyright (c) 2020, NXP Drone and Rover Team
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * UAVCAN data structure definition.
 *
 * AUTOGENERATED, DO NOT EDIT.
 *
 * Source File:
 * /home/cis/drones/s32k-bms/software/rddrone-bms772/src/nxp_bms/BMS_v1/public_regulated_data_types/uavcan/time/510.GetSynchronizationMasterInfo.0.1.uavcan
 *
 * Template:
 * ServiceType.j2
 *
 * Generated at:  2020-11-02 13:23:10.473384 UTC
 * Is deprecated: no
 * Fixed port ID: 510
 * Full name:     uavcan.time.GetSynchronizationMasterInfo
 * Version:       0.1
 *
 */

#ifndef UAVCAN_TIME_GETSYNCHRONIZATIONMASTERINFO
#define UAVCAN_TIME_GETSYNCHRONIZATIONMASTERINFO
#include <canard_dsdl.h>

#include <uavcan/time/GetSynchronizationMasterInfo/Request_0_1.h>
#include <uavcan/time/GetSynchronizationMasterInfo/Response_0_1.h>

#define UAVCAN_TIME_GET_SYNCHRONIZATION_MASTER_INFO_PORT_ID 510

#define UAVCAN_TIME_GET_SYNCHRONIZATION_MASTER_INFO_MSG_SIZE 6



typedef struct uavcan_time_get_synchronization_master_infoType
{
	uavcan_time_get_synchronization_master_info_request request;
	uavcan_time_get_synchronization_master_info_response response;
} uavcan_time_get_synchronization_master_info;

void uavcan_time_get_synchronization_master_info_serializeToBuffer(uavcan_time_get_synchronization_master_info* msg, uint8_t* const buffer, const size_t starting_bit)
{
    uavcan_time_get_synchronization_master_info_request_serializeToBuffer(&msg->request, buffer, starting_bit + 0);
    uavcan_time_get_synchronization_master_info_response_serializeToBuffer(&msg->response, buffer, starting_bit + 0);
}

void uavcan_time_get_synchronization_master_info_deserializeFromBuffer(uavcan_time_get_synchronization_master_info* msg, const uint8_t* const buffer, const size_t buf_size, const size_t starting_bit)
{

        
        uavcan_time_get_synchronization_master_info_request_deserializeFromBuffer(&msg->request, buffer, buf_size, starting_bit + 0);
        
        uavcan_time_get_synchronization_master_info_response_deserializeFromBuffer(&msg->response, buffer, buf_size, starting_bit + 0);

    return msg;
}

#endif // UAVCAN_TIME_GETSYNCHRONIZATIONMASTERINFO
