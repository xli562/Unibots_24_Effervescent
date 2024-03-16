/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2018, EAIBOT, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/
#include <stdint.h>
#include <stdlib.h>
//#include <core/base/typedef.h>

#ifndef ORADAR_PROTOCOL_H_
#define ORADAR_PROTOCOL_H_

#ifdef __cplusplus
extern "C" {
#endif

#define TMPBUFF_SIZE (1024)
#define MAX_BLOCK_POINT_NUM (100)
#define MAX_BLOCK_SIZE (MAX_BLOCK_POINT_NUM * 4)
#define POINT_CIRCLE_MAX_SIZE (4096)
#define POINT_PKG_MAX_SIZE (200)
#define POINT_PER_PACK (12)

#define SET_TIME_OUT    (10) //unit:s
#define HEAD_FLAG   (0xF5A5)        
#define TAIL_FLAG   (0x31F2)
#define HEAD_LEN    (5)

typedef enum
{
    ORADAR_MS200 = 1,
    ORADAR_MS300 = 2,
}oradar_lidar_type_id;

typedef enum {
    ORADAR_TYPE_SERIAL = 0x0,/**< serial type.*/
    ORADAR_TYPC_UDP = 0x1,/**< socket udp type.*/
    ORADAR_TYPE_TCP = 0x1,/**< socket tcp type.*/
} device_type_id;

typedef enum
{
    SET_ROTATION_SPEED = 0xA1,
    SET_RUN_MODE = 0xA2,
}CMD;

typedef enum
{
    WRITE_PARAM = 0xC1,
    WRITE_PARAM_RESPONSE = 0xC2,
    READ_PARAM = 0xC3,
    READ_PARAM_RESPONSE = 0xC4,
}CMD_TYPE;

typedef struct uart_comm_st
{
    uint16_t head_flag;
    uint8_t cmd;
    uint8_t cmd_type;
    uint8_t payload_len;
    uint8_t data[10];
}uart_comm_t;

typedef struct point_data_st
{
    unsigned short distance;
    unsigned short intensity;
    float angle;
} point_data_t;

typedef enum frame_head_flag_et
{
    HEAD_FLAG_NONE,
    HEAD_FLAG_OK,
} frame_head_flag_t;

typedef enum protocol_version_et
{
    VERSION_NONE = 0,
    VERSION_MS200,
} protocol_version_t;

typedef struct __attribute__((packed))
{
    uint16_t distance;
    uint8_t confidence;
} OradarLidarPoint;

typedef struct __attribute__((packed))
{
    uint8_t header;
    uint8_t ver_len;
    uint16_t speed;
    uint16_t start_angle;
    OradarLidarPoint point[POINT_PER_PACK];
    uint16_t end_angle;
    uint16_t timestamp;
    uint8_t crc8;
} OradarLidarFrame;

typedef struct
{
    frame_head_flag_t head_flag;
    protocol_version_t version;
    double speed;
    unsigned char cmd_mode;
    unsigned char point_num;
    float start_angle;
    float end_angle;
    uint16_t timestamp;
    unsigned short crc_value;
} parsed_data_st;

typedef struct
{
    point_data_t data[POINT_CIRCLE_MAX_SIZE + 1];
    unsigned short vailtidy_point_num;
    double speed;
} full_scan_data_st;

typedef struct
{
    point_data_t data[POINT_PKG_MAX_SIZE + 1];
    unsigned short vailtidy_point_num;
    double speed;
} one_scan_data_st;

#ifdef __cplusplus
}
#endif


#endif  // ORADAR_PROTOCOL_H_
