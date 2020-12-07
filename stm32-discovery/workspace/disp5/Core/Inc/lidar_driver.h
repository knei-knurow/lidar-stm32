/*
 * lidar_driver.h
 *
 *  Created on: Dec 4, 2020
 *      Author: bartl
 */

#ifndef LIDAR_DRIVER_H
#define LIDAR_DRIVER_H

#include "main.h"

#define POINTS 1800	//Maximum amount of points to be possibly stored.

#define BLOCK_SIZE 132
#define ULTRA_CABINS_IN_RESPONSE 32
#define CABIN_SIZE 4
#define ANGLE_OFFSET_14_8 3
#define ANGLE_OFFSET_07_0 2
#define ULTRA_CABIN_0_OFFSET 4

#define RECV_STATUS_NOT_SYNCED 0
#define RECV_STATUS_SYNCED_ONCE 1
#define RECV_STATUS_SYNCED_TWICE_MAP_READY 2

#define RPLIDAR_VARBITSCALE_X2_SRC_BIT  9
#define RPLIDAR_VARBITSCALE_X4_SRC_BIT  11
#define RPLIDAR_VARBITSCALE_X8_SRC_BIT  12
#define RPLIDAR_VARBITSCALE_X16_SRC_BIT 14
#define RPLIDAR_VARBITSCALE_X2_DEST_VAL 512
#define RPLIDAR_VARBITSCALE_X4_DEST_VAL 1280
#define RPLIDAR_VARBITSCALE_X8_DEST_VAL 1792
#define RPLIDAR_VARBITSCALE_X16_DEST_VAL 3328
#define RPLIDAR_RESP_MEASUREMENT_QUALITY_SHIFT  2
#define RPLIDAR_VARBITSCALE_GET_SRC_MAX_VAL_BY_BITS(_BITS_) \
    (  (((0x1<<(_BITS_)) - RPLIDAR_VARBITSCALE_X16_DEST_VAL)<<4) + \
       ((RPLIDAR_VARBITSCALE_X16_DEST_VAL - RPLIDAR_VARBITSCALE_X8_DEST_VAL)<<3) + \
       ((RPLIDAR_VARBITSCALE_X8_DEST_VAL - RPLIDAR_VARBITSCALE_X4_DEST_VAL)<<2) + \
       ((RPLIDAR_VARBITSCALE_X4_DEST_VAL - RPLIDAR_VARBITSCALE_X2_DEST_VAL)<<1) + \
       RPLIDAR_VARBITSCALE_X2_DEST_VAL - 1)


//#### LIDAR COMMANDS ####
const uint8_t rplidar_reset[] = {0xA5, 0x40};
const uint8_t rplidar_scan_ultra[] = {0xA5, 0x82, 0x05, 0x02, 0x00, 0x00, 0x00, 0x00, (0x00 ^ 0xa5 ^ 0x82 ^ 0x05 ^ 0x02^ 0x00 ^ 0x00 ^ 0x00 ^ 0x00)};

//#### UNUSED COMMANDS ####
const uint8_t rplidar_get_salmplerate[] = {0xA5, 0x59};
const uint8_t rplidar_get_health[] = {0xA5, 0x52};
const uint8_t rplidar_get_info[] = {0xA5, 0x50};
const uint8_t rplidar_get_conf[] = {0xA5, 0x84};
const uint8_t rplidar_stop_scan[] = {0xA5, 0x25};
const uint8_t rplidar_startscan[] = {0xA5, 0x20};



//Arrays needed by "varbitscale_decode" func
static const uint32_t VBS_SCALED_BASE[] = {
	RPLIDAR_VARBITSCALE_X16_DEST_VAL,
	RPLIDAR_VARBITSCALE_X8_DEST_VAL,
	RPLIDAR_VARBITSCALE_X4_DEST_VAL,
	RPLIDAR_VARBITSCALE_X2_DEST_VAL,
	0,
};

static const uint32_t VBS_SCALED_LVL[] = {
	4,
	3,
	2,
	1,
	0,
};

static const uint32_t VBS_TARGET_BASE[] = {
	(0x1 << RPLIDAR_VARBITSCALE_X16_SRC_BIT),
	(0x1 << RPLIDAR_VARBITSCALE_X8_SRC_BIT),
	(0x1 << RPLIDAR_VARBITSCALE_X4_SRC_BIT),
	(0x1 << RPLIDAR_VARBITSCALE_X2_SRC_BIT),
	0,
};

typedef struct lidar_map{
	uint8_t recv_status = RECV_STATUS_NOT_SYNCED;
	uint16_t cnt = 0;
	float amax = 0;
	float dmax = 0;
	float amin = 0;
	float dmin = 1000.0;
	float angles[POINTS] = {0.0};
	float distances[POINTS] = {0.0};
} lidar_map;




uint32_t _varbitscale_decode(uint32_t scaled, uint32_t & scaleLevel);

#endif

