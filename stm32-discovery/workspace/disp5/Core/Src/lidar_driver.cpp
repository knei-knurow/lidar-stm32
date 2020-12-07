/*
 * lidar_driver.cpp
 *
 *  Created on: Dec 4, 2020
 *      Author: bartl
 */

#include "lidar_driver.h"
#include "main.h"


//Copied from SDK.
uint32_t _varbitscale_decode(uint32_t scaled, uint32_t & scaleLevel){
    for (size_t i = 0; i < 5; ++i){
        int remain = ((int)scaled - (int)VBS_SCALED_BASE[i]);
        if (remain >= 0) {
            scaleLevel = VBS_SCALED_LVL[i];
            return VBS_TARGET_BASE[i] + (remain << scaleLevel);
        }
    }
    return 0;
}



