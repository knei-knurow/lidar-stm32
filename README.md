# LiDAR Visualizations on STM32
This software cyclically generates a 2D map of enviroment using RPLIDAR A3M1 and prints it on the integrated display of STM32F746G-DISCO board.

## Table of contents
- [What is it?](#what-is-it)
    - [About RPLIDAR A3M1](#about-rplidar-a3m1)
    - [About STM32f746g-Disco](#about-stm32f746g-disco)
- [Gallery](#gallery)
- [Setting up the project](#settng-up-the-project)
- [Thanks](#thanks)

## Brief summary
PC version of this project can be found here -> 
   
RPLIDAR A3M1 is connected through UART to STM32 board. The speed of motor is controlled through hardware PWM on STM32. The MCU sets up continuous measurement and then receives packets of data from LiDAR continuously through DMA, and processes the data in UART interrupt. When a full 360° map is generated, the map is printed on the screen. The software automatically scales the map, so that it fits perfectly to the screen. The most distant point is marked, and the distance[m] from it is printed on the screen.

## Implementation of RPLIDAR A3M1
RPLIDAR A3M1 is currently one of the most advanced RPLIDAR devices. This software is operating with the fastest possible data exchange mode (dense/ultra-capsuled mode). Data receiving and parsing code part of code created with respect to [RPLIDAR SDK](https://github.com/Slamtec/rplidar_sdk) equivalents, but with slightly different approach (less memory operations, different method of storing raw data). 

### Connection
RPLIDAR A3M1 is connected to the MCU through UART6(DMA). 
* MCU_UART6_RX(D0)  --- LIDAR_TX
* MCU_UART6_TX(D1)  --- LIDAR_RX
* MCU_TIM12_CH1(D6) --- LIDAR_PWM
* +5V               --- +5V
* GND               --- GND

### Data analysis (UART interrupt)
The data is stored in a buffer containing three 132B "data blocks" received from the lidar: "Previous", "Current", "Next". When a new block is completely received through DMA, an interrupt is handled. This block is the "Current" block. The interrupt sets up listening for the "Next" block, and the map data consisting of angle + distance pairs is calculated using the comparison between "Current" and "Previous" block. Two consecutive blocks are necessary to calculate the data.

The calculated data is then stored in `f_angles[]` and `f_distances[]` array, where for any point `k`, `f_angles[k]` corresponds to the angle (float \[0°-360°\[ ) and f_distances[`k`] is the distance from the LiDAR in milimetres. Use this data as you wish.

### Display
The display 

### About STM32f746g-Disco

### Setting up the project



## Gallery

## Thanks
