# Lidar Visualizations on STM32
This software cyclically generates a 2D map of enviroment using RPLIDAR A3M1 and prints it on the integrated display of STM32F746G-DISCO board.

## Table of contents
- [What is it?](#what-is-it)
    - [About RPLIDAR A3M1](#about-rplidar-a3m1)
    - [About STM32f746g-Disco](#about-stm32f746g-disco)
- [Gallery](#gallery)
- [Setting up the project](#settng-up-the-project)
- [Thanks](#thanks)

## Brief summary
**Desktop version** of this project developed by my friend can be found here -> https://github.com/knei-knurow/lidar-visualizations. It has many more advanced features which I did not implement in the STM32 version including histogram, adjustable zoom, adjustable "gap filling" between measured points, different display modes and mouse support.
   
RPLIDAR A3M1 is connected through UART to STM32 board. The speed of motor is controlled through hardware PWM on STM32. The MCU sets up continuous measurement and then receives packets of data from lidar continuously through DMA, and processes the data in UART interrupt. When a full 360° map is generated, the map is printed on the screen. The software automatically scales the map, so that it fits perfectly to the screen. The most distant point is marked, and the distance[m] from it is printed on the screen.

### IDE

The project was created in STM32CubeIDE, using HAL library, code generated by CUBE (in separate files), BSP and CMSIS drivers. The code is written in C++, so every time Cube has to update generated C code, main.cpp is renamed to main.c, and after the update, it is renamed back to main.cpp.

## Implementation of RPLIDAR A3M1
RPLIDAR A3M1 is currently one of the most advanced RPLIDAR devices. This software is operating with the fastest possible data exchange mode (dense/ultra-capsuled mode). Data receiving and parsing code part of code created with respect to [RPLIDAR SDK](https://github.com/Slamtec/rplidar_sdk) equivalents, but with slightly different approach (less memory operations, different method of storing raw data, low-level approach). 

### Connection
RPLIDAR A3M1 is connected to the MCU through UART6 controlled by DMA2. 
* MCU_UART6_RX(D0)  --- LIDAR_TX
* MCU_UART6_TX(D1)  --- LIDAR_RX
* MCU_TIM12_CH1(D6) --- LIDAR_PWM
* +5V               --- +5V
* GND               --- GND

### Data analysis (UART interrupt)
The data is stored in a buffer containing three 132B "data blocks" received from the lidar: "Previous", "Current", "Next". When a new block is completely received through DMA, an interrupt is handled. This block is the "Current" block. The interrupt sets up listening for the "Next" block, and the map data consisting of angle + distance pairs is calculated using the comparison between "Current" and "Previous" block. Two consecutive blocks are always necessary to calculate the data. 

The calculated data is then stored in `f_angles[]` and `f_distances[]` array, where for any point `k`, `f_angles[k]` corresponds to the angle (float \[0°-360°\[ ) and f_distances[`k`] is the distance from the lidar in milimetres. Use this data as you wish. 

### Graphical rendering
When a complete 360° map is complete, it is transformed into a 2D image placed in one of two present layers

Note that the MCU itself does not have enough RAM to store a full 480x272 32bpp display matrix. Both layers are stored in external SDRAM chip on the board, and they are accessed using BSP_LCD functions (BSP - Board Support Package by STM32). The layers are kept in separate memory banks, in order to prevent delays related to memory access issues.

The image layers are repetitively swapped so, that the currently edited layer is never visible.


# Setting up and building the project
0. Clone the repository and make sure you have the latest versions of STM32CubeIDE installed.
1. Open the downloaded .project file

<img src="/img/steps/step1.png" width="480" align="center"/>

2. Select "lidar-stm32/stm32-discovery/workspace" as the workspace path

<img src="/img/steps/step2.png" width="480" align="center" />

3. Check if import is successful

![](/img/steps/step3.png?raw=true)

4. Close "Information Center" tab :)

![](/img/steps/step4.png?raw=true)

5. Rename the `main.cpp` file to `main.c`

![](/img/steps/step6.png?raw=true)

7. Open .ioc device configuration file

![](/img/steps/step7.png?raw=true)

8. Edit the configuration if you want. Then, launch "Device Configuration Tool Code Generation".

![](/img/steps/step8.png?raw=true)

9. Rename `main.c` back  to main.c **and comment out MX_LTDC_INIT() after each time you do it** (main.cpp, line 268)

![](/img/steps/step10.png?raw=true)

10. Built the project (CTRL+B)

![](/img/steps/step11.png?raw=true)

11. Have fun :)

Gallery

## Thanks

This project was developed within Electronics and Computer Science Club in Knurów (KNEI for short) where lots of amazing projects and ideas come from. Have a look at our website - https://knei.pl/ - unfortunately, at the moment, only available in Polish. Check out our GitHub too - https://github.com/knei-knurow.

Numerous packages with colourful electronic gadgets like RPLIDAR have been granted to us by our friends from KAMAMI.pl.
