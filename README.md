# Lidar Visualizations on STM32
This software cyclically generates a 2D map of environment using RPLIDAR A3M1 and prints it on the integrated display of STM32F746G-DISCO board.

## Table of contents
- [Gallery](#gallery)
- [Desktop version](#desktop-version)
 - [Implementation of RPLIDAR on STM32](#implementation-of-rplidar-on-stm32)
   - [IDE](#ide)
   - [Connection](#connection)
   - [Data analysis (UART interrupt)](#data-analysis-uart-interrupt)
   - [Graphical rendering](#graphical-rendering)
- [Setting up and building the project](#setting-up-and-building-the-project)
- [Editing configuration with Cube](#editing-configuration-with-cube)
- [Sources](#sources)
- [Thanks](#thanks)

## Gallery 

### Video: [https://www.youtube.com/watch?v=OUqbe0YduO4](https://www.youtube.com/watch?v=OUqbe0YduO4)

Garden
<p align = "center">
<img src="img/gallery/garden.jpg" width="576"/>
</p>

Room
<p align = "center">
<img src="img/gallery/room.jpg" width="576"/>
</p>

Garage
<p align = "center">
<img src="img/gallery/garage.jpg" width="576"/>
</p>

## Desktop version
Desktop version of this project developed by my friend can be found here -> https://github.com/knei-knurow/lidar-visualizations. It has many more advanced features which I did not implement in the STM32 version including histogram, adjustable zoom, adjustable "gap filling" between measured points, different display modes and mouse support. I highly recommend you to try RPLIDAR with it, apart from implementing it on STM32 platform. 

## Implementation of RPLIDAR on STM32
RPLIDAR A3M1 is connected through UART to STM32 board. The speed of motor is controlled through hardware PWM on STM32. The MCU sets up continuous measurement and then receives packets of data from lidar continuously through DMA, and processes the data in UART interrupt. When a full 360° map is generated, the map (with colors representing the distances) is printed on the screen. The software automatically scales the map, so that it fits perfectly to the screen. The most distant point is marked, and the distance[m] from it is printed on the screen. 

### IDE
The project was created in STM32CubeIDE, using HAL library, code generated by CUBE (in separate files), BSP and CMSIS drivers. The code is written in C++, so every time Cube has to update generated C code, main.cpp is renamed to main.c, and after the update, it is renamed back to main.cpp.

### Connection
RPLIDAR A3M1 is connected to the MCU through UART6 (Baud rate 256000) controlled by DMA2. 
* MCU_UART6_RX(D0)  --- LIDAR_TX
* MCU_UART6_TX(D1)  --- LIDAR_RX
* MCU_TIM12_CH1(D6) --- LIDAR_PWM
* +5V               --- +5V
* GND               --- GND

### Data analysis (UART interrupt)
The software uses the fastest data transmission mode of RPLIDAR A3M1 (dense/ultra-capsuled mode). Data parsing functions are similar of these in SDK, but with more low-level approach.

The data is stored in a buffer containing three 132B "data blocks" received from the lidar: "Previous", "Current", "Next". When a new block is completely received through DMA, an interrupt is handled. This block is the "Current" block. The interrupt sets up listening for the "Next" block, and the map data consisting of angle + distance pairs is calculated using the comparison between "Current" and "Previous" block. Two consecutive blocks are always necessary to calculate the data. 

The calculated data is then stored in `f_angles[]` and `f_distances[]` array, where for any point `k`, `f_angles[k]` corresponds to the angle (float \[0°-360°\[ ) and f_distances[`k`] is the distance from the lidar in millimetres. Use this data as you wish. 

### Graphical rendering
When a complete 360° map is complete, it is transformed into a 2D image placed in one of two present layers

Note that the MCU itself does not have enough RAM to store a full 480x272 32bpp display matrix. Both layers are stored in external SDRAM chip on the board, and they are accessed using BSP_LCD functions (BSP - Board Support Package by STM32). The layers are kept in separate memory banks, in order to prevent delays related to memory access issues.

The image layers are repetitively swapped so that the currently edited layer is never visible.


# Setting up and building the project
0. Clone the repository and make sure you have the latest versions of STM32CubeIDE installed.
1. Open the downloaded .project file

<p align="center">
<img src="/img/steps/step1.png" width="576"/>
</p>


2. Select "lidar-stm32/stm32-discovery/workspace" as the workspace path

<p align="center">
<img src="/img/steps/step2.png" width="576"/>
</p>

3. Check if import is successful

<p align="center">
<img src="/img/steps/step3.png" width="576"/>
</p>

4. Close "Information Center" tab :)

<p align="center">
<img src="/img/steps/step4.png"/>
</p>


5. Rename the `main.cpp` file to `main.c`

<p align="center">
<img src="/img/steps/step6.png" width="576"  />
</p>

7. Open .ioc device configuration file

<p align="center">
<img src="/img/steps/step7.png" width="576"  />
</p>

8. Edit the configuration if you want. Then, launch "Device Configuration Tool Code Generation".

<p align="center">
<img src="/img/steps/step8.png" width="576"  />
</p>

9. Rename `main.c` back  to main.c **and comment out `MX_LTDC_INIT();` after each time you do it** (main.cpp, line 268)

<p align="center">
<img src="/img/steps/step10.png" width="576"  />
</p>

10. Build the project (CTRL+B)

<p align="center">
<img src="/img/steps/step11.png" width="576"  />
</p>

11. Have fun :)

## Editing configuration with Cube
For any changes in Cube configuration tool, repeat steps 5-10 carefully.

## Sources

This project was created with use of the following sources
- RPLIDAR A3M1 Datasheet
- RPLIDAR Communication protocol specifications
- RPLIDAR SDK sources

... all of which can be found on [RPLIDAR A Series support page](https://www.slamtec.com/en/Support#rplidar-a-series) and
- [UM2298 User manual: STM32Cube BSP drivers development guidelines](https://www.st.com/resource/en/user_manual/dm00440740-stm32cube-bsp-drivers-development-guidelines-stmicroelectronics.pdf)
- [AN4861 Application note: LCD-TFT display controller (LTDC) on STM32 MCUs](https://www.st.com/resource/en/application_note/dm00287603-lcdtft-display-controller-ltdc-on-stm32-mcus-stmicroelectronics.pdf)
- [UM1907 User manual: Discovery kit for STM32F7 Series with STM32F746NG MCU](https://www.st.com/resource/en/user_manual/dm00190424-discovery-kit-for-stm32f7-series-with-stm32f746ng-mcu-stmicroelectronics.pdf)

provided by ST Microelectronics. 

Last access: 2020-12-30 for all sources.


## Thanks

This project was developed within Electronics and Computer Science Club in Knurów (KNEI for short) where lots of amazing projects and ideas come from. Have a look at our website - https://knei.pl/ - unfortunately, at the moment, only available in Polish. Check out our GitHub too - https://github.com/knei-knurow.

Numerous packages with colourful electronic gadgets like RPLIDAR have been granted to us by our friends from KAMAMI.pl.
