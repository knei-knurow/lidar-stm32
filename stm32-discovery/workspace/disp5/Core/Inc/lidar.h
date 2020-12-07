#pragma once

#include "main.h"
#include "stm32746g_discovery.h"
#include "stm32746g_discovery_lcd.h"
#include "lidar_driver.h"

const int WIDTH = 480;
const int HEIGHT = 272;
const int CHANNELS = 4;
const int ORIGIN_X = WIDTH / 2;
const int ORIGIN_Y = HEIGHT / 2;


//format: ARGB8888
const float MAX_ANGLE_DIFF_IN_LINE = 5.0;
const uint32_t COLOR_BACKGROUND = 0xff101018;
const uint32_t COLOR_BACKGROUND_GRID = 0xff242430;
const uint32_t COLOR_GRID = 0xff242430;
const uint32_t COLOR_CLOUD2 = 0xff00ffff;
const uint32_t COLOR_CLOUD1 = 0xffff00ff;
const uint32_t COLOR_CLOUD0 = 0xffffff00;


uint32_t make_color(uint8_t r, uint8_t g, uint8_t b);

//Structure "map" is defined in lidar_driver
void draw_connected_cloud_from_map(lidar_map &map, float scale,  int y_offset, float lightness, bool marks);

//Get coordinates from angle and distance.
float pt_getX(float phi, float dist,  float k);
float pt_getY(float phi, float dist, float k);

//Drawing functions
void draw_mark( unsigned x, unsigned y, unsigned a, unsigned b, uint32_t color);
void draw_pixel( unsigned x, unsigned y, uint32_t color);
void draw_point( unsigned x, unsigned y, uint32_t color, float lightness = 1.0);
void draw_point( unsigned x, unsigned y, float lightness = 1.0);
void draw_line( float x0, float y0, float x1, float y1, uint32_t color);
void draw_colorful_line(float x0, float y0, float x1, float y1, float dmax);
void draw_ray( float x0, float y0, float x1, float y1, const uint32_t color);
void draw_background( uint32_t color);
void draw_grid( uint32_t color);
void draw_cloud_bars_fromArrays( float * angles, float * distances, int size, float max);

uint32_t calc_color(float v, float lightness = 1.0);

