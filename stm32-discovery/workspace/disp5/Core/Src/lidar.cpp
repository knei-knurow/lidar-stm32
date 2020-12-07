#include <math.h>
#include "main.h"
#include "lidar.h"
#include "lidar_driver.h"
#include "stm32746g_discovery.h"
#include "stm32746g_discovery_lcd.h"
#include "characters.h"


#define MIN(a,b) (((a)<(b))?(a):(b))
#define MAX(a,b) (((a)>(b))?(a):(b))


uint32_t make_color(uint8_t r, uint8_t g, uint8_t b)
{
	return (0xff << 24) | (r << 16) | (g << 8) | b;
}


void draw_connected_cloud_from_map(lidar_map &map, float scale, int y_offset, float lightness, bool marks) {


	if(map.cnt == 0) return;
	if(map.dmax == 0) return;
	if(scale == 0) scale = (HEIGHT) * 0.7 / map.dmax;


	int cnt = 1;

	float first_pt_x = pt_getX(map.angles[1], map.distances[1], scale);	//auto first_pt = cyl_to_cart(cloud.pts[0], scale);
	float first_pt_y = pt_getY(map.angles[1], map.distances[1], scale);
	float first_angle = map.angles[1];
	float first_dist = map.distances[1];

	float last_pt_x = first_pt_x;	//auto last_pt = first_pt;
	float last_pt_y = first_pt_y;
	float last_angle = first_angle;
	float last_dist = first_dist;

	float pt_max_x = pt_getX(map.amax, map.dmax, scale);
	float pt_max_y = pt_getY(map.amax, map.dmax, scale);

	float max_cart_dist = hypot(pt_max_x - (WIDTH/2), pt_max_y - (HEIGHT/2));

	for(int i = 1; i < map.cnt-5; i+=4) {
		float angle = map.angles[i];
		float distance = map.distances[i];


			float pt_x = pt_getX(angle, distance, scale);
			float pt_y = pt_getY(angle, distance, scale);

			if(distance > 0 && last_dist > 0 && ((angle - last_angle) < MAX_ANGLE_DIFF_IN_LINE))
			draw_colorful_line(float(last_pt_x), float(last_pt_y + y_offset), float(pt_x), float(pt_y) + y_offset, max_cart_dist);

			last_angle = angle;
			last_dist = distance;
			last_pt_x = pt_x;
			last_pt_y = pt_y;
			cnt++;

	}

	if (marks) {
			draw_point( pt_max_x, pt_max_y + y_offset, make_color(255, 255, 255));
			draw_mark(  pt_max_x, pt_max_y+ y_offset, unsigned(map.dmax / 1000), unsigned(map.dmax) % 1000, make_color(255, 255, 255));
	}
}


float pt_getX(float phi, float dist,  float k) {
return round(dist * std::sin(phi * (acos(-1) / 180.0)) * k) + ORIGIN_X;
}


float pt_getY(float phi, float dist, float k){
return round(dist * std::cos(phi * (acos(-1) / 180.0)) * k) + ORIGIN_Y;
}



void draw_mark(unsigned x, unsigned y, unsigned a, unsigned b, uint32_t color) {
	draw_point(  x, y, color);
	auto str = std::to_string(a) + "." + std::to_string(b);
	x += -12;
	y += 5;
	for (int ch : str) {
		if (ch == '.') ch = CHAR_DOT;
		else ch -= '0';

		for (uint32_t cy = 0; cy < CHAR_HEIGHT; cy++) {
			for (uint32_t cx = 0; cx < CHAR_MAT[ch][cy].size(); cx++) {
				if (CHAR_MAT[ch][cy][cx] == '#')
					draw_pixel(  x + cx, y + cy, color);
			}
		}
		x += CHAR_MAT[ch][0].size() + 1;
	}
}

void draw_pixel(unsigned x, unsigned y,  uint32_t color){
	if(x <= WIDTH && y <= HEIGHT) BSP_LCD_DrawPixel(x, y, color);
}

void draw_point(unsigned x, unsigned y, uint32_t color, float lightness) {

	uint8_t r = (color << 8) >> 24;
	uint8_t g = (color << 16)>> 24;
	uint8_t b = (color << 24)>> 24;

	r *= lightness;
	g *= lightness;
	b *= lightness;

	color = make_color(r, g, b);

	for (auto cx : { -1, 0, 1 }) {
		for (auto cy : { -1, 0, 1 }) {
				draw_pixel(x + cx, y + cy, color);

		}
	}
}

void draw_point(unsigned x, unsigned y, float lightness) {
	uint32_t c = calc_color(x, y);
	draw_point(x, y, c, lightness);
}

void draw_line(float x0, float y0, float x1, float y1, uint32_t color) {
	float x = x1 - x0, y = y1 - y0;
	const float max = MAX(fabs(x), fabs(y));
	x /= max; y /= max;
	for (float n = 0; n < max; n++) {
		draw_point(x0, y0, color);
		x0 += x; y0 += y;
	}
}

void draw_colorful_line(float x0, float y0, float x1, float y1, float dmax_cart){
	float x = x1 - x0, y = y1 - y0;
	const float max = MAX(fabs(x), fabs(y));
	x /= max; y /= max;
	for (float n = 0; n < max; n++) {
		float dist = hypot(fabs(x0 - (float)(WIDTH/2)), fabs(y0 - (float)(HEIGHT/2)));
		uint32_t color = calc_color((float) dist /  (float) dmax_cart, 1.0);
		draw_point(x0, y0, color, 1.0);
		x0 += x; y0 += y;
	}
}


void draw_ray(float x0, float y0, float x1, float y1, uint32_t color) {
	float x = x1 - x0, y = y1 - y0;
	const float max = MAX(fabs(x), fabs(y));
	x /= max; y /= max;
	while (x0 < WIDTH && x0 >= 0 && y0 < HEIGHT && y0 >= 0) {
		draw_point(x0, y0, color);
		x0 += x; y0 += y;
	}
}



void draw_grid(uint32_t color) {
	for (int x = 0; x < WIDTH; x += WIDTH / 8) {
		for (int y = 0; y < HEIGHT; y++) {
			draw_pixel(x, y, color);
		}
	}
	for (int y = 0; y < HEIGHT; y += HEIGHT / 8) {
		for (int x = 0; x < WIDTH; x++) {
			draw_pixel(x, y, color);
		}
	}
}



void draw_cloud_bars_fromArrays(float * angles, float * distances, int size, float max) {
	unsigned max_width = 80;
	for (int j = 0; j < HEIGHT; j++) {
		float dist = distances[size_t(j * size / HEIGHT)];
		unsigned width = unsigned(round(dist / max * max_width));

		for (uint32_t i = 0; i < width; i++) {
			draw_pixel(i, j, calc_color(float(j * size / HEIGHT) / float(size)));
		}
	}
}


uint32_t calc_color(float v, float lightness){
	if (v >= 0 && v <= 0.33f){
		uint32_t c0 = COLOR_CLOUD0;
		uint32_t c1 = COLOR_CLOUD1;

		uint8_t r = (c0 << 8) >> 24;
		uint8_t g = (c0 << 16)>> 24;
		uint8_t b = (c0 << 24)>> 24;

		r *=  v / 0.34f;
		g *=  v / 0.34f;
		b *=  v / 0.34f;


		uint8_t r1 = (c1 << 8) >> 24;
		uint8_t g1 = (c1 << 16)>> 24;
		uint8_t b1 = (c1 << 24)>> 24;

		r1 *=  1.0 - v / 0.34f;
		g1 *=  1.0 - v / 0.34f;
		b1 *=  1.0 - v / 0.34f;

		return make_color((r + r1) * lightness, (g + g1) * lightness, (b +b1) * lightness);
	}
	else if (v <= 0.66f){
		v -= 0.33;
		uint32_t c0 = COLOR_CLOUD2;
		uint32_t c1 = COLOR_CLOUD0;

		uint8_t r = (c0 << 8) >> 24;
		uint8_t g = (c0 << 16)>> 24;
		uint8_t b = (c0 << 24)>> 24;

		r *=  v / 0.34f;
		g *=  v / 0.34f;
		b *=  v / 0.34f;


		uint8_t r1 = (c1 << 8) >> 24;
		uint8_t g1 = (c1 << 16)>> 24;
		uint8_t b1 = (c1 << 24)>> 24;

		r1 *=  1.0 - v / 0.34f;
		g1 *=  1.0 - v / 0.34f;
		b1 *=  1.0 - v / 0.34f;

		return make_color((r + r1) * lightness, (g + g1) * lightness, (b +b1) * lightness);
	}
	else if (v <= 1.0f) {

		v -= 0.66;
		uint32_t c0 = COLOR_CLOUD1;
		uint32_t c1 = COLOR_CLOUD2;

		uint8_t r = (c0 << 8) >> 24;
		uint8_t g = (c0 << 16)>> 24;
		uint8_t b = (c0 << 24)>> 24;

		r *=  v / 0.34f;
		g *=  v / 0.34f;
		b *=  v / 0.34f;


		uint8_t r1 = (c1 << 8) >> 24;
		uint8_t g1 = (c1 << 16)>> 24;
		uint8_t b1 = (c1 << 24)>> 24;

		r1 *=  1.0 - v / 0.34f;
		g1 *=  1.0 - v / 0.34f;
		b1 *=  1.0 - v / 0.34f;
		return make_color((r + r1) * lightness, (g + g1) * lightness, (b +b1) * lightness);

	}else{
		return make_color(255 * lightness, 255 * lightness, 255 * lightness);
	}
}

