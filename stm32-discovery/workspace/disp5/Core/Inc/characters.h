#pragma once
#include <string>

enum Chars {
	CHAR_0,
	CHAR_1,
	CHAR_2,
	CHAR_3,
	CHAR_4,
	CHAR_5,
	CHAR_6,
	CHAR_7,
	CHAR_8,
	CHAR_9,
	CHAR_DOT,
};

const unsigned CHAR_HEIGHT = 8;

const std::string CHAR_MAT[][CHAR_HEIGHT] = {
	// 0
	{
		".####.",
		"#....#",
		"#....#",
		"#....#",
		"#....#",
		"#....#",
		"#....#",
		".####.",
	},
	// 1
	{
		".#",
		".#",
		".#",
		".#",
		".#",
		".#",
		".#",
		".#",
	},
	// 2
	{
		".####.",
		"#....#",
		".....#",
		".####.",
		"#.....",
		"#.....",
		"#.....",
		"######",
	},
	// 3
	{
		".####.",
		"#....#",
		".....#",
		"..###.",
		".....#",
		".....#",
		"#....#",
		".####.",
	},
	// 4
	{
		"#....#",
		"#....#",
		"#....#",
		".#####",
		".....#",
		".....#",
		".....#",
		".....#",
	},
	// 5
	{
		"######",
		"#.....",
		"#.....",
		"#####.",
		".....#",
		".....#",
		"#....#",
		".####.",
	},
	// 6
	{
		".####.",
		"#....#",
		"#.....",
		"#####.",
		"#....#",
		"#....#",
		"#....#",
		".####."
	},
	// 7
	{
		"######",
		".....#",
		".....#",
		".....#",
		".....#",
		".....#",
		".....#",
		".....#",
	},
	// 8
	{
		".####.",
		"#....#",
		"#....#",
		".####.",
		"#....#",
		"#....#",
		"#....#",
		".####.",
	},
	// 9
	{
		".####.",
		"#....#",
		"#....#",
		".#####",
		".....#",
		".....#",
		"#....#",
		".####.",
	},
	// dot
	{
		"..",
		"..",
		"..",
		"..",
		"..",
		"..",
		"##",
		"##",
	},
};