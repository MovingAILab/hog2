/*
 *  $Id: MonoFont.cpp
 *  hog2
 *
 *  Created by Nathan Sturtevant on 04/12/04.
 *  Modified by Nathan Sturtevant on 06/27/21.
 *
 * This file is part of HOG2. See https://github.com/nathansttt/hog2 for licensing information.
 *
 */

#include <string.h>
#include "MonoFont.h"

/**
 * \param height Height of the text
 * \param location Where to draw the text; depends on the alignment and baseline
 * \param text The text to draw
 */
void MonoFont::GetTextLines(std::vector<Graphics::Display::lineInfo> &lines,
							Graphics::point location,
							const char *text, float height,
							const rgbColor &color,
							Graphics::textAlign align, Graphics::textBaseline base)
{
	lines.resize(0);
	Graphics::point loc = location;
	// 1. Get character width / space size / text length
	auto length = strlen(text);
	float charLength = 5.0f*height/9.0f + 0.5f*height/9.0f;
	float textLength = length*charLength-0.5f*height/9.0f;
//	float charLength = height/2.0f + height/4.0f;
//	float textLength = length*charLength-height/4.0f;
	
	// 2. Get top/left location of text
	loc = location;
	switch (base)
	{
		case Graphics::textBaselineTop:
			break;
		case Graphics::textBaselineMiddle:
			loc.y -= height*0.5f;
			break;
		case Graphics::textBaselineBottom:
			loc.y -= height;
			break;
	}
	switch (align)
	{
		case Graphics::textAlignCenter:
			loc.x -= textLength*0.5f;
			break;
		case Graphics::textAlignLeft:
			break;
		case Graphics::textAlignRight:
			loc.x -= textLength;
			break;
	}
	
	// 3. Draw text
	for (int x = 0; x < length; x++)
	{
		DrawChar(lines, text[x], loc, height, color);
		loc.x += charLength;
	}
}

/**
 * \param height Height of the text
 * \param location Where to draw the text; depends on the alignment and baseline
 * \param text The text to draw
 */
void MonoFont::DrawText(Graphics::Display &display,
						Graphics::point location,
						const char *text, float height,
						const rgbColor &color,
						Graphics::textAlign align, Graphics::textBaseline base)
{
	Graphics::point loc = location;
	// 1. Get character width / space size / text length
	auto length = strlen(text);
	float charLength = 5.0f*height/9.0f + 0.5f*height/9.0f;
	float textLength = length*charLength-0.5f*height/9.0f;

	// 2. Get top/left location of text
	loc = location;
	switch (base)
	{
		case Graphics::textBaselineTop:
			break;
		case Graphics::textBaselineMiddle:
			loc.y -= height*0.5f;
			break;
		case Graphics::textBaselineBottom:
			loc.y -= height;
			break;
	}
	switch (align)
	{
		case Graphics::textAlignCenter:
			loc.x -= textLength*0.5f;
			break;
		case Graphics::textAlignLeft:
			break;
		case Graphics::textAlignRight:
			loc.x -= textLength;
			break;
	}
	
	// 3. Draw text
	for (int x = 0; x < length; x++)
	{
		DrawChar(display, text[x], loc, height, color);
		loc.x += charLength;
	}
}
/*
 *  _ _   0, 1
 * |X|X|  2, 3/, 4\, 5, 6/, 7\, 8
 * |\|/|  9, 10, 11, 12, 13
 *  - -   14, 15
 * |/|\|  16, 17, 18, 19, 20
 * |X|X|  21, 22/, 23\, 24, 25/, 26\, 27
 *  - -   28, 29
 *
 */

void MonoFont::DrawChar(Graphics::Display &display, char c, point3d where, float height, rgbColor color)
{
	uint32_t bMap = GetBitmap(toupper(c));
	for (int x = 0; x < 30; x++)
	{
		if (bMap&0x1)
		{
			switch (x) {
				case 0: DrawLine(display, where, 0, 0, 1, 0, height, color); break;
				case 1: DrawLine(display, where, 1, 0, 1, 0, height, color); break;
				case 2: DrawLine(display, where, 0, 0, 0, 1, height, color); break;
				case 3: DrawLine(display, where, 0, 1, 1, -1, height, color); break;
				case 4: DrawLine(display, where, 0, 0, 1, 1, height, color); break;
				case 5: DrawLine(display, where, 1, 0, 0, 1, height, color); break;
				case 6: DrawLine(display, where, 1, 1, 1, -1, height, color); break;
				case 7: DrawLine(display, where, 1, 0, 1, 1, height, color); break;
				case 8: DrawLine(display, where, 2, 0, 0, 1, height, color); break;
				case 9: DrawLine(display, where, 0, 1, 0, 1, height, color); break;
				case 10: DrawLine(display, where, 0, 1, 1, 1, height, color); break;
				case 11: DrawLine(display, where, 1, 1, 0, 1, height, color); break;
				case 12: DrawLine(display, where, 1, 2, 1, -1, height, color); break;
				case 13: DrawLine(display, where, 2, 1, 0, 1, height, color); break;
				case 14: DrawLine(display, where, 0, 2, 1, 0, height, color); break;
				case 15: DrawLine(display, where, 1, 2, 1, 0, height, color); break;
				case 16: DrawLine(display, where, 0, 2, 0, 1, height, color); break;
				case 17: DrawLine(display, where, 0, 3, 1, -1, height, color); break;
				case 18: DrawLine(display, where, 1, 2, 0, 1, height, color); break;
				case 19: DrawLine(display, where, 1, 2, 1, 1, height, color); break;
				case 20: DrawLine(display, where, 2, 2, 0, 1, height, color); break;
				case 21: DrawLine(display, where, 0, 3, 0, 1, height, color); break;
				case 22: DrawLine(display, where, 0, 4, 1, -1, height, color); break;
				case 23: DrawLine(display, where, 0, 3, 1, 1, height, color); break;
				case 24: DrawLine(display, where, 1, 3, 0, 1, height, color); break;
				case 25: DrawLine(display, where, 1, 4, 1, -1, height, color); break;
				case 26: DrawLine(display, where, 1, 3, 1, 1, height, color); break;
				case 27: DrawLine(display, where, 2, 3, 0, 1, height, color); break;
				case 28: DrawLine(display, where, 0, 4, 1, 0, height, color); break;
				case 29: DrawLine(display, where, 1, 4, 1, 0, height, color); break;
			}
		}
		bMap >>= 1;
	}
}

void MonoFont::DrawChar(std::vector<Graphics::Display::lineInfo> &display, char c, point3d where, float height, rgbColor color)
{
	uint32_t bMap = GetBitmap(toupper(c));
	for (int x = 0; x < 30; x++)
	{
		if (bMap&0x1)
		{
			switch (x) {
				case 0: DrawLine(display, where, 0, 0, 1, 0, height, color); break;
				case 1: DrawLine(display, where, 1, 0, 1, 0, height, color); break;
				case 2: DrawLine(display, where, 0, 0, 0, 1, height, color); break;
				case 3: DrawLine(display, where, 0, 1, 1, -1, height, color); break;
				case 4: DrawLine(display, where, 0, 0, 1, 1, height, color); break;
				case 5: DrawLine(display, where, 1, 0, 0, 1, height, color); break;
				case 6: DrawLine(display, where, 1, 1, 1, -1, height, color); break;
				case 7: DrawLine(display, where, 1, 0, 1, 1, height, color); break;
				case 8: DrawLine(display, where, 2, 0, 0, 1, height, color); break;
				case 9: DrawLine(display, where, 0, 1, 0, 1, height, color); break;
				case 10: DrawLine(display, where, 0, 1, 1, 1, height, color); break;
				case 11: DrawLine(display, where, 1, 1, 0, 1, height, color); break;
				case 12: DrawLine(display, where, 1, 2, 1, -1, height, color); break;
				case 13: DrawLine(display, where, 2, 1, 0, 1, height, color); break;
				case 14: DrawLine(display, where, 0, 2, 1, 0, height, color); break;
				case 15: DrawLine(display, where, 1, 2, 1, 0, height, color); break;
				case 16: DrawLine(display, where, 0, 2, 0, 1, height, color); break;
				case 17: DrawLine(display, where, 0, 3, 1, -1, height, color); break;
				case 18: DrawLine(display, where, 1, 2, 0, 1, height, color); break;
				case 19: DrawLine(display, where, 1, 2, 1, 1, height, color); break;
				case 20: DrawLine(display, where, 2, 2, 0, 1, height, color); break;
				case 21: DrawLine(display, where, 0, 3, 0, 1, height, color); break;
				case 22: DrawLine(display, where, 0, 4, 1, -1, height, color); break;
				case 23: DrawLine(display, where, 0, 3, 1, 1, height, color); break;
				case 24: DrawLine(display, where, 1, 3, 0, 1, height, color); break;
				case 25: DrawLine(display, where, 1, 4, 1, -1, height, color); break;
				case 26: DrawLine(display, where, 1, 3, 1, 1, height, color); break;
				case 27: DrawLine(display, where, 2, 3, 0, 1, height, color); break;
				case 28: DrawLine(display, where, 0, 4, 1, 0, height, color); break;
				case 29: DrawLine(display, where, 1, 4, 1, 0, height, color); break;
			}
		}
		bMap >>= 1;
	}
}

uint32_t MonoFont::GetBitmap(char c)
{
	uint32_t index[128] = {
		0x0, //   0
		0x0, //   1
		0x0, //   2
		0x0, //   3
		0x0, //   4
		0x0, //   5
		0x0, //   6
		0x0, //   7
		0x0, //   8
		0x0, //   9
		0x0, //  10
		0x0, //  11
		0x0, //  12
		0x0, //  13
		0x0, //  14
		0x0, //  15
		0x0, //  16
		0x0, //  17
		0x0, //  18
		0x0, //  19
		0x0, //  20
		0x0, //  21
		0x0, //  22
		0x0, //  23
		0x0, //  24
		0x0, //  25
		0x0, //  26
		0x0, //  27
		0x0, //  28
		0x0, //  29
		0x0, //  30
		0x0, //  31
		0x0, //  32        
		(1<<5)|(1<<11)|(1<<24), //  33       !
		(1<<5)|(1<<8), //  34       "
		0x0, //  35       #
		0x0, //  36       $
		0x0, //  37       %
		0x0, //  38       &
		(1<<5), //  39       '
		(1<<3)|(1<<9)|(1<<16)|(1<<23), //  40       (
		(1<<7)|(1<<13)|(1<<20)|(1<<25), //  41       )
		(1<<10)|(1<<11)|(1<<12)|(1<<14)|(1<<15)|(1<<17)|(1<<18)|(1<<19), //  42       *
		0x0, //  43       +
		(1<<22), //  44       ,
		(1<<15)|(1<<14), //  45       -
		(1<<22)|(1<<23), //  46       .
		(1<<8)|(1<<12)|(1<<18)|(1<<22), //  47       /
		//  48       0
		(1<<3)|(1<<7)|(1<<9)|(1<<12)|(1<<13)|(1<<16)|(1<<17)|(1<<20)|(1<<23)|(1<<25),
		//  49       1
		(1<<3)|(1<<5)|(1<<11)|(1<<18)|(1<<24)|(1<<28)|(1<<29),
		//  50       2
		(1<<3)|(1<<1)|(1<<8)|(1<<13)|(1<<15)|(1<<17)|(1<<21)|(1<<28)|(1<<29),
		//  51       3
		(1<<0)|(1<<7)|(1<<13)|(1<<15)|(1<<20)|(1<<25)|(1<<28),
		//  52       4
		(1<<3)|(1<<9)|(1<<14)|(1<<15)|(1<<11)|(1<<18)|(1<<24),
		//  53       5
		(1<<0)|(1<<1)|(1<<2)|(1<<9)|(1<<14)|(1<<19)|(1<<27)|(1<<28)|(1<<29),
		//  54       6
		(1<<0)|(1<<1)|(1<<2)|(1<<9)|(1<<14)|(1<<19)|(1<<27)|(1<<28)|(1<<29)|(1<<16)|(1<<21),
		//  55       7
		(1<<0)|(1<<1)|(1<<8)|(1<<12)|(1<<18)|(1<<24),
		//  56       8
		(1<<0)|(1<<1)|(1<<2)|(1<<8)|(1<<9)|(1<<13)|(1<<14)|(1<<15)|(1<<16)|(1<<20)|(1<<21)|(1<<27)|(1<<28)|(1<<29),
		//  57       9
		(1<<0)|(1<<1)|(1<<2)|(1<<8)|(1<<9)|(1<<13)|(1<<14)|(1<<15)|(1<<20)|(1<<27)|(1<<28)|(1<<29),
		(1<<24)|(1<<11), //  58       :
		(1<<22)|(1<<11), //  59       ;
		0x0, //  60       <
		0x0, //  61       =
		0x0, //  62       >
		(1<<2)|(1<<1)|(1<<0)|(1<<8)|(1<<12)|(1<<24), //  63       ?
		0x0, //  64       @
				 //  65       A
		(1<<1)|(1<<3)|(1<<8)|(1<<9)|(1<<13)|(1<<14)|(1<<15)|(1<<16)|(1<<20)|(1<<21)|(1<<27),
		//  66       B
		(1<<0)|(1<<2)|(1<<7)|(1<<9)|(1<<12)|(1<<14)|(1<<16)|(1<<19)|(1<<21)|(1<<27)|(1<<28)|(1<<29),
		//  67       C
		(1<<0)|(1<<1)|(1<<2)|(1<<9)|(1<<16)|(1<<21)|(1<<28)|(1<<29),
		//  68       D
		(1<<0)|(1<<2)|(1<<7)|(1<<9)|(1<<13)|(1<<16)|(1<<20)|(1<<21)|(1<<25)|(1<<28),
		//  69       E
		(1<<0)|(1<<1)|(1<<2)|(1<<9)|(1<<14)|(1<<16)|(1<<21)|(1<<28)|(1<<29),
		//  70       F
		(1<<0)|(1<<1)|(1<<2)|(1<<9)|(1<<14)|(1<<16)|(1<<21),
		//  71       G
		(1<<0)|(1<<1)|(1<<2)|(1<<8)|(1<<9)|(1<<15)|(1<<16)|(1<<20)|(1<<21)|(1<<25)|(1<<28),
		//  72       H
		(1<<2)|(1<<8)|(1<<9)|(1<<13)|(1<<14)|(1<<15)|(1<<16)|(1<<20)|(1<<21)|(1<<27),
		//  73       I
		(1<<0)|(1<<1)|(1<<5)|(1<<11)|(1<<18)|(1<<24)|(1<<28)|(1<<29),
		//  74       J
		(1<<1)|(1<<8)|(1<<13)|(1<<20)|(1<<21)|(1<<25)|(1<<28),
		//  75       K
		(1<<2)|(1<<9)|(1<<16)|(1<<21)|(1<<14)|(1<<12)|(1<<19)|(1<<8)|(1<<27),
		//  76       L
		(1<<2)|(1<<9)|(1<<16)|(1<<21)|(1<<28)|(1<<29),
		//  77       M
		(1<<2)|(1<<9)|(1<<16)|(1<<21)|(1<<8)|(1<<13)|(1<<20)|(1<<27)|(1<<0)|(1<<1)|(1<<5)|(1<<11),
		//  78       N
		(1<<2)|(1<<9)|(1<<16)|(1<<21)|(1<<0)|(1<<7)|(1<<13)|(1<<20)|(1<<27),
		//  79       O
		(1<<2)|(1<<9)|(1<<16)|(1<<21)|(1<<8)|(1<<13)|(1<<20)|(1<<27)|(1<<0)|(1<<1)|(1<<28)|(1<<29),
		//  80       P
		(1<<2)|(1<<9)|(1<<16)|(1<<21)|(1<<0)|(1<<14)|(1<<7)|(1<<12),
		//  81       Q
		(1<<2)|(1<<9)|(1<<16)|(1<<21)|(1<<8)|(1<<13)|(1<<20)|(1<<0)|(1<<1)|(1<<28)|(1<<25)|(1<<26),
		//  82       R
		(1<<2)|(1<<9)|(1<<16)|(1<<21)|(1<<0)|(1<<14)|(1<<7)|(1<<13)|(1<<15)|(1<<19)|(1<<27),
		//  83       S
		(1<<0)|(1<<1)|(1<<2)|(1<<9)|(1<<14)|(1<<15)|(1<<20)|(1<<27)|(1<<28)|(1<<29),
		//  84       T
		(1<<0)|(1<<1)|(1<<5)|(1<<11)|(1<<18)|(1<<24),
		//  85       U
		(1<<2)|(1<<9)|(1<<16)|(1<<21)|(1<<8)|(1<<13)|(1<<20)|(1<<27)|(1<<28)|(1<<29),
		//  86       V
		(1<<2)|(1<<9)|(1<<16)|(1<<21)|(1<<8)|(1<<13)|(1<<20)|(1<<25)|(1<<28),
		//  87       W
		(1<<2)|(1<<9)|(1<<16)|(1<<21)|(1<<8)|(1<<13)|(1<<20)|(1<<25)|(1<<28)|(1<<18)|(1<<24),
		//  88       X
		(1<<2)|(1<<21)|(1<<10)|(1<<17)|(1<<12)|(1<<19)|(1<<8)|(1<<27),
		//  89       Y
		(1<<2)|(1<<10)|(1<<18)|(1<<12)|(1<<24)|(1<<8),
		//  90       Z
		(1<<0)|(1<<1)|(1<<8)|(1<<12)|(1<<17)|(1<<21)|(1<<28)|(1<<29),
		// 91
		(1<<0)|(1<<2)|(1<<9)|(1<<16)|(1<<21)|(1<<28), //  91       [  // 0 2 9 16 21 28 // 1 8 13 20 27 29
		0x0, //  92       (backslash)
		(1<<1)|(1<<8)|(1<<13)|(1<<20)|(1<<27)|(1<<29), //  93       ]
		0x0, //  94       ^
		(1<<28)|(1<<29), //  95       _
		0x0, //  96       `
		0x0, //  97       a
		0x0, //  98       b
		0x0, //  99       c
		0x0, // 100       d
		0x0, // 101       e
		0x0, // 102       f
		0x0, // 103       g
		0x0, // 104       h
		0x0, // 105       i
		0x0, // 106       j
		0x0, // 107       k
		0x0, // 108       l
		0x0, // 109       m
		0x0, // 110       n
		0x0, // 111       o
		0x0, // 112       p
		0x0, // 113       q
		0x0, // 114       r
		0x0, // 115       s
		0x0, // 116       t
		0x0, // 117       u
		0x0, // 118       v
		0x0, // 119       w
		0x0, // 120       x
		0x0, // 121       y
		0x0, // 122       z
		0x0, // 123       {
		0x0, // 124       |
		0x0, // 125       }
		0x0, // 126       ~
		0x0, // 127       ESC
	};
	return index[(int)c];
}

/**
 * \param where The top left corner of the text
 * \param scale The height of the text
 */
void MonoFont::DrawLine(Graphics::Display &display, point3d where, int startx, int starty, int offsetx, int offsety, float scale, rgbColor color)
{
	//float unit = scale/21;
//	float edgeLength = scale/4.0f;
//	float lineWidth = scale/9.0f;
	float lineWidth = scale/9.0f;
	float edgeLength = 2.0f*lineWidth;//scale/4.0f;
	Graphics::point p1(where.x+startx*edgeLength+lineWidth/2,
					   where.y+starty*edgeLength+lineWidth/2);
	Graphics::point p2(where.x+startx*edgeLength+lineWidth/2+offsetx*edgeLength,
					   where.y+starty*edgeLength+lineWidth/2+offsety*edgeLength);
	display.DrawLine(p1, p2, lineWidth, color);
	// double unit = scale/4;
	// glVertex2f(where.x+startx*unit, where.y-starty*unit);
	// glVertex2f(where.x+startx*unit+offsetx*unit, where.y-starty*unit-offsety*unit);
}



//struct lineInfo {
//	point start, end;
//	rgbColor c;
//	float width;
//	bool arrow;
//};
void MonoFont::DrawLine(std::vector<Graphics::Display::lineInfo> &lines, point3d where, int startx, int starty, int offsetx, int offsety, float scale, rgbColor color)
{
	//float unit = scale/21;
	float lineWidth = scale/9.0f;
	float edgeLength = 2.0f*lineWidth;//scale/4.0f;
	Graphics::point p1(where.x+startx*edgeLength+lineWidth/2,
					   where.y+starty*edgeLength+lineWidth/2);
	Graphics::point p2(where.x+startx*edgeLength+lineWidth/2+offsetx*edgeLength,
					   where.y+starty*edgeLength+lineWidth/2+offsety*edgeLength);
	lines.push_back({p1, p2, color, lineWidth, false});
}

