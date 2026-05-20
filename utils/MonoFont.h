/*
 *  $Id: TextBox.h
 *  hog2
 *
 *  Created by Nathan Sturtevant on 10/18/06.
 *  Modified by Nathan Sturtevant on 02/29/20.
 *
 * This file is part of HOG2. See https://github.com/nathansttt/hog2 for licensing information.
 *
 */

#include <ctype.h>
#include <stdint.h>

#ifndef MONOFONT_H
#define MONOFONT_H

#include "Graphics.h"

class MonoFont {
public:
	void DrawText(Graphics::Display &display,
				  Graphics::point location, const char *text, float height,
				  const rgbColor &color = Colors::black,
				  Graphics::textAlign align = Graphics::textAlignLeft,
				  Graphics::textBaseline base = Graphics::textBaselineBottom);
	void GetTextLines(std::vector<Graphics::Display::lineInfo> &lines,
					  Graphics::point location, const char *text, float height,
					  const rgbColor &color = Colors::black,
					  Graphics::textAlign align = Graphics::textAlignLeft,
					  Graphics::textBaseline base = Graphics::textBaselineBottom);
private:
	void DrawLine(Graphics::Display &display, point3d where, int startx, int starty, int offsetx, int offsety, float scale, rgbColor color);
	void DrawChar(Graphics::Display &display, char c, point3d where, float height, rgbColor color);
	void DrawLine(std::vector<Graphics::Display::lineInfo> &lines, point3d where, int startx, int starty, int offsetx, int offsety, float scale, rgbColor color);
	void DrawChar(std::vector<Graphics::Display::lineInfo> &lines, char c, point3d where, float height, rgbColor color);
	uint32_t GetBitmap(char c);
};
#endif

