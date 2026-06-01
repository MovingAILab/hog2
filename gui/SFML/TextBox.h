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

//#include "Constants.h"
#include <ctype.h>
#include <stdint.h>
#include "Graphics.h"
#include "GLUtil.h"

#ifndef TEXTBOX_H
#define TEXTBOX_H

class TextBox {
public:
	TextBox(char *text, int charLine, point3d topLeft, point3d bottomRight, double duration=4, bool deform=true, bool scrolling = false); // start coord (3), velocity(3)
	~TextBox();
  void stepTime(double amount);
	void draw();
	point3d getLocation();
	void destroy();
	void setColor(rgbColor _myColor) { myColor = _myColor; }
private:
	void drawLine(point3d where, int startx, int starty, int offsetx, int offsety, double scale);
	void drawChar(char c, point3d where, double height);
	uint32_t getBitmap(char c);
	point3d tl, br;
	rgbColor myColor;
	char *text;
	int charLine;
	bool deform, scrolling;
	double elapsedTime;
	double duration;
	GLuint dList;
};

#endif

