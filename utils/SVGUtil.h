//
//  SVGUtil.hpp
//  hog2 glut
//
//  Created by Nathan Sturtevant on 10/29/15.
//  Copyright © 2015 University of Denver. All rights reserved.
//

#ifndef SVGUtil_h
#define SVGUtil_h

#include <stdio.h>
#include <string>
#include "Constants.h"
#include "Graphics.h"

namespace SVG {
	enum svgAlignment {
		kLeft,
		kRight,
		kCenter
	};

enum svgBaseline {
	kBottom,
	kMiddle,
	kTop
};

}

std::string SVGFrameRect(float x, float y, float width, float height, float border, rgbColor c);

std::string SVGDrawRect(float x, float y, float width, float height, rgbColor c);
std::string SVGDrawRect(float x, float y, float width, float height, const char *gradient);
std::string SVGFrameCircle(double x, double y, double radius, double border, rgbColor c);
std::string SVGDrawCircle(double x, double y, double radius, rgbColor c);
std::string SVGDrawEllipse(double x, double y, double rx, double ry, rgbColor c);
std::string SVGDrawEllipse(Graphics::rect &r, rgbColor c);

std::string SVGFrameNGon(double x, double y, double radius, int segments, float rotation, int border, rgbColor c);
std::string SVGDrawNGon(double x, double y, double radius, int segments, float rotation, rgbColor c);

std::string SVGDrawLine(int x1, int y1, int x2, int y2, int width, rgbColor c, bool center = true);
std::string SVGDrawLine(float x1, float y1, float x2, float y2, float width, rgbColor c);
std::string SVGDrawLineSegments(const std::vector<Graphics::point> &lines, float width, rgbColor c);
std::string SVGBeginLinePath(float width, rgbColor c);
std::string SVGAddLinePath(float x1, float y1);
std::string SVGAddLinePath(float x1, float y1, float x2, float y2);
std::string SVGEndLinePath();
std::string SVGDrawText(float x1, float y1, const char *txt, rgbColor c, double size, const char *typeface = 0,
						SVG::svgAlignment align = SVG::kCenter,
						SVG::svgBaseline base = SVG::kMiddle);
std::string SVGDrawStrokedText(float x1, float y1, const char *txt, rgbColor c, rgbColor strokeColor, double size);

std::string SVGDefineGradient(bool horizontal, bool vertical, rgbColor c1, rgbColor c2, const char *name);

std::string MakeSVG(const Graphics::Display &disp, int width, int height, int viewport = -1, const char *comment = 0, bool crisp = true);
void MakeSVG(const Graphics::Display &disp, const char *filename, int width, int height, int viewport = -1, const char *comment = 0, bool crisp = true);

#endif /* SVGUtil_hpp */
