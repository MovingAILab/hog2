//
//  SVGUtil.cpp
//  hog2 glut
//
//  Created by Nathan Sturtevant on 10/29/15.
//  Copyright © 2015 University of Denver. All rights reserved.
//

#include "SVGUtil.h"
#include <iostream>
#include <fstream>

/*
//<svg height="500" width="500">
//<defs>
//<radialGradient id="grad1" cx="60%" cy="40%" r="50%" fx="70%" fy="40%">
//<stop offset="0%" style="stop-color:rgb(255,255,255);stop-opacity:0.5" />
//<stop offset="100%" style="stop-color:rgb(0,0,255);stop-opacity:1" />
//</radialGradient>
//</defs>
//<ellipse cx="200" cy="200" rx="55" ry="55" fill="url(#grad1)" />
//Sorry, your browser does not support inline SVG.
//</svg>
*/

std::string SVGGetRGB(rgbColor c)
{
	std::string s;
	s = "rgb(";
	s += std::to_string(int(c.r*255)) + "," + std::to_string(int(c.g*255)) + "," + std::to_string(int(c.b*255));
	s += ")";
	return s;
}

std::string SVGDefineGradient(bool horizontal, bool vertical, rgbColor c1, rgbColor c2, const char *name)
{
	std::string s;
	s += "<defs><linearGradient id=\"";
	s += name;
	s += "\" x1=\"0%\" y1=\"0%\" x2=\"0%\" y2=\"100%\">";
	s += "<stop offset=\"0%\" style=\"stop-color:";
	s += SVGGetRGB(c1);
	s += ";stop-opacity:1\" />";
	s += "<stop offset=\"100%\" style=\"stop-color:";
	s += SVGGetRGB(c2);
	s += ";stop-opacity:1\" />";
	s += "</linearGradient></defs>";
	
	return s;
}

std::string SVGFrameCircle(double x, double y, double radius, double border, rgbColor c)
{
	std::string s;
	s += "<circle cx=\"" + to_string_with_precision(x);
	s += "\" cy=\"" + to_string_with_precision(y);
	s += "\" r=\""+to_string_with_precision(radius)+"\" style=\"fill:none;stroke:"+SVGGetRGB(c)+";stroke-width:"+to_string_with_precision(border)+"\" />";
	return s;
}

std::string SVGDrawEllipse(Graphics::rect &r, rgbColor c)
{
	return SVGDrawEllipse((r.right+r.left)*0.5, (r.top+r.bottom)*0.5f, (r.right-r.left)*0.5f, (r.bottom-r.top)*0.5f, c);
}

std::string SVGDrawEllipse(double x, double y, double rx, double ry, rgbColor c)
{
	//double epsilon = 0.5;
	std::string s;
	s += "<ellipse cx=\"" + to_string_with_precision(x);
	s += "\" cy=\"" + to_string_with_precision(y);
	s += "\" rx=\""+to_string_with_precision(rx);
	s += "\" ry=\""+to_string_with_precision(ry);
	s += "\" style=\"fill:"+SVGGetRGB(c)+";stroke-width:1\" />";
	return s;

//	s += "<circle cx=\"" + std::to_string(10*x);
//	s += "\" cy=\"" + std::to_string(10*y);
//	s += "\" r=\""+std::to_string(radius*10)+"\" style=\"fill:"+SVGGetRGB(c)+";stroke-width:1\" />";
}

std::string SVGDrawCircle(double x, double y, double radius, rgbColor c)
{
	//double epsilon = 0.5;
	std::string s;
	s += "<circle cx=\"" + to_string_with_precision(x);
	s += "\" cy=\"" + to_string_with_precision(y);
	s += "\" r=\""+to_string_with_precision(radius)+"\" style=\"fill:"+SVGGetRGB(c)+";stroke-width:1\" />";
	return s;

//	s += "<circle cx=\"" + std::to_string(10*x);
//	s += "\" cy=\"" + std::to_string(10*y);
//	s += "\" r=\""+std::to_string(radius*10)+"\" style=\"fill:"+SVGGetRGB(c)+";stroke-width:1\" />";
}

std::string SVGDrawRect(float x, float y, float width, float height, const char *gradient)
{
	double epsilon = 0.05;
	std::string s;
	s += "<rect x=\"" + std::to_string(10*x-epsilon);
	s += "\" y=\"" + std::to_string(10*y-epsilon);
	s += "\" width=\""+std::to_string(width*10+2*epsilon)+"\" height=\""+std::to_string(height*10+2*epsilon)+"\" ";
	s += "fill=\"url(#";
	s += gradient;
	s += ")\"";
	s += " style=\"stroke-width:1\" />";
	return s;
}

std::string SVGDrawRect(float x, float y, float width, float height, rgbColor c)
{
	double epsilon = 0.0;//0.55;
	std::string s;
	s += "<rect x=\"" + to_string_with_precision(x-epsilon);
	s += "\" y=\"" + to_string_with_precision(y-epsilon);
	s += "\" width=\""+to_string_with_precision(width+2*epsilon, 8)+"\" height=\""+to_string_with_precision(height+2*epsilon, 8)+"\" style=\"fill:"+SVGGetRGB(c)+";\" />";
//	s += "\" width=\""+to_string_with_precision(width+2*epsilon, 8)+"\" height=\""+to_string_with_precision(height+2*epsilon, 8)+"\" style=\"fill:"+SVGGetRGB(c)+";stroke:"+SVGGetRGB(c)+";stroke-width:5%\" />";
	return s;
}

std::string SVGFrameRect(const Graphics::rect &r, float border, rgbColor c)
{
	return SVGFrameRect(r.left, r.top, r.right-r.left, r.bottom-r.top, border, c);
}

std::string SVGFrameRect(float x, float y, float width, float height, float border, rgbColor c)
{
	double epsilon = 0;//0.05;//0.5;
	std::string s;
	s += "<rect x=\"" + to_string_with_precision(x-epsilon+0);
	s += "\" y=\"" + to_string_with_precision(y-epsilon+0);
	s += "\" width=\""+to_string_with_precision(width+2*epsilon-0)+"\" height=\""+to_string_with_precision(height+2*epsilon-0)+"\" style=\"fill:none;stroke:"+SVGGetRGB(c);
	s += ";stroke-width:"+std::to_string(border)+"\" />";
	return s;
}

std::string SVGFrameNGon(double _x, double _y, double radius, int segments, float rotation, int border, rgbColor c)
{
	double resolution = TWOPI/segments;
	std::string s;
	s += "<polygon points=\"";

	for (int x = 0; x <= segments; x++)
	{
		s += to_string_with_precision(_x+sin(resolution*x+rotation*TWOPI/360.0)*radius);
		s += ",";
		s += to_string_with_precision(_y+cos(resolution*x+rotation*TWOPI/360.0)*radius);
		s += " ";
	}
	s += "\" style=\"fill:none;stroke:purple;stroke-width:1\" />";
	return s;
}

std::string SVGDrawNGon(double _x, double _y, double radius, int segments, float rotation, rgbColor c)
{
	double resolution = TWOPI/segments;
	std::string s;
	s += "<polygon points=\"";
	
	for (int x = 0; x <= segments; x++)
	{
		s += to_string_with_precision(_x+sin(resolution*x+rotation*TWOPI/360.0)*radius);
		s += ",";
		s += to_string_with_precision(_y+cos(resolution*x+rotation*TWOPI/360.0)*radius);
		s += " ";
	}
	s += "\" style=\"fill:"+SVGGetRGB(c)+"\" />";
	return s;
}

//void DrawCircle(GLdouble _x, GLdouble _y, GLdouble tRadius, int segments = 32, float rotation = 0);
//void FrameCircle(GLdouble _x, GLdouble _y, GLdouble tRadius, GLdouble lineWidth, int segments = 32, float rotation = 0);

std::string SVGBeginLinePath(float width, rgbColor c)
{
	return "<path stroke-linejoin=\"round\" stroke-linecap=\"round\" stroke=\""+SVGGetRGB(c)+"\" stroke-width=\""+std::to_string(width)+"\" fill=\"none\" d=\"";
//
//	s += "M "+std::to_string(lines[0].x)+" "+std::to_string(lines[0].y)+" L";
//
//	for (int x = 1; x < lines.size(); x++)
//	{
//		s += " "+std::to_string(lines[x].x)+" "+std::to_string(lines[x].y)+" ";
//	}
//	//	s += "\" stroke=\"black\" stroke-width=\""+std::to_string(width)+"\" fill=\"none\" vector-effect: \"non-scaling-stroke\";/>\n";
//	s += "\" stroke=\""+SVGGetRGB(c)+"\" stroke-width=\""+std::to_string(width)+"\" fill=\"none\" />\n";
//
}

std::string SVGAddLinePath(float x1, float y1)
{
	return "L "+std::to_string(x1)+" "+std::to_string(y1)+" ";
}

std::string SVGAddLinePath(float x1, float y1, float x2, float y2)
{
	return "M "+std::to_string(x1)+" "+std::to_string(y1)+" L "+std::to_string(x2)+" "+std::to_string(y2)+" ";
}

std::string SVGEndLinePath()
{ return "\" />\n"; }


std::string SVGDrawLineSegments(const std::vector<Graphics::point> &lines, float width, rgbColor c)
{
	std::string s = "<path d=\"";

	s += "M "+std::to_string(lines[0].x)+" "+std::to_string(lines[0].y)+" L";
	
	for (size_t x = 1; x < lines.size(); x++)
	{
		s += " "+std::to_string(lines[x].x)+" "+std::to_string(lines[x].y)+" ";
	}
//	s += "\" stroke=\"black\" stroke-width=\""+std::to_string(width)+"\" fill=\"none\" vector-effect: \"non-scaling-stroke\";/>\n";
	s += "\" stroke=\""+SVGGetRGB(c)+"\" stroke-width=\""+std::to_string(width)+"\" fill=\"none\" />\n";
	return s;
}


std::string SVGDrawLine(float x1, float y1, float x2, float y2, float width, rgbColor c)
{
	std::string s;
	s = "<line x1 = \"" + std::to_string(x1) + "\" ";
	s +=      "y1 = \"" + std::to_string(y1) + "\" ";
	s +=      "x2 = \"" + std::to_string(x2) + "\" ";
	s +=      "y2 = \"" + std::to_string(y2) + "\" ";
	s += "style=\"stroke:"+SVGGetRGB(c);
	s += ";stroke-width:"+std::to_string(width)+"\" stroke-linecap=\"round\" />";
	return s;
}

std::string SVGDrawLine(int x1, int y1, int x2, int y2, int width, rgbColor c, bool center)
{
	std::string s;
	float offset = center?0.5:0;
	s = "<line x1 = \"" + std::to_string(x1+offset) + "\" ";
	s +=      "y1 = \"" + std::to_string(y1+offset) + "\" ";
	s +=      "x2 = \"" + std::to_string(x2+offset) + "\" ";
	s +=      "y2 = \"" + std::to_string(y2+offset) + "\" ";
	s += "style=\"stroke:"+SVGGetRGB(c);
	s += ";stroke-width:"+std::to_string(width)+"\" stroke-linecap=\"round\" />";
	return s;
	
//	std::string s;
//	int offset = center?5:0;
//	s = "<line x1 = \"" + std::to_string(10*x1+offset) + "\" ";
//	s +=      "y1 = \"" + std::to_string(10*y1+offset) + "\" ";
//	s +=      "x2 = \"" + std::to_string(10*x2+offset) + "\" ";
//	s +=      "y2 = \"" + std::to_string(10*y2+offset) + "\" ";
//	s += "style=\"stroke:"+SVGGetRGB(c);
//	s += ";stroke-width:"+std::to_string(width)+"\" stroke-linecap=\"round\" />";
//	return s;
}

std::string SVGDrawText(float x1, float y1, const char *txt, rgbColor c, double size, const char *typeface, SVG::svgAlignment align, SVG::svgBaseline base)
{
	std::string s;
//	s =  "<text x=\""+std::to_string(x1*10+2)+"\" y=\""+std::to_string(y1*10-1)+"\" text-anchor=\"middle\" style=\"fill:"+SVGGetRGB(c);
//	s += "; font-family:Helvetica, sans-serif; font-size:"+std::to_string(size*10)+"px\">";
	// Helvetica Impact
	s =  "<text x=\""+std::to_string(x1)+"\" y=\""+std::to_string(y1)+"\" text-anchor=\"";
	switch (align) {
		case SVG::kLeft: s += "start"; break;
		case SVG::kRight: s += "end"; break;
		case SVG::kCenter: s += "middle"; break;
	}//middle
	s += "\" style=\"fill:"+SVGGetRGB(c);
	s += "; font-family:";
	if (typeface)
		s += typeface;
	else
		s += "Helvetica";
	s += ", sans-serif; font-size:"+std::to_string(size)+"px\"";
	switch (base) {
		case SVG::kBottom:
			s += " dominant-baseline=\"auto\">";
			break;
		case SVG::kTop:
			s += " dominant-baseline=\"hanging\">";
			break;
		case SVG::kMiddle:
			s += " dominant-baseline=\"middle\">";
			break;
	}
	s += txt;
	s += "</text>";
	//	s += "<text x=\""+std::to_string(x1*10+2)+"\" y=\""+std::to_string(y1*10-1)+"\" style=\"fill:"+SVGGetRGB(c);
	//	s += "; font-family:Impact, sans-serif; font-size:"+std::to_string(size*10)+"px; stroke:"+SVGGetRGB(notC)+"; stroke-width:0px\">";
	//	s += txt;
	//	s += "</text>";
	return s;
}

std::string SVGDrawStrokedText(float x1, float y1, const char *txt, rgbColor c, rgbColor strokeColor, double size)
{
	std::string s;
	s =  "<text x=\""+std::to_string(x1*10+2)+"\" y=\""+std::to_string(y1*10-1)+"\" style=\"fill:"+SVGGetRGB(c);
	s += "; font-family:Impact, sans-serif; font-size:"+std::to_string(size*10)+"px; stroke:"+SVGGetRGB(strokeColor)+"; stroke-width:"+std::to_string(size)+"px; stroke-linecap:round;stroke-linejoin:round\">";
	s += txt;
	s += "</text>\n";
	return s;
}


float WidthToSVG(float w, float xMultiplier, float yMultiplier)
{
//	return std::min(xMultiplier, yMultiplier)*w;
	return xMultiplier*((w+1)/2.0f-1.0f/2.0f);
}

float PointToSVG(float p, float multiplier)
{
	return (p+1)*multiplier/2.0f;
}

void PointToSVG(Graphics::point &p, float xmultiplier, float ymultiplier)
{
	p.x = (p.x+1)*xmultiplier/2.0f;
	p.y = (p.y+1)*ymultiplier/2.0f;
}

void RectToSVG(Graphics::rect &r, float xmultiplier, float ymultiplier)
{
	Graphics::point p1(r.left, r.top);
	Graphics::point p2(r.right, r.bottom);
	PointToSVG(p1, xmultiplier, ymultiplier);
	PointToSVG(p2, xmultiplier, ymultiplier);
	r.left = p1.x;
	r.right = p2.x;
	r.top = p1.y;
	r.bottom = p2.y;
}

void DrawLineCommand(const Graphics::Display &disp, const std::vector<Graphics::Display::data> &drawCommands, int height, std::string &s, int viewport, int width, size_t &x) {
	if (viewport == -1)
	{
		s += SVGBeginLinePath(WidthToSVG(disp.ViewportToGlobalHOGX(drawCommands[x].line.width, drawCommands[x].viewport, width, height), width, height), drawCommands[x].line.c);
	}
	else {
		s += SVGBeginLinePath(WidthToSVG(drawCommands[x].line.width, width, height), drawCommands[x].line.c);
	}
	bool first = true;
	while (drawCommands[x].what == Graphics::Display::kLine &&
		   (first ||
			(drawCommands[x].line.width == drawCommands[x-1].line.width &&
			 drawCommands[x].line.c == drawCommands[x-1].line.c)))
	{
		const Graphics::Display::lineInfo &o = drawCommands[x].line; //disp.lines[x];
		if (first)
		{
			if (viewport == -1)
			{
				Graphics::point p1 = o.start, p2 = o.end;
				p1 = disp.ViewportToGlobalHOG(p1, drawCommands[x].viewport, width, height);
				p2 = disp.ViewportToGlobalHOG(p2, drawCommands[x].viewport, width, height);
				s += SVGAddLinePath(PointToSVG(p1.x, width), PointToSVG(p1.y, height),
									PointToSVG(p2.x, width), PointToSVG(p2.y, height));
			}
			else {
				s += SVGAddLinePath(PointToSVG(o.start.x, width), PointToSVG(o.start.y, height),
									PointToSVG(o.end.x, width), PointToSVG(o.end.y, height));
			}
			first = false;
		}
		else if (drawCommands[x-1].line.end == o.start)
		{
			if (viewport == -1)
			{
				Graphics::point p2 = o.end;
				p2 = disp.ViewportToGlobalHOG(p2, drawCommands[x].viewport, width, height);
				s += SVGAddLinePath(PointToSVG(p2.x, width), PointToSVG(p2.y, height));
			}
			else {
				s += SVGAddLinePath(PointToSVG(o.end.x, width), PointToSVG(o.end.y, height));
			}
		}
		else {
			if (viewport == -1)
			{
				Graphics::point p1 = o.start, p2 = o.end;
				p1 = disp.ViewportToGlobalHOG(p1, drawCommands[x].viewport, width, height);
				p2 = disp.ViewportToGlobalHOG(p2, drawCommands[x].viewport, width, height);
				s += SVGAddLinePath(PointToSVG(p1.x, width), PointToSVG(p1.y, height),
									PointToSVG(p2.x, width), PointToSVG(p2.y, height));
			}
			else {
				s += SVGAddLinePath(PointToSVG(o.start.x, width), PointToSVG(o.start.y, height),
									PointToSVG(o.end.x, width), PointToSVG(o.end.y, height));
			}
		}
		//					s += SVGDrawLine((o.start.x+1)*width/2.0, (o.start.y+1)*height/2.0, (o.end.x+1)*width/2.0, (o.end.y+1)*height/2.0, o.width, o.c);
		if (o.arrow)
		{
			Graphics::point newEnd = o.end*0.975f+o.start*0.025f;
			Graphics::point p1 = o.end-o.start;
			Graphics::point p2 = o.start;
			Graphics::point p3 = o.end;

			p2.z = 1;
			p2 = p1*p2;
			p2.normalise();
			p2 *= (o.end-newEnd).length();
			if (viewport == -1)
			{
				Graphics::point t1 = disp.ViewportToGlobalHOG(newEnd+p2, drawCommands[x].viewport, width, height);
				Graphics::point t2 = disp.ViewportToGlobalHOG(newEnd-p2, drawCommands[x].viewport, width, height);
				Graphics::point end = disp.ViewportToGlobalHOG(o.end, drawCommands[x].viewport, width, height);

				s += SVGAddLinePath(PointToSVG(t1.x, width), PointToSVG(t1.y, height));
				//								PointToSVG(p3.x, width), PointToSVG(p3.y, height));
				s += SVGAddLinePath(PointToSVG(t2.x, width), PointToSVG(t2.y, height),
									PointToSVG(end.x, width), PointToSVG(end.y, height));
			}
			else {
				s += SVGAddLinePath(PointToSVG(newEnd.x+p2.x, width), PointToSVG(newEnd.y+p2.y, height));
				//								PointToSVG(p3.x, width), PointToSVG(p3.y, height));
				s += SVGAddLinePath(PointToSVG(newEnd.x-p2.x, width), PointToSVG(newEnd.y-p2.y, height),
									PointToSVG(p3.x, width), PointToSVG(p3.y, height));
			}
		}
		x++;
		if (x >= drawCommands.size())
			break;
	}
	x--;
	s += SVGEndLinePath();
}

void FillRectCommand(const Graphics::Display &disp, const std::vector<Graphics::Display::data> &drawCommands, int height, std::string &s, int viewport, int width, size_t &x)
{
	{
		const Graphics::Display::drawInfo &o = drawCommands[x].shape;
		// If we have multiple rects of the same style, group them together
		// to make the .svg file smaller
		if (drawCommands.size() > x+1 && drawCommands[x+1].what == Graphics::Display::kFillRectangle &&
			drawCommands[x+1].shape.c == o.c)
		{
			s += "<g style=\"fill:"+SVGGetRGB(o.c)+";\">\n";
			
			while (true)
			{
				const Graphics::Display::drawInfo &o1 = drawCommands[x].shape;
				Graphics::rect r = o1.r;
				if (viewport == -1)
				{
					r = disp.ViewportToGlobalHOG(r, drawCommands[x].viewport, width, height);
				}
				float x1 = PointToSVG(r.left, width);
				float y1 = PointToSVG(r.top, height);
				float w1 = PointToSVG(r.right, width)-PointToSVG(r.left, width);
				float h1 = PointToSVG(r.bottom, height)-PointToSVG(r.top, height);
				s += " <rect x=\"" + to_string_with_precision(x1);
				s += "\" y=\"" + to_string_with_precision(y1);
				s += "\" width=\""+to_string_with_precision(w1)+"\" height=\""+to_string_with_precision(h1)+"\"/>\n";
				
				x++;
				if (drawCommands.size() > x && drawCommands[x].what == Graphics::Display::kFillRectangle &&
					drawCommands[x].shape.c == o.c)
				{
					
				}
				else {
					x--;
					break;
				}
			}
			s += "</g>";
		}
		else {
			Graphics::rect r = o.r;
			if (viewport == -1)
			{
				r = disp.ViewportToGlobalHOG(r, drawCommands[x].viewport, width, height);
			}
			
			s += SVGDrawRect(PointToSVG(r.left, width), PointToSVG(r.top, height),
							 PointToSVG(r.right, width)-PointToSVG(r.left, width),
							 PointToSVG(r.bottom, height)-PointToSVG(r.top, height),
							 o.c);
		}
	}
}

void HandleCommand(const Graphics::Display &disp, const std::vector<Graphics::Display::data> &drawCommands, std::string &s, int width, int height, int viewport)
{
	for (size_t x = 0; x < drawCommands.size(); x++)
	{
		if ((drawCommands[x].viewport != viewport) && (viewport != -1))
			continue;
		if (disp.viewports[drawCommands[x].viewport].active == false)
			continue;
		switch (drawCommands[x].what)
		{
			case Graphics::Display::kLine:
				DrawLineCommand(disp, drawCommands, height, s, viewport, width, x);
				break;
			case Graphics::Display::kFillRectangle:
				FillRectCommand(disp, drawCommands, height, s, viewport, width, x);
				break;
			case Graphics::Display::kFrameRectangle:
			{
				const Graphics::Display::drawInfo &o = drawCommands[x].shape;
				Graphics::rect r = o.r;
				float w = o.width;
				if (viewport == -1)
				{
					r = disp.ViewportToGlobalHOG(r, drawCommands[x].viewport, width, height);
					w = disp.ViewportToGlobalHOGX(w, drawCommands[x].viewport, width, height);
				}
				RectToSVG(r, width, height);
				s += SVGFrameRect(r, WidthToSVG(w, width, height), o.c);
//				s += SVGFrameRect(PointToSVG(r.left, width), PointToSVG(r.top, height),
//								  PointToSVG(r.right, width)-PointToSVG(r.left, width),
//								  PointToSVG(r.bottom, height)-PointToSVG(r.top, height),
//								  WidthToSVG(w, width, height), o.c);
				break;
			}
			case Graphics::Display::kFillOval:
			{
				const Graphics::Display::drawInfo &o = drawCommands[x].shape;
				Graphics::rect r = o.r;
				if (viewport == -1)
				{
					r = disp.ViewportToGlobalHOG(r, drawCommands[x].viewport, width, height);
				}
				RectToSVG(r, width, height);
				s += SVGDrawEllipse(r, o.c);
//				s += SVGDrawCircle(((r.left+r.right)/2.0+1)*width/2.0, ((r.top+r.bottom)/2.0+1)*height/2.0, width*(r.right-r.left)/4.0, o.c);
				break;
			}
			case Graphics::Display::kFrameOval:
			{
				const Graphics::Display::drawInfo &o = drawCommands[x].shape;
				Graphics::rect r = o.r;
				float w = o.width;
				if (viewport == -1)
				{
					r = disp.ViewportToGlobalHOG(r, drawCommands[x].viewport, width, height);
					w = disp.ViewportToGlobalHOGX(w, drawCommands[x].viewport, width, height);
				}
				s += SVGFrameCircle(((r.left+r.right)/2.0+1)*width/2.0, ((r.top+r.bottom)/2.0+1)*height/2.0, width*(r.right-r.left)/4.0,
									WidthToSVG(w, width, height), o.c);
				
				break;
			}
			case Graphics::Display::kFillNGon:
			{
				const Graphics::Display::shapeInfo &i = drawCommands[x].polygon;
				Graphics::point p = i.center;
				Graphics::point p2 = i.center;
				p2.x += i.radius;
				//int w = i.radius;
				if (viewport == -1)
				{
					p = disp.ViewportToGlobalHOG(p, drawCommands[x].viewport, width, height);
					p2 = disp.ViewportToGlobalHOG(p2, drawCommands[x].viewport, width, height);
//					w = disp.ViewportToGlobalHOGX(w, drawCommands[x].viewport, width, height);
				}
				PointToSVG(p, width, height);
				PointToSVG(p2, width, height);
				s += SVGDrawNGon(p.x, p.y, p2.x-p.x, i.segments, i.rotate, i.c);
				//				s += SVGDrawNGon(i.center.x, i.center.y, i.radius, i.segments, i.rotate, i.c);
				break;
			}
			case Graphics::Display::kFrameNGon:
			{
				const Graphics::Display::shapeInfo &i = drawCommands[x].polygon;
				Graphics::point p = i.center;
				Graphics::point p2 = i.center;
				p2.x += i.radius;
				if (viewport == -1)
				{
					p = disp.ViewportToGlobalHOG(p, drawCommands[x].viewport, width, height);
					p2 = disp.ViewportToGlobalHOG(p2, drawCommands[x].viewport, width, height);
				}
				PointToSVG(p, width, height);
				PointToSVG(p2, width, height);
				s += SVGFrameNGon(p.x, p.y, p2.x-p.x, i.segments, i.rotate, 1/*border*/, i.c);
				break;
			}
			case Graphics::Display::kFrameTriangle:
			{
				Graphics::point p1, p2, p3;
				const Graphics::Display::triangleInfo &i = drawCommands[x].triangle;
				p1 = i.p1;
				p2 = i.p2;
				p3 = i.p3;
				if (viewport == -1)
				{
					p1 = disp.ViewportToGlobalHOG(p1, drawCommands[x].viewport, width, height);
					p2 = disp.ViewportToGlobalHOG(p2, drawCommands[x].viewport, width, height);
					p3 = disp.ViewportToGlobalHOG(p3, drawCommands[x].viewport, width, height);
				}
				PointToSVG(p1, width, height);
				PointToSVG(p2, width, height);
				PointToSVG(p3, width, height);
				s += "<polygon points=\""+
				std::to_string(p1.x)+","+std::to_string(p1.y)+" "+
				std::to_string(p2.x)+","+std::to_string(p2.y)+" "+
				std::to_string(p3.x)+","+std::to_string(p3.y)+"\" ";
				s += "style=\"fill:none;stroke:"+SVGGetRGB(i.c)+";stroke-width:"+std::to_string(i.width)+"\" />";
//				s += "style=\"fill:"+SVGGetRGB(i.c);
				//style="fill:lime;stroke:purple;stroke-width:1" />
				break;
			}
			case Graphics::Display::kFillTriangle:
			{
				Graphics::point p1, p2, p3;
				const Graphics::Display::triangleInfo &i = drawCommands[x].triangle;
				p1 = i.p1;
				p2 = i.p2;
				p3 = i.p3;
				if (viewport == -1)
				{
					p1 = disp.ViewportToGlobalHOG(p1, drawCommands[x].viewport, width, height);
					p2 = disp.ViewportToGlobalHOG(p2, drawCommands[x].viewport, width, height);
					p3 = disp.ViewportToGlobalHOG(p3, drawCommands[x].viewport, width, height);
				}
				PointToSVG(p1, width, height);
				PointToSVG(p2, width, height);
				PointToSVG(p3, width, height);
				s += "<polygon points=\""+
				std::to_string(p1.x)+","+std::to_string(p1.y)+" "+
				std::to_string(p2.x)+","+std::to_string(p2.y)+" "+
				std::to_string(p3.x)+","+std::to_string(p3.y)+"\" ";
				s += "style=\"fill:"+SVGGetRGB(i.c)+";\" />";
			}
		}
	}
}

void MakeSVG(const Graphics::Display &disp, const char *filename, int width, int height, int viewport, const char *comment, bool crisp)
{
	std::string s = MakeSVG(disp, width, height, viewport, comment, crisp);
	FILE *f = fopen(filename, "w+");
	if (f == 0)
	{
		printf("Error: Could not open file '%s'\n", filename);
		return;
	}
	fputs(s.c_str(), f);
	fclose(f);
//	std::fstream myFile;
//	std::fstream svgFile;
//	svgFile.open(filename, std::fstream::out | std::fstream::trunc);
//	svgFile << s;
//	svgFile.close();
}

std::string MakeSVG(const Graphics::Display &disp, int width, int height, int viewport, const char *comment, bool crisp)
{
	std::string s;
	s = "<svg xmlns=\"http://www.w3.org/2000/svg\" version=\"1.1\" width = \""+std::to_string(width)+"\" height = \""+std::to_string(height)+"\" ";
	s += "viewBox = \"0 0 "+std::to_string(width)+" "+std::to_string(height)+"\" ";
//	s += "viewBox = \"-1 -1 "+std::to_string(width+2)+" "+std::to_string(height+2)+"\" ";
	//s += "preserveAspectRatio = \"none\" shape-rendering=\"crispEdges\""; // crispEdges
	//s += " preserveAspectRatio = \"none\"";
	if (crisp)
		s += " shape-rendering=\"crispEdges\""; // crispEdges
	s += ">\n";

//	s += SVGDrawRect(0, 0, width, height, Colors::white);
	
	HandleCommand(disp, disp.backgroundDrawCommands, s, width, height, viewport);
	HandleCommand(disp, disp.drawCommands, s, width, height, viewport);

	for (size_t x = 0; x < disp.text.size(); x++)
	{
		const auto &i = disp.text[x];
		if (i.viewport == viewport || viewport == -1)
		{
			SVG::svgBaseline b;
			Graphics::point p;
			float w;
			if (viewport == -1)
			{
				p = disp.ViewportToGlobalHOG(i.loc, i.viewport, width, height);
				w = disp.ViewportToGlobalHOGX(i.size, i.viewport, width, height);
			}
			else {
				p = i.loc;
				w = i.size;
			}
			w = WidthToSVG(w, width, height);
			switch (i.base)
			{
				case Graphics::textBaselineTop: b = SVG::kTop; break;
				case Graphics::textBaselineMiddle: b = SVG::kMiddle; break;
				case Graphics::textBaselineBottom: b = SVG::kBottom; break;
			}
			if (i.align == Graphics::textAlignLeft)
			{
				s += SVGDrawText(PointToSVG(p.x, width),
								 PointToSVG(p.y, height), // TODO - size is not being converted properly
								 i.s.c_str(), i.c, w, i.typeface.c_str(), SVG::kLeft, b);
				// was i.size*width/2.0
			}
			else if (i.align == Graphics::textAlignCenter)
			{
				s += SVGDrawText(PointToSVG(p.x, width),
								 PointToSVG(p.y, height),
								 i.s.c_str(), i.c, w, i.typeface.c_str(), SVG::kCenter, b);
			}
			else {
				s += SVGDrawText(PointToSVG(p.x, width),
								 PointToSVG(p.y, height),
								 i.s.c_str(), i.c, w, i.typeface.c_str(), SVG::kRight, b);
			}
			s += "\n";
		}
	}
	static std::vector<Graphics::point> outputPoints;
	for (size_t x = 0; x < disp.lineSegments.size(); x++)
	{
		const auto &i = disp.lineSegments[x];
		if (i.viewport == viewport || viewport == -1)
		{
			outputPoints = i.points;
			for (auto &j : outputPoints)
			{
				if (viewport == -1)
					j = disp.ViewportToGlobalHOG(j, i.viewport, width, height);
				PointToSVG(j, width, height);
			}
//			s += SVGDrawLineSegments(outputPoints, i.size, i.c);
			s += SVGDrawLineSegments(outputPoints, WidthToSVG(i.size, width, height), i.c);
			
			s += "\n";
		}
	}

	
	s += "\n";
	if (comment)
	{
		s += "<!--";
		s += comment;
		s += "-->\n";
	}
	s += "</svg>";
	return s;
}
