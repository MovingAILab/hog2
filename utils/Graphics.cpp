//
//  Graphics.cpp
//  hog2
//
//  Created by Nathan Sturtevant on 7/10/16.
//  Copyright © 2016 NS Software. All rights reserved.
//

#include "Graphics.h"

namespace Graphics {

bool fastCrossTest(float p0_x, float p0_y, float p1_x, float p1_y,
				   float p2_x, float p2_y, float p3_x, float p3_y, float *i_x, float *i_y);

bool line2d::crosses(line2d which) const
{
	if ((which.start == start) || (which.end == end) ||
		(which.start == end) || (which.end == start))
		return false;
	
	//	//input x1,y1 input x2,y2
	//input u1,v1 input u2,v2
	line2d here(start, end);
	double maxx1, maxx2, maxy1, maxy2;
	double minx1, minx2, miny1, miny2;
	if (here.start.x > here.end.x)
	{ maxx1 = here.start.x; minx1 = here.end.x; }
	else
	{ maxx1 = here.end.x; minx1 = here.start.x; }
	
	if (here.start.y > here.end.y)
	{ maxy1 = here.start.y; miny1 = here.end.y; }
	else
	{ maxy1 = here.end.y; miny1 = here.start.y; }
	
	if (which.start.x > which.end.x)
	{ maxx2 = which.start.x; minx2 = which.end.x; }
	else
	{ maxx2 = which.end.x; minx2 = which.start.x; }
	
	if (which.start.y > which.end.y)
	{ maxy2 = which.start.y; miny2 = which.end.y; }
	else
	{ maxy2 = which.end.y; miny2 = which.start.y; }
	
	if (fless(maxx1,minx2) || fless(maxx2, minx1) || fless(maxy1, miny2) || fless(maxy2, miny1))
		return false;
	
	return fastCrossTest(start.x, start.y, end.x, end.y,
						 which.start.x, which.start.y, which.end.x, which.end.y,
						 0, 0);
}

// Returns 1 if the lines intersect, otherwise 0. In addition, if the lines
// intersect the intersection point may be stored in the floats i_x and i_y.
// taken from: http://stackoverflow.com/questions/563198/how-do-you-detect-where-two-line-segments-intersect
//
bool fastCrossTest(float p0_x, float p0_y, float p1_x, float p1_y,
				   float p2_x, float p2_y, float p3_x, float p3_y, float *i_x, float *i_y)
{
	float s1_x, s1_y, s2_x, s2_y;
	s1_x = p1_x - p0_x;     s1_y = p1_y - p0_y;
	s2_x = p3_x - p2_x;     s2_y = p3_y - p2_y;
	
	float s, t;
	s = (-s1_y * (p0_x - p2_x) + s1_x * (p0_y - p2_y)) / (-s2_x * s1_y + s1_x * s2_y);
	t = ( s2_x * (p0_y - p2_y) - s2_y * (p0_x - p2_x)) / (-s2_x * s1_y + s1_x * s2_y);
	
	if (s >= 0 && s <= 1 && t >= 0 && t <= 1)
	{
		// Collision detected
		//        if (i_x != NULL)
		//            *i_x = p0_x + (t * s1_x);
		//        if (i_y != NULL)
		//            *i_y = p0_y + (t * s1_y);
		return 1;
	}
	
	return 0; // No collision
}


//bool PointInRect(const point3d &p, const rect &r)
//{
//	return p.y >= r.top && p.x >= r.left && p.y <= r.bottom && p.x <= r.right;
//}

bool PointInRect(const point &p, const rect &r)
{
	return p.y >= r.top && p.x >= r.left && p.y <= r.bottom && p.x <= r.right;
}

// TODO: Handle rounded rect properly
bool PointInRect(const point &p, const roundedRect &r)
{
	return PointInRect(p, r.r);
}

point BezierHelper(const point &from1, const point &to1, const point &from2, const point &to2, float mix)
{
	return (from1*(1-mix)+to1*mix)*(1-mix) + (from2*(1-mix)+to2*mix)*mix;
}

namespace Tween {

float Linear(const float t)
{
	return t;
}

float EaseInSine(const float t)
{
	return 1.0f - static_cast<float>(std::cos(t * M_PI_2));
}

float EaseOutSine(const float t)
{
	return static_cast<float>(std::sin(t * M_PI_2));
}

float EaseInOutSine(const float t)
{
	return -static_cast<float>((std::cos(M_PI * t) - 1.0f) / 2.0f);
}
}

float LinearInterpolate(const float start, const float end, const float percent) {
	return start + (end - start) * Clamp(percent, 0.0f, 1.0f);
}

void InterpolateRect(rect &current, const rect &from, const rect &to, float percent) {
	current.left = LinearInterpolate(from.left, to.left, percent);
	current.top = LinearInterpolate(from.top, to.top, percent);
	current.right = LinearInterpolate(from.right, to.right, percent);
	current.bottom = LinearInterpolate(from.bottom, to.bottom, percent);
}

Display::Display()
{
	currViewport = 0;
	numViewports = 1;
	backgroundFrame = foregroundFrame = 0;
	drawingBackground = false;
	// Default viewport
	AddViewport({-1, -1, 1, 1}, kScaleToSquare);
}


void Display::StartFrame()
{
	drawCommands.clear();
	text.clear();
	lineSegments.clear();
	foregroundFrame++;
}

void Display::EndFrame()
{
}

void Display::StartBackground()
{
	if (backgroundFrame != foregroundFrame)
	{
		backgroundDrawCommands.clear();
		backgroundText.clear();
		backgroundLineSegments.clear();
	}
	backgroundFrame = foregroundFrame;
	drawingBackground = true;
}

void Display::EndBackground()
{
	drawingBackground = false;
}

void Display::ForceBackgroundRedraw()
{
	// We don't necessarily know where we are in the cycle when this is called,
	// so force re-draw on the next cycle
	backgroundFrame = foregroundFrame+1;
}

bool Display::BackgroundNeedsRedraw() const
{
	return backgroundFrame >= foregroundFrame;
}

void Display::SetNumViewports(uint8_t v)
{
	numViewports = v;
	while (viewports.size() < numViewports)
		AddViewport({-1, -1, 1, 1}, kScaleToSquare);
}

void Display::SetViewport(uint8_t v)
{
	currViewport = v;
}

void Display::FrameRect(roundedRect r, rgbColor c, float lineWidth)
{
	rrInfo i = {r, c, lineWidth};
	if (drawingBackground)
		backgroundDrawCommands.push_back({i, kFrameRoundedRectangle, currViewport});
	else
		drawCommands.push_back({i, kFrameRoundedRectangle, currViewport});
}

void Display::FrameRect(rect r, rgbColor c, float lineWidth)
{
	drawInfo i = {r, c, lineWidth};
	if (drawingBackground)
		backgroundDrawCommands.push_back({i, kFrameRectangle, currViewport});
	else
		drawCommands.push_back({i, kFrameRectangle, currViewport});
}

void Display::FrameSquare(point p, float radius, rgbColor c, float lineWidth)
{
	drawInfo i = {{p.x-radius, p.y-radius, p.x+radius, p.y+radius}, c, lineWidth};
	if (drawingBackground)
		backgroundDrawCommands.push_back({i, kFrameRectangle, currViewport});
	else
		drawCommands.push_back({i, kFrameRectangle, currViewport});
	
}

void Display::FillSquare(point p, float radius, rgbColor c)
{
	drawInfo i = {{p.x-radius, p.y-radius, p.x+radius, p.y+radius}, c, 0};
	if (drawingBackground)
		backgroundDrawCommands.push_back({i, kFillRectangle, currViewport});
	else
		drawCommands.push_back({i, kFillRectangle, currViewport});
}

void Display::FillRect(roundedRect r, rgbColor c)
{
	rrInfo i = {r, c, 0};
	if (drawingBackground)
		backgroundDrawCommands.push_back({i, kFillRoundedRectangle, currViewport});
	else
		drawCommands.push_back({i, kFillRoundedRectangle, currViewport});
}

void Display::FillRect(rect r, rgbColor c)
{
	drawInfo i = {r, c, 0};
	if (drawingBackground)
		backgroundDrawCommands.push_back({i, kFillRectangle, currViewport});
	else
		drawCommands.push_back({i, kFillRectangle, currViewport});
}

void Display::FrameCircle(rect r, rgbColor c, float lineWidth)
{
	drawInfo i = {r, c, lineWidth};
	if (drawingBackground)
		backgroundDrawCommands.push_back({i, kFrameOval, currViewport});
	else
		drawCommands.push_back({i, kFrameOval, currViewport});
}

void Display::FrameCircle(point p, float radius, rgbColor c, float lineWidth)
{
	drawInfo i = {{p.x-radius, p.y-radius, p.x+radius, p.y+radius}, c, lineWidth};
	if (drawingBackground)
		backgroundDrawCommands.push_back({i, kFrameOval, currViewport});
	else
		drawCommands.push_back({i, kFrameOval, currViewport});
}

void Display::FillCircle(rect r, rgbColor c)
{
	drawInfo i = {r, c, 0};
	if (drawingBackground)
		backgroundDrawCommands.push_back({i, kFillOval, currViewport});
	else
		drawCommands.push_back({i, kFillOval, currViewport});
}

void Display::FillCircle(point p, float radius, rgbColor c)
{
	drawInfo i = {{p.x-radius, p.y-radius, p.x+radius, p.y+radius}, c, 0};
	if (drawingBackground)
		backgroundDrawCommands.push_back({i, kFillOval, currViewport});
	else
		drawCommands.push_back({i, kFillOval, currViewport});
}

void Display::FillTriangle(const triangle &t, rgbColor c)
{
	triangleInfo i = {t.p1, t.p2, t.p3, c, 0};
	if (drawingBackground)
		backgroundDrawCommands.push_back({i, kFillTriangle, currViewport});
	else
		drawCommands.push_back({i, kFillTriangle, currViewport});
}

void Display::FillTriangle(point p1, point p2, point p3, rgbColor c)
{
	triangleInfo i = {p1, p2, p3, c, 0};
	if (drawingBackground)
		backgroundDrawCommands.push_back({i, kFillTriangle, currViewport});
	else
		drawCommands.push_back({i, kFillTriangle, currViewport});
}

void Display::FrameTriangle(point p1, point p2, point p3, float lineWidth, rgbColor c)
{
	triangleInfo i = {p1, p2, p3, c, lineWidth};
	if (drawingBackground)
		backgroundDrawCommands.push_back({i, kFrameTriangle, currViewport});
	else
		drawCommands.push_back({i, kFrameTriangle, currViewport});
}

void Display::FrameTriangle(const triangle &t, float lineWidth, rgbColor c)
{
	triangleInfo i = {t.p1, t.p2, t.p3, c, lineWidth};
	if (drawingBackground)
		backgroundDrawCommands.push_back({i, kFrameTriangle, currViewport});
	else
		drawCommands.push_back({i, kFrameTriangle, currViewport});
}

void Display::FillNGon(point p, float radius, int sides, float rotation, rgbColor c)
{
	shapeInfo i = {p, c, radius, sides, rotation, 0};
	if (drawingBackground)
		backgroundDrawCommands.push_back({i, kFillNGon, currViewport});
	else
		drawCommands.push_back({i, kFillNGon, currViewport});
}

void Display::FrameNGon(point p, float radius, float width, int sides, float rotation, rgbColor c)
{
	shapeInfo i = {p, c, radius, sides, rotation, width};
	
	if (drawingBackground)
		backgroundDrawCommands.push_back({i, kFrameNGon, currViewport});
	else
		drawCommands.push_back({i, kFrameNGon, currViewport});
}


void Display::DrawLine(point start, point end, float lineWidth, rgbColor c)
{
	lineInfo i = {start, end, c, lineWidth, false};
	if (drawingBackground)
		backgroundDrawCommands.push_back({i, currViewport});
	else
		drawCommands.push_back({i, currViewport});
}

void Display::DrawArrow(point start, point end, float lineWidth, rgbColor c)
{
	lineInfo i = {start, end, c, lineWidth, true};
	if (drawingBackground)
		backgroundDrawCommands.push_back({i, currViewport});
	else
		drawCommands.push_back({i, currViewport});
}

void Display::DrawText(const char *textString, point location, rgbColor c, float height,
					   textAlign align, textBaseline base, const char *typeface)
{
	textInfo i = {std::string(textString), location, c, height, std::string((typeface==0)?"Helvetica":typeface), align, base, currViewport};
	if (drawingBackground)
		backgroundText.push_back(i);
	else
		text.push_back(i);
}

void Display::DrawText(const char *textString, point location, rgbColor c, float height, textAlign align, const char *typeface)
{
	textInfo i = {std::string(textString), location, c, height, std::string((typeface==0)?"Helvetica":typeface), align, textBaselineBottom, currViewport};
	if (drawingBackground)
		backgroundText.push_back(i);
	else
		text.push_back(i);
}

void Display::DrawText(const char *textString, point location, rgbColor c, float height, const char *typeface)
{
	textInfo i = {std::string(textString), location, c, height, std::string((typeface==0)?"Helvetica":typeface), textAlignLeft, textBaselineBottom, currViewport};
	if (drawingBackground)
		backgroundText.push_back(i);
	else
		text.push_back(i);
}

void Display::DrawLineSegments(const std::vector<point> &points, float lineWidth, rgbColor c)
{
	segments s = {c, lineWidth, points, false, currViewport};
	if (drawingBackground)
		backgroundLineSegments.push_back(s);
	else
		lineSegments.push_back(s);
}

void Display::FillLineSegments(const std::vector<point> &points, float lineWidth, rgbColor c)
{
	segments s = {c, lineWidth, points, true, currViewport};
	if (drawingBackground)
		backgroundLineSegments.push_back(s);
	else
		lineSegments.push_back(s);
}

}

void Graphics::Display::ReinitViewports(const Graphics::rect &r, viewportType v)
{
	viewports.resize(1);
	viewports[0].bounds = r;
	viewports[0].finalBound = r;
	viewports[0].type = v;
	viewports[0].active = true;
	numViewports = 1;
}

/* Adds a new viewport to the existing viewports and
 * returns the new viewport numbers
 */
int Graphics::Display::AddViewport(const Graphics::rect &r, viewportType v)
{
//	pRecContext pContextInfo = GetContext(windowID);
//	if (pContextInfo->numPorts >= MAXPORTS)
//	{
//		printf("Cannot add viewport - reached limit of %d [constant MAXPORTS]\n", MAXPORTS);
//		return -1;
//	}
	viewports.resize(viewports.size()+1);
	//pContextInfo->numPorts++;
	numViewports = viewports.size();
	viewports[numViewports-1].bounds = r;
	viewports[numViewports-1].finalBound = r;
	viewports[numViewports-1].type = v;
	viewports[numViewports-1].active = true;
	return numViewports-1;
}

/* Adds a new viewport to the existing viewports and
 * returns the new viewport numbers. Will animate from initial to final location
 */
int Graphics::Display::AddViewport(const Graphics::rect &initial, const Graphics::rect &fin, viewportType v)
{
//	pRecContext pContextInfo = GetContext(windowID);
//	if (pContextInfo->numPorts >= MAXPORTS)
//	{
//		printf("Cannot add viewport - reached limit of %d [constant MAXPORTS]\n", MAXPORTS);
//		return -1;
//	}
	numViewports++;
	viewports[numViewports-1].bounds = initial;
	viewports[numViewports-1].finalBound = fin;
	viewports[numViewports-1].type = v;
	viewports[numViewports-1].active = true;
	return numViewports-1;
}

void Graphics::Display::MoveViewport(int viewport, const rect &newLocation, float duration, TweenFunc func)
{
	viewports[viewport].startBound = viewports[viewport].bounds;
	viewports[viewport].finalBound = newLocation;
	viewports[viewport].animationDuration = duration;
    viewports[viewport].tweenFunc = std::move(func);
	viewports[viewport].frameCount = 0;
}

float Graphics::Display::GlobalHOGToViewportX(float x, int v) const
{
	return GlobalHOGToViewportX(x, v, windowWidth, windowHeight);
}

float Graphics::Display::GlobalHOGToViewportX(float x, int v, int width, int height) const
{
	Graphics::point input(x, 0.f, 0.f);
	Graphics::point input2(0, 0.f, 0.f);
	Graphics::point result = GlobalHOGToViewport(input, v, width, height);
	Graphics::point result2 = GlobalHOGToViewport(input2, v, width, height);
	return result.x-result2.x;
}

float Graphics::Display::ViewportToGlobalHOGX(float x, int v) const
{
	return ViewportToGlobalHOGX(x, v, windowWidth, windowHeight);
}

float Graphics::Display::ViewportToGlobalHOGX(float x, int v, int width, int height) const
{
	Graphics::point input(x, 0.f, 0.f);
	Graphics::point input2(0, 0.f, 0.f);
	Graphics::point result = ViewportToGlobalHOG(input, v, width, height);
	Graphics::point result2 = ViewportToGlobalHOG(input2, v, width, height);
	return result.x-result2.x;
}

// This code doesn't look to be correct
//float Graphics::Display::GlobalHOGToViewportY(float y, int v)
//{
//	Graphics::point input(0.f, y, 0.f);
//	Graphics::point result = ViewportToGlobalHOG(input, v);
////	result.y -= pContextInfo->viewports[v].bounds.bottom;
////	result.y = pContextInfo->viewports[v].bounds.top - result.y;
//	//	if (v == 1)
////		printf("Y:%f -> %f\n", y, ((result.y+1.0))/2.0);
//	return -((result.y-1.0)*pContextInfo->windowHeight)/2.0;
//}

Graphics::point Graphics::Display::GlobalHOGToViewport(Graphics::point where, int viewport) const
{
	return GlobalHOGToViewport(where, viewport, windowWidth, windowHeight);
}

Graphics::point Graphics::Display::GlobalHOGToViewport(Graphics::point where, int viewport, int width, int height) const
{
	return GlobalHOGToViewport(viewports[viewport], where, width, height);
}

Graphics::point Graphics::Display::ViewportToGlobalHOG(Graphics::point where, int viewport) const
{
	return ViewportToGlobalHOG(where, viewport, windowWidth, windowHeight);
}

Graphics::point Graphics::Display::ViewportToGlobalHOG(Graphics::point where, int viewport, int width, int height) const
{
	return ViewportToGlobalHOG(viewports[viewport], where, width, height);
}

Graphics::rect Graphics::Display::GlobalHOGToViewport(const Graphics::rect &loc, int port) const
{
	return GlobalHOGToViewport(loc, port, windowWidth, windowHeight);
}

Graphics::rect Graphics::Display::GlobalHOGToViewport(const Graphics::rect &loc, int port, int width, int height) const
{
	return Graphics::rect(GlobalHOGToViewport(viewports[port], {loc.left, loc.top}, width, height),
						  GlobalHOGToViewport(viewports[port], {loc.right, loc.bottom}, width, height));
}

Graphics::rect Graphics::Display::ViewportToGlobalHOG(const Graphics::rect &loc, int port) const
{
	return ViewportToGlobalHOG(loc, port, windowWidth, windowHeight);
}

Graphics::rect Graphics::Display::ViewportToGlobalHOG(const Graphics::rect &loc, int port, int width, int height) const
{
	return Graphics::rect(ViewportToGlobalHOG(viewports[port], {loc.left, loc.top}, width, height),
						  ViewportToGlobalHOG(viewports[port], {loc.right, loc.bottom}, width, height));
}

Graphics::point Graphics::Display::GlobalHOGToViewport(const viewport &v, Graphics::point where) const
{
	return GlobalHOGToViewport(v, where, windowWidth, windowHeight);
}

Graphics::point Graphics::Display::GlobalHOGToViewport(const viewport &v, Graphics::point where, int width, int height) const
{
	if (v.type == kScaleToFill) 		// just scale regular -1/1 axes into the rectangle
	{
		// gets offset into rect
		where.x -= v.bounds.left;
		where.x /= (v.bounds.right-v.bounds.left);
		where.x = where.x*2.0f-1.0f;
		where.y -= v.bounds.bottom;
		where.y /= (v.bounds.top-v.bounds.bottom);
		where.y = -where.y*2.0f+1.0f;
		return where;
	}
	else if (v.type == kScaleToSquare)
	{
		float localWidth = v.bounds.right-v.bounds.left;
		float localHeight = v.bounds.bottom-v.bounds.top;
		float actualWidth = windowWidth*localWidth;
		float actualHeight = windowHeight*localHeight;
		float xRatio = actualWidth/actualHeight;
		float yRatio = actualHeight/actualWidth;
		xRatio = std::max(xRatio, 1.f);
		yRatio = std::max(yRatio, 1.f);
		
		where.x *= xRatio;
		where.x -= v.bounds.left;
		where.x /= (v.bounds.right-v.bounds.left);
		where.x = where.x*2.0f-1.0f;

		
		where.y *= yRatio;
		where.y -= v.bounds.bottom;
		where.y /= (v.bounds.top-v.bounds.bottom);
		where.y = (-where.y)*2.0f+1.0f;

		
		return where;

	}
	else {
		printf("Unknown scale type\n");
		exit(0);
	}
}

Graphics::point Graphics::Display::ViewportToGlobalHOG(const viewport &v, Graphics::point where) const
{
	return ViewportToGlobalHOG(v, where, windowWidth, windowHeight);
}

Graphics::point Graphics::Display::ViewportToGlobalHOG(const viewport &v, Graphics::point where, int wWidth, int wHeight) const
{
	if (v.type == kScaleToFill)
	{
		where.x = (where.x+1.0f)/2.0f;
		where.x *= (v.bounds.right-v.bounds.left);
		where.x += v.bounds.left;

		where.y = -(where.y-1.0f)/2.0f;
		where.y *= (v.bounds.top-v.bounds.bottom);
		where.y += v.bounds.bottom;
		return where;
	}
	else if (v.type == kScaleToSquare)
	{
		float localWidth = v.bounds.right-v.bounds.left;
		float localHeight = v.bounds.bottom-v.bounds.top;
		float actualWidth = wWidth*localWidth;
		float actualHeight = wHeight*localHeight;
		float xRatio = actualWidth/actualHeight;
		float yRatio = actualHeight/actualWidth;
		xRatio = std::max(xRatio, 1.f);
		yRatio = std::max(yRatio, 1.f);
		
		where.x = (where.x+1.0f)/2.0f;
		where.x *= (v.bounds.right-v.bounds.left);
		where.x += v.bounds.left;
		where.x /= xRatio;
		
		where.y = -(where.y-1.0f)/2.0f;
		where.y *= (v.bounds.top-v.bounds.bottom);
		where.y += v.bounds.bottom;
		where.y /= yRatio;
		return where;

	}
	else {
		printf("Unknown scale type\n");
		exit(0);
	}
}
