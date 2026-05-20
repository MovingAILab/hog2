//
//  Graphics.h
//  hog2
//
//  Created by Nathan Sturtevant on 7/10/16.
//  Copyright © 2016 NS Software. All rights reserved.
//

#ifndef Graphics_h
#define Graphics_h

#include <vector>
#include <math.h>
//#include "Constants.h" // TODO: needs to be renamed, if data structures are to be more widely re-used
#include "Constants.h"
#include "Colors.h"
#include "FPUtil.h"
#include <cstdint>
#include <functional>
#include <algorithm>

// TODO: move back into namespace - but lots of code has to be updated; task for later
enum viewportType {
	kScaleToSquare,
	kScaleToFill
};

namespace Graphics {
struct  point;
enum textAlign {
	textAlignCenter,
	textAlignLeft,
	textAlignRight
};

enum textBaseline {
	textBaselineTop,
	textBaselineMiddle,
	textBaselineBottom
};


struct point {
	//point(point3d p) :x(p.x), y(p.y), z(p.z) {}
	point(float x = 0, float y = 0, float z = 0)
	:x(x), y(y), z(z) {}
	float x, y, z;

	bool operator==(const point &p) const
	{ return fequal(p.x, x) && fequal(p.y, y) && fequal(p.z, z); }
    bool operator!=(const point &p) const
    { return !(p==*this); }

	point &operator+=(const float v)
	{ x += v; y += v; z += v; return *this; }
	point &operator-=(const float v)
	{ x -= v; y -= v; z -= v; return *this; }
	point &operator*=(const float v)
	{ x *= v; y *= v; z *= v; return *this; }
	point &operator/=(const float v)
	{ x /= v; y /= v; z /= v; return *this; }

	point &operator+=(const point &i)
	{ x+=i.x; y+=i.y; z+=i.z; return *this; }
	point &operator-=(const point &i)
	{ x-=i.x; y-=i.y; z-=i.z; return *this; }

	point operator*(float i) const
	{ point p = *this; p*=i; return p; }
	point operator+(const point &i) const
	{ point p = *this; p+=i; return p; }
	point operator-(const point &i) const
	{ point p = *this; p-=i; return p; }
	point operator/(const float f) const
	{ point p = *this; p.x/=f;p.y/=f;p.z/=f; return p; }


	float length() const
	{ return sqrtf(x * x + y * y + z * z); }
	float squaredLength() const
	{ return (x * x + y * y + z * z); }
	void normalise()
	{
		float length = this->length();
		if (length != 0)
		{
			x /= length; y /= length; z /= length;
		}
		else {
			x = 0; y = 0; z = 0;
		}
	}
	static float Dot(point a, point b)
	{ return a.x * b.x + a.y*b.y + a.z*b.z; }
	// cross product
	point operator*(const point &val) const
	{
		point result;
		result.x = this->y*val.z - this->z*val.y;
		result.y = this->z*val.x - this->x*val.z;
		result.z = this->x*val.y - this->y*val.x;
		result.normalise();
		return result;
	}
};

// TODO: Remove backwards compatibilty
#define point3d Graphics::point


/**
 * A generic vector (essentially the same as a point, but offers normalization)
 */
//class recVec {
//public:
//	recVec() { x = y = z = 0; }
//	recVec(double x_i, double y_i, double z_i) :x(x_i), y(y_i), z(z_i) {}
//	void normalise();
//	double length() const;
//	recVec GetNormal(recVec v)
//	{
//		recVec n;
//		
//		v.normalise();
//		this->normalise();
//		
//		n.x = this->y * v.z - this->z * v.y;
//		n.y = this->z * v.x - this->x * v.z;
//		n.z = this->x * v.y - this->y * v.x;
//		n.normalise();
//		
//		return n;
//	}
//	double x,y,z;
//	recVec &operator+=(const recVec &v)
//	{ x += v.x; y += v.y; z += v.z; return *this; }
//	recVec &operator-=(const recVec &v)
//	{ x -= v.x; y -= v.y; z -= v.z; return *this; }
//	recVec operator-(const recVec &v)
//	{ recVec n = *this; n-=v; return n; }
//	recVec &operator*=(double val)
//	{ x *= val; y *= val; z *= val; return *this; }
//	recVec operator*(const recVec &val) const
//	{
//		recVec result;
//		result.x = this->y*val.z - this->z*val.y;
//		result.y = this->z*val.x - this->x*val.z;
//		result.z = this->x*val.y - this->y*val.x;
//		result.normalise();
//		return result;
//	}
//};
//bool operator==(const recVec &l1, const recVec &l2);
//std::ostream& operator<<(std::ostream &out, const recVec &loc);

class line2d {
public:
	line2d() {}
	line2d(Graphics::point a, Graphics::point b) :start(a), end(b) {}
	bool crosses(line2d which) const;
	Graphics::point start;
	Graphics::point end;
};

static bool operator==(const line2d &l1, const line2d &l2)
{
	return l1.start == l2.start && l1.end == l2.end;
}


struct triangle {
	triangle() {}
	triangle(point p1, point p2, point p3)
	:p1(p1), p2(p2), p3(p3) {}
	point p1, p2, p3;
};

struct rect {
	rect() {}
	rect(point center, float rad) : left(center.x-rad), top(center.y-rad), right(center.x+rad), bottom(center.y+rad) {}
	rect(point tl, point br) :left(tl.x), top(tl.y), right(br.x), bottom(br.y) {}
	rect(float l, float t, float r, float b)
	:left(l), top(t), right(r), bottom(b) {}
	float left, top, right, bottom;
	rect inset(float delta)
	{ return rect(left+delta, top+delta, right-delta, bottom-delta); }
	rect expand(float delta)
	{ return rect(left-delta, top-delta, right+delta, bottom+delta); }
	rect &operator+=(const rect &val)
	{
		left += val.left;
		right += val.right;
		top += val.top;
		bottom += val.bottom;
		return *this;
	}
	rect &operator*=(const point &val)
	{
		left = left*val.x;
		right = right*val.x;
		top = top*val.y;
		bottom = bottom*val.y;
		return *this;
	}
	rect &operator|=(const rect &val)
	{
		left = std::min(left, val.left);
		right = std::max(right, val.right);
		top = std::min(top, val.top);
		bottom = std::max(bottom, val.bottom);
		return *this;
	}
	bool operator==(const rect &val) const
	{
		return fequal(left, val.left) && fequal(right, val.right) &&
			fequal(top, val.top) && fequal(bottom, val.bottom);
	}
	bool operator!=(const rect &val) const
	{
		return !(val==*this);
	}
	void lerp(const rect &val, float percentage)
	{
		left = left*(1-percentage)+val.left*percentage;
		right = right*(1-percentage)+val.right*percentage;
		top = top*(1-percentage)+val.top*percentage;
		bottom = bottom*(1-percentage)+val.bottom*percentage;
	}
};

struct roundedRect {
	roundedRect(rect r, float rad) :r(r), rad(rad) {}
	rect r;
	float rad;
};


inline std::ostream &operator<<(std::ostream &o, const rect&r)
{ o << r.left << ", " << r.top  << ", " << r.right << ", " << r.bottom; return o; }

inline std::ostream &operator<<(std::ostream &o, const point&r)
{ o << "(" << r.x << ", " << r.y  << ", " << r.z << ")"; return o; }

inline std::ostream &operator<<(std::ostream &out, const triangle &t)
{
	out << "[" << t.p1 << ", " << t.p2 << ", " << t.p3 << "]";
	return out;
}

//bool PointInRect(const point3d &p, const rect &r);
bool PointInRect(const point &p, const rect &r);
bool PointInRect(const point &p, const roundedRect &r);
point BezierHelper(const point &from1, const point &to1, const point &from2, const point &to2, float mix);

template <typename T>
T Clamp(const T& n, const T& lower, const T& upper) {
	return std::max(lower, std::min(n, upper));
}

using TweenFunc = std::function<float(float)>;

namespace Tween {

float Linear(float t);

float EaseInSine(float t);

float EaseOutSine(float t);

float EaseInOutSine(float t);
}

float LinearInterpolate(float start, float end, float percent);

void InterpolateRect(rect &current, const rect &from, const rect &to, float percent);

struct viewport {
	Graphics::rect bounds;
	Graphics::rect finalBound;
	viewportType type;
	bool active; // Is this viewport valid
	Graphics::rect startBound;
	float animationDuration;
	TweenFunc tweenFunc;
	unsigned frameCount;
};

/*
 * This class represents an abstract display.
 *
 * All hog2 classes do their drawing to the display. This is an abstraction for a true
 * display. Any actual display then must collect the data from this display and show it,
 * as appropriate.
 */
class Display {
public:
	Display();
	void StartFrame();
	void EndFrame();
	void StartBackground();
	void EndBackground();
	void ForceBackgroundRedraw();
	bool BackgroundNeedsRedraw() const;
	void SetViewport(uint8_t v);
	void SetNumViewports(uint8_t v);
	int GetNumViewports() { return numViewports; }
	void FrameRect(rect r, rgbColor c, float lineWidth);
	void FillRect(rect r, rgbColor c);
	void FrameRect(roundedRect r, rgbColor c, float lineWidth);
	void FillRect(roundedRect r, rgbColor c);

	void FrameSquare(point p, float radius, rgbColor c, float lineWidth);
	void FillSquare(point p, float radius, rgbColor c);
	void FrameCircle(rect r, rgbColor c, float lineWidth); // FIXME: Should be a point and a radius!
	void FrameCircle(point r, float radius, rgbColor c, float lineWidth); // FIXME: Should be a point and a radius!
	void FillCircle(rect r, rgbColor c);
	void FillCircle(point p, float radius, rgbColor c);
	void FillTriangle(const triangle &t, rgbColor c);
	void FillTriangle(point p1, point p2, point p3, rgbColor c);
	void FrameTriangle(const triangle &t, float lineWidth, rgbColor c);
	void FrameTriangle(point p1, point p2, point p3, float lineWidth, rgbColor c);


	void FillNGon(point p, float radius, int sides, float rotation, rgbColor c);
	void FrameNGon(point p, float radius, float width, int sides, float rotation, rgbColor c);

	void DrawLine(point start, point end, float lineWidth, rgbColor c);
	void DrawLineSegments(const std::vector<point> &points, float lineWidth, rgbColor c);
	void FillLineSegments(const std::vector<point> &points, float lineWidth, rgbColor c);
	void DrawArrow(point start, point end, float lineWidth, rgbColor c);
	void DrawText(const char *text, point location, rgbColor c, float height, const char *typeface = 0);
	void DrawText(const char *text, point location, rgbColor c, float height, textAlign align, const char *typeface = 0);
	void DrawText(const char *text, point location, rgbColor c, float height, textAlign align, textBaseline base, const char *typeface = 0);

	struct rrInfo {
		roundedRect r;
		rgbColor c;
		float width;
	};
	struct drawInfo {
		rect r;
		rgbColor c;
		float width;
	};
	struct triangleInfo {
		point p1, p2, p3;
		rgbColor c;
		float width;
	};
	struct shapeInfo {
		point center;
		rgbColor c;
		float radius;
		int segments;
		float rotate;
		float width;
	};
	struct lineInfo {
		point start, end;
		rgbColor c;
		float width;
		bool arrow;
	};
	struct textInfo {
		std::string s;
		point loc;
		rgbColor c;
		float size;
		std::string typeface;
		textAlign align;
		textBaseline base;
		uint8_t viewport;
	};
	enum tDrawClass
	{
		kFillRoundedRectangle,
		kFrameRoundedRectangle,
		kFillRectangle,
		kFrameRectangle,
		kFillTriangle,
		kFrameTriangle,
		kFillOval,
		kFrameOval,
		kFillNGon,
		kFrameNGon,
		kLine
	};
	struct data {
		data(shapeInfo d, tDrawClass t, uint8_t view)
		{
			what = t;
			polygon = d;
			viewport = view;
		}
		data(drawInfo d, tDrawClass t, uint8_t view)
		{
			what = t;
			shape = d;
			viewport = view;
		}
		data(rrInfo d, tDrawClass t, uint8_t view)
		{
			what = t;
			rr = d;
			viewport = view;
		}
		data(lineInfo l, uint8_t view)
		{
			what = kLine;
			line = l;
			viewport = view;
		}
		data(triangleInfo t, tDrawClass d, uint8_t view)
		{
			what = d;
			triangle = t;
			viewport = view;
		}
		tDrawClass what;
		union {
			drawInfo shape;
			rrInfo rr;
			shapeInfo polygon;
			lineInfo line;
			triangleInfo triangle;
		};
		uint8_t viewport;
	};
	struct segments {
		rgbColor c;
		float size;
		std::vector<point> points;
		bool fill;
		uint8_t viewport;
	};
	// These are dynamic items that change from frame to frame
	std::vector<data> drawCommands;
	std::vector<textInfo> text;
	std::vector<segments> lineSegments;
	// These are static items that don't usually change from frame to frame
	std::vector<data> backgroundDrawCommands;
	std::vector<textInfo> backgroundText;
	std::vector<segments> backgroundLineSegments;
	uint64_t backgroundFrame;
	uint64_t foregroundFrame;

	/* Removes all active viewports and adds this one as the first.
	 * rect coordinates are in HOG coordinates.
	 */
	void ReinitViewports(const Graphics::rect &r, viewportType v);
	/* Adds a new viewport to the existing viewports and
	 * returns the new viewport numbers
	 */
	int AddViewport(const Graphics::rect &r, viewportType v);
	int AddViewport(const Graphics::rect &initial, const Graphics::rect &fin, viewportType v);
	void MoveViewport(int viewport, const rect &newLocation, float duration = 1.5, TweenFunc func = Tween::Linear);

	Graphics::point ViewportToGlobalHOG(Graphics::point where, int viewport) const;
	Graphics::point ViewportToGlobalHOG(Graphics::point where, int viewport, int wWidth, int wHeight) const;
	Graphics::rect  ViewportToGlobalHOG(const Graphics::rect &loc, int viewport) const;
	Graphics::rect  ViewportToGlobalHOG(const Graphics::rect &loc, int viewport, int wWidth, int wHeight) const;
	Graphics::point ViewportToGlobalHOG(const viewport &v, Graphics::point where) const;
	Graphics::point ViewportToGlobalHOG(const viewport &v, Graphics::point where, int wWidth, int wHeight) const;
	float ViewportToGlobalHOGX(float x, int v) const;
	float ViewportToGlobalHOGX(float x, int v, int wWidth, int wHeight) const;
	Graphics::point GlobalHOGToViewport(const viewport &v, Graphics::point where) const;
	Graphics::point GlobalHOGToViewport(const viewport &v, Graphics::point where, int wWidth, int wHeight) const;
	Graphics::point GlobalHOGToViewport(Graphics::point where, int viewport) const;
	Graphics::point GlobalHOGToViewport(Graphics::point where, int viewport, int wWidth, int wHeight) const;
	Graphics::rect GlobalHOGToViewport(const Graphics::rect &loc, int viewport) const;
	Graphics::rect GlobalHOGToViewport(const Graphics::rect &loc, int viewport, int wWidth, int wHeight) const;
	float GlobalHOGToViewportX(float x, int v) const;
	float GlobalHOGToViewportX(float x, int v, int wWidth, int wHeight) const;

	uint8_t currViewport;
	uint8_t numViewports;
	std::vector<viewport> viewports;
	int windowWidth, windowHeight; // Ratio is needed for scaling viewports
private:
	bool drawingBackground;

//	int numPorts, currPort;
};

}

#endif /* Graphics_h */
