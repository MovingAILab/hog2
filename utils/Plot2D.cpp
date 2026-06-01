/*
 *  Plot2D.cpp
 *  hog2
 *
 *  Created by Nathan Sturtevant on 4/18/07.
 *  Copyright 2007 Nathan Sturtevant, University of Alberta. All rights reserved.
 *
 */

#include "Plot2D.h"
#include <math.h>
#include <cstring>
#include <assert.h>
#include <stdio.h>

namespace Plotting {

Line::Line(const char *label, tPlotType ptype)
:plotType(ptype)
{
	width = 1.0;
	ClearColor();
	strncpy(name,label,1024);
	xMin = DBL_MAX;
	xMax = -DBL_MAX;
	yMin = DBL_MAX;
	yMax = -DBL_MAX;
	hidden = false;
	changedLine = false;
}

void Line::Clear()
{
	x.resize(0);
	y.resize(0);
	xMin = DBL_MAX;
	xMax = -DBL_MAX;
	yMin = DBL_MAX;
	yMax = -DBL_MAX;
}

void Line::AddPoint(double _y)
{
	changedLine = true;
	if (x.size() == 0)
		AddPoint(0, _y);
	else
		AddPoint(x.back()+1, _y);
}

void Line::AddPoint(double _x, double _y)
{
	changedLine = true;
	if (_x < xMin)
		xMin = _x;
	if (_x > xMax)
		xMax = _x;
	if (_y < yMin)
		yMin = _y;
	if (_y > yMax)
		yMax = _y;
	x.push_back(_x);
	y.push_back(_y);
	
	//printf("New %s bounds (%1.2f, %1.2f, %1.2f, %1.2f)\n", name, xMin, yMax, xMax, yMin);
	
}

double Line::DistanceToLine(double xp, double yp)
{
	double bestDist = (y[0]-yp)*(y[0]-yp)+(x[0]-xp)*(x[0]-xp);
	// Linear search? - can be sped up with binary search if needed
	for (unsigned int m = 1; m < x.size()-1; m++)
	{
		if (fless((y[m]-yp)*(y[m]-yp)+(x[m]-xp)*(x[m]-xp), bestDist))
		{
			bestDist = (y[m]-yp)*(y[m]-yp)+(x[m]-xp)*(x[m]-xp);
		}
	}
	return sqrt(bestDist);
}

double Line::VerticalDistanceToLine(double xp, double yp)
{
	double horizDist = fabs(y[0]-yp);
	int horizIndex = 0;
	// Linear search? - can be sped up with binary search if needed
	for (unsigned int m = 1; m < x.size()-1; m++)
	{
		if (fabs(y[m]-yp) < horizDist)
		{
			horizDist = fabs(y[m]-yp);
			horizIndex = m;
		}
	}
	return fabs(x[horizIndex]-xp);
}

void Line::Smooth(unsigned int windowSize)
{
	double sum=0;
	// setup initial window
	for (unsigned int m = 0; (m < x.size()) && (m < windowSize); m++)
	{
		sum += y[m];
		y[m/2] = sum/(double)(m+1);
	}
	for (unsigned int m = windowSize/2; m < x.size()-windowSize/2-1; m++)
	{
		y[m] = sum/(double)windowSize;
		sum-=y[m-windowSize/2];
		sum+=y[m+windowSize/2];
	}
	for (unsigned int m = x.size()-windowSize/2-1; m < x.size(); m++)
	{
		y[m] = sum/(double)(windowSize/2+x.size()-1-m);
		sum-=y[m-windowSize/2];
	}
}

void Line::ClearColor()
{
	r = -1;
}

void Line::SetColor(const rgbColor &c)
{
	r = c.r;
	g = c.g;
	b = c.b;
}

void Line::SetColor(double _r, double _g, double _b)
{
	r = _r; g = _g; b = _b;
}

//void Line::OpenGLDraw() const
//{
//	if (hidden)
//		return;
//	switch (plotType)
//	{
//		case kLinePlot:
//		{
//			glBegin(GL_LINE_STRIP);
//			if (r != -1)
//			{
//				glColor3f(r, g, b);
//			}
//			for (unsigned int t = 0; t < x.size(); t++)
//				glVertex2d(x[t], y[t]);
//			glEnd();
//		}
//			break;
//		case kImpulsePlot:
//		{
//			glBegin(GL_LINES);
//			if (r != -1)
//			{
//				glColor3f(r, g, b);
//			}
//			for (unsigned int t = 0; t < x.size(); t++)
//			{
//				glVertex2d(x[t], 0);
//				glVertex2d(x[t], y[t]);
//			}
//			glEnd();
//		}
//			break;
//	}
//}

void Line::Draw(Graphics::Display &display, double xOff, double yOff, double xScale, double yScale) const
{
	if (x.size() == 0 || hidden)
		return;
	
	// Only does line plots right now
	std::vector<Graphics::point> points;
	for (unsigned int t = 0; t < x.size(); t++)
	{
		points.push_back({
			(float)((x[t]+xOff)*xScale),
			-(float)((y[t]+yOff)*yScale)
		});
	}
	//		printf("(%f, %f) -> (%f, %f)",
	//			   (float)((x[0]+xOff)*xScale), (float)-((y[0]+yOff)*yScale),
	//			   (float)((x[x.size()-1]+xOff)*xScale), (float)-((y[x.size()-1]+yOff)*yScale)
	//			   );
	float baseLineSize = 1.0/150.0;
	display.DrawLineSegments(points, width*baseLineSize, rgbColor(r, g, b));
//	display.DrawLineSegments(points, width*xScale, rgbColor(r, g, b));
}

void Point::Draw(Graphics::Display &display, double xOff, double yOff, double xScale, double yScale) const
{
	double xLoc = (x+xOff)*xScale;
	double yLoc = -(y+yOff)*yScale;
	display.FillCircle({(float)xLoc, (float)yLoc}, std::min(xScale, yScale)*r, c);
}

Plot2D::Plot2D()
{
	ResetAxis();
	drawMouse = true;
	lastSelectedLine = -1;
	recomputeBorders = true;
}

void Plot2D::Clear()
{
	lines.clear();
	points.clear();
	ResetAxis();
}

void Plot2D::SetXAxisLabel(const char *l)
{
	xLabel = l;
}

void Plot2D::SetYAxisLabel(const char *l)
{
	yLabel = l;

}


void Plot2D::AddLine(Line *l)
{
	lines.push_back(l);
	
	if (l->GetMaxX() > xMax)
		xMax = l->GetMaxX();
	if (l->GetMaxY() > yMax)
		yMax = l->GetMaxY();
	if (l->GetMinX() < xMin)
		xMin = l->GetMinX();
	if (l->GetMinY() < yMin)
		yMin = l->GetMinY();
	
	if (!forceAxis)
		ResetAxis();
	
	dLeft = xMin;
	dRight = xMax;
	dTop = yMax;
	dBottom = yMin;
	
}

void Plot2D::AddPoint(const Point &p)
{
	points.push_back(p);
	
	if (p.x > xMax)
		xMax = p.x;
	if (p.y > yMax)
		yMax = p.y;
	if (p.x < xMin)
		xMin = p.x;
	if (p.y < yMin)
		yMin = p.y;
	
	xMax = std::max(xMax, yMax);
	yMax = xMax;
	xMin = std::min(xMin, yMin);
	yMin = xMin;
	
	dLeft = xMin;
	dRight = xMax;
	dTop = yMax;
	dBottom = yMin;
	
	int val = 1;
	while (true)
	{
		val *= 2;
		if (val > xMax)
		{
			dRight = val;
			dTop = val;
			break;
		}
	}
}

void Plot2D::IncludeInX(double x)
{
	if (x > xMax)
		xMax = x;
	if (x < xMin)
		xMin = x;
	
	xMax = std::max(xMax, yMax);
	yMax = xMax;
	xMin = std::min(xMin, yMin);
	yMin = xMin;
	
	dLeft = xMin;
	dRight = xMax;
	dTop = yMax;
	dBottom = yMin;
	
	int val = 1;
	while (true)
	{
		val *= 2;
		if (val > xMax)
		{
			dRight = val;
			dTop = val;
			break;
		}
	}
}

void Plot2D::IncludeInY(double y)
{
	if (y > yMax)
		yMax = y;
	if (y < yMin)
		yMin = y;
	
	xMax = std::max(xMax, yMax);
	yMax = xMax;
	xMin = std::min(xMin, yMin);
	yMin = xMin;
	
	dLeft = xMin;
	dRight = xMax;
	dTop = yMax;
	dBottom = yMin;
	
	int val = 1;
	while (true)
	{
		val *= 2;
		if (val > xMax)
		{
			dRight = val;
			dTop = val;
			break;
		}
	}

}


void Plot2D::Zoom(double amt)
{
	printf("Got amt %1.2f\n", amt);
	double cx, cy;
	amt = 1-0.01*amt;
	cx = xMin+(xMax-xMin)/2;
	cy = yMin+(yMax-yMin)/2;
	double width = (xMax-xMin)*amt;
	double height = (yMax-yMin)*amt;
	
	xMin = cx - width/2;
	xMax = cx + width/2;
	yMin = cy - height/2;
	yMax = cy + height/2;
	
	dLeft = xMin;
	dRight = xMax;
	dTop = yMax;
	dBottom = yMin;
}

void Plot2D::SetAxis(double minx, double miny, double maxx, double maxy)
{
	forceAxis = true;
	xMax = maxx;
	xMin = minx;
	yMax = maxy;
	yMin = miny;
}

void Plot2D::ResetAxis()
{
	forceAxis = false;
	recomputeBorders = false;
	xMin = DBL_MAX;
	xMax = -DBL_MAX;
	yMin = DBL_MAX;
	yMax = -DBL_MAX;
	
	// recompute max/min stuff
	for (unsigned int x = 0; x < lines.size(); x++)
	{
		Line *l = lines[x];
		
		if (l->GetMaxX() > xMax)
			xMax = l->GetMaxX();
		if (l->GetMaxY() > yMax)
			yMax = l->GetMaxY();
		if (l->GetMinX() < xMin)
			xMin = l->GetMinX();
		if (l->GetMinY() < yMin)
			yMin = l->GetMinY();
	}
	
	for (const auto &p : points)
	{
		if (p.x > xMax)
			xMax = p.x;
		if (p.y > yMax)
			yMax = p.y;
		if (p.x < xMin)
			xMin = p.x;
		if (p.y < yMin)
			yMin = p.y;
	}
	
	if (xMin > xMax) // valid points
	{
		xMin = 0;
		xMax = 10;
		yMin = 0;
		yMax = 10;
	}
	
	dLeft = xMin;
	dRight = xMax;
	dTop = yMax;
	dBottom = yMin;
}

void Plot2D::OffsetCurrMouse(double deltaX, double deltaY)
{
	mouseXLoc += deltaX;
	mouseYLoc += deltaY;
	
	if (lines.size() == 0)
		return;
	
	double minDist = lines[0]->DistanceToLine(mouseXLoc, mouseYLoc);
	int minIndex = 0;
	for (unsigned int x = 1; x < lines.size(); x++)
	{
		if (lines[x]->IsHidden())
			continue;
		
		if (fless(lines[x]->DistanceToLine(mouseXLoc, mouseYLoc), minDist))
		{
			minIndex = x;
			minDist = lines[x]->DistanceToLine(mouseXLoc, mouseYLoc);
		}
		//		if (lines[x]->pointOnLine(mouseXLoc, mouseYLoc))
		//			printf("Near: %s\n", lines[x]->getLabel());
	}
	if (minIndex != lastSelectedLine)
	{
		// set colors here instead
		//			if (lastSelectedLine != -1)
		//				lines[lastSelectedLine]->unselect();
		//			lines[minIndex]->select();
		lastSelectedLine = minIndex;
		//			printf("Near: %s\n", lines[minIndex]->getLabel());
		//			submitMessage(lines[minIndex]->getLabel());
	}
}

//	void Plot2D::SetCurrMouse(double lastX, double lastY, Rect &winRect)
//	{
//		double tpW = (dRight-dLeft)*.05;
//		double tpH = (dTop-dBottom)*.05;
//		mouseXLoc = dLeft-tpW+(dRight+2*tpW - dLeft)*(lastX/winRect.right);
//		mouseYLoc = dBottom-tpH+(dTop+2*tpH - dBottom)*(1-(lastY/winRect.bottom));
//
//		if (lines.size() == 0)
//			return;
//		
//		double minDist = lines[0]->DistanceToLine(mouseXLoc, mouseYLoc);
//		int minIndex = 0;
//		for (unsigned int x = 1; x < lines.size(); x++)
//		{
//			if (fless(lines[x]->DistanceToLine(mouseXLoc, mouseYLoc), minDist))
//			{
//				minIndex = x;
//				minDist = lines[x]->DistanceToLine(mouseXLoc, mouseYLoc);
//			}
//	//		if (lines[x]->pointOnLine(mouseXLoc, mouseYLoc))
//	//			printf("Near: %s\n", lines[x]->getLabel());
//		}
//		if (minIndex != lastSelectedLine)
//		{
////			if (lastSelectedLine != -1)
////				lines[lastSelectedLine]->unselect();
////			lines[minIndex]->select();
//			lastSelectedLine = minIndex;
//			//printf("Near: %s\n", lines[minIndex]->getLabel());
//			//submitMessage(lines[minIndex]->getLabel());
//		}
//	}
//
//	void Plot2D::Recenter(double lastX, double lastY, Rect &winRect)
//	{
//		double tpW = (dRight-dLeft)*.05;
//		double tpH = (dTop-dBottom)*.05;
//		double XLoc = dLeft-tpW+(dRight+2*tpW - dLeft)*(lastX/winRect.right);
//		double YLoc = dBottom-tpH+(dTop+2*tpH - dBottom)*(1-(lastY/winRect.bottom));
//
//		xMin = XLoc - (dRight-dLeft)/2;
//		xMax = XLoc + (dRight-dLeft)/2;
//		yMin = YLoc - (dTop-dBottom)/2;
//		yMax = YLoc + (dTop-dBottom)/2;
//		
//		dLeft = xMin;
//		dRight = xMax;
//		dTop = yMax;
//		dBottom = yMin;
//		
//	}


void Plot2D::SmoothLines()
{
	for (unsigned int x = 0; x < lines.size(); x++)
		lines[x]->Smooth(50);
	ResetAxis();
}

void Plot2D::NormalizeAxes()
{
	xMax = std::max(xMax, yMax);
	yMax = xMax;
	xMin = std::min(xMin, yMin);
	yMin = xMin;
	
	dLeft = xMin;
	dRight = xMax;
	dTop = yMax;
	dBottom = yMin;
}

//void Plot2D::OpenGLDraw() const
//{
//	GLint matrixMode;
//	
//	if (recomputeBorders)
//		assert(false);
//	//			ResetAxis();
//	
//	glGetIntegerv(GL_MATRIX_MODE, &matrixMode);
//	
//	glEnable(GL_BLEND); // for text fading
//	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA); // ditto
//	
//	// set orthograhic 1:1  pixel transform in local view coords
//	glDisable(GL_DEPTH_TEST);
//	
//	glMatrixMode(GL_PROJECTION);
//	glPushMatrix();
//	glLoadIdentity();
//	glMatrixMode(GL_MODELVIEW);
//	glPushMatrix();
//	glLoadIdentity();
//	glColor3f(0, 1, 0);
//	
//	// left, right, bottom, top, near, far
//	double tpW = (dRight-dLeft)*.05;
//	double tpH = (dTop-dBottom)*.05;
//	glOrtho(dLeft-tpW, dRight+tpW, dBottom-tpH, dTop+tpH, -1, 1);
//	//printf("Drawing axis (%1.2f, %1.2f, %1.2f, %1.2f)\n", dLeft, dTop, dRight, dBottom);
//	
//	glColor3f(1, 1, 1); // draw axis
//	glBegin(GL_LINES);
//	glVertex2d(dLeft-tpW, 0); glVertex2d(dRight+tpW, 0);
//	glVertex2d(0, dBottom-tpH); glVertex2d(0, dTop+tpH);
//	glEnd();
//	
//	for (unsigned int x = 0; x < lines.size(); x++)
//	{
//		if (lines[x]->GetChanged())
//			assert(false);
//		//recomputeBorders = true;
//		lines[x]->OpenGLDraw();
//	}
//	
//	glLineWidth(3.0);
//	if (lastSelectedLine != -1)
//		lines[lastSelectedLine]->OpenGLDraw();
//	glLineWidth(1.0);
//	
//	// draw mouse - temporary hack
//	if (drawMouse)
//	{
//		glColor4f(0, 1, 0, .5); // draw axis
//		glBegin(GL_LINES);
//		glVertex2d(mouseXLoc, dBottom-tpH); glVertex2d(mouseXLoc, dTop+tpH);
//		glVertex2d(dLeft-tpW, mouseYLoc); glVertex2d(dRight+tpW, mouseYLoc);
//		glEnd();
//	}
//	
//	glPopMatrix(); // GL_MODELVIEW
//	glMatrixMode(GL_PROJECTION);
//	glPopMatrix();
//	glMatrixMode(matrixMode);
//	
//	glEnable(GL_DEPTH_TEST);
//}

point3d Plot2D::MakeHOG(double x, double y) const
{
	return point3d((x+xOffset)*xScale, -(y+yOffset)*yScale, 0);
}

double Plot2D::MakeHOGWidth(double w) const
{
	// If we scale the lines as above, they can get really large. The lines
	// need to always stay the same (relative) size.
	float baseLineSize = 1.0f/150.0f;
	return w*baseLineSize;
//	return w*xScale;
}

void Plot2D::Draw(Graphics::Display &display) const
{
	xOffset = (dRight-dLeft)/2.0-dRight;
	yOffset = (dTop-dBottom)/2.0-dTop;
	xScale = 0.9*2.0/(dRight-dLeft);
	yScale = 0.9*2.0/(dTop-dBottom);
	
	float lineAxisWeight = 2;//*baseLineSize;
	float lineTicWeight = 1;//*baseLineSize;

	display.FillRect({-1, -1, 1, 1}, Colors::white);
	display.DrawLine(MakeHOG(dLeft, 0), MakeHOG(dRight, 0), MakeHOGWidth(lineAxisWeight), Colors::black); // x-axis
	display.DrawLine(MakeHOG(0, dTop), MakeHOG(0, dBottom), MakeHOGWidth(lineAxisWeight), Colors::black); // y-axis

	double fontSize = MakeHOGWidth(10);//4;//*MakeHOGWidth((dTop-dBottom)/100);
	display.DrawText(xLabel.c_str(), MakeHOG(dRight+(dTop-dBottom)/100, 0), Colors::black, fontSize, Graphics::textAlignLeft, Graphics::textBaselineMiddle);

	display.DrawText(yLabel.c_str(), MakeHOG(0, dTop+(dTop-dBottom)/100), Colors::black, fontSize, Graphics::textAlignCenter, Graphics::textBaselineBottom);

	for (double x = 0; x < dRight; x+=dRight/10)
	{
		display.DrawLine(MakeHOG(x, 0)+point3d(0, fontSize/3), MakeHOG(x, 0)-point3d(0, fontSize/3), MakeHOGWidth(lineTicWeight), Colors::black); // x-axis
//		display.DrawLine(MakeHOG(x, (dTop-dBottom)/100), MakeHOG(x, -(dTop-dBottom)/100), MakeHOGWidth(lineTicWeight), Colors::black); // x-axis
	}
	for (double y = 0; y < dTop; y+=dTop/10)
	{
		display.DrawLine(MakeHOG(0, y)+point3d(fontSize/3,0), MakeHOG(0, y)-point3d(fontSize/3,0), MakeHOGWidth(lineTicWeight), Colors::black); // y-axis
//		display.DrawLine(MakeHOG((dRight-dLeft)/100, y), MakeHOG(-(dRight-dLeft)/100, y), MakeHOGWidth(lineTicWeight), Colors::black); // x-axis
	}
	
	for (unsigned int x = 0; x < lines.size(); x++)
	{
		lines[x]->Draw(display, xOffset, yOffset, xScale, yScale);
	}
	
	for (const auto i : points)
		i.Draw(display, xOffset, yOffset, xScale, yScale);
}

}
