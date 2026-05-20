/*
 *  MapOverlay.cpp
 *  hog
 *
 *  Created by Nathan Sturtevant on 3/21/06.
 *  Copyright 2006 Nathan Sturtevant. All rights reserved.
 *
 */

#include "MapOverlay.h"
#include "FPUtil.h"
#include "Constants.h"
#include "SVGUtil.h"
#include "Colors.h"

const double stripResolution = 20;

MapOverlay::MapOverlay(Map *_m)
:m(_m), maxVal(0), minVal(0.0)
{
	values.resize(m->GetMapWidth()*m->GetMapHeight());
	colorMap = 4;
//	displayList = 0;
	drawBorders = false;
}

void MapOverlay::Clear()
{
//	if (displayList)
//	{
//		glDeleteLists(displayList, 1);
//		displayList = 0;
//	}
	
	for (unsigned int x = 0; x < values.size(); x++)
		values[x] = ignoreVal;

	maxVal = DBL_MIN;
	minVal = DBL_MAX;
}

void MapOverlay::resetValues()
{
//	if (displayList)
//	{
//		glDeleteLists(displayList, 1);
//		displayList = 0;
//	}
	maxVal = DBL_MIN;
	minVal = DBL_MAX;
	for (unsigned int t = 0; t < values.size(); t++)
	{
		if (!fequal(values[t], ignoreVal))
		{
			if (fgreater(values[t], maxVal))
				maxVal = values[t];
			if (fless(values[t], minVal))
				minVal = values[t];
		}
	}
}

void MapOverlay::SetOverlayValue(int x, int y, double value)
{
	if ((x < 0) || (x >= m->GetMapWidth()) || (y < 0) || (y >= m->GetMapHeight()))
		return;
//	if (displayList)
//	{
//		glDeleteLists(displayList, 1);
//		displayList = 0;
//	}
	
	values[y*m->GetMapWidth()+x] = value;
	if (value > maxVal)
		maxVal = value;
	if (value < minVal)
		minVal = value;
//	printf("Min: %f; max: %f\n", minVal, maxVal);
	//resetValues();
}

double MapOverlay::GetOverlayValue(int x, int y)
{
	if ((x < 0) || (x >= m->GetMapWidth()) || (y < 0) || (y >= m->GetMapHeight()))
		return 0;
	return values[y*m->GetMapWidth()+x];
}

rgbColor MapOverlay::GetValueColor(double value) const
{
	int v = (int)value;
	auto loc = colors.find(v);
	if (loc == colors.end() || colorMap != -1)
		return Colors::GetColor(value, minVal, maxVal, 0);
	return loc->second;
}

//void MapOverlay::OpenGLDraw() const
//{
//	if (displayList)
//	{
//		glCallList(displayList);
//	}
//	else if (!fequal(minVal, maxVal)) // we get unddefined behavior when they are equal`
//	{
//		
//		displayList = glGenLists(1);
//		glNewList(displayList, GL_COMPILE_AND_EXECUTE);
//		
//		glBegin(GL_QUADS);
//		glNormal3f(0, 0, -1);
//		for (unsigned int t = 0; t < values.size(); t++)
//		{
//			//printf("Drawing: (%ld, %ld): %f\n", t%m->GetMapWidth(), t/m->GetMapWidth(), values[t]);
//			if (fequal(values[t], ignoreVal))
//			{
//				//glColor4f(0.5, 0.5, 0.5, 0.5);
//				continue;
//			}
//			else {
//				rgbColor r;
//				if (colorMap == customColorMap)
//					r = GetValueColor(values[t]);
//				else
//					r = Colors::GetColor(values[t], minVal, maxVal, colorMap);
//				glColor3f(r.r, r.g, r.b);
//			}
//			unsigned int last;
//			for (last = t+1;
//					 last < values.size() && (last%(m->GetMapWidth()) != 0) &&
//					 fequal(values[t], values[last]); last++)
//			{}
//			last -= 1;
//			GLdouble coverage = 1.0;
//			GLdouble a, b, c, radius;
//			m->GetCoord((int)(t%(m->GetMapWidth())), (int)(t/m->GetMapWidth()), a, b, c, radius);
//			glVertex3f(a-coverage*radius, b+coverage*radius, c-0.0001);//+1.01*radius);
//			glVertex3f(a-coverage*radius, b-coverage*radius, c-0.0001);//+1.01*radius);
//			m->GetCoord((int)(last%(m->GetMapWidth())), (int)(last/m->GetMapWidth()), a, b, c, radius);
//			glVertex3f(a+coverage*radius, b-coverage*radius, c-0.0001);//+1.01*radius);
//			glVertex3f(a+coverage*radius, b+coverage*radius, c-0.0001);//+1.01*radius);
//			t = last;
//		}
//		glEnd();
//
//		// borders between regions
//		if (drawBorders)
//		{
//			glLineWidth(2.0);
//			glColor3f(0.0, 0.0, 0.0);
//			glBegin(GL_LINES);
//			for (int y = 0; y < m->GetMapHeight()-1; y++)
//			{
//				for (int x = 0; x < m->GetMapWidth()-1; x++)
//				{
//					int index = x+y*m->GetMapWidth();
//					if (values[index] != values[index+1])
//					{
//						GLdouble a, b, c, r;
//						m->GetCoord(x, y, a, b, c, r);
//						glVertex3f(a+r, b+r, c-1.12*r);
//						glVertex3f(a+r, b-r, c-1.12*r);
//					}
//					if (values[index] != values[index+m->GetMapWidth()])
//					{
//						GLdouble a, b, c, r;
//						m->GetCoord(x, y, a, b, c, r);
//						glVertex3f(a-r, b+r, c-1.12*r);
//						glVertex3f(a+r, b+r, c-1.12*r);
//					}
//				}
//			}
//			glEnd();
//		}
//		
//
//		if (1)// drawColorBar
//		{
//			// black background
//			glBegin(GL_QUADS);
//			glNormal3f(0, 0, -1);
//			GLdouble a, b, c, radius;
//			m->GetCoord(0, 0, a, b, c, radius);
//			double wide = radius*2.0*m->GetMapWidth()/100.0;
//			glColor3f(0.2, 0.2, 0.2);
//			glVertex3f(a-wide, b+2*m->GetMapHeight()*radius+wide, c-4*radius);
//			glVertex3f(a-4*wide, b+2*m->GetMapHeight()*radius+wide, c-4*radius);
//			glVertex3f(a-4*wide, b-wide, c-4*radius);
//			glVertex3f(a-wide, b-wide, c-4*radius);
//			glEnd();
//
//			char text[255];
//			sprintf(text, "%1.2f", minVal);
//			glColor3f(1.0, 1.0, 1.0);
//			DrawText(a-wide, b+2*m->GetMapHeight()*radius+wide, c-3*radius, 0.1, text);
//			sprintf(text, "%1.2f", maxVal);
//			DrawText(a-wide, b-wide+0.05, c-3*radius, 0.1, text);
//
//			// white border
//			glBegin(GL_LINE_LOOP);
//			glColor3f(1.0, 1.0, 1.0);
//			glVertex3f(a-wide, b+2*m->GetMapHeight()*radius+wide, c-4*radius-radius/10.0);
//			glVertex3f(a-4*wide, b+2*m->GetMapHeight()*radius+wide, c-4*radius-radius/10.0);
//			glVertex3f(a-4*wide, b-wide, c-4*radius);
//			glVertex3f(a-wide, b-wide, c-4*radius);
//			glEnd();
//
//			// color bar on the side
//			glBegin(GL_QUAD_STRIP);
//			glNormal3f(0, 0, -1);
//			m->GetCoord(0, 0, a, b, c, radius);
//			for (int x = 0; x <= stripResolution; x++)
//			{
//				rgbColor r = Colors::GetColor(minVal + (maxVal-minVal)*((double)x/stripResolution),
//									  minVal, maxVal, colorMap);
//				glColor3f(r.r, r.g, r.b);
//				glVertex3f(a-2*wide, b+2*m->GetMapHeight()*radius*(1.0-(double)x/stripResolution), c-4*radius-radius/8.0);
//				glVertex3f(a-3*wide, b+2*m->GetMapHeight()*radius*(1.0-(double)x/stripResolution), c-4*radius-radius/8.0);
//			}
//			glEnd();
//			
//			if (maxVal-minVal != 0)
//			{
//				double mult = 1.0;
//				double range = 1.0;
//				while (1)
//				{
//					if (!(fless((maxVal-minVal)*mult, 10.0)))
//						mult /= 10;
//					else if (fless((maxVal-minVal)*mult, 1.0))
//						mult *= 10;
//					else {
//						if ((maxVal-minVal)*mult >= 5)
//							range = 1.0;
//						else if ((maxVal-minVal)*mult >= 3)
//							range = 0.5;
//						else
//							range = 0.1;
//						break;
//					}
//				}
//				glBegin(GL_LINES);
//				glColor3f(1.0, 1.0, 1.0);
//				//double top = (double)((int)(maxVal*mult))/mult;
//				double bottom = (double)((int)(minVal*mult))/mult;
//				//			printf("min/max %f, %f, stepped min/max %f, %f\n",
//				//						 minVal, maxVal, bottom, top);
//				//			printf("Range: %f, mult %f\n", range, mult);
//				for (double x = bottom; !fgreater(x, maxVal); x+=range/mult)
//				{
//					if (fequal(x, 0))
//						glColor3f(1.0, 0, 0);
//					glVertex3f(a-1*wide, b+2*m->GetMapHeight()*radius*(1.0-(x-minVal)/(maxVal-minVal)), c-4*radius-radius/9.0);
//					glVertex3f(a-4*wide, b+2*m->GetMapHeight()*radius*(1.0-(x-minVal)/(maxVal-minVal)), c-4*radius-radius/9.0);
//					if (fequal(x, 0))
//						glColor3f(1.0, 1.0, 1.0);
//				}
//				for (double x = bottom-range/mult; !fless(x, minVal); x-=range/mult)
//				{
//					if (fequal(x, 0))
//						glColor3f(1.0, 0, 0);
//					glVertex3f(a-1*wide, b+2*m->GetMapHeight()*radius*(1.0-(x-minVal)/(maxVal-minVal)), c-4*radius-radius/9.0);
//					glVertex3f(a-4*wide, b+2*m->GetMapHeight()*radius*(1.0-(x-minVal)/(maxVal-minVal)), c-4*radius-radius/9.0);
//					if (fequal(x, 0))
//						glColor3f(1.0, 1.0, 1.0);
//				}
//				glEnd();
//			}
//		}
//		
//		glEndList();
//	}
//}

void MapOverlay::Draw(Graphics::Display &display) const
{
	for (unsigned int t = 0; t < values.size(); t++)
	{
		if (fequal(values[t], ignoreVal))
		{
			continue;
		}
		else {
			rgbColor r = Colors::GetColor(values[t], minVal, maxVal, colorMap);
			double x, y, z, rad;
			m->GetCoord(static_cast<int>(t%m->GetMapWidth()), static_cast<int>(t/m->GetMapWidth()), x, y, z, rad);
			display.FillRect({static_cast<float>(x-rad), static_cast<float>(y-rad), static_cast<float>(x+rad), static_cast<float>(y+rad)}, r);
			//s += SVGDrawRect(t%m->GetMapWidth()+1, t/m->GetMapWidth()+1, 1, 1, r);
		}
	}

}

std::string MapOverlay::SVGDraw() const
{
	std::string s;
	char s1[255];
	char s2[255];
	sprintf(s1, "%1.2f", maxVal);
	sprintf(s2, "%1.2f", minVal);
	s += "\n";
	int margin = std::max(1.0, m->GetMapWidth()/40.0);
	s += SVGDefineGradient(true, true,
						   Colors::GetColor(maxVal, minVal, maxVal, colorMap),
						   Colors::GetColor(minVal, minVal, maxVal, colorMap), "sideBar");
	s += "\n";
	s += SVGDrawRect(-margin, 1, margin, m->GetMapHeight(), "sideBar");
	s += SVGFrameRect(-margin, 1, margin, m->GetMapHeight(), 1, Colors::darkgray);
	s += "\n";
	s += SVGDrawText(0, 1+margin, s1, Colors::black, margin);
	s += "\n";
	s += SVGDrawText(0, m->GetMapHeight()+1, s2, Colors::black, margin);
	s += "\n";
	for (unsigned int t = 0; t < values.size(); t++)
	{
		if (fequal(values[t], ignoreVal))
		{
			continue;
		}
		else {
			rgbColor r = Colors::GetColor(values[t], minVal, maxVal, colorMap);
			s += SVGDrawRect(t%m->GetMapWidth()+1, t/m->GetMapWidth()+1, 1, 1, r);
		}
	}
	
	rgbColor black = {0, 0, 0};

//	// terrain borders
//	for (int y = 0; y < m->GetMapHeight()-1; y++)
//	{
//		for (int x = 0; x < m->GetMapWidth()-1; x++)
//		{
//			if ((m->GetTerrainType(x, y)>>terrainBits) != (m->GetTerrainType(x+1, y)>>terrainBits))
//			{
//				s += SVGDrawLine(x+1, y, x+1, y+1, 2, black, false);
//			}
//			if ((m->GetTerrainType(x, y)>>terrainBits) != (m->GetTerrainType(x, y+1)>>terrainBits))
//			{
//				s += SVGDrawLine(x, y+1, x+1, y+1, 2, black, false);
//			}
//		}
//	}
	
	return s;
	
	// value borders
	for (int x = 0; x < m->GetMapWidth()-1; x++)
	{
		for (int y = 0; y < m->GetMapHeight()-1; y++)
		{
			int index = x+y*m->GetMapWidth();
			if (values[index] != values[index+1])
			{
				s += SVGDrawLine(x+1, y, x+1, y+1, 2, black, false);
			}
			if (values[index] != values[index+m->GetMapWidth()])
			{
				s += SVGDrawLine(x, y+1, x+1, y+1, 2, black, false);
			}
		}
	}

	return s;
}
