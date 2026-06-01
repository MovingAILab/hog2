//
//  ScreenTransition.cpp
//  puzzleSample
//
//  Created by Nathan Sturtevant on 5/22/19.
//  Copyright © 2019 University of Denver. All rights reserved.
//

#include "ScreenTransition.h"

LineTransition::LineTransition(int seg, int lps, rgbColor clr)
{
	numSegments = seg;
	linesPerSegment = lps;
	color = clr;
}

void LineTransition::SetColor(rgbColor c)
{
	color = c;
}

void LineTransition::Reset(float t)
{
	mTime = t;
}

bool LineTransition::Step(float delta)
{
	mTime += delta;
	if (mTime > 1)
		mTime = 1;
	if (mTime < 0)
		mTime = 0;
	if (mTime >= 1 || mTime <= 0)
		return true;
	return false;
}
void LineTransition::Draw(Graphics::Display &d)
{
//	numSegments = seg;
//	linesPerSegment = lps;
	float segmentHeight = 2.0/numSegments;
	for (int x = -2; x < numSegments+2; x++)
	{
		Graphics::rect r(-2, -1+x*segmentHeight, 2, -1+x*segmentHeight+mTime*segmentHeight);
		d.FillRect(r, color);
	}
}

FallingBoxTransition::FallingBoxTransition(int dimension)
:dim(dimension){}

void FallingBoxTransition::Reset(float t)
{
	mTime = t;
}
bool FallingBoxTransition::Step(float delta)
{
	mTime += delta;
	if (mTime > 1)
		mTime = 1;
	if (mTime < 0)
		mTime = 0;
	if (mTime >= 1 || mTime <= 0)
		return true;
	return false;
}

void FallingBoxTransition::Draw(Graphics::Display &d)
{
	float s = 1.0f/dim;
	for (float x = -2; x < 2; x += 2*s)
	{
		for (float y = -2; y < 2; y += 2*s)
		{
			d.FillSquare({x+s, y+s}, s*mTime, Colors::black);
		}
	}
}
