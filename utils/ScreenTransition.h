//
//  ScreenTransition.h
//  puzzleSample
//
//  Created by Nathan Sturtevant on 5/22/19.
//  Copyright © 2019 University of Denver. All rights reserved.
//

#ifndef ScreenTransition_h
#define ScreenTransition_h

#include <stdio.h>
#include "Graphics.h"

class ScreenTransition {
public:
	// Time 0 is empty; time 1 is full
	virtual void Reset(float time) = 0;
	// Adjust time (forward or backwards)
	// return true when complete (0 or 1)
	virtual bool Step(float time) = 0;
	virtual void Draw(Graphics::Display &d) = 0;
};

class LineTransition : public ScreenTransition {
public:
	LineTransition(int numSegments, int linesPerSegment, rgbColor color = Colors::black);
	virtual void Reset(float t);
	virtual bool Step(float delta) ;
	virtual void Draw(Graphics::Display &d) ;
	void SetColor(rgbColor c);
private:
	float mTime;
	int numSegments, linesPerSegment;
	rgbColor color;
};

class FallingBoxTransition  : public ScreenTransition {
public:
	FallingBoxTransition(int dimension);
	virtual void Reset(float t);
	virtual bool Step(float delta) ;
	virtual void Draw(Graphics::Display &d) ;
private:
	float mTime;
	int dim;
};

#endif /* ScreenTransition_h */
