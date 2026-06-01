/*
 *  FlipSide.cpp
 *  hog2
 *
 *  Created by Nathan Sturtevant on 5/22/07.
 *  Copyright 2007 Nathan Sturtevant, University of Alberta. All rights reserved.
 *
 */

#include "FlipSide.h"

FlipSide::FlipSide(int _width)
:width(_width)
{
}

FlipSide::~FlipSide()
{
}

void FlipSide::GetSuccessors(const FlipSideState &stateID, std::vector<FlipSideState> &neighbors) const
{
	std::vector<flipMove> acts;
	GetActions(stateID, acts);
	neighbors.resize(0);
	for (unsigned int x = 0; x < acts.size(); x++)
	{
		FlipSideState s = stateID;
		ApplyAction(s, acts[x]);
		neighbors.push_back(s);
	}
}

void FlipSide::GetActions(const FlipSideState &stateID, std::vector<flipMove> &actions) const
{
	actions.resize(0);
	for (unsigned int x = 0; x < stateID.width-2; x++)
	{
		for (unsigned int y = 0; y < stateID.width-2; y++)
		{
			actions.push_back(flipMove(x, y));
		}
	}
}

flipMove FlipSide::GetAction(const FlipSideState &, const FlipSideState &) const
{
	assert(false);
	return flipMove();
}

void FlipSide::ApplyAction(FlipSideState &s, flipMove a) const
{
	for (int x = 0; x < 3; x++)
	{
		int tmp = s.puzzle[a.top+x];
		s.puzzle[a.top+x] = s.puzzle[s.width+a.bottom+x];
		s.puzzle[s.width+a.bottom+x] = tmp;
	}
}

double FlipSide::HCost(const FlipSideState &state1, const FlipSideState &state2) const
{
	assert(state1.width==state2.width);
	std::vector<unsigned int> xloc(state2.width*2);
	std::vector<unsigned int> yloc(state2.width*2);
	double hval = 0;
	
	for (unsigned int x = 0; x < state2.width; x++)
	{
		for (unsigned int y = 0; y < 2; y++)
		{
			xloc[state2.puzzle[x + y*state2.width]] = x;
			yloc[state2.puzzle[x + y*state2.width]] = y;
		}
	}
	for (unsigned int x = 0; x < state1.width; x++)
	{
		for (unsigned int y = 0; y < 2; y++)
		{
			if ((yloc[state1.puzzle[x + y*state1.width]] == y) &&
					(xloc[state1.puzzle[x + y*state1.width]] == x))
			{
			}
			else if (yloc[state1.puzzle[x + y*state1.width]] == y)
			{
				int tmp = (xloc[state1.puzzle[x + y*state1.width]] - x)/4;
				hval += (1+tmp)*2;
			}
			else {
				int tmp = ((xloc[state1.puzzle[x + y*state1.width]] - x)+1)/4;
				hval += 1+tmp*2;
			}

			int tmp = (xloc[state1.puzzle[x + y*state1.width]] - x)/2;
			if (xloc[state1.puzzle[x + y*state1.width]] != x)
			{
				if ((tmp%2) != (yloc[state1.puzzle[x + y*state1.width]] - y))
					tmp+=1;
				else if (tmp == 0)
					tmp += 2;
			}
			hval += tmp;
		}
	}
	return ceil(hval/6);
}

double FlipSide::GCost(const FlipSideState &, const FlipSideState &) const
{
	return 1;
}

bool FlipSide::GoalTest(const FlipSideState &state, const FlipSideState &goal) const
{
	return (state == goal);
}

uint64_t FlipSide::GetStateHash(const FlipSideState &state) const
{
	uint64_t hashVal = 0;
	for (unsigned int x = 0; x < state.puzzle.size(); x++)
	{
		hashVal = hashVal*state.puzzle.size() + state.puzzle[x];
	}
	return hashVal;
}

uint64_t FlipSide::GetActionHash(flipMove act) const
{
	return (act.top<<16)|act.bottom;
}

//void FlipSide::OpenGLDraw() const
//{
//}
//
//
//void FlipSide::OpenGLDraw(const FlipSideState &s) const
//{
//	glLineWidth(1.0);
//	glEnable(GL_LINE_SMOOTH);
//	
//	float w = width;
//	float h = 2;
//	float fscale = 120;
//	
//	for (int y = 0; y < 2; y++)
//	{
//		for (int x = 0; x < width; x++)
//		{
//			glPushMatrix();
//			glColor3f(0.0, 1.0, 0.0);
//			glTranslatef(x*2.0/w-1.0+1/(2*w), (1+y)*2.0/h-1.0-1/(2*h), -0.001);
//			glScalef(1.0/(w*fscale), 1.0/(h*fscale), 1);
//			glRotatef(180, 0.0, 0.0, 1.0);
//			glRotatef(180, 0.0, 1.0, 0.0);
//			//glTranslatef((float)x/width-0.5, (float)y/2-0.5, 0);
//			//if (s.puzzle[x+y*width] > 9)
//			//	glutStrokeCharacter(GLUT_STROKE_ROMAN, '0'+(((s.puzzle[x+y*width])/10)%10));
//			//glutStrokeCharacter(GLUT_STROKE_ROMAN, '0'+((s.puzzle[x+y*width])%10));
//			//glTranslatef(-x/width+0.5, -y/2+0.5, 0);
//			glPopMatrix();
//		}
//	}
//	
//	glBegin(GL_LINES);
//	for (int y = 0; y <= 2; y++)
//	{
//		for (int x = 0; x <= width; x++)
//		{
//			glVertex3f(x*2.0/w-1.0, -1, -0.001);
//			glVertex3f(x*2.0/w-1.0, 1, -0.001);
//			glVertex3f(-1, (y)*2.0/h-1.0, -0.001);
//			glVertex3f(1, (y)*2.0/h-1.0, -0.001);
//		}
//	}
//	glEnd();
//	
//	//glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
//	//glEnable(GL_BLEND);
//	//glEnable(GL_LINE_SMOOTH);
//	//output(200, 225, "This is antialiased.");
//	
//	//int width, height;
//	//std::vector<int> puzzle;
//}
