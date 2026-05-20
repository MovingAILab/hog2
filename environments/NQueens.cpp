//
//  NQueens.cpp
//  hog2 glut
//
//  Created by Nathan Sturtevant on 9/18/11.
//  Copyright 2011 University of Denver. All rights reserved.
//

#include "NQueens.h"
#include "Constants.h"

void NQueens::GetSuccessors(const NQueenState &nodeID, std::vector<NQueenState> &neighbors) const
{
	
}

void NQueens::GetActions(const NQueenState &nodeID, std::vector<NQueenAction> &actions) const
{
	actions.resize(0);
	for (unsigned int x = 0; x < nodeID.locs.size(); x++)
	{
		for (unsigned int y = 0; y < nodeID.locs.size(); y++)
		{
			if (nodeID.locs[x] == y)
				continue;

			NQueenAction a;
			a.loc = x;
			a.value = y;
			actions.push_back(a);
		}
	}
}

NQueenAction NQueens::GetAction(const NQueenState &s1, const NQueenState &s2) const
{
	assert(false);
	NQueenAction a;
	return a;
}

void NQueens::ApplyAction(NQueenState &s, NQueenAction a) const
{
	s.locs[a.loc] = a.value;
}

void NQueens::GetNextState(const NQueenState &s, NQueenAction a, NQueenState &s2) const
{
	s2 = s;
	ApplyAction(s2, a);
}

/** Goal Test if the goal is stored **/
bool NQueens::GoalTest(const NQueenState &node) const
{
	for (int x = 0; x < node.locs.size(); x++)
	{
		int up = node.locs[x];
		if (up == -1) return false;		
		int down = up;
		int middle = up;
		
		for (int y = 1; y < node.locs.size(); y++)
		{
			up -= 1;
			down += 1;
			
			if ((x-y < 0) && (x+y >= node.locs.size()))
				break;

			if ((x+y < node.locs.size()) &&
				((node.locs[x+y] == up) ||
				 (node.locs[x+y] == down) ||
				 (node.locs[x+y] == middle)))
			{
				return false;
			}
			if ((x-y >= 0) &&
				((node.locs[x-y] == up) ||
				 (node.locs[x-y] == down) ||
				 (node.locs[x-y] == middle)))
			{
				return false;
			}
		}
	}
	return true;
}
int NQueens::NumCollisions(const NQueenState &node, int row, int column) const
{
	if (row > node.locs.size())
		return -1;
	int count = 0;
	int x = row;
	{
		int up = column;
		if (up == -1)
		{
			// ill formed state
			return -1;
		}
		int down = up;
		int middle = up;
		
		for (int y = 1; y < node.locs.size(); y++)
		{
			up -= 1;
			down += 1;
			
			if ((x-y < 0) && (x+y >= node.locs.size()))
				break;
			
			if (x+y < node.locs.size())
			{
				if (node.locs[x+y] == up) count++;
				if (node.locs[x+y] == down) count++;
				if (node.locs[x+y] == middle) count++;
			}
			if (x-y >= 0)
			{
				if (node.locs[x-y] == up) count++;
				if (node.locs[x-y] == down) count++;
				if (node.locs[x-y] == middle) count++;
			}
		}
	}
	return count;
}


int NQueens::NumCollisions(const NQueenState &node, int row) const
{
	if (row > node.locs.size())
		return -1;
	int count = 0;
	int x = row;
	{
		int up = node.locs[x];
		if (up == -1)
		{
			// ill formed state
			return -1;
		}
		int down = up;
		int middle = up;
		
		for (int y = 1; y < node.locs.size(); y++)
		{
			up -= 1;
			down += 1;
			
			if ((x-y < 0) && (x+y >= node.locs.size()))
				break;
			
			if (x+y < node.locs.size())
			{
				if (node.locs[x+y] == up) count++;
				if (node.locs[x+y] == down) count++;
				if (node.locs[x+y] == middle) count++;
			}
			if (x-y >= 0)
			{
				if (node.locs[x-y] == up) count++;
				if (node.locs[x-y] == down) count++;
				if (node.locs[x-y] == middle) count++;
			}
		}
	}
	return count;
}

int NQueens::NumCollisions(const NQueenState &node) const
{
	int count = 0;
	for (int x = 0; x < node.locs.size(); x++)
	{
		int up = node.locs[x];
		if (up == -1) 
		{
			// ill formed state
			continue;
		}
		int down = up;
		int middle = up;
		
		for (int y = 1; y < node.locs.size(); y++)
		{
			up -= 1;
			down += 1;

			if ((x-y < 0) && (x+y >= node.locs.size()))
				break;
			
			if (x+y < node.locs.size())
			{
				if (node.locs[x+y] == up) count++;
				if (node.locs[x+y] == down) count++;
				if (node.locs[x+y] == middle) count++;
			}
			if (x-y >= 0)
			{
				if (node.locs[x-y] == up) count++;
				if (node.locs[x-y] == down) count++;
				if (node.locs[x-y] == middle) count++;
			}
		}
	}
	return count;
}

//void NQueens::OpenGLDraw() const
//{
//	// no basic environment to draw
//}
//
//void NQueens::OpenGLDrawBackground(float r, float g, float b)
//{
//	glBegin(GL_QUADS);
//	glColor3f(r, g, b);
//	glVertex3d(-1.0,  1.0, 0.01);
//	glVertex3d(-1.0, -1.0, 0.01);
//	glVertex3d( 1.0, -1.0, 0.01);
//	glVertex3d( 1.0,  1.0, 0.01);
//	glEnd();
//}
//
//void NQueens::OpenGLDrawBackground(const NQueenState&s, float r, float g, float b, int firstRow, int lastRow)
//{
//	double fract = s.locs.size();
//	fract = 2.0/fract;
//	glColor3f(r, g, b);
//	glBegin(GL_QUADS);
//	glVertex3d(-1.0+firstRow*fract, 1, 0);
//	glVertex3d(-1.0+firstRow*fract, -1, 0);
//	glVertex3d(-1.0+lastRow*fract,  -1, 0);
//	glVertex3d(-1.0+lastRow*fract,  1, 0);
//	glEnd();
//}
//
//
//void NQueens::OpenGLDraw(const NQueenState&s) const
//{		
//	double fract = s.locs.size();
//	fract = 2.0/fract;
//	glBegin(GL_LINES);
//	glColor3f(1.0, 1.0, 1.0);
//	for (unsigned int x = 0; x <= s.locs.size(); x++)
//	{
//		glVertex3d(-1.0+x*fract, -1, 0);
//		glVertex3d(-1.0+x*fract,  1, 0);
//
//		glVertex3d(-1, -1.0+x*fract, 0);
//		glVertex3d( 1, -1.0+x*fract, 0);
//	}
//	glEnd();
//	glBegin(GL_QUADS);
//	for (unsigned int x = 0; x < s.locs.size(); x++)
//	{
//		int count;
//		if ((count = NumCollisions(s, x)) == 0)
//			glColor3f(0.2, 1.0, 0.2);
//		else
//			glColor3f(1.0, 0.1, 0.1);
//		glVertex3d(-1.0+x*fract, -1.0+s.locs[x]*fract, 0.0);
//		glVertex3d(-1.0+x*fract, -1.0+(1+s.locs[x])*fract, 0.0);
//		glVertex3d(-1.0+(x+1)*fract, -1.0+(1+s.locs[x])*fract, 0.0);
//		glVertex3d(-1.0+(x+1)*fract, -1.0+s.locs[x]*fract, 0.0);
//	}
//	glEnd();
//
//	if (0) // draw lines
//	{
//		glColor3f(1.0, 0, 0);
//		glLineWidth(3.0);
//		glBegin(GL_LINES);
//		for (int x = 0; x < s.locs.size(); x++)
//		{
//			int up = s.locs[x];
//			if (up == -1) continue;
//			int down = up;
//			for (int y = 1; y < s.locs.size(); y++)
//			{
//				up -= 1;
//				down += 1;
//				if (y == 1)
//					continue;
//				
//				// root location
//				glVertex3d(-1.0+x*fract+fract*0.5+(y-1)*fract, -1.0+s.locs[x]*fract+fract*0.5, 0.0);
//				// offset: right
//				glVertex3d(-1.0+x*fract+fract*0.5+y*fract, -1.0+s.locs[x]*fract+fract*0.5, 0.0);
//				
//				// root location
//				glVertex3d(-1.0+x*fract+fract*0.5-(y-1)*fract, -1.0+s.locs[x]*fract+fract*0.5, 0.0);
//				// offset: left
//				glVertex3d(-1.0+x*fract+fract*0.5-y*fract, -1.0+s.locs[x]*fract+fract*0.5, 0.0);
//				
//				// root location
//				glVertex3d(-1.0+x*fract+fract*0.5-(y-1)*fract, -1.0+(up+1)*fract+fract*0.5, 0.0);
//				// offset: up-left
//				glVertex3d(-1.0+x*fract+fract*0.5-y*fract, -1.0+up*fract+fract*0.5, 0.0);
//				
//				// root location
//				glVertex3d(-1.0+x*fract+fract*0.5-(y-1)*fract, -1.0+(down-1)*fract+fract*0.5, 0.0);
//				// offset: down-left
//				glVertex3d(-1.0+x*fract+fract*0.5-y*fract, -1.0+down*fract+fract*0.5, 0.0);
//				
//				// root location
//				glVertex3d(-1.0+x*fract+fract*0.5+(y-1)*fract, -1.0+(up+1)*fract+fract*0.5, 0.0);
//				// offset: up-right
//				glVertex3d(-1.0+x*fract+fract*0.5+y*fract, -1.0+up*fract+fract*0.5, 0.0);
//				
//				// root location
//				glVertex3d(-1.0+x*fract+fract*0.5+(y-1)*fract, -1.0+(down-1)*fract+fract*0.5, 0.0);
//				// offset: down-right
//				glVertex3d(-1.0+x*fract+fract*0.5+y*fract, -1.0+down*fract+fract*0.5, 0.0);
//				
//			}
//		}
//		glEnd();
//	}
//	glLineWidth(1.0);
//}
//
//void NQueens::OpenGLDrawConflicts(const NQueenState&s) const
//{
//	for (unsigned int x = 0; x < s.locs.size(); x++)
//	{
//		int count = NumCollisions(s, x);
//		GLLabelState(s, x, s.locs[x], count);
//		if (count != 0)
//		{
//			for (int y = 0; y < s.locs.size(); y++)
//			{
//				if (y != s.locs[x])
//					GLLabelState(s, x, y, NumCollisions(s, x, y));
//			}
//		}
//	}
//}
//
//
//void NQueens::GLLabelState(const NQueenState &s, int x, int y, int number) const
//{
//	glDisable(GL_LIGHTING);
//    glEnable(GL_LINE_SMOOTH);
//    glDisable(GL_DEPTH_TEST);
//	glLineWidth(3.0);
//
//	int w = (int)s.locs.size();
//	int h = (int)s.locs.size();
//	glPushMatrix();
//	glColor3f(1.0, 1.0, 1.0);
//	glTranslatef(x*2.0/w-1.0, (1+y)*2.0/h-1.0, -0.001);
//	glScalef(1.0/(w*120.0), 1.0/(h*120.0), 1);
//	glRotatef(180, 0.0, 0.0, 1.0);
//	glRotatef(180, 0.0, 1.0, 0.0);
//	//glTranslatef((float)x/width-0.5, (float)y/height-0.5, 0);
//	//if (number > 9)
//	//	glutStrokeCharacter(GLUT_STROKE_ROMAN, '0'+(((number)/10)%10));
//	//if (number > 0)
//	//	glutStrokeCharacter(GLUT_STROKE_ROMAN, '0'+((number)%10));
//	//glTranslatef(-x/width+0.5, -y/height+0.5, 0);
//	glPopMatrix();
//
//
//    glEnable(GL_DEPTH_TEST);
//	glEnable(GL_LIGHTING);
//    glDisable(GL_LINE_SMOOTH);
//	glLineWidth(1.0);
//}
//
///** Draw the transition at some percentage 0...1 between two states */
//void NQueens::OpenGLDraw(const NQueenState&, const NQueenState&, float) const
//{
//	
//}
//
//void NQueens::OpenGLDraw(const NQueenState&, const NQueenAction&) const
//{
//	return;
//}
