/*
 *  Fling.cpp
 *  hog2
 *
 *  Created by Nathan Sturtevant on 3/5/10.
 *  Copyright 2010 NS Software. All rights reserved.
 *
 */

#include "Fling.h"
#include <stdint.h>
#include <string.h>
#include <algorithm>

void FlingBoard::SetPiece(int which)
{
	uint64_t val = (1ull<<which);
	board |= val;
}

void FlingBoard::ClearPiece(int which)
{
	uint64_t val = (1ull<<which);
	board &= (~val);
}

void FlingBoard::SetHole(int which)
{
	uint64_t val = (1ull<<which);
	holes |= val;
}

void FlingBoard::ClearHole(int which)
{
	uint64_t val = (1ull<<which);
	holes &= (~val);
}

void FlingBoard::SetObstacle(int which)
{
	uint64_t val = (1ull<<which);
	obstacles |= val;
}

void FlingBoard::ClearObstacle(int which)
{
	uint64_t val = (1ull<<which);
	obstacles &= (~val);
}

bool FlingBoard::HasPiece(int offset) const
{
	uint64_t val = (1ull<<offset);
	return (board & val);
}

bool FlingBoard::HasPiece(int x, int y) const
{
	uint64_t val = (1ull<<(y*width+x));
	return (board & val);
}

bool FlingBoard::HasHole(int offset) const
{
	uint64_t val = (1ull<<offset);
	return (holes & val);
}

bool FlingBoard::HasHole(int x, int y) const
{
	uint64_t val = (1ull<<(y*width+x));
	return (holes & val);
}

bool FlingBoard::HasObstacle(int offset) const
{
	uint64_t val = (1ull<<offset);
	return (obstacles & val);
}

bool FlingBoard::HasObstacle(int x, int y) const
{
	uint64_t val = (1ull<<(y*width+x));
	return (obstacles & val);
}

void FlingBoard::AddFling(unsigned int x, unsigned int y)
{
	assert(x < width);
	assert(y < height);
	//if (board[y*width+x] == false)
	if (HasPiece(y*width+x) == false)
	{
		SetPiece(y*width+x);
		//board[y*width+x] = true;
		locs.push_back({y*width+x, currId++});
		
		std::sort(locs.begin(), locs.end(), std::greater<std::pair<int,int>>());
	}
}

void FlingBoard::AddFling(unsigned int offset)
{
	assert(offset < width*height);
	//assert(offset < board.size());
	assert(HasPiece(offset) == false);
	//assert(board[offset] == false);
	SetPiece(offset);
	//board[offset] = true;
	locs.push_back({offset, currId++});
	std::sort(locs.begin(), locs.end(), std::greater<std::pair<int,int>>());
}

void FlingBoard::RemoveFling(unsigned int x, unsigned int y)
{
	assert(x < width);
	assert(y < height);
	RemoveFling(y*width+x);
}

void FlingBoard::RemoveFling(unsigned int offset)
{
	ClearPiece(offset);
	//board[offset] = false;
	for (unsigned int x = 0; x < width*height; x++)
	{
		if (locs[x].first == offset)
		{
			// TODO: just use erase here
			locs[x] = locs.back();
			locs.pop_back();
			std::sort(locs.begin(), locs.end(), std::greater<std::pair<int,int>>());
			return;
		}
	}
}

bool FlingBoard::CanMove(int which, int x, int y) const
{
	int xx = which%width;
	int yy = which/width;

	xx+=x; yy += y;
	
	bool first = true;
	while ((xx >= 0) && (xx < width) && (yy >= 0) && (yy < height))
	{
		//if (board[yy*width+xx])
		if (HasPiece(yy*width+xx) || (HasObstacle(yy*width+xx)))
			return !first;
		if (HasHole(yy*width+xx))
			return false;
		first = false;
		xx+=x; yy += y;
	}
	return false;

}

int FlingBoard::GetIndexInLocs(int offset) const
{
	// could do a binary search; TODO: profile to see if necessary
	for (size_t i = 0; i < locs.size(); i++)
	{
		if (locs[i].first == offset)
		{
			return i;
		}
	}
	assert(!"Didn't find index of location!");
	return -1;
}

int FlingBoard::GetIndexInLocs(int x, int y) const
{
	return GetIndexInLocs(y*width+x);
}

void FlingBoard::Move(int which, int x, int y)
{
	int xx = which%width;
	int yy = which/width;

	int index = GetIndexInLocs(xx, yy);
	
	int lastx = xx;
	int lasty = yy;
	xx+=x; yy += y;
	while ((xx >= 0) && (xx < width) && (yy >= 0) && (yy < height))
	{
		//board[lasty*width+lastx] = board[yy*width+xx];
		if (HasPiece(yy*width+xx))
		{
			SetPiece(lasty*width+lastx);
			locs[index].first = lasty*width+lastx;
			index = GetIndexInLocs(xx, yy);
		}
		else if (HasObstacle(yy*width+xx))
		{
			SetPiece(lasty*width+lastx);
			locs[index].first = lasty*width+lastx;
			// no piece can be on the obstacle, so we
			// let that location be cleared
			std::sort(locs.begin(), locs.end(), std::greater<std::pair<int,int>>());
//			locs.resize(0);
//			for (int t = width*height-1; t >= 0; t--)
//				if (HasPiece(t))//[t])
//					locs.push_back(t);
			return;
		}
		else {
			ClearPiece(lasty*width+lastx);
		}
		lastx = xx;
		lasty = yy;
		xx+=x; yy += y;
	}
	ClearPiece(lasty*width+lastx);
	locs.erase(locs.begin()+index);
	std::sort(locs.begin(), locs.end(), std::greater<std::pair<int,int>>());
	//board[lasty*width+lastx] = false;
//	locs.resize(0);
//	for (int t = width*height-1; t >= 0; t--)
//		if (HasPiece(t))//[t])
//			locs.push_back(t);
}

int FlingBoard::LocationAfterAction(FlingMove m)
{
	int xx = m.startLoc%width;
	int yy = m.startLoc/width;
	int x, y;

	switch (m.dir)
	{
		case kRight: x = 1;  y = 0;  break;
		case kLeft:  x = -1; y = 0;  break;
		case kUp:    x = 0;  y = -1; break;
		case kDown:  x = 0;  y = 1;  break;
	}
	
	int lastx = xx;
	int lasty = yy;
	xx+=x; yy += y;
	while ((xx >= 0) && (xx < width) && (yy >= 0) && (yy < height))
	{
		//board[lasty*width+lastx] = board[yy*width+xx];
		if (HasPiece(yy*width+xx) || HasObstacle(yy*width+xx))
		{
			return lasty*width+lastx;
		}
		lastx = xx;
		lasty = yy;
		xx+=x; yy += y;
	}
	assert(!"No collision found for piece");
	return -1;
}


void GetMirror(const FlingBoard &in, FlingBoard &out, bool h, bool v)
{
	out = in;
	out.Reset();
	for (size_t x = 0; x < in.width; x++)
	{
		for (size_t y = 0; y < in.height; y++)
		{
			if (in.HasPiece(x, y))
			{
				int newx = x;
				int newy = y;
				if (h)
					newx = in.width-x-1;
				if (v)
					newy = in.height-y-1;
				out.AddFling(newx, newy);
			}
		}
	}
}

void ShiftToCorner(FlingBoard &in)
{
	bool done = false;
	while (1)
	{
		// find piece
		for (size_t x = 0; x < in.width; x++)
		{
			if (in.HasPiece(x, 0))
			{
				done = true;
				break;
			}
		}
		
		if (done) break;
		// move over
		for (size_t y = 1; y < in.height; y++)
		{
			for (size_t x = 0; x < in.width; x++)
			{
				if (in.HasPiece(x, y))
				{
					in.AddFling(x, y-1);
					in.RemoveFling(x, y);
				}
			}
		}
	}
	while (1)
	{
		for (size_t y = 0; y < in.height; y++)
		{
			if (in.HasPiece(0, y))
			{
				return;
			}
		}
		
		for (size_t x = 1; x < in.width; x++)
		{
			for (size_t y = 0; y < in.height; y++)
			{
				if (in.HasPiece(x, y))
				{
					in.AddFling(x-1, y);
					in.RemoveFling(x, y);
				}
			}
		}
	}

}

uint64_t GetCanonicalHash(uint64_t which)
{
	Fling f;
	FlingBoard b;
	f.GetStateFromHash(which, b);
	ShiftToCorner(b);
	which = f.GetStateHash(b);

	FlingBoard flip;
	GetMirror(b, flip, true, true);
	ShiftToCorner(flip);
	
	if (f.GetStateHash(flip) < which)
		which = f.GetStateHash(flip);

	GetMirror(b, flip, true, false);
	ShiftToCorner(flip);
	if (f.GetStateHash(flip) < which)
		which = f.GetStateHash(flip);

	GetMirror(b, flip, false, true);
	ShiftToCorner(flip);
	if (f.GetStateHash(flip) < which)
		which = f.GetStateHash(flip);

	return which;
}

Fling::Fling()
{
	specificGoalLoc = false;
	specificGoalPanda = false;
	initBinomial();
	//		initBinomialSums();
}

void Fling::SetGoalPanda(int which)
{
	specificGoalPanda = true;
	goalLoc = which;
}

void Fling::ClearGoalPanda()
{
	specificGoalPanda = false;
}

void Fling::SetGoalLoc(int val)
{
	specificGoalLoc = true;
	goalLoc = val;
}

void Fling::ClearGoalLoc()
{
	specificGoalLoc = false;
}

bool Fling::GoalTest(const FlingBoard &node, const FlingBoard &goal) const
{
	if (specificGoalLoc)
		return (node.locs.size() == 1 && node.locs[0].first == goalLoc);
	if (specificGoalPanda)
	{
		return (node.locs.size() == 1 && node.locs[0].second == goalLoc);
	}
	return (node.locs.size() == 1);
}


void Fling::GetSuccessors(const FlingBoard &nodeID, std::vector<FlingBoard> &neighbors) const
{
	neighbors.resize(0);
	for (unsigned int x = 0; x < nodeID.locs.size(); x++)
	{
		if (nodeID.CanMove(nodeID.locs[x].first, 1, 0))
		{
			FlingBoard b(nodeID);
			b.Move(nodeID.locs[x].first, 1, 0);
			neighbors.push_back(b);
		}
		if (nodeID.CanMove(nodeID.locs[x].first, -1, 0))
		{
			FlingBoard b(nodeID);
			b.Move(nodeID.locs[x].first, -1, 0);
			neighbors.push_back(b);
		}
		if (nodeID.CanMove(nodeID.locs[x].first, 0, 1))
		{
			FlingBoard b(nodeID);
			b.Move(nodeID.locs[x].first, 0, 1);
			neighbors.push_back(b);
		}
		if (nodeID.CanMove(nodeID.locs[x].first, 0, -1))
		{
			FlingBoard b(nodeID);
			b.Move(nodeID.locs[x].first, 0, -1);
			neighbors.push_back(b);
		}
	}
}

void Fling::GetActions(const FlingBoard &nodeID, std::vector<FlingMove> &actions) const
{
	actions.resize(0);
	FlingMove m;
	for (size_t x = 0; x < nodeID.locs.size(); x++)
	{
		if (nodeID.CanMove(nodeID.locs[x].first, 1, 0))
		{
			m.startLoc = nodeID.locs[x].first;
			m.dir = kRight;
			actions.push_back(m);
		}
		if (nodeID.CanMove(nodeID.locs[x].first, -1, 0))
		{
			m.startLoc = nodeID.locs[x].first;
			m.dir = kLeft;
			actions.push_back(m);
		}
		if (nodeID.CanMove(nodeID.locs[x].first, 0, 1))
		{
			m.startLoc = nodeID.locs[x].first;
			m.dir = kDown;
			actions.push_back(m);
		}
		if (nodeID.CanMove(nodeID.locs[x].first, 0, -1))
		{
			m.startLoc = nodeID.locs[x].first;
			m.dir = kUp;
			actions.push_back(m);
		}
	}
}

bool Fling::LegalMove(const FlingBoard &s, FlingMove a)
{
	if (!s.HasPiece(a.startLoc))
		return false;
	switch (a.dir)
	{
		case kRight: return s.CanMove(a.startLoc, 1, 0); break;
		case kLeft: return s.CanMove(a.startLoc, -1, 0); break;
		case kDown: return s.CanMove(a.startLoc, 0, 1); break;
		case kUp: return s.CanMove(a.startLoc, 0, -1); break;
	}
	return false;
}

void Fling::ApplyAction(FlingBoard &s, FlingMove a) const
{
	switch (a.dir)
	{
		case kRight: s.Move(a.startLoc, 1, 0); break;
		case kLeft: s.Move(a.startLoc, -1, 0); break;
		case kDown: s.Move(a.startLoc, 0, 1); break;
		case kUp: s.Move(a.startLoc, 0, -1); break;
	}
}

void Fling::UndoAction(FlingBoard &s, FlingMove a) const
{
	assert(false);
}

void Fling::GetNextState(const FlingBoard &f, FlingMove a, FlingBoard &t) const
{
	t = f;
	ApplyAction(t, a);
}

FlingMove Fling::GetAction(const FlingBoard &s1, const FlingBoard &s2) const
{
	std::vector<FlingMove> m1;
	GetActions(s1, m1);
	FlingMove between = {0, kUp};
	FlingBoard tmp;
	bool found = false;
	for (int x = 0; x < m1.size(); x++)
	{
		GetNextState(s1, m1[x], tmp);
		if (tmp == s2)
		{
			found = true;
			between = m1[x];
			break;
		}
	}
	assert(found == true);
	return between;
}


uint64_t Fling::GetStateHash(const FlingBoard &node) const
{
	uint64_t hash = 0;
	bool onBoard = false;
	for (unsigned int x = 0; x < node.locs.size(); x++)
	{
		hash |= (1ull<<node.locs[x].first);
		if (node.locs[x].second == goalLoc)
			onBoard = true;
//		std::cout << "Storing piece at " << node.locs[x] << std::endl;
//		printf("0x%llX (out:%d)\n", hash, x);
	}
	if (specificGoalPanda && onBoard)
		hash |= (1ull<<63);
	//	printf("0x%llX (out)\n", hash);
	return hash;
//	return 0;
}

void Fling::GetStateFromHash(uint64_t parent, FlingBoard &s) const
{
//	printf("0x%llX (in)\n", parent);
	s.Reset();
	for (int x = 0; x < s.width*s.height; x++)
	{
		if (1 == ((parent>>x)&0x1))
		{
//			std::cout << "Setting piece at " << x << std::endl;
			s.AddFling(x);
		}
	}
}

uint64_t Fling::GetActionHash(FlingMove act) const
{
	return 0;
}

//void Fling::OpenGLDraw(const FlingBoard&b) const
//{
//	double radius = 1.0/(1+max(b.width, b.height));
//	double diameter = radius*2;
//	double xLoc = -1+radius;
//	double yLoc = -1+radius;
//
//	glColor3f(0.0, 0.0, 0.5); //
//	glBegin(GL_QUADS);
//	glVertex3f(-1+diameter, -1+diameter, 0);
//	glVertex3f(-1+diameter, -1+(diameter*b.height)+diameter, 0);
//	glVertex3f(-1+diameter*(b.width)+diameter, -1+diameter*(b.height)+diameter, 0);
//	glVertex3f(-1+diameter*(b.width)+diameter, -1+diameter, 0);
//	glEnd();
//	
//	glLineWidth(2.0);
//	glColor3f(1.0, 1.0, 1.0); // white
//	glBegin(GL_LINES);
//	for (double x = 0; x <= b.width; x++)
//	{
//		glVertex3f(-1+(x+1)*diameter, -1+diameter, 0);
//		glVertex3f(-1+(x+1)*diameter, -1+diameter*b.height+diameter, -0.01);
//		xLoc += diameter;
//	}
//	for (double y = 0; y <= b.height; y++)
//	{
//		yLoc += diameter;
//		glVertex3f(-1+diameter, -1+(y+1)*diameter, 0);
//		glVertex3f(-1+diameter*b.width+diameter, -1+(y+1)*diameter, -0.01);
//	}
//	glEnd();
//	glLineWidth(1.0);
//	
//	xLoc = -1+radius;
//	for (double x = 0; x < b.width; x++)
//	{
//		xLoc += diameter;
//		yLoc = -1+radius;
//		for (double y = 0; y < b.height; y++)
//		{
//			yLoc += diameter;
//			if (b.HasPiece(x, y))
//			{
//				rgbColor r = Colors::GetColor(b.locs[b.GetIndexInLocs(x, y)].second, 0, b.currId, 9); // 4
//				glColor3f(r.r, r.g, r.b);
//				DrawSphere(xLoc, yLoc, 0, radius*0.8);
//			}
//			else if (b.HasHole(x, y))
//			{
//				glColor3f(0, 0, 0);
//				DrawBox(xLoc, yLoc, 0, radius);
//			}
//			else if (b.HasObstacle(x, y))
//			{
//				glColor3f(1, 1, 1);
//				DrawBox(xLoc, yLoc, 0, radius);
//			}
//		}
//	}
//}

//void Fling::OpenGLDrawPlain(const FlingBoard&b) const
//{
//	double radius = 1.0/(1+max(b.width, b.height));
//	double diameter = radius*2;
//	double xLoc;
//	double yLoc;
//	double r = radius*0.85;
//
//	glColor3f(1.0, 1.0, 1.0); //
//	glBegin(GL_QUADS);
//	glVertex3f(-2, -2, 0);
//	glVertex3f(-2, +2, 0);
//	glVertex3f(+2, +2, 0);
//	glVertex3f(+2, -2, 0);
//	glEnd();
//	
////	glLineWidth(2.0);
////	glColor3f(1.0, 1.0, 1.0); // white
////	glBegin(GL_LINES);
////	for (double x = 0; x <= b.width; x++)
////	{
////		glVertex3f(-1+(x+1)*diameter, -1+diameter, 0);
////		glVertex3f(-1+(x+1)*diameter, -1+diameter*b.height+diameter, -0.01);
////		xLoc += diameter;
////	}
////	for (double y = 0; y <= b.height; y++)
////	{
////		yLoc += diameter;
////		glVertex3f(-1+diameter, -1+(y+1)*diameter, 0);
////		glVertex3f(-1+diameter*b.width+diameter, -1+(y+1)*diameter, -0.01);
////	}
////	glEnd();
//	glLineWidth(1.0);
//	
//	xLoc = -1+radius;
//	glDisable(GL_LIGHTING);
//	glLineWidth(8.0);
//	glColor4f(0.0, 0.0, 0.0, 1.0); // ffd700
//	//	glColor4f(0.0, 1.0, 0.0, 0.5); // ffd700
//	for (double x = 0; x < b.width; x++)
//	{
//		xLoc += diameter;
//		yLoc = -1+radius;
//		for (double y = 0; y < b.height; y++)
//		{
//			yLoc += diameter;
//			//rgbColor r = GetColor(x+y*b.width, 0, b.width*b.height, 4);
//			
//			if (b.HasPiece(x, y))
//			{
//				glBegin(GL_QUADS);
//				glVertex3f(xLoc-r, yLoc-r-r, -0.02);
//				glVertex3f(xLoc-r, yLoc+r-r, -0.02);
//				glVertex3f(xLoc+r, yLoc+r-r, -0.02);
//				glVertex3f(xLoc+r, yLoc-r-r, -0.02);
//				glEnd();
//			}
//			//DrawSphere(xLoc, yLoc, 0, radius);
//		}
//	}
//	glLineWidth(1.0);
//	glEnable(GL_LIGHTING);
//}


//void Fling::OpenGLDrawAlternate(const FlingBoard &b) const
//{
//	double radius = 1.0/(1+max(b.width, b.height));
//	double diameter = radius*2;
//	double xLoc = -1+radius;
//	double yLoc;
//	double r = radius*0.80;
//
//	glDisable(GL_LIGHTING);
//	glLineWidth(8.0);
//	glColor4f(0.0, 0.5, 0.0, 1.0); // ffd700
////	glColor4f(0.0, 1.0, 0.0, 0.5); // ffd700
//	for (double x = 0; x < b.width; x++)
//	{
//		xLoc += diameter;
//		yLoc = -1+radius;
//		for (double y = 0; y < b.height; y++)
//		{
//			yLoc += diameter;
//			//rgbColor r = GetColor(x+y*b.width, 0, b.width*b.height, 4);
//
//			if (b.HasPiece(x, y))
//			{
//				glBegin(GL_QUADS);
//				glVertex3f(xLoc-r, yLoc-r, -0.02);
//				glVertex3f(xLoc-r, yLoc+r, -0.02);
//				glVertex3f(xLoc+r, yLoc+r, -0.02);
//				glVertex3f(xLoc+r, yLoc-r, -0.02);
//				glEnd();
//			}
//			//DrawSphere(xLoc, yLoc, 0, radius);
//		}
//	}
//	glLineWidth(1.0);
//	glEnable(GL_LIGHTING);
//}


//void Fling::OpenGLDraw(const FlingBoard&b, const FlingMove &m) const
//{
//	double radius = 1.0/(1+max(b.width, b.height));
//	double diameter = radius*2;
//	double xLoc = -1+radius+diameter;
//	double yLoc = -1+radius+diameter;
//	
//	glColor3f(1.0, 1.0, 1.0);
//	glLineWidth(10);
//	glBegin(GL_TRIANGLES);
//
////	int x = b.locs[m.startLoc]%b.width;
////	int y = b.locs[m.startLoc]/b.width;
//	int x = m.startLoc%b.width;
//	int y = m.startLoc/b.width;
//	switch (m.dir)
//	{
//		case kLeft:
//			glVertex3f(xLoc+x*diameter, yLoc+y*diameter-radius/2, -radius);
//			glVertex3f(xLoc+x*diameter, yLoc+y*diameter+radius/2, -radius);
//			glVertex3f(xLoc+x*diameter-radius, yLoc+y*diameter, -radius);
//			break;
//		case kRight:
//			glVertex3f(xLoc+x*diameter, yLoc+y*diameter+radius/2, -radius);
//			glVertex3f(xLoc+x*diameter, yLoc+y*diameter-radius/2, -radius);
//			glVertex3f(xLoc+x*diameter+radius, yLoc+y*diameter, -radius);
//			break;
//		case kUp:
//			glVertex3f(xLoc+x*diameter+radius/2, yLoc+y*diameter, -radius);
//			glVertex3f(xLoc+x*diameter-radius/2, yLoc+y*diameter, -radius);
//			glVertex3f(xLoc+x*diameter, yLoc+y*diameter-radius, -radius);
//			break;
//		case kDown:
//			glVertex3f(xLoc+x*diameter-radius/2, yLoc+y*diameter, -radius);
//			glVertex3f(xLoc+x*diameter+radius/2, yLoc+y*diameter, -radius);
//			glVertex3f(xLoc+x*diameter, yLoc+y*diameter+radius, -radius);
//			break;
//	}
//	glEnd();
//	glLineWidth(1.0);
//}

bool Fling::GetXYFromPoint(const FlingBoard &b, point3d loc, int &x, int &y) const
{
	double radius = 1.0/(1+max(b.width, b.height));
	double diameter = radius*2;

	loc.x = loc.x-diameter+1;
	loc.y = loc.y-diameter+1;
	loc.x /= diameter;
	loc.y /= diameter;
	x = loc.x;
	y = loc.y;
	
	if (x >= 0 && x < b.width && y >= 0 && y < b.height)
		return true;
	return false;
}
//
//void Fling::GLLabelState(const FlingBoard&b, const char *text) const
//{
//	glDisable(GL_LIGHTING);
//    glEnable(GL_LINE_SMOOTH);
//    glDisable(GL_DEPTH_TEST);
//	glLineWidth(3.0);
//
//	double radius = 1.0/(1+max(b.width, b.height));
//	double diameter = radius*2;
//	double xLoc = -1+radius;
//	double yLoc;
//	
//	for (double x = 0; x < b.width; x++)
//	{
//		xLoc += diameter;
//		yLoc = -1+radius;
//		for (double y = 0; y < b.height; y++)
//		{
//			yLoc += diameter;
//			glColor3f(1.0, 1.0, 1.0);
//			if (b.HasPiece(x, y))
//			{
//				const char *p;
//				
//				glPushMatrix();
//				glTranslatef(xLoc-radius+1/152.38, yLoc-radius+8/152.38, 0);
//				glScalef(.05/152.38, -.05/152.38, .05/152.38);
//				glTranslatef(0, 0, -2*radius);
//
//				//glScalef(0.09f, -0.08f, 1.0);
//				//glTranslatef(0, 0, -2*radius);
//				for (p = text; *p; p++)
//				{
//					//printf("%c", *p);
//					//glutStrokeCharacter(GLUT_STROKE_ROMAN, *p);
//				}
//				glPopMatrix();
//				//printf("\n");
//				// draw text
////				const char *c;
//////				glPushMatrix();
////				glScalef(0.09f, -0.08f, 1.0);
//////				glTranslatef(x, y, 2*radius);
////				for (c=text; *c != '\0'; c++)
////				{
////					glutStrokeCharacter(GLUT_STROKE_ROMAN , *c);
////				}
//////				glPopMatrix();
//			}
//		}
//	}
//
//    glEnable(GL_DEPTH_TEST);
//	glEnable(GL_LIGHTING);
//    glDisable(GL_LINE_SMOOTH);
//	glLineWidth(1.0);
//
//}

void Fling::IncrementRank(FlingBoard &b) const
{
	IncrementRank(b, 0);
}

int Fling::IncrementRank(FlingBoard &b, int piece) const
{
	int currPieceLocation = b.GetPieceLocation(piece);
	int newPieceLocation;
	// piece reached end of board
	if (currPieceLocation == b.height*b.width-1-piece)
	{
		newPieceLocation = IncrementRank(b, piece+1)+1;
	}
	else {
		newPieceLocation = currPieceLocation+1;
	}
	b.ClearPiece(currPieceLocation);
	b.SetPiece(newPieceLocation);
	b.locs[piece].first = newPieceLocation;
	return newPieceLocation;
}


int64_t Fling::getMaxSinglePlayerRank(int spots, int numPieces)
{
	return binomial(spots, numPieces);
}

int64_t Fling::getMaxSinglePlayerRank2(int spots, int numPieces)
{
	return binomial(spots-(numPieces-2), 2);
}

int64_t Fling::getMaxSinglePlayerRank2(int spots, int numPieces, int64_t firstIndex)
{
	int NUM_SPOTS = spots;
	int NUM_PIECES = numPieces;
	unsigned int ls = 2;
	int i = 0;
	for (; ls > 0; ++i)
	{
		int64_t value;
		if (ls > 0)
		{
			value = binomial(NUM_SPOTS-(NUM_PIECES-2) - i - 1, ls - 1);
		}
		else {
			value = 0;
		}
		if (firstIndex < value)
		{
			ls--;
		}
		else {
			firstIndex -= value;
		}
	}
	return binomial(NUM_SPOTS-i,NUM_PIECES-2);
}

int64_t Fling::rankPlayer(FlingBoard &s)
{
	int NUM_SPOTS = s.width*s.height;
	int NUM_PIECES = (int)s.locs.size();

	int64_t r2 = 0;
	int last = NUM_SPOTS-1;
	for (int x = 0; x < NUM_PIECES; x++)
	{
		int64_t tmp = binomialSum(last, NUM_SPOTS-s.locs[NUM_PIECES-1-x].first-1, NUM_PIECES-1-x);
		r2 += tmp;
		last = NUM_SPOTS-s.locs[NUM_PIECES-1-x].first-1-1;
	}
	return r2;
}

void Fling::rankPlayer(FlingBoard &s, int64_t &index1, int64_t &index2)
{
	int NUM_SPOTS = s.width*s.height;
	int NUM_PIECES = (int)s.locs.size();
	index1 = 0;
	int tot = NUM_SPOTS-1-(NUM_PIECES-2);
	int last = tot;
	for (int x = 0; x < 2; x++)
	{
		int64_t tmp = binomialSum(last, tot-s.locs[NUM_PIECES-1-x].first, (2)-1-x);
		index1 += tmp;
		last = tot-s.locs[NUM_PIECES-1-x].first-1;
	}
	
	index2 = 0;
	last = NUM_SPOTS-s.locs[NUM_PIECES-1-1].first-1-1;
	for (int x = 2; x < NUM_PIECES; x++)
	{
		int64_t tmp = binomialSum(last, NUM_SPOTS-s.locs[NUM_PIECES-1-x].first-1, NUM_PIECES-1-x);
		index2 += tmp;
		last = NUM_SPOTS-s.locs[NUM_PIECES-1-x].first-1-1;
	}
}

void Fling::rankPlayerFirstTwo(FlingBoard &s, int64_t &index1)
{
	int NUM_SPOTS = s.width*s.height;
	int NUM_PIECES = (int)s.locs.size();

	index1 = 0;
	int tot = NUM_SPOTS-1-(NUM_PIECES-2);
	int last = tot;
	for (int x = 0; x < 2; x++)
	{
		int64_t tmp = binomialSum(last, tot-s.locs[NUM_PIECES-1-x].first, (2)-1-x);
		index1 += tmp;
		last = tot-s.locs[NUM_PIECES-1-x].first-1;
	}
}

void Fling::rankPlayerRemaining(FlingBoard &s, int64_t &index2)
{
	int NUM_SPOTS = s.width*s.height;
	int NUM_PIECES = (int)s.locs.size();

	int last;
	index2 = 0;
	last = NUM_SPOTS-s.locs[NUM_PIECES-1-1].first-1-1;
	for (int x = 2; x < NUM_PIECES; x++)
	{
		int64_t tmp = binomialSum(last, NUM_SPOTS-s.locs[NUM_PIECES-1-x].first-1, NUM_PIECES-1-x);
		index2 += tmp;
		last = NUM_SPOTS-s.locs[NUM_PIECES-1-x].first-1-1;
	}
}


// returns true if it is a valid unranking given existing pieces
bool Fling::unrankPlayer(int64_t theRank, int pieces, FlingBoard &s)
{
	int NUM_SPOTS = s.width*s.height;
	int NUM_PIECES = pieces;

//	int tag = who + 1;
	unsigned int ls = NUM_PIECES;
//	memset(s.board, 0, NUM_SPOTS*sizeof(int));
	s.Reset();
	s.locs.resize(pieces);
	for (int i=0; ls > 0; ++i)
	{
		int64_t value;
		if (ls > 0)
		{
			value = binomial(NUM_SPOTS - i - 1, ls - 1);
		}
		else {
			value = 0;
		}
		if (theRank < value)
		{
			s.SetPiece(i);
			//s.board[i] = true;
			s.locs[ls-1].first = i;
			s.locs[ls-1].second = ls-1;
			ls--;
		}
		else {
			s.ClearPiece(i);
			//s.board[i] = 0;
			theRank -= value;
		}
	}
	for (int x = 1; x < NUM_PIECES; x++)
		assert(s.locs[x-1] > s.locs[x]);
	return true;
}

//
//
//void Fling::initBinomialSums()
//{
//	if (theSums.size() == 0)
//	{
//		theSums.resize((NUM_PIECES+1)*(NUM_SPOTS+1));
//		//		sums.resize(NUM_PIECES+1);
//		for (int x = 0; x <= NUM_PIECES; x++)
//		{
//			//			sums[x].resize(NUM_SPOTS+1);
//			int64_t result = 0;
//			for (int y = 0; y <= NUM_SPOTS; y++)
//			{
//				result+=binomial(y, x);
//				//				sums[x][y] = result;
//				theSums[x*(NUM_SPOTS+1)+y] = result;
//			}
//		}
//	}
//}
//
int64_t Fling::binomialSum(unsigned int n1, unsigned int n2, unsigned int k)
{
	//	static std::vector<std::vector<int64_t> > sums;
	//assert(theSums[k*(NUM_SPOTS+1)+n1]-theSums[k*(NUM_SPOTS+1)+n2] == sums[k][n1]-sums[k][n2]);
	int64_t result = 0;
	for (int x = n1; x > n2; x--)
		result += binomial(x, k);
	return result;
	//return theSums[k*(NUM_SPOTS+1)+n1]-theSums[k*(NUM_SPOTS+1)+n2];
	//return sums[k][n1]-sums[k][n2];
}

const int maxPieces = 14;

void Fling::initBinomial()
{
	if (binomials.size() == 0)
	{
		for (int x = 0; x <= 56; x++)
		{
			for (int y = 0; y <= maxPieces; y++)
			{
				binomials.push_back(bi(x, y));
			}
		}
	}
}

int64_t Fling::binomial(unsigned int n, unsigned int k)
{
	//assert(bi(n, k) == binomials[n*(1+NUM_PLAYERS*NUM_PIECES)+k]);
	return binomials[n*(1+maxPieces)+k];
}

int64_t Fling::bi(unsigned int n, unsigned int k)
{
	int64_t num = 1;
	const unsigned int bound = (n - k);
	while(n > bound)
	{
		num *= n--;
	}
	
	int64_t den = 1;
	while(k > 1)
	{
		den *= k--;
	}
	return num / den;
}
