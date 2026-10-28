//
//  TOH.hpp
//  hog2 glut
//
//  Created by Nathan Sturtevant on 10/20/15.
//  Copyright © 2015 University of Denver. All rights reserved.
//

#ifndef TOH_hpp
#define TOH_hpp

#include <stdio.h>
#include <cstdint>
#include <cassert>
#include <math.h>
#include "SearchEnvironment.h"
#include "PDBHeuristic.h"

struct TOHMove {
	TOHMove(uint8_t s, uint8_t d) :source(s), dest(d) {}
	TOHMove() {}
	uint8_t source;
	uint8_t dest;
};

template <int numDisks, int numPegs = 4>
struct TOHState {
	TOHState()
	{
		for (int x = 0; x < numPegs; x++)
		{
			counts[x] = 0;
		}
		for (int x = 0; x < numDisks; x++)
		{
			disks[numPegs-1][x] = numDisks-x;
		}
		counts[numPegs-1] = numDisks;
	}
	
	void Reset()
	{
		for (int x = 0; x < numPegs; x++)
		{
			counts[x] = 0;
		}
		for (int x = 0; x < numDisks; x++)
		{
			disks[numPegs-1][x] = numDisks-x;
		}
		counts[numPegs-1] = numDisks;
	}
	void StandardStart()
	{
		for (int x = 0; x < numPegs; x++)
		{
			counts[x] = 0;
		}
		for (int x = 0; x < numDisks; x++)
		{
			disks[0][x] = numDisks-x;
		}
		counts[0] = numDisks;
	}
	
	int GetDiskCountOnPeg(int whichPeg) const
	{
		assert(whichPeg >= 0 && whichPeg < numPegs);
		return counts[whichPeg];
	}
	
	int GetDiskOnPeg(int whichPeg, int whichDisk) const
	{
		assert(whichPeg >= 0 && whichPeg < numPegs);
		assert(whichDisk >= 0 && whichDisk < counts[whichPeg]);
		return disks[whichPeg][whichDisk];
	}
	
	// if this is slow, we can add a "big" disk to every peg to
	// avoid the "if" statement.
	int GetSmallestDiskOnPeg(int whichPeg) const
	{
		assert(whichPeg >= 0 && whichPeg < numPegs);
		int count = GetDiskCountOnPeg(whichPeg);
		if (count == 0)
			return numDisks+1;
		return GetDiskOnPeg(whichPeg, count-1);
	}
	
	uint8_t disks[numPegs][numDisks];
	uint8_t counts[numPegs];
};

template <int D, int numPegs>
static std::ostream &operator<<(std::ostream &out, const TOHState<D, numPegs> &s)
{
	for (int x = 0; x < numPegs; x++)
	{
		out << "(" << x << ") ";
		for (int y = 0; y < s.GetDiskCountOnPeg(x); y++)
			out << s.GetDiskOnPeg(x, y) << " ";
	}
	return out;
}

template <int D, int numPegs>
static bool operator==(const TOHState<D, numPegs> &l1, const TOHState<D, numPegs> &l2) {
	for (int x = 0; x < numPegs; x++)
	{
		if (l1.GetDiskCountOnPeg(x) != l2.GetDiskCountOnPeg(x))
			return false;
		for (int y = 0; y < l1.GetDiskCountOnPeg(x); y++)
		{
			if (l1.GetDiskOnPeg(x, y)!= l2.GetDiskOnPeg(x, y))
				return false;
		}
	}
	return true;
}

template <int D, int numPegs>
static bool operator!=(const TOHState<D, numPegs> &l1, const TOHState<D, numPegs> &l2) {
	return !(l1 == l2);
}

static std::ostream &operator<<(std::ostream &out, const TOHMove &m)
{
	out << "(" << +m.source << ", " << +m.dest << ")";
	return out;
}

static bool operator==(const TOHMove &m1, const TOHMove &m2) {
	return m1.source == m2.source && m1.dest == m2.dest;
}

// integer exponentiation - used to build the base-numPegs hash of a TOHState
static inline uint64_t TOHIntPow(uint64_t base, int exponent)
{
	uint64_t result = 1;
	for (int i = 0; i < exponent; i++)
		result *= base;
	return result;
}

template <int numDisks, int numPegs = 4>
class TOH : public SearchEnvironment<TOHState<numDisks, numPegs>, TOHMove> {
public:
	TOH() {}
	~TOH() {}
	void GetSuccessors(const TOHState<numDisks, numPegs> &nodeID, std::vector<TOHState<numDisks, numPegs>> &neighbors) const;
	void GetActions(const TOHState<numDisks, numPegs> &nodeID, std::vector<TOHMove> &actions) const;
	void ApplyAction(TOHState<numDisks, numPegs> &s, TOHMove a) const;
	bool InvertAction(TOHMove &a) const;
	
	/** Heuristic value between two arbitrary nodes. **/
	double HCost(const TOHState<numDisks, numPegs> &node1, const TOHState<numDisks, numPegs> &node2) const;
	double GCost(const TOHState<numDisks, numPegs> &node1, const TOHState<numDisks, numPegs> &node2) const { return 1; }
	double GCost(const TOHState<numDisks, numPegs> &node, const TOHMove &act) const { return 1; }
	bool GoalTest(const TOHState<numDisks, numPegs> &node, const TOHState<numDisks, numPegs> &goal) const;
	
	uint64_t GetStateHash(const TOHState<numDisks, numPegs> &node) const;
	void GetStateFromHash(uint64_t parent, TOHState<numDisks, numPegs> &s) const;
	uint64_t GetMaxHash() const { return TOHIntPow(numPegs, numDisks); }
	uint64_t GetNumStates(TOHState<numDisks, numPegs> &s) const;
	uint64_t GetActionHash(TOHMove act) const;
	
	std::string GetName() { return "TOH("+std::to_string(numDisks)+","+std::to_string(numPegs)+")"; }
	
	// x-coordinate of the center of the given peg; pegs are evenly spaced
	// across the [-1, 1] range of the display, one per numPegs-wide region
	float GetPegLocation(int peg) const;
	
	void Draw(Graphics::Display &display, std::string str) const; // draws text
	void Draw(Graphics::Display &display) const; // draws the base and lines
	void Draw(Graphics::Display &display, const TOHState<numDisks, numPegs> &s) const; // draws the disks when not animating
	void Draw(Graphics::Display &display, const TOHState<numDisks, numPegs> &l1, const TOHState<numDisks, numPegs> &l2, float v) const; // animation for optimal solution
	void Draw(Graphics::Display &display, const TOHState<numDisks, numPegs> &l1, int selectedPeg, int nextPeg, float v) const; // vertical animation for when user is solving
	void Draw(Graphics::Display &display, const TOHState<numDisks, numPegs> &l1, int startPeg, float px); // horizontal animation for when user is solving
	void Draw(Graphics::Display &display, const TOHState<numDisks, numPegs>&, TOHMove&) const;
	
	bool Click(int &peg, float px);
	int GetHoveredPeg(const float &px);
	bool Drag(const TOHState<numDisks, numPegs> &currState, int peg);
	bool Release(const TOHState<numDisks, numPegs> &currState, int &peg, point3d loc, TOHState<numDisks, numPegs> &nextState, int &userMoveCount);
	
protected:
private:
	void DrawDisk(Graphics::Display &display, Graphics::rect r, rgbColor c, float ratio) const; // draws a disk block with 3D-style ovals on the top and bottom
	
	// caches
	mutable std::vector<TOHMove> acts;
	mutable TOHState<numDisks, numPegs> tmp;
	
};



template <int numDisks, int numPegs>
void TOH<numDisks, numPegs>::GetSuccessors(const TOHState<numDisks, numPegs> &nodeID, std::vector<TOHState<numDisks, numPegs>> &neighbors) const
{
	neighbors.resize(0);
	GetActions(nodeID, acts);
	for (auto act : acts)
	{
		this->GetNextState(nodeID, act, tmp);
		neighbors.push_back(tmp);
	}
}

template <int numDisks, int numPegs>
void TOH<numDisks, numPegs>::GetActions(const TOHState<numDisks, numPegs> &s, std::vector<TOHMove> &actions) const
{
	actions.resize(0);
	for (int i = 0; i < numPegs; i++)
	{
		for (int j = i+1; j < numPegs; j++)
		{
			if (s.GetSmallestDiskOnPeg(i) < s.GetSmallestDiskOnPeg(j))
			{
				if (s.GetDiskCountOnPeg(i) > 0)
					actions.push_back(TOHMove(i, j));
			}
			else {
				if (s.GetDiskCountOnPeg(j) > 0)
					actions.push_back(TOHMove(j, i));
			}
		}
	}
}


template <int numDisks, int numPegs>
void TOH<numDisks, numPegs>::ApplyAction(TOHState<numDisks, numPegs> &s, TOHMove m) const
{
	s.disks[m.dest][s.counts[m.dest]] = s.disks[m.source][s.counts[m.source]-1];
	s.counts[m.dest]++;
	s.counts[m.source]--;
}

template <int numDisks, int numPegs>
bool TOH<numDisks, numPegs>::InvertAction(TOHMove &a) const
{
	uint8_t tmp = a.source;
	a.source = a.dest;
	a.dest = tmp;
	return true;
}


/** Heuristic value between two arbitrary nodes. **/
template <int numDisks, int numPegs>
double TOH<numDisks, numPegs>::HCost(const TOHState<numDisks, numPegs> &node1, const TOHState<numDisks, numPegs> &node2) const
{
	// NOTE: this is using the standard goal state; arbitrary goal states
	// are more expensive to check
	return numDisks - node1.GetDiskCountOnPeg(numPegs-1);
}

template <int numDisks, int numPegs>
bool TOH<numDisks, numPegs>::GoalTest(const TOHState<numDisks, numPegs> &node, const TOHState<numDisks, numPegs> &goal) const
{
	// NOTE: This goal test is only from standard start to standard goal
	return (node.GetDiskCountOnPeg(numPegs-1) == numDisks && node.GetDiskOnPeg(numPegs-1, 0) == numDisks);
	// NOTE: this is using the standard goal state; arbitrary goal states
	// are more expensive to check
	return (node == goal);
}


template <int numDisks, int numPegs>
uint64_t TOH<numDisks, numPegs>::GetStateHash(const TOHState<numDisks, numPegs> &node) const
{
	uint64_t hash = 0;
	for (int x = 0; x < numPegs; x++)
	{
		for (int y = 0; y < node.GetDiskCountOnPeg(x); y++)
		{
			hash += static_cast<uint64_t>(x) * TOHIntPow(numPegs, node.GetDiskOnPeg(x, y)-1);
		}
	}
	return hash;
}

template <int numDisks, int numPegs>
uint64_t TOH<numDisks, numPegs>::GetNumStates(TOHState<numDisks, numPegs> &s) const
{
	return TOHIntPow(numPegs, numDisks);
}

template <int numDisks, int numPegs>
void TOH<numDisks, numPegs>::GetStateFromHash(uint64_t hash, TOHState<numDisks, numPegs> &s) const
{
	for (int x = 0; x < numPegs; x++)
		s.counts[x] = 0;
	for (int x = numDisks-1; x >= 0; x--)
	{
		int nextPeg = (hash / TOHIntPow(numPegs, x)) % numPegs;
		s.disks[nextPeg][s.counts[nextPeg]] = x+1;
		s.counts[nextPeg]++;
	}
}

template <int numDisks, int numPegs>
uint64_t TOH<numDisks, numPegs>::GetActionHash(TOHMove act) const
{
	return (act.source<<8)|act.dest;
}

// Draw for text area
template <int numDisks, int numPegs>
void TOH<numDisks, numPegs>::Draw(Graphics::Display &display, std::string str) const
{
	Graphics::rect r1(-1, -1, 1, -0.8); // background for text area
	display.FillRect(r1, Colors::lightgray);
	
	display.DrawText(str.c_str(), Graphics::point{-0.9, -0.9}, Colors::black, 0.075,
					 Graphics::textAlignLeft, Graphics::textBaselineMiddle);
}

template <int numDisks, int numPegs>
float TOH<numDisks, numPegs>::GetPegLocation(int peg) const
{
	// the screen spans [-1, 1] (2 units); divide it into numPegs regions
	// and place the peg at the center of its region
	return -1.0f + (2.0f*peg+1.0f)/numPegs;
}

// Draw for pegs and base
template <int numDisks, int numPegs>
void TOH<numDisks, numPegs>::Draw(Graphics::Display &display) const
{
	display.FillRect({-1, 0.7, 1, 0.90}, {0.6, 0.4, 0.2}); // brown base
	display.FillRect({-1, 0.87, 1, 1.0}, Colors::brown); // brown base

	for (int p = 0; p < numPegs; p++)
	{
		float loc = GetPegLocation(p);
		Graphics::rect r1(loc-0.01, 0, loc+0.01, 0.8); // peg
		display.FillRect(r1, Colors::gray);
	}
	
}

// Draw for still state
template <int numDisks, int numPegs>
void TOH<numDisks, numPegs>::Draw(Graphics::Display &display, const TOHState<numDisks, numPegs> &s) const
{
	for (int x = 0; x < numPegs; x++)
	{
		float loc = GetPegLocation(x);
		for (int y = 0; y < s.GetDiskCountOnPeg(x); y++)
		{
			int which = s.GetDiskOnPeg(x, y);
			float halfwidth = (0.9f*(2.0f/numPegs)/2.0f)*((which+1)/float(numDisks));
			
			rgbColor color(0.0, 1.0-float(which)/float(numDisks), 1.0);
			Graphics::rect r(loc-halfwidth,
							 -y*0.8/(1+float(numDisks))-0.8/(1+float(numDisks))+0.8,
							 loc+halfwidth,
							 -y*0.8/(1+float(numDisks))+0.8);
			DrawDisk(display, r, color, (which+1)/float(numDisks));
			//display.FillRect(r, color);
		}
	}
}

// Draw for animating optimal solve
template <int numDisks, int numPegs>
void TOH<numDisks, numPegs>::Draw(Graphics::Display &display, const TOHState<numDisks, numPegs> &s, const TOHState<numDisks, numPegs> &s2, float v) const
{
	TOHMove m = this->GetAction(s, s2);
	
	int animatingDisk = s.GetSmallestDiskOnPeg(m.source);
	int finalHeight = s.GetDiskCountOnPeg(m.dest);
	
	for (int x = 0; x < numPegs; x++)
	{
		float loc = GetPegLocation(x);
		for (int y = 0; y < s.GetDiskCountOnPeg(x); y++)
		{
			int which = s.GetDiskOnPeg(x, y);
			float halfwidth = (0.9f*(2.0f/numPegs)/2.0f)*((which+1)/float(numDisks));
			if (which != animatingDisk) // first, draws every disk except for the animating one
			{
				Graphics::rect r1 = {static_cast<float>(loc-halfwidth), static_cast<float>(0.8-0.8/(1+float(numDisks))-y*0.8/(1+float(numDisks))), static_cast<float>(loc+halfwidth), static_cast<float>(0.8-y*0.8/(1+float(numDisks)))};
				DrawDisk(display, r1, {0.0, static_cast<float>(1.0-float(which)/float(numDisks)), 1.0}, (which+1)/float(numDisks));
			}
			else {
				int targetPeg = m.dest;
				float targetLoc = GetPegLocation(targetPeg);
				Graphics::rect r1;
				Graphics::rect r2;
				
				if (v <= 0.333) { // up
					v *= 3;
					r1 = {static_cast<float>(loc-halfwidth), static_cast<float>(0.8-0.8/(1+float(numDisks))-y*0.8/(1+float(numDisks))), static_cast<float>(loc+halfwidth), static_cast<float>(0.8-y*0.8/(1+float(numDisks)))};
					
					r2 = {static_cast<float>(loc-halfwidth), static_cast<float>(-0.5-0.8/(1+float(numDisks))), static_cast<float>(loc+halfwidth), static_cast<float>(-0.5)};
				}
				else if (v <= 0.666) { // horizontal
					v = (v - 0.333) * 3;
					r1 = {static_cast<float>(loc-halfwidth), static_cast<float>(-0.5-0.8/(1+float(numDisks))), static_cast<float>(loc+halfwidth), static_cast<float>(-0.5)};
					
					r2 = {static_cast<float>(targetLoc-halfwidth), static_cast<float>(-0.5-0.8/(1+float(numDisks))), static_cast<float>(targetLoc+halfwidth), static_cast<float>(-0.5)};
				}
				else { // down
					v = (v - 0.666) * 3;
					r1 = {static_cast<float>(targetLoc-halfwidth), static_cast<float>(-0.5-0.8/(1+float(numDisks))), static_cast<float>(targetLoc+halfwidth), static_cast<float>(-0.5)};
					
					r2 = {static_cast<float>(targetLoc-halfwidth), static_cast<float>(0.8-0.8/(1+float(numDisks))-finalHeight*0.8/(1+float(numDisks))), static_cast<float>(targetLoc+halfwidth), static_cast<float>(0.8-finalHeight*0.8/(1+float(numDisks)))};
				}
				
				r1.lerp(r2, v);
				DrawDisk(display, r1, {0.0, static_cast<float>(1.0-float(which)/float(numDisks)), 1.0}, (which+1)/float(numDisks));
			}
		}
	}
	
	
}

// Draw for vertical animation when user is solving
template <int numDisks, int numPegs>
void TOH<numDisks, numPegs>::Draw(Graphics::Display &display, const TOHState<numDisks, numPegs> &s, int selectedPeg, int nextPeg, float v) const
{
	int animatingDisk = s.GetSmallestDiskOnPeg(selectedPeg);
	int finalHeight = s.GetDiskCountOnPeg(nextPeg);
	
	for (int x = 0; x < numPegs; x++)
	{
		float loc = GetPegLocation(x);
		for (int y = 0; y < s.GetDiskCountOnPeg(x); y++)
		{
			int which = s.GetDiskOnPeg(x, y);
			float halfwidth = (0.9f*(2.0f/numPegs)/2.0f)*((which+1)/float(numDisks));
			if (which != animatingDisk) // first, draws every disk except for the animating one
			{
				//display.FillRect(
				Graphics::rect r = {static_cast<float>(loc-halfwidth), static_cast<float>(0.8-0.8/(1+float(numDisks))-y*0.8/(1+float(numDisks))), static_cast<float>(loc+halfwidth), static_cast<float>(0.8-y*0.8/(1+float(numDisks)))};
				//);
				DrawDisk(display, r, {0.0, static_cast<float>(1.0-float(which)/float(numDisks)), 1.0},  (which+1)/float(numDisks));
			}
			else {
				float nextLoc = GetPegLocation(nextPeg);
				Graphics::rect r1;
				Graphics::rect r2;
				
				if (v <= 0.333) { // up for the first third
					v *= 3;
					r1 = {static_cast<float>(loc-halfwidth), static_cast<float>(0.8-0.8/(1+float(numDisks))-y*0.8/(1+float(numDisks))), static_cast<float>(loc+halfwidth), static_cast<float>(0.8-y*0.8/(1+float(numDisks)))};
					
					r2 = {static_cast<float>(loc-halfwidth), static_cast<float>(-0.5-0.8/(1+float(numDisks))), static_cast<float>(loc+halfwidth), static_cast<float>(-0.5)};
				}
				else { // down for the last third. the second third is animated by Draw(display, s, startPeg, px)
					v = (v - 0.666) * 3;
					r1 = {static_cast<float>(nextLoc-halfwidth), static_cast<float>(-0.5-0.8/(1+float(numDisks))), static_cast<float>(nextLoc+halfwidth), static_cast<float>(-0.5)};
					
					r2 = {static_cast<float>(nextLoc-halfwidth), static_cast<float>(0.8-0.8/(1+float(numDisks))-finalHeight*0.8/(1+float(numDisks))), static_cast<float>(nextLoc+halfwidth), static_cast<float>(0.8-finalHeight*0.8/(1+float(numDisks)))};
				}
				
				r1.lerp(r2, v);
				DrawDisk(display, r1, Colors::purple, (which+1)/float(numDisks));
				//display.FillRect(r1, Colors::purple);
			}
		}
	}
	
	
}

// Draw for horizontal animation when user is solving
template <int numDisks, int numPegs>
void TOH<numDisks, numPegs>::Draw(Graphics::Display &display, const TOHState<numDisks, numPegs> &s, int startPeg, float px)
{
	int animatingDisk = s.GetSmallestDiskOnPeg(startPeg);
	
	// if the mouse is hovering over a peg, highlight that peg
	int hoveredPeg = GetHoveredPeg(px);
	if (hoveredPeg != -1)
	{
		float hoveredLoc = GetPegLocation(hoveredPeg);
		Graphics::rect p(hoveredLoc-0.01, 0, hoveredLoc+0.01, 0.8);
		
		// highlight invalid pegs in red and valid pegs in purple
		if (s.GetSmallestDiskOnPeg(startPeg) > s.GetSmallestDiskOnPeg(hoveredPeg))
		{
			display.FillRect(p, Colors::red);
		}
		else {
			display.FillRect(p, Colors::purple);
		}
		
	}
	
	for (int x = 0; x < numPegs; x++)
	{
		float loc = GetPegLocation(x);
		for (int y = 0; y < s.GetDiskCountOnPeg(x); y++)
		{
			int which = s.GetDiskOnPeg(x, y);
			float halfwidth = (0.9f*(2.0f/numPegs)/2.0f)*((which+1)/float(numDisks));
			if (which != animatingDisk) // first, draws every disk except for the animating one
			{
				//display.FillRect(
				Graphics::rect r = {static_cast<float>(loc-halfwidth), static_cast<float>(0.8-0.8/(1+float(numDisks))-y*0.8/(1+float(numDisks))), static_cast<float>(loc+halfwidth), static_cast<float>(0.8-y*0.8/(1+float(numDisks)))};
				DrawDisk(display, r, {0.0, static_cast<float>(1.0-float(which)/float(numDisks)), 1.0}, (which+1)/float(numDisks));
			}
			else
			{ // draws the animating disk
				Graphics::rect r1 = {static_cast<float>(px-halfwidth), static_cast<float>(-0.5-0.8/(1+float(numDisks))), static_cast<float>(px+halfwidth), -0.5f};
				//display.FillRect(r1, Colors::purple);
				DrawDisk(display, r1, Colors::purple, (which+1)/float(numDisks));
			}
		}
	}
	
}

template <int numDisks, int numPegs>
void TOH<numDisks, numPegs>::Draw(Graphics::Display &display, const TOHState<numDisks, numPegs>&, TOHMove&) const
{
	// nothing here as in OpenGLDraw
}

// Draws a single disk block, adding a flattened oval "cap" at the top and
// bottom of the block to give it a 3D appearance. The bottom oval matches
// the block color; the top oval is 10% lighter, suggesting a lit top face.
template <int numDisks, int numPegs>
void TOH<numDisks, numPegs>::DrawDisk(Graphics::Display &display, Graphics::rect r, rgbColor c, float ratio) const
{
	display.FillRect(r, c);
	
	float ovalHeight = (r.bottom - r.top) * 0.5f * (ratio+0.5f)/1.5f;
	rgbColor lighter = rgbColor::mix(c, rgbColor(1, 1, 1), 0.5f);
	
	Graphics::rect bottomOval(r.left, r.bottom - ovalHeight / 2, r.right, r.bottom + ovalHeight / 2);
	Graphics::rect topOval(r.left, r.top - ovalHeight / 2, r.right, r.top + ovalHeight / 2);
	
	display.FillOval(bottomOval, c);
	display.FillOval(topOval, lighter);
	
	if ((topOval.top+topOval.bottom)/2.0f > 0)
	{
		Graphics::rect r1((r.left+r.right)/2.0f-0.01, topOval.top, (r.left+r.right)/2.0f+0.01, (topOval.top+topOval.bottom)/2.0f); // peg
		display.FillRect(r1, Colors::gray);
	}
	else {
		Graphics::rect r1((r.left+r.right)/2.0f-0.01, (topOval.top+topOval.bottom)/2.0f-0.0015f, (r.left+r.right)/2.0f+0.01, (topOval.top+topOval.bottom)/2.0f+0.0015f); // peg
		display.FillOval(r1, Colors::black);
	}
}

template <int numDisks, int numPegs>
bool TOH<numDisks, numPegs>::Click(int &peg, float px)
{
	peg = GetHoveredPeg(px);
	
	return true;
}

template <int numDisks, int numPegs>
int TOH<numDisks, numPegs>::GetHoveredPeg(const float &px)
{
	// area accepted as "peg" goes a little beyond the peg's exact location,
	// proportionally to how wide each peg's region is
	float tolerance = 0.4f/numPegs;
	for (int p = 0; p < numPegs; p++)
	{
		float loc = GetPegLocation(p);
		if (loc-tolerance <= px && px <= loc+tolerance)
			return p;
	}
	
	return -1;
}


template <int numDisks, int numPegs>
bool TOH<numDisks, numPegs>::Drag(const TOHState<numDisks, numPegs> &currState, int peg)
{
	if (peg == -1) // if in empty space
		return false;
	
	if (currState.GetDiskCountOnPeg(peg) == 0) // if the peg has no disks
		return false;
	
	return true;
}

template <int numDisks, int numPegs>
bool TOH<numDisks, numPegs>::Release(const TOHState<numDisks, numPegs> &currState, int &peg, point3d loc, TOHState<numDisks, numPegs> &nextState, int &userMoveCount)
{
	if (peg == -1) // no disk to release
		return false;
	
	int nextPeg = GetHoveredPeg(loc.x);
	
	nextState = currState;
	
	if (peg == nextPeg)
		return true;
	
	if (nextPeg != -1 && currState.GetSmallestDiskOnPeg(peg) < currState.GetSmallestDiskOnPeg(nextPeg)) // if the next peg is actually a valid next peg
	{
		TOHMove m = TOHMove(peg, nextPeg);
		ApplyAction(nextState, m);
		userMoveCount++;
		
		return true;
	}
	
	return false;
}


template <int patternDisks, int totalDisks, int offset=0, int numPegs=4, uint64_t pdbBits=8>
class TOHPDB : public PDBHeuristic<TOHState<patternDisks, numPegs>, TOHMove, TOH<patternDisks, numPegs>, TOHState<totalDisks, numPegs>, pdbBits> {
public:
	TOHPDB(TOH<patternDisks, numPegs> *e, const TOHState<totalDisks, numPegs> &s)
	:PDBHeuristic<TOHState<patternDisks, numPegs>, TOHMove, TOH<patternDisks, numPegs>, TOHState<totalDisks, numPegs>, pdbBits>(e) { this->SetGoal(s); }
	virtual ~TOHPDB() {}
	
	TOHState<totalDisks, numPegs> GetStateFromAbstractState(TOHState<patternDisks, numPegs> &start) const
	{
		int diff = totalDisks - patternDisks;
		
		TOHState<totalDisks, numPegs> tmp;
		for (int x = 0; x < numPegs; x++)
		{
			tmp.counts[x] = start.counts[x];
			for (int y = 0; y < tmp.counts[x]; y++)
			{
				tmp.disks[x][y] = start.disks[x][y]+diff-offset;
			}
		}
		return tmp;
	}
	//
	// 6 5
	//
	// 4 3
	//
	// 2
	//
	// 1
	virtual uint64_t GetAbstractHash(const TOHState<totalDisks, numPegs> &s, int threadID = 0) const
	{
		int diff = totalDisks - patternDisks;
		uint64_t hash = 0;
		for (int x = 0; x < numPegs; x++)
		{
			for (int y = 0; y < s.GetDiskCountOnPeg(x); y++)
			{
				// 6 total 2 pattern
				if ((s.GetDiskOnPeg(x, y) > diff-offset) && (s.GetDiskOnPeg(x, y) <= totalDisks-offset))
					hash += static_cast<uint64_t>(x) * TOHIntPow(numPegs, s.GetDiskOnPeg(x, y)-1-diff+offset);
			}
		}
		return hash;
	}
	
	virtual uint64_t GetPDBSize() const
	{
		return TOHIntPow(numPegs, patternDisks);
	}
	virtual uint64_t GetPDBHash(const TOHState<patternDisks, numPegs> &s, int threadID = 0) const
	{
		return this->env->GetStateHash(s);
	}
	virtual void GetStateFromPDBHash(uint64_t hash, TOHState<patternDisks, numPegs> &s, int threadID = 0) const
	{
		this->env->GetStateFromHash(hash, s);
	}
	
	virtual bool Load(const char *prefix)
	{
		return false;
	}
	virtual void Save(const char *prefix)
	{
		FILE *f = fopen(GetFileName(prefix).c_str(), "w+");
		if (f == 0)
		{
			fprintf(stderr, "Error saving");
			return;
		}
		PDBHeuristic<TOHState<patternDisks, numPegs>, TOHMove, TOH<patternDisks, numPegs>, TOHState<totalDisks, numPegs>, pdbBits>::Save(f);
		fclose(f);
	}
	
	virtual std::string GetFileName(const char *prefix)
	{
		std::string s = prefix;
		s += "TOH"+std::to_string(numPegs)+"+"+std::to_string(patternDisks)+"+"+std::to_string(totalDisks)+".pdb";
		return s;
	}
};

#endif /* TOH_hpp */
