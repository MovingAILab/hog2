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
#include <math.h>
#include "SearchEnvironment.h"
#include "PDBHeuristic.h"

struct TOHMove {
	TOHMove(uint8_t s, uint8_t d) :source(s), dest(d) {}
	TOHMove() {}
	uint8_t source;
	uint8_t dest;
};

template <int numDisks>
struct TOHState {
	TOHState()
	{
		for (int x = 0; x < 4; x++)
		{
			counts[x] = 0;
		}
		for (int x = 0; x < numDisks; x++)
		{
			disks[3][x] = numDisks-x;
		}
		counts[3] = numDisks;
	}
	
	void Reset()
	{
		for (int x = 0; x < 4; x++)
		{
			counts[x] = 0;
		}
		for (int x = 0; x < numDisks; x++)
		{
			disks[3][x] = numDisks-x;
		}
		counts[3] = numDisks;
	}
	void StandardStart()
	{
		for (int x = 0; x < 4; x++)
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
		return counts[whichPeg];
	}
	
	int GetDiskOnPeg(int whichPeg, int whichDisk) const
	{
		return disks[whichPeg][whichDisk];
	}

	// if this is slow, we can add a "big" disk to every peg to
	// avoid the "if" statement.
	int GetSmallestDiskOnPeg(int whichPeg) const
	{
		int count = GetDiskCountOnPeg(whichPeg);
		if (count == 0)
			return numDisks+1;
		return GetDiskOnPeg(whichPeg, count-1);
	}

	uint8_t disks[4][numDisks];
	uint8_t counts[4];
};

template <int D>
static std::ostream &operator<<(std::ostream &out, const TOHState<D> &s)
{
	for (int x = 0; x < 4; x++)
	{
		out << "(" << x << ") ";
		for (int y = 0; y < s.GetDiskCountOnPeg(x); y++)
			out << s.GetDiskOnPeg(x, y) << " ";
	}
	return out;
}

template <int D>
static bool operator==(const TOHState<D> &l1, const TOHState<D> &l2) {
	for (int x = 0; x < 4; x++)
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

template <int D>
static bool operator!=(const TOHState<D> &l1, const TOHState<D> &l2) {
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


template <int disks>
class TOH : public SearchEnvironment<TOHState<disks>, TOHMove> {
public:
	TOH() :pruneActions(false) {}
	~TOH() {}
	void GetSuccessors(const TOHState<disks> &nodeID, std::vector<TOHState<disks>> &neighbors) const;
	void GetActions(const TOHState<disks> &nodeID, std::vector<TOHMove> &actions) const;
	void ApplyAction(TOHState<disks> &s, TOHMove a) const;
	bool InvertAction(TOHMove &a) const;

	/** Heuristic value between two arbitrary nodes. **/
	double HCost(const TOHState<disks> &node1, const TOHState<disks> &node2) const;
	double GCost(const TOHState<disks> &node1, const TOHState<disks> &node2) const { return 1; }
	double GCost(const TOHState<disks> &node, const TOHMove &act) const { return 1; }
	bool GoalTest(const TOHState<disks> &node, const TOHState<disks> &goal) const;

	uint64_t GetStateHash(const TOHState<disks> &node) const;
	void GetStateFromHash(uint64_t parent, TOHState<disks> &s) const;
	uint64_t GetMaxHash() const { return (1ull)<<(disks*2ull); }
	uint64_t GetNumStates(TOHState<disks> &s) const;
	uint64_t GetActionHash(TOHMove act) const;

	std::string GetName() { return "TOH("+std::to_string(disks)+")"; }
//	void OpenGLDraw() const;
//	void OpenGLDraw(const TOHState<disks>&) const;
//	/** Draw the transition at some percentage 0...1 between two TOHState<disks>s */
//	void OpenGLDraw(const TOHState<disks>&, const TOHState<disks>&, float) const;
//	void OpenGLDraw(const TOHState<disks>&, const TOHMove&) const;

    void Draw(Graphics::Display &display, std::string str) const; // draws text
	void Draw(Graphics::Display &display) const; // draws the base and lines
	void Draw(Graphics::Display &display, const TOHState<disks> &s) const; // draws the disks when not animating
	void Draw(Graphics::Display &display, const TOHState<disks> &l1, const TOHState<disks> &l2, float v) const; // animation for optimal solution
    void Draw(Graphics::Display &display, const TOHState<disks> &l1, int selectedPeg, int nextPeg, float v) const; // vertical animation for when user is solving
    void Draw(Graphics::Display &display, const TOHState<disks> &l1, int startPeg, float px); // horizontal animation for when user is solving
    void Draw(Graphics::Display &display, const TOHState<disks>&, TOHMove&) const;
    
    bool Click(int &peg, float px);
    int GetHoveredPeg(const float &px);
    bool Drag(const TOHState<disks> &currState, int peg);
    bool Release(const TOHState<disks> &currState, int &peg, point3d loc, TOHState<disks> &nextState, int &userMoveCount);
	
	bool pruneActions;
protected:
private:
	// caches
	mutable std::vector<TOHMove> acts;
	mutable TOHState<disks> tmp;

};



template <int disks>
void TOH<disks>::GetSuccessors(const TOHState<disks> &nodeID, std::vector<TOHState<disks>> &neighbors) const
{
	neighbors.resize(0);
	GetActions(nodeID, acts);
	for (auto act : acts)
	{
		this->GetNextState(nodeID, act, tmp);
		neighbors.push_back(tmp);
	}
}

template <int disks>
void TOH<disks>::GetActions(const TOHState<disks> &s, std::vector<TOHMove> &actions) const
{
	bool goalOrdered = false;
	if (pruneActions)
	{
		goalOrdered = true;
		for (int x = 0; x < s.GetDiskCountOnPeg(3); x++)
		{
			if (s.GetDiskOnPeg(3, x) != disks-x)
			{
				goalOrdered = false;
				break;
			}
		}
	}
	
	actions.resize(0);
	if (s.GetSmallestDiskOnPeg(0) < s.GetSmallestDiskOnPeg(1))
	{
		if (s.GetDiskCountOnPeg(0) > 0)
			actions.push_back(TOHMove(0, 1));
	}
	else {
		if (s.GetDiskCountOnPeg(1) > 0)
			actions.push_back(TOHMove(1, 0));
	}
	if (s.GetSmallestDiskOnPeg(0) < s.GetSmallestDiskOnPeg(2))
	{
		if (s.GetDiskCountOnPeg(0) > 0)
			actions.push_back(TOHMove(0, 2));
	}
	else {
		if (s.GetDiskCountOnPeg(2) > 0)
			actions.push_back(TOHMove(2, 0));
	}
	
	if (s.GetSmallestDiskOnPeg(0) < s.GetSmallestDiskOnPeg(3))
	{
		if (s.GetDiskCountOnPeg(0) > 0)
			actions.push_back(TOHMove(0, 3));
	}
	else {
		if (s.GetDiskCountOnPeg(3) > 0 && !goalOrdered)
			actions.push_back(TOHMove(3, 0));
	}
	if (s.GetSmallestDiskOnPeg(1) < s.GetSmallestDiskOnPeg(2))
	{
		if (s.GetDiskCountOnPeg(1) > 0)
			actions.push_back(TOHMove(1, 2));
	}
	else {
		if (s.GetDiskCountOnPeg(2) > 0)
			actions.push_back(TOHMove(2, 1));
	}
	if (s.GetSmallestDiskOnPeg(1) < s.GetSmallestDiskOnPeg(3))
	{
		if (s.GetDiskCountOnPeg(1) > 0)
			actions.push_back(TOHMove(1, 3));
	}
	else {
		if (s.GetDiskCountOnPeg(3) > 0 && !goalOrdered)
			actions.push_back(TOHMove(3, 1));
	}
	if (s.GetSmallestDiskOnPeg(2) < s.GetSmallestDiskOnPeg(3))
	{
		if (s.GetDiskCountOnPeg(2) > 0)
			actions.push_back(TOHMove(2, 3));
	}
	else {
		if (s.GetDiskCountOnPeg(3) > 0 && !goalOrdered)
			actions.push_back(TOHMove(3, 2));
	}
}


template <int disks>
void TOH<disks>::ApplyAction(TOHState<disks> &s, TOHMove m) const
{
	s.disks[m.dest][s.counts[m.dest]] = s.disks[m.source][s.counts[m.source]-1];
	s.counts[m.dest]++;
	s.counts[m.source]--;
}

template <int disks>
bool TOH<disks>::InvertAction(TOHMove &a) const
{
	uint8_t tmp = a.source;
	a.source = a.dest;
	a.dest = tmp;
	return true;
}


/** Heuristic value between two arbitrary nodes. **/
template <int disks>
double TOH<disks>::HCost(const TOHState<disks> &node1, const TOHState<disks> &node2) const
{
	// NOTE: this is using the standard goal state; arbitrary goal states
	// are more expensive to check
	return disks - node1.GetDiskCountOnPeg(3);
}

template <int disks>
bool TOH<disks>::GoalTest(const TOHState<disks> &node, const TOHState<disks> &goal) const
{
	// NOTE: This goal test is only from standard start to standard goal
	return (node.GetDiskCountOnPeg(3) == disks && node.GetDiskOnPeg(3, 0) == disks);
//	return (node.GetDiskCountOnPeg(3) == 1 && node.GetDiskOnPeg(3, 0) == disks);
	// NOTE: this is using the standard goal state; arbitrary goal states
	// are more expensive to check
	return (node == goal);
	//return (node.GetDiskCountOnPeg(3)==disks);
}


template <int disks>
uint64_t TOH<disks>::GetStateHash(const TOHState<disks> &node) const
{
	uint64_t hash = 0;
	for (int x = 0; x < 4; x++)
	{
		for (int y = 0; y < node.GetDiskCountOnPeg(x); y++)
		{
			hash |= (uint64_t(x)<<(2*(node.GetDiskOnPeg(x, y)-1)));
		}
	}
	return hash;
}

template <int disks>
uint64_t TOH<disks>::GetNumStates(TOHState<disks> &s) const
{
	return 1ull<<(2*disks);
}

template <int disks>
void TOH<disks>::GetStateFromHash(uint64_t hash, TOHState<disks> &s) const
{
	for (int x = 0; x < 4; x++)
		s.counts[x] = 0;
	for (int x = disks-1; x >= 0; x--)
	{
		int nextPeg = (hash>>(2*x))&0x3;
		s.disks[nextPeg][s.counts[nextPeg]] = x+1;
		s.counts[nextPeg]++;
	}
}

template <int disks>
uint64_t TOH<disks>::GetActionHash(TOHMove act) const
{
	return (act.source<<8)|act.dest;
}

//template <int disks>
//void TOH<disks>::OpenGLDraw() const
//{
//	glColor3f(0.5, 0.5, 0.5);
//	DrawCylinder(-0.75, 0, 0, 0, 0.01, 0.8);
//	DrawCylinder(-0.25, 0, 0, 0, 0.01, 0.8);
//	DrawCylinder( 0.25, 0, 0, 0, 0.01, 0.8);
//	DrawCylinder( 0.75, 0, 0, 0, 0.01, 0.8);
//	glColor3f(0.6, 0.4, 0.2);
//	glPushMatrix();
//	glScalef(1.0, 0.05, 0.25);
//	DrawBox(0, 0.4/0.05+1, 0, 1.0);
//	glPopMatrix();
//	//	DrawBox(0, 0, 0, 0.5);
//}
//
//template <int disks>
//void TOH<disks>::OpenGLDraw(const TOHState<disks>&s) const
//{
//	glColor3f(0.0, 0.0, 1.0);
//	double offset[4] = {-0.75, -0.25, 0.25, 0.75};
//	for (int x = 0; x < 4; x++)
//	{
//		for (int y = 0; y < s.GetDiskCountOnPeg(x); y++)
//		{
//			int which = s.GetDiskOnPeg(x, y);
//			glColor3f(0.0, 1.0-float(which)/float(disks), 1.0);
//			DrawCylinder(offset[x], 0.4-0.4/(1+float(disks))-y*0.8/(1+float(disks)), 0,
//						 0.02, 0.04+0.2*which/float(disks), 0.8/(1+float(disks)));
//		}
//	}
//}
//
///** Draw the transition at some percentage 0...1 between two TOHState<disks>s */
//template <int disks>
//void TOH<disks>::OpenGLDraw(const TOHState<disks>&s, const TOHState<disks>&s2, float interval) const
//{
//	TOHMove m = this->GetAction(s, s2);
//	int animatingDisk = s.GetSmallestDiskOnPeg(m.source);
//	int initialHeight = s.GetDiskCountOnPeg(m.source)-1;
//	int finalHeight = s.GetDiskCountOnPeg(m.dest);
//	
//	glColor3f(0.0, 0.0, 1.0);
//	double offset[4] = {-0.75, -0.25, 0.25, 0.75};
//	for (int x = 0; x < 4; x++)
//	{
//		for (int y = 0; y < s.GetDiskCountOnPeg(x); y++)
//		{
//			int which = s.GetDiskOnPeg(x, y);
//			if (which != animatingDisk)
//			{
//				glColor3f(0.0, 1.0-float(which)/float(disks), 1.0);
//				DrawCylinder(offset[x], 0.4-0.4/(1+float(disks))-y*0.8/(1+float(disks)), 0,
//							 0.02, 0.04+0.2*which/float(disks), 0.8/(1+float(disks)));
//			}
//		}
//	}
//	glColor3f(0.0, 1.0-float(animatingDisk)/float(disks), 1.0);
//	if (interval <= 0.333)
//	{
//		interval *= 3;
//		DrawCylinder(offset[m.source], 0.4-0.4/(1+float(disks))-initialHeight*0.8/(1+float(disks)) - (interval)*(disks+1-initialHeight)*0.8/(1+float(disks)), 0,
//					 0.02, 0.04+0.2*animatingDisk/float(disks), 0.8/(1+float(disks)));
//	}
//	else if (interval <= 0.666)
//	{
//		interval *= 3;
//		DrawCylinder((2-interval)*offset[m.source]+(interval-1)*offset[m.dest], 0.4-0.4/(1+float(disks))-0.8-0.2*sin((interval-1)*PI), 0,
//					 0.02, 0.04+0.2*animatingDisk/float(disks), 0.8/(1+float(disks)));
//	}
//	else {
//		DrawCylinder(offset[m.dest], 0.4-0.4/(1+float(disks))-finalHeight*0.8/(1+float(disks)) -
//					 ((1.0-interval)/0.334)*(disks+1-finalHeight)*0.8/(1+float(disks)), 0,
//					 0.02, 0.04+0.2*animatingDisk/float(disks), 0.8/(1+float(disks)));
//	}
//}
//
//template <int disks>
//void TOH<disks>::OpenGLDraw(const TOHState<disks>&, const TOHMove&) const
//{
//	
//}


// Draw for text area
template <int disks>
void TOH<disks>::Draw(Graphics::Display &display, std::string str) const
{
    Graphics::rect r1(-1, -1, 1, -0.8); // background for text area
    display.FillRect(r1, Colors::lightgray);
    
    display.DrawText(str.c_str(), Graphics::point{-0.9, -0.9}, Colors::black, 0.075,
                     Graphics::textAlignLeft, Graphics::textBaselineMiddle);
}

// Draw for pegs and base
template <int disks>
void TOH<disks>::Draw(Graphics::Display &display) const
{
	Graphics::rect r1(-0.75-0.01, 0, -0.75+0.01, 0.9); // peg

	display.FillRect(r1, Colors::gray);
    
	r1.left += 0.5; r1.right += 0.5; // adds margin of space to the left and right
	display.FillRect(r1, Colors::gray);
	r1.left += 0.5; r1.right += 0.5;
	display.FillRect(r1, Colors::gray);
	r1.left += 0.5; r1.right += 0.5;
	display.FillRect(r1, Colors::gray);
	r1.left += 0.5; r1.right += 0.5;
    
    display.FillRect({-1, 0.8, 1, 0.92}, {0.6, 0.4, 0.2}); // brown base
}

// Draw for still state
template <int disks>
void TOH<disks>::Draw(Graphics::Display &display, const TOHState<disks> &s) const
{
	double offset[4] = {-0.75, -0.25, 0.25, 0.75};
	for (int x = 0; x < 4; x++)
	{
		for (int y = 0; y < s.GetDiskCountOnPeg(x); y++)
		{
			int which = s.GetDiskOnPeg(x, y);
			rgbColor color(0.0, 1.0-float(which)/float(disks), 1.0);
			Graphics::rect r(offset[x]-0.04-0.2*which/float(disks),
							 -y*0.8/(1+float(disks))-0.8/(1+float(disks))+0.8,
							 offset[x]+0.04+0.2*which/float(disks),
							 -y*0.8/(1+float(disks))+0.8);

			display.FillRect(r, color);
		}
	}
}

// Draw for animating optimal solve
template <int disks>
void TOH<disks>::Draw(Graphics::Display &display, const TOHState<disks> &s, const TOHState<disks> &s2, float v) const
{
    TOHMove m = this->GetAction(s, s2);
    
    int animatingDisk = s.GetSmallestDiskOnPeg(m.source);
    int finalHeight = s.GetDiskCountOnPeg(m.dest);
    
    float offset[4] = {-0.75, -0.25, 0.25, 0.75}; // x-positions of the pegs
    for (int x = 0; x < 4; x++)
    {
        for (int y = 0; y < s.GetDiskCountOnPeg(x); y++)
        {
            int which = s.GetDiskOnPeg(x, y);
            float halfwidth = 0.04+0.2*which/float(disks);
            if (which != animatingDisk) // first, draws every disk except for the animating one
            {
                display.FillRect({static_cast<float>(offset[x]-halfwidth), static_cast<float>(0.8-0.8/(1+float(disks))-y*0.8/(1+float(disks))), static_cast<float>(offset[x]+halfwidth), static_cast<float>(0.8-y*0.8/(1+float(disks)))}, {0.0, static_cast<float>(1.0-float(which)/float(disks)), 1.0});
            }
            else {
                int targetPeg = m.dest;
                Graphics::rect r1;
                Graphics::rect r2;
                
                if (v <= 0.333) { // up
                    v *= 3;
                    r1 = {static_cast<float>(offset[x]-halfwidth), static_cast<float>(0.8-0.8/(1+float(disks))-y*0.8/(1+float(disks))), static_cast<float>(offset[x]+halfwidth), static_cast<float>(0.8-y*0.8/(1+float(disks)))};
                    
                    r2 = {static_cast<float>(offset[x]-halfwidth), static_cast<float>(-0.5-0.8/(1+float(disks))), static_cast<float>(offset[x]+halfwidth), static_cast<float>(-0.5)};
                }
                else if (v <= 0.666) { // horizontal
                    v = (v - 0.333) * 3;
                    r1 = {static_cast<float>(offset[x]-halfwidth), static_cast<float>(-0.5-0.8/(1+float(disks))), static_cast<float>(offset[x]+halfwidth), static_cast<float>(-0.5)};
                    
                    r2 = {static_cast<float>(offset[targetPeg]-halfwidth), static_cast<float>(-0.5-0.8/(1+float(disks))), static_cast<float>(offset[targetPeg]+halfwidth), static_cast<float>(-0.5)};
                }
                else { // down
                    v = (v - 0.666) * 3;
                    r1 = {static_cast<float>(offset[targetPeg]-halfwidth), static_cast<float>(-0.5-0.8/(1+float(disks))), static_cast<float>(offset[targetPeg]+halfwidth), static_cast<float>(-0.5)};
                    
                    r2 = {static_cast<float>(offset[targetPeg]-halfwidth), static_cast<float>(0.8-0.8/(1+float(disks))-finalHeight*0.8/(1+float(disks))), static_cast<float>(offset[targetPeg]+halfwidth), static_cast<float>(0.8-finalHeight*0.8/(1+float(disks)))};
                }
                
                r1.lerp(r2, v);
                display.FillRect(r1, Colors::purple);
            }
        }
    }

   
}

// Draw for vertical animation when user is solving
template <int disks>
void TOH<disks>::Draw(Graphics::Display &display, const TOHState<disks> &s, int selectedPeg, int nextPeg, float v) const
{
    int animatingDisk = s.GetSmallestDiskOnPeg(selectedPeg);
    int finalHeight = s.GetDiskCountOnPeg(nextPeg);
    
    float offset[4] = {-0.75, -0.25, 0.25, 0.75};
    for (int x = 0; x < 4; x++)
    {
        for (int y = 0; y < s.GetDiskCountOnPeg(x); y++)
        {
            int which = s.GetDiskOnPeg(x, y);
            float halfwidth = 0.04+0.2*which/float(disks);
            if (which != animatingDisk) // first, draws every disk except for the animating one
            {
                display.FillRect({static_cast<float>(offset[x]-halfwidth), static_cast<float>(0.8-0.8/(1+float(disks))-y*0.8/(1+float(disks))), static_cast<float>(offset[x]+halfwidth), static_cast<float>(0.8-y*0.8/(1+float(disks)))}, {0.0, static_cast<float>(1.0-float(which)/float(disks)), 1.0});
            }
            else {
                Graphics::rect r1;
                Graphics::rect r2;
                
                if (v <= 0.333) { // up for the first third
                    v *= 3;
                    r1 = {static_cast<float>(offset[x]-halfwidth), static_cast<float>(0.8-0.8/(1+float(disks))-y*0.8/(1+float(disks))), static_cast<float>(offset[x]+halfwidth), static_cast<float>(0.8-y*0.8/(1+float(disks)))};
                    
                    r2 = {static_cast<float>(offset[x]-halfwidth), static_cast<float>(-0.5-0.8/(1+float(disks))), static_cast<float>(offset[x]+halfwidth), static_cast<float>(-0.5)};
                }
                else { // down for the last third. the second third is animated by Draw(display, s, startPeg, px)
                    v = (v - 0.666) * 3;
                    r1 = {static_cast<float>(offset[nextPeg]-halfwidth), static_cast<float>(-0.5-0.8/(1+float(disks))), static_cast<float>(offset[nextPeg]+halfwidth), static_cast<float>(-0.5)};
                    
                    r2 = {static_cast<float>(offset[nextPeg]-halfwidth), static_cast<float>(0.8-0.8/(1+float(disks))-finalHeight*0.8/(1+float(disks))), static_cast<float>(offset[nextPeg]+halfwidth), static_cast<float>(0.8-finalHeight*0.8/(1+float(disks)))};
                }
                
                r1.lerp(r2, v);
                display.FillRect(r1, Colors::purple);
            }
        }
    }

   
}

// Draw for horizontal animation when user is solving
template <int disks>
void TOH<disks>::Draw(Graphics::Display &display, const TOHState<disks> &s, int startPeg, float px)
{
    int animatingDisk = s.GetSmallestDiskOnPeg(startPeg);
    
    float offset[4] = {-0.75, -0.25, 0.25, 0.75}; // the x-positions for each peg
    
    // if the mouse is hovering over a peg, highlight that peg
    int hoveredPeg = GetHoveredPeg(px);
    if (hoveredPeg != -1)
    {
        Graphics::rect p(offset[hoveredPeg]-0.01, 0, offset[hoveredPeg]+0.01, 0.8);
        
        // highlight invalid pegs in red and valid pegs in purple
        if (s.GetSmallestDiskOnPeg(startPeg) > s.GetSmallestDiskOnPeg(hoveredPeg))
            display.FillRect(p, Colors::red);
        else
            display.FillRect(p, Colors::purple);
        
    }

    for (int x = 0; x < 4; x++)
    {
        for (int y = 0; y < s.GetDiskCountOnPeg(x); y++)
        {
            int which = s.GetDiskOnPeg(x, y);
            float halfwidth = 0.04+0.2*which/float(disks);
            if (which != animatingDisk) // first, draws every disk except for the animating one
            {
                display.FillRect({static_cast<float>(offset[x]-halfwidth), static_cast<float>(0.8-0.8/(1+float(disks))-y*0.8/(1+float(disks))), static_cast<float>(offset[x]+halfwidth), static_cast<float>(0.8-y*0.8/(1+float(disks)))}, {0.0, static_cast<float>(1.0-float(which)/float(disks)), 1.0});
            }
            else
            { // draws the animating disk
                Graphics::rect r1 = {static_cast<float>(px-halfwidth), static_cast<float>(-0.5-0.8/(1+float(disks))), static_cast<float>(px+halfwidth), -0.5f};
                display.FillRect(r1, Colors::purple);
            }
        }
    }

}

template <int disks>
void TOH<disks>::Draw(Graphics::Display &display, const TOHState<disks>&, TOHMove&) const
{
	// nothing here as in OpenGLDraw
}

template <int disks>
bool TOH<disks>::Click(int &peg, float px) 
{
    peg = GetHoveredPeg(px);
    
    return true;
}

template <int disks>
int TOH<disks>::GetHoveredPeg(const float &px)
{
    int peg = -1;
    
    if (-0.75-0.1 <= px && px <= -0.75+0.1) // area accepted as "peg" goes a little beyond peg boundary
    {
        peg = 0;
    }
    else if (-0.25-0.1 <= px && px <= -0.25+0.1)
    {
        peg = 1;
    }
    else if (0.25-0.1 <= px && px <= 0.25+0.1)
    {
        peg = 2;
    }
    else if (0.75-0.1 <= px && px <= 0.75+0.1)
    {
        peg = 3;
    }
    
    return peg;
}


template <int disks>
bool TOH<disks>::Drag(const TOHState<disks> &currState, int peg)
{
    if (peg == -1) // if in empty space
        return false;
    
    if (currState.GetDiskCountOnPeg(peg) == 0) // if the peg has no disks
        return false;
    
    return true;
}

template <int disks>
bool TOH<disks>::Release(const TOHState<disks> &currState, int &peg, point3d loc, TOHState<disks> &nextState, int &userMoveCount)
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


template <int patternDisks, int totalDisks, int offset=0>
class TOHPDB : public PDBHeuristic<TOHState<patternDisks>, TOHMove, TOH<patternDisks>, TOHState<totalDisks>> {
public:
	TOHPDB(TOH<patternDisks> *e, const TOHState<totalDisks> &s)
	:PDBHeuristic<TOHState<patternDisks>, TOHMove, TOH<patternDisks>, TOHState<totalDisks>>(e) { this->SetGoal(s); }
	virtual ~TOHPDB() {}

	TOHState<totalDisks> GetStateFromAbstractState(TOHState<patternDisks> &start) const
	{
		int diff = totalDisks - patternDisks;
		
		TOHState<totalDisks> tmp;
		for (int x = 0; x < 4; x++)
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
	virtual uint64_t GetAbstractHash(const TOHState<totalDisks> &s, int threadID = 0) const
	{
		int diff = totalDisks - patternDisks;
		uint64_t hash = 0;
		for (int x = 0; x < 4; x++)
		{
			for (int y = 0; y < s.GetDiskCountOnPeg(x); y++)
			{
				// 6 total 2 pattern
				if ((s.GetDiskOnPeg(x, y) > diff-offset) && (s.GetDiskOnPeg(x, y) <= totalDisks-offset))
					hash |= (uint64_t(x)<<(2*(s.GetDiskOnPeg(x, y)-1-diff+offset)));
			}
		}
		return hash;
	}

	virtual uint64_t GetPDBSize() const
	{
		return 1ull<<(2*patternDisks);
	}
	virtual uint64_t GetPDBHash(const TOHState<patternDisks> &s, int threadID = 0) const
	{
		return this->env->GetStateHash(s);
	}
	virtual void GetStateFromPDBHash(uint64_t hash, TOHState<patternDisks> &s, int threadID = 0) const
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
		PDBHeuristic<TOHState<patternDisks>, TOHMove, TOH<patternDisks>, TOHState<totalDisks>>::Save(f);
		fclose(f);
	}
	
	virtual std::string GetFileName(const char *prefix)
	{
		std::string s = prefix;
		s += "TOH4+"+std::to_string(patternDisks)+"+"+std::to_string(totalDisks)+".pdb";
		return s;
	}
};

#endif /* TOH_hpp */
