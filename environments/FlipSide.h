/*
 *  FlipSide.h
 *  hog2
 *
 *  Created by Nathan Sturtevant on 5/22/07.
 *  Copyright 2007 Nathan Sturtevant, University of Alberta. All rights reserved.
 *
 */

#ifndef FLIPSIDE_H
#define FLIPSIDE_H

#include <stdint.h>
#include <iostream>
#include "SearchEnvironment.h"
#include "UnitSimulation.h"

class FlipSideState {
public:
	FlipSideState()
{ width = 5;
		puzzle.resize(width*2);
		for (unsigned int x = 0; x < puzzle.size(); x++)
			puzzle[x] = x;
}
	FlipSideState(int _width)
	:width(_width)
{
	puzzle.resize(width*2);
	for (unsigned int x = 0; x < puzzle.size(); x++)
		puzzle[x] = x;
}
unsigned int width;
std::vector<int> puzzle;
};

class flipMove {
public:
	flipMove() { top = 0; bottom = 0; }
	flipMove(int t, int b) :top(t), bottom(b) {}
	uint16_t top, bottom;
};

static std::ostream& operator <<(std::ostream & out, const FlipSideState &loc)
{
	for (unsigned int x = 0; x < loc.puzzle.size(); x++)
	{
		if (x == loc.puzzle.size()/2)
			out << std::endl;
		out << loc.puzzle[x] << " ";
	}
	out << std::endl;
	return out;
}

static bool operator==(const FlipSideState &l1, const FlipSideState &l2)
{
	if (l1.width != l2.width)
		return false;
	for (unsigned int x = 0; x < l1.puzzle.size(); x++)
		if (l1.puzzle[x] != l2.puzzle[x])
			return false;
	return true;
}

static bool operator==(const flipMove &l1, const flipMove &l2)
{
	return l1.top == l2.top && l1.bottom == l2.bottom;
}

class FlipSide : public SearchEnvironment<FlipSideState, flipMove> {
public:
	FlipSide(int width = 5);
	~FlipSide();
	void GetSuccessors(const FlipSideState &stateID, std::vector<FlipSideState> &neighbors) const;
	void GetActions(const FlipSideState &stateID, std::vector<flipMove> &actions) const;
	flipMove GetAction(const FlipSideState &s1, const FlipSideState &s2) const;
	void ApplyAction(FlipSideState &s, flipMove a) const;
	bool InvertAction(flipMove &) const { return true; } // applying the same action inverts it

	//OccupancyInterface<FlipSideState, flipMove> *GetOccupancyInfo() { return 0; }

	double HCost(const FlipSideState &) const {
		fprintf(stderr, "ERROR: Single State HCost not implemented for FlipSide\n");
		exit(1); return -1.0;}
	double HCost(const FlipSideState &state1, const FlipSideState &state2) const;
	double GCost(const FlipSideState &state1, const FlipSideState &state2) const;
	double GCost(const FlipSideState &, const flipMove &) const { return 1.0; }
	bool GoalTest(const FlipSideState &state, const FlipSideState &goal) const;

	bool GoalTest(const FlipSideState &) const{
		fprintf(stderr, "ERROR: Single State Goal Test not implemented for FlipSide\n");
		exit(1); return false;}

	uint64_t GetStateHash(const FlipSideState &state) const;
	uint64_t GetActionHash(flipMove act) const;
//	void OpenGLDraw() const;
//	void OpenGLDraw(const FlipSideState &s) const;
//	void OpenGLDraw(const FlipSideState &, const flipMove &)  const{ /* currently not drawing moves */ }
//	void OpenGLDraw(const FlipSideState&, const FlipSideState&, float) const { /* currently not drawing moves */ }

	void StoreGoal(FlipSideState &){}
	void ClearGoal(){}
	bool IsGoalStored() const {return false;}
private:
		int width;
};

typedef UnitSimulation<FlipSideState, flipMove, FlipSide> FlipPuzzleSimulation;

#endif
