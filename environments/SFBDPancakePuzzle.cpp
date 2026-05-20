/*
 *  SFBDPancakePuzzle.cpp
 *  hog2
 *
 *  Created by Nathan Sturtevant on 1/14/10.
 *  Copyright 2010 NS Software. All rights reserved.
 *
 */

#include "SFBDPancakePuzzle.h"

SFBDPancakeEnvironment::SFBDPancakeEnvironment(int theSize)
{
	pan = new BurnedPancakePuzzle(theSize);
}

SFBDPancakeEnvironment::~SFBDPancakeEnvironment()
{
	delete pan;
}

void SFBDPancakeEnvironment::GetSuccessors(const pancakeStatePair &stateID, std::vector<pancakeStatePair> &neighbors) const
{
	assert(false);
}

void SFBDPancakeEnvironment::GetActions(const pancakeStatePair &stateID, std::vector<pancakeMovePair> &actions) const
{
	std::vector<pancakeMovePair> acts;
	bool fromStart = true;

	int startScore = MeasureLocalHeuristic(stateID.start, stateID.goal);
	int goalScore = MeasureLocalHeuristic(stateID.goal, stateID.start);
	
	if (startScore > goalScore)
	{
		fromStart = false;
//		assert(pan->HCost(stateID.start, stateID.goal)==pan->HCost(stateID.goal, stateID.start));
//		std::cout << "Start score: [" << startScore << "] " << stateID.start << std::endl;
//		std::cout << "Goal score: [" << goalScore << "] " << stateID.goal << std::endl;
	}

	actions.clear();
	for (unsigned i = stateID.start.puzzle.size(); i >= 2; i--)
		actions.push_back(pancakeMovePair(i, fromStart));
//	for (unsigned i = 2; i < stateID.start.puzzle.size(); i++)
//		actions.push_back(pancakeMovePair(i, fromStart));
}

int SFBDPancakeEnvironment::MeasureLocalHeuristic(const BurnedPancakePuzzleState &a, const BurnedPancakePuzzleState &b) const
{
	double hc = pan->HCost(a, b);
	int result = 0;
	std::vector<unsigned> acts;
	BurnedPancakePuzzleState a1 = a;
	pan->GetActions(a1, acts);
	for (unsigned int x = 0; x < acts.size(); x++)
	{
		pan->ApplyAction(a1, acts[x]);
//		for (unsigned int y = 0; y < acts.size(); y++)
//		{
//			if (x==y) continue;
//			pan->ApplyAction(a1, acts[y]);
			double c = pan->HCost(a1, b);
			result += (c>hc)?(1):0;
//			pan->ApplyAction(a1, acts[y]);
//		}
		pan->ApplyAction(a1, acts[x]);
	}
	return -result;
	//	std::vector<int> goal_locs(a.puzzle.size());
//	for (unsigned i = 0; i < a.puzzle.size(); i++) {
//		goal_locs[abs(b.puzzle[i])] = i;
//	}
//	
//	int total = 0;
//	//	int chunkSize = 1;
//	for (unsigned int i = 2; i < a.puzzle.size()-1; i++) {
//		int diff1 = abs(goal_locs[abs(a.puzzle[i])] - goal_locs[abs(a.puzzle[i+1])]);
//		int diff2 = abs(goal_locs[abs(a.puzzle[i])] - goal_locs[abs(a.puzzle[i-1])]);
//		if ((diff1 > 1) && (diff2 > 1)) total--;
//		if (diff1 > 1) total--;
//		//		if (diff2 > 1) total++;
//	}
//	return total;
}

// best so far, but needs to be cheaper (regular pancake)
//	double hc = pan->HCost(a, b);
//	int result = 0;
//	std::vector<unsigned> acts;
//	BurnedPancakePuzzleState a1 = a;
//	pan->GetActions(a1, acts);
//	for (unsigned int x = 0; x < acts.size(); x++)
//	{
//		pan->ApplyAction(a1, acts[x]);
//		result += (pan->HCost(a1, b)>hc)?1:0;
//		pan->ApplyAction(a1, acts[x]);
//	}
//	return result;

// current best! (regular pancake)
//int SFBDPancakeEnvironment::MeasureLocalHeuristic(const PancakePuzzleState &a, const PancakePuzzleState &b) const
//{
//	std::vector<int> goal_locs(a.puzzle.size());
//	for (unsigned i = 0; i < a.puzzle.size(); i++) {
//		goal_locs[b.puzzle[i]] = i;
//	}
//	
//	int total = 0;
//	//	int chunkSize = 1;
//	for (unsigned int i = 1; i < a.puzzle.size()-1; i++) {
//		int diff1 = abs(goal_locs[a.puzzle[i]] - goal_locs[a.puzzle[i+1]]);
//		//		int diff2 = abs(goal_locs[a.puzzle[0]] - goal_locs[a.puzzle[i+1]]);
//		if (diff1 > 1) total--;
//		//		if (diff2 > 1) total++;
//	}
//	return total;
//}

// marginally better
//std::vector<int> goal_locs(a.puzzle.size());
//for (unsigned i = 0; i < a.puzzle.size(); i++) {
//	goal_locs[b.puzzle[i]] = i;
//}
//
//int total = 0;
////	int chunkSize = 1;
//for (unsigned int i = 2; i < a.puzzle.size()-1; i++) {
//	int diff1 = abs(goal_locs[a.puzzle[i]] - goal_locs[a.puzzle[i+1]]);
//	int diff2 = abs(goal_locs[a.puzzle[i]] - goal_locs[a.puzzle[i-1]]);
//	if ((diff1 > 1) && (diff2 > 1)) total--;
//	if (diff1 > 1) total--;
//	//		if (diff2 > 1) total++;
//}
//return total;


pancakeMovePair SFBDPancakeEnvironment::GetAction(const pancakeStatePair &, const pancakeStatePair &) const
{
	assert(false);
	return pancakeMovePair(1, false);
}

void SFBDPancakeEnvironment::ApplyAction(pancakeStatePair &s, pancakeMovePair a) const
{
	if (a.applyToStart)
		pan->ApplyAction(s.start, a.theAction);
	else
		pan->ApplyAction(s.goal, a.theAction);
}

bool SFBDPancakeEnvironment::InvertAction(pancakeMovePair &) const
{
	return true;
}

double SFBDPancakeEnvironment::HCost(const pancakeStatePair &state1, const pancakeStatePair &) const
{
	return pan->HCost(state1.start, state1.goal);
}

double SFBDPancakeEnvironment::GCost(const pancakeStatePair &, const pancakeMovePair &) const
{
	return 1;
}

double SFBDPancakeEnvironment::GCost(const pancakeStatePair &, const pancakeStatePair &) const
{
	return 1;
}

bool SFBDPancakeEnvironment::GoalTest(const pancakeStatePair &state, const pancakeStatePair &) const
{
	return (state.start == state.goal);	
}

uint64_t SFBDPancakeEnvironment::GetStateHash(const pancakeStatePair &state) const
{
	return pan->GetStateHash(state.start)^pan->GetStateHash(state.goal);
}

uint64_t SFBDPancakeEnvironment::GetActionHash(pancakeMovePair act) const
{
	return (act.theAction<<1)|(act.applyToStart?1:0);
}

//void SFBDPancakeEnvironment::OpenGLDraw() const
//{
//}
//
//void SFBDPancakeEnvironment::OpenGLDraw(const pancakeStatePair &) const
//{
//}
//
//void SFBDPancakeEnvironment::OpenGLDraw(const pancakeStatePair &, const pancakeMovePair &) const
//{
//	// if we want to draw a set of moves we use this to do so
//}
//
