/*
 *  MNAgentPuzzle.h
 *  hog2
 *
 *  Created by Nathan Sturtevant on 9/25/10.
 *  Copyright 2010 University of Denver. All rights reserved.
 *
 */

#include "SearchEnvironment.h"
#include <vector>
#include <string.h>
#include <iostream>
#include <iomanip>


#ifndef MNAGENTPUZZLE_H
#define MNAGENTPUZZLE_H

enum tAgentAction {
	kAgentStay = 0,
	kAgentLeft = 1,
	kAgentRight = 2,
	kAgentUp = 3,
	kAgentDown = 4
};

const uint64_t filled = 0xFFFFFFFFFFFFFFFFll;

class MNAgentPuzzleState {
public:
	MNAgentPuzzleState() { width = height = -1; }
	MNAgentPuzzleState(unsigned int _width, unsigned int _height)
	:width(_width), height(_height), locations(height*width)
	{
		//locations.resize(height*width);
		numAgents = 0;
		currentAgent = 0;
	}
	void BlockCell(int x, int y)
	{
		assert(locations[y*width+x] == 0);
		locations[y*width+x] = filled;
	}
	void AddAgent(int x, int y)
	{
		locations[y*width+x] |= (1<<numAgents);
		numAgents++;
	}
	int width, height;
	unsigned int numAgents;
	int currentAgent;
	std::vector<uint64_t> locations;
};

static std::ostream& operator <<(std::ostream & out, const MNAgentPuzzleState &loc)
{
	for (int y = 0; y < loc.height; y++)
	{
		for (int x = 0; x < loc.width; x++)
		{
			if (loc.locations[y*loc.width+x] == filled)
				out << "X";
			else if (loc.locations[y*loc.width+x] == 0)
				out << ".";
			else {
				for (int t = 0; t < 64; t++)
				{
					if ((loc.locations[y*loc.width+x]&((uint64_t)1<<t)) != 0)
					{
						//out << std::setw (2)  << t;
						out << t;
						break;
					}
				}
			}
		}
		out << std::endl;
	}
	return out;
}

static bool operator==(const MNAgentPuzzleState &l1, const MNAgentPuzzleState &l2) {
	for (unsigned int x = 0; x < l1.locations.size(); x++)
		if (l1.locations[x] != l2.locations[x])
			return false;
	return true;
}

class MNAgentEnvironment : public SearchEnvironment<MNAgentPuzzleState, tAgentAction>
{
public:
	MNAgentEnvironment() { domainAbstractionSize = -1; }
	virtual void GetSuccessors(const MNAgentPuzzleState &nodeID, std::vector<MNAgentPuzzleState> &neighbors) const;
	virtual void GetActions(const MNAgentPuzzleState &nodeID, std::vector<tAgentAction> &actions) const;
	//virtual int GetNumSuccessors(const MNAgentPuzzleState &stateID) const;
	virtual tAgentAction GetAction(const MNAgentPuzzleState &s1, const MNAgentPuzzleState &s2) const;
	virtual void ApplyAction(MNAgentPuzzleState &s, tAgentAction a) const;
	
	virtual void GetNextState(const MNAgentPuzzleState &, tAgentAction , MNAgentPuzzleState &) const;
	
	virtual bool InvertAction(tAgentAction &a) const;	
	
	/** Heuristic value between two arbitrary nodes. **/
	virtual double HCost(const MNAgentPuzzleState &node1, const MNAgentPuzzleState &node2) const;
	
	/** Heuristic value between node and the stored goal. Asserts that the
	 goal is stored **/
	virtual double HCost(const MNAgentPuzzleState &node) const
	{ assert(bValidSearchGoal); return HCost(node, searchGoal); }
	
	virtual double GCost(const MNAgentPuzzleState &node1, const MNAgentPuzzleState &node2) const;
	virtual double GCost(const MNAgentPuzzleState &node, const tAgentAction &act) const;
	virtual bool GoalTest(const MNAgentPuzzleState &node, const MNAgentPuzzleState &goal) const;
	
	/** Goal Test if the goal is stored **/
	virtual bool GoalTest(const MNAgentPuzzleState &node) const
	{ return bValidSearchGoal&&(node == searchGoal); }
	
	void SetDomainAbstractionSize(int val) { domainAbstractionSize = val; }
	virtual uint64_t GetStateHash(const MNAgentPuzzleState &node) const;
	virtual uint64_t GetActionHash(tAgentAction act) const;
	
	//virtual double GetPathLength(std::vector<MNAgentPuzzleState> &neighbors);
		
//	virtual void OpenGLDraw() const;
//	virtual void OpenGLDraw(const MNAgentPuzzleState&) const;
//	/** Draw the transition at some percentage 0...1 between two states */
//	virtual void OpenGLDraw(const MNAgentPuzzleState&, const MNAgentPuzzleState&, float) const;
//	virtual void OpenGLDraw(const MNAgentPuzzleState&, const tAgentAction&) const;
private:
	void FindLegalMoves(MNAgentPuzzleState &s,
						tAgentAction &curr,
						std::vector<tAgentAction> &actions,
						int realMoves,
						int depth,
						std::vector<bool> &moved) const;
	int domainAbstractionSize;
};

#endif
