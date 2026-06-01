/*
 *  GraphRefinementEnvironment.h
 *  hog2
 *
 *  Created by Nathan Sturtevant on 4/15/10.
 *  Copyright 2010 NS Software. All rights reserved.
 *
 */

#ifndef GRAPHREFINEMENTENVIRONMENT_H
#define GRAPHREFINEMENTENVIRONMENT_H

#include <stdint.h>
#include <unordered_map>
#include <iostream>
#include "GraphEnvironment.h"
#include "GraphAbstraction.h"

class GraphRefinementEnvironment : public GraphEnvironment
{
public:
	GraphRefinementEnvironment(GraphAbstraction *ga, int planLevel, GraphHeuristic *gh, Map *m = 0);
	~GraphRefinementEnvironment();
	
//	void SetPlanningLevel(int planLevel);
//	int GetPlanningLevel();
	
	virtual void GetSuccessors(const graphState &stateID, std::vector<graphState> &neighbors) const;
	virtual void GetActions(const graphState &stateID, std::vector<graphMove> &actions) const;
	virtual bool GoalTest(const graphState &state, const graphState &goal) const;
	virtual bool GoalTest(const graphState &) const { assert(false); return false; }
	virtual void SetUseAbstractGoal(bool use, int level) { useAbstractGoal = use; abstractGoalLevel = level; }
	//void SetPlanningCorridor(std::vector<graphState> &corridor, int level);
	void SetPlanningCorridor(std::vector<graphState> &corridor, int level, int start = 0);
	double HCost(const graphState &state1, const graphState &state2) const;
	double HCost(const graphState &) const { assert(false); return false; }
private:
	GraphAbstraction *ga;
	typedef std::unordered_map<graphState, bool> CorridorCheck;
	int planLevel, corridorLevel, abstractGoalLevel;
	bool useAbstractGoal;
	CorridorCheck corridorTable;
};

#endif
