/*
 *  GraphRefinementEnvironment.cpp
 *  hog2
 *
 *  Created by Nathan Sturtevant on 4/15/10.
 *  Copyright 2010 NS Software. All rights reserved.
 *
 */

#include "GraphRefinementEnvironment.h"

//GraphAbstraction *ga;
//int planLevel, corridorLevel;
//	CorridorCheck corridorTable;

GraphRefinementEnvironment::GraphRefinementEnvironment(GraphAbstraction *ga, int planLevel, GraphHeuristic *gh, Map *ma)
:GraphEnvironment(ma, ga->GetAbstractGraph(planLevel), gh)
{
	this->planLevel = planLevel;
	this->ga = ga;
	corridorLevel = 0;
	useAbstractGoal = false;
}

GraphRefinementEnvironment::~GraphRefinementEnvironment()
{
}

//void GraphRefinementEnvironment::SetPlanningLevel(int planLevel)
//{
//}
//
//int GraphRefinementEnvironment::GetPlanningLevel()
//{
//}

void GraphRefinementEnvironment::GetSuccessors(const graphState &stateID, std::vector<graphState> &neighbors) const
{
	GraphEnvironment::GetSuccessors(stateID, neighbors);
	//printf("%d initial successors\n", (int)neighbors.size());
	if (corridorTable.size() > 0)
	{
		for (unsigned int x = 0; x < neighbors.size(); x++)
		{
			//graphState parent = 
			//Graph *g = ga->GetAbstractGraph(planLevel);
			if (corridorTable.find(ga->GetNthParent(g->GetNode(neighbors[x]),
													corridorLevel)->GetNum()) == corridorTable.end())
			{
//				printf("Excluding %d from expansion due to corridor\n", neighbors[x]);
				neighbors[x] = neighbors.back();
				neighbors.pop_back();
				x--;
			}
		}
	}
	//printf("%d final successors\n", (int)neighbors.size());
}

void GraphRefinementEnvironment::GetActions(const graphState &stateID, std::vector<graphMove> &actions) const
{
	GraphEnvironment::GetActions(stateID, actions);
	assert(false);
}

bool GraphRefinementEnvironment::GoalTest(const graphState &state, const graphState &goal) const
{
	if (useAbstractGoal)
	{
		return (ga->GetNthParent(g->GetNode(state), abstractGoalLevel)->GetNum() == goal);
	}
	return (state == goal);
}

void GraphRefinementEnvironment::SetPlanningCorridor(std::vector<graphState> &corridor, int level, int start)
{
	corridorTable.clear();
	corridorLevel = level;
	for (unsigned int x = start; x < corridor.size(); x++)
		corridorTable[corridor[x]] = true;
}

double GraphRefinementEnvironment::HCost(const graphState &state1, const graphState &state2) const
{
	if (useAbstractGoal)
	{
		return ga->h(ga->GetAbstractGraph(planLevel)->GetNode(state1),
					 ga->GetAbstractGraph(abstractGoalLevel)->GetNode(state2));
	}
	if (h)
		return max(h->HCost(state1, state2), ga->h(ga->GetAbstractGraph(planLevel)->GetNode(state1),
												   ga->GetAbstractGraph(planLevel)->GetNode(state2)));				   
	return ga->h(ga->GetAbstractGraph(planLevel)->GetNode(state1),
				 ga->GetAbstractGraph(planLevel)->GetNode(state2));
}
