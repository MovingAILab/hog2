/*
 *  LocalSensingUnit.h
 *  hog2
 *
 *  Created by Nathan Sturtevant on 6/12/09.
 *  Copyright 2009 NS Software. All rights reserved.
 *
 */

#include "Unit.h"
#include <unordered_map>

template <class state, class action>
struct stateInfo {
	stateInfo() { gCost = -1; exploreCost = -1; }
	
	double gCost;
	double fCost;
	double exploreCost;
	action fromParent;
	state theState;
	int currentNeighbor;
};

template <class state, class action, class environment>
class LocalSensingUnit : public Unit<state, action, environment> {
public:
	LocalSensingUnit(state &loc, state &goal);

	const char *GetName() { return "LocalSensingUnit"; }

	bool MakeMove(environment *, OccupancyInterface<state,action> *, SimulationInfo<state,action,environment> *, action& a);

	void OpenGLDraw(const environment *e, const SimulationInfo<state,action,environment> *) const;

	void UpdateLocation(environment *, state &s, bool success, SimulationInfo<state,action,environment> *)
	{
		if (success) 
		{
			std::cout << "Updating location from " << currentLoc << " to " << s << std::endl;
			currentLoc = s;
		}
		else
			std::cout << "Move failed, not updating location" << std::endl;
	}
	void GetGoal(state &s) { s = goalLoc; }
	void GetLocation(state &s) { s = currentLoc; }
	bool Done() { return false; return (currentLoc == goalLoc);} 
	
	/** log an stats that may have been computed during the last run */
	void LogStats(StatCollection *) {}
	/** log any final one-time stats before a simulation is ended */
	void LogFinalStats(StatCollection *) {}
private:
	
	typedef std::unordered_map<uint64_t, stateInfo<state, action>, Hash64 > LSStateStorage;
	LSStateStorage hashTable;
	
	bool init;
	state currentLoc, startLoc, goalLoc;
	action lastAction;
	double currentIterationCost, currentPathCost;
	double nextIterationCost;
};


template <class state, class action, class environment>
LocalSensingUnit<state, action, environment>::LocalSensingUnit(state &loc, state &goal)
:currentLoc(loc), startLoc(loc), goalLoc(goal)
{
	init = false;
}

template <class state, class action, class environment>
bool LocalSensingUnit<state, action, environment>::MakeMove(environment *e, OccupancyInterface<state,action> *,
															SimulationInfo<state,action,environment> *, action& a)
{
	if (!init)
	{
		currentPathCost = 0.0;
		nextIterationCost = DBL_MAX;
		
		stateInfo<state, action> &si = hashTable[e->GetStateHash(currentLoc)];
		
		si.exploreCost = 0;
		si.gCost = 0;
		si.fCost = e->HCost(currentLoc, goalLoc);
		si.currentNeighbor = 0;
		si.theState = currentLoc;
		currentIterationCost = si.fCost;
		init = true;
	}		
	
	std::cout << "\nInside LocalSensingUnit::MakeMove; current cost " << currentPathCost << " limit " << currentIterationCost << std::endl;
	std::vector<state> neighbors;
	e->GetSuccessors(currentLoc, neighbors);
	
	stateInfo<state, action> &si = hashTable[e->GetStateHash(currentLoc)];
	
	// haven't been here before
	if (si.gCost == -1)
	{
		std::cout << "Exploring " << currentLoc << " for the first time" << std::endl;
		si.exploreCost = currentPathCost;
		si.gCost = currentPathCost;
		si.fCost = currentPathCost + e->HCost(currentLoc, goalLoc);
		si.fromParent = lastAction;
		si.currentNeighbor = 0;
		si.theState = currentLoc;
		
		if (fgreater(si.fCost, currentIterationCost)) // beyond boundary
		{
			std::cout << ": Cost " << currentPathCost << " is above limit " << currentIterationCost << std::endl;
			if (fless(currentPathCost, nextIterationCost))
				nextIterationCost = currentPathCost + e->HCost(currentLoc, goalLoc);
			assert(e->InvertAction(lastAction)); // actions must be invertable
			a = lastAction;
			currentPathCost -= e->GCost(currentLoc, a);
			return true;
		}
		else if (neighbors.size() == 0)
		{
			std::cout << ": no neighbors" << currentIterationCost << std::endl;
			assert(e->InvertAction(lastAction)); // actions must be invertable
			a = lastAction;
			currentPathCost -= e->GCost(currentLoc, a);
			return true;
		}
		else { // take first action
			std::cout << ": taking action " << 0 << " of " << neighbors.size() << std::endl;
			lastAction = a = e->GetAction(currentLoc, neighbors[si.currentNeighbor++]);
			currentPathCost += e->GCost(currentLoc, a);
			return true;
		}
	}
	else { // have been here; take next action
		std::cout << "Exploring " << currentLoc << " NOT for the first time" << std::endl;
		if (fless(currentPathCost, si.gCost))
		{
			std::cout << ": Reducing g-cost of " << currentLoc << " to " << currentPathCost << " from " << si.gCost << std::endl;
			si.gCost = currentPathCost;
			si.fCost = currentPathCost + e->HCost(currentLoc, goalLoc);
			si.fromParent = lastAction;
		}
		if (fgreater(currentPathCost, si.gCost))
		{
			std::cout << ": On suboptimal path to " << currentPathCost << std::endl;
			assert(e->InvertAction(lastAction)); // actions must be invertable
			a = lastAction;
			currentPathCost -= e->GCost(currentLoc, a);
			return true;
		}
		if (fgreater(currentIterationCost, si.exploreCost))
		{
			std::cout << ": exploring on a new iteration " << si.exploreCost << " -> " << currentIterationCost << std::endl;
			si.exploreCost = currentIterationCost;
			si.currentNeighbor = 0;
		}
		if (fgreater(si.fCost, currentIterationCost)) // beyond boundary
		{
			std::cout << ": Cost " << currentPathCost << " is above limit " << currentIterationCost << std::endl;
			if (fless(currentPathCost, nextIterationCost))
				nextIterationCost = currentPathCost + e->HCost(currentLoc, goalLoc);
			assert(e->InvertAction(lastAction)); // actions must be invertable
			a = lastAction;
			currentPathCost -= e->GCost(currentLoc, a);
			return true;
		}
		
		if (si.currentNeighbor >= neighbors.size()) // no more neighbors
		{
			std::cout << ": No more unexplored neighbors." << std::endl;
			if (si.gCost == 0) // at root; go to next iteration - no action now
			{
				std::cout << ": Updating next iteration limit to " << nextIterationCost << std::endl;
				currentIterationCost = nextIterationCost;
				nextIterationCost = DBL_MAX;
				return false;
			}
			// go back to parent
			std::cout << "Finished -- inverting action to get here: " << si.fromParent;
			lastAction = si.fromParent;
			assert(e->InvertAction(lastAction)); // actions must be invertable
			a = lastAction;
			currentPathCost -= e->GCost(currentLoc, a);
			std::cout << " and taking action " << a << " to get back " << std::endl;
			return true;
		}
//		else if (!fequal(si.gCost, currentPathCost)) // got here with too high a cost
//		{
//			std::cout << ": Cost " << currentPathCost << " is above best path here " << si.gCost << std::endl;
//			assert(e->InvertAction(lastAction)); // actions must be invertable
//			a = lastAction;
//			currentPathCost -= e->GCost(currentLoc, a);
//			return true;
//		}
		else {
			std::cout << ": taking action " << si.currentNeighbor << " of " << neighbors.size() << std::endl;
			lastAction = a = e->GetAction(currentLoc, neighbors[si.currentNeighbor++]);
			currentPathCost += e->GCost(currentLoc, a);
			return true;
		}
	}
	std::cout << " No action taken\n ";
}

template <class state, class action, class environment>
void LocalSensingUnit<state, action, environment>::OpenGLDraw(const environment *e, const SimulationInfo<state,action,environment> *si) const
{
	PublicUnitInfo<state, action, environment> i;
	si->GetPublicUnitInfo(si->GetCurrentUnit(), i);
	e->SetColor(1.0, 0.0, 0.0, 1.0);
	if (fgreater(si->GetSimulationTime(), i.nextTime))
		e->OpenGLDraw(i.currentState);
	else
		e->OpenGLDraw(i.lastState, i.currentState,
						(si->GetSimulationTime()-i.lastTime)/(i.nextTime-i.lastTime));
	//e->OpenGLDraw(currentLoc);

	e->SetColor(0.0, 1.0, 0.0, 1.0);
	e->OpenGLDraw(goalLoc);

	e->SetColor(0.0, 0.0, 0.5, 0.25);
	for (typename LSStateStorage::const_iterator it = hashTable.begin(); it != hashTable.end(); it++)
	{
		if ((*it).second.theState == currentLoc)
		{
		}
		else {
			e->OpenGLDraw((*it).second.theState);
		}
	}
}
