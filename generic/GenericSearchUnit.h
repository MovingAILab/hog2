/** 
* @file GenericSearchUnit.h
* @package hog2
* @brief A generic search unit which uses templates
* @author Nathan Sturtevant
* @author Modified from SearchUnit.h by Renee Jansen
*
* This file is part of HOG2.
* HOG : http://www.cs.ualberta.ca/~nathanst/hog.html
* HOG2: http://code.google.com/p/hog2/
*
* HOG2 is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation; either version 2 of the License, or
* (at your option) any later version.
* 
* HOG2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
* 
* You should have received a copy of the GNU General Public License
* along with HOG2; if not, write to the Free Software
* Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include <vector>

#include "Unit.h"
#include "Map.h"
#include "GenericSearchAlgorithm.h"
#include "SpreadExecSearchAlgorithm.h"
#include "AbsMapUnit.h"
//#include "ReservationProvider.h" // occupancy interface - change later
//#include "SearchEnvironment.h"

#ifndef GENERICSEARCHUNIT_H
#define GENERICSEARCHUNIT_H

/**
* A general unit which collects path information from a GenericSearchAlgorithm and
* incrementally executes that path in the world
*/

template <class state, class action, class environment> 
class GenericSearchUnit : public Unit<state,action,environment> {
public:
	GenericSearchUnit(state &start, state &goal, GenericSearchAlgorithm<state,action,environment> *alg);
	GenericSearchUnit(state &start, Unit<state,action,environment> *target, GenericSearchAlgorithm<state,action,environment> *alg);
	virtual ~GenericSearchUnit();
	virtual const char *GetName() { if (algorithm) return algorithm->GetName(); return "None"; }
	virtual GenericSearchAlgorithm<state,action,environment>* getAlgorithm() { return algorithm; }

	virtual bool Done() {return (loc == goal);}
	virtual void GetGoal(state &s) { s=goal;} 
	
	virtual void SetTarget(Unit<state,action,environment> *u) { target = u; }
	virtual Unit<state,action,environment>* GetTarget() { return target; }

	virtual bool MakeMove(environment *env, OccupancyInterface<state,action> *oi, SimulationInfo<state,action,environment> *si, action& a);
	
	virtual void UpdateLocation(environment *env, state &l, bool success, SimulationInfo<state,action,environment> *si);
	
	virtual void GetLocation(state& s) {s=loc;}
//	virtual void OpenGLDraw(const environment *, const SimulationInfo<state,action,environment> *) const;
	//void printRoundStats(FILE *f);
	void LogStats(StatCollection *stats);
	void LogFinalStats(StatCollection *stats);
	
// 	void SetColor(GLfloat _r, GLfloat _g, GLfloat _b) { r=_r; g=_g; b=_b; }
// 	void GetColor(GLfloat& _r, GLfloat& _g, GLfloat& _b) { _r=r; _g=g; _b=b; }
protected:
	virtual void AddPathToCache(environment *env, std::vector<state> &path);
	bool getCachedMove(action &a);
	int nodesExpanded;
	int nodesTouched;
	std::vector<action> moves; // states? actions? what? 
	//	path *p;
	GenericSearchAlgorithm<state,action,environment> *algorithm;

	Unit<state,action,environment> *target;
	//GLfloat r, g, b;
	state loc, goal, lastloc;
	double targetTime, lastTime;
	bool onTarget;
};


template <class state, class action, class environment>
GenericSearchUnit<state,action,environment>::GenericSearchUnit(state &start, state &targ, GenericSearchAlgorithm<state,action,environment> *alg)
{

	loc = start;
	goal = targ;
	algorithm = alg;
	nodesExpanded = 0;
	nodesTouched = 0;
	
	targetTime = 0;
	
	target=0;
	
	GLfloat _r,_g,_b;
	_r = (double)rand() / RAND_MAX;
	_g = (double)rand() / RAND_MAX;
	_b = (double)rand() / RAND_MAX;
	this->SetColor(_r,_g,_b);

}

template <class state, class action, class environment>
GenericSearchUnit<state,action,environment>::GenericSearchUnit(state &start, Unit<state,action,environment> *targ, GenericSearchAlgorithm<state,action,environment> *alg)
{	
	target = targ; 
	targ->getLocation(goal);
	
	loc = start;
	algorithm = alg;
	nodesExpanded = 0;
	nodesTouched = 0; 
	
	targetTime = 0; 

}

template <class state, class action, class environment>
GenericSearchUnit<state,action,environment>::~GenericSearchUnit()
{
}

template <class state, class action, class environment>
bool GenericSearchUnit<state,action,environment>::MakeMove(environment *theEnv, OccupancyInterface<state,action> *, SimulationInfo<state,action,environment> *si, action& a)
{
	if (getCachedMove(a))
		return true;
	
	// Check if we have a target defined - i.e. if we're following another unit
	if (target)
	{
		// check if target has moved from goal state - update goal if necessary
		state targpos;
		target->GetLocation(targpos);
		if (~(targpos == goal))
			goal = targpos;
	}
	
	// find a path to the goal 
	
	state from = loc;
	state to = goal; 
	
	if (from == to)
	{
		if (!onTarget)
		{
//			if (verbose)
//			{
//				printf("STAY ON TARGET!\n");
//				printf("%p target time %1.4f\n", (void*)this, targetTime);
//			}
			if (si)
				targetTime = si->GetSimulationTime();
		}
		onTarget = true;
		// return kStay ??? 
	}
	else
		onTarget = false;

	std::vector<state> path;

	algorithm->GetPath(theEnv,from, to, path); 
	
	nodesExpanded+=algorithm->GetNodesExpanded();
	nodesTouched+=algorithm->GetNodesTouched();

	// returning an empty path means there is no path between the start and goal
	if (path.size()==0)
	{
//		if (verbose)
//			printf("SU %s: Path returned NIL\n", this->GetName());
		return false;
		//return kStay **************
	}
		

	// a valid path must have at least 2 nodes and start where the unit is located
	assert((path.size() > 1) && (path[0]==loc));
	
	AddPathToCache(theEnv, path);

	assert(moves.size() > 0);

	//a = moves.back();
	//moves.pop_back();	
	a = moves.front();
	moves.erase(moves.begin());
	return true;
}

//template <class state, class action, class environment>
//void GenericSearchUnit<state,action,environment>::OpenGLDraw(const environment *theEnv, const SimulationInfo<state,action,environment> *si) const
//{
//	// Draw current + goal states as states. May need to find something
//	// different for the goal
//	//printf("Drawing %p at \n", this);
//	{
//		PublicUnitInfo<state, action, environment> i;
//		si->GetPublicUnitInfo(si->GetCurrentUnit(), i);
//		
//		GLfloat _r,_g,_b;
//		this->GetColor(_r,_g,_b);
//		theEnv->SetColor(_r, _g, _b);
//
//		if (fgreater(si->GetSimulationTime(), i.nextTime))
//			theEnv->OpenGLDraw(i.currentState);
//		else
//			theEnv->OpenGLDraw(i.lastState, i.currentState,
//							   (si->GetSimulationTime()-i.lastTime)/(i.nextTime-i.lastTime));
//		theEnv->SetColor(1.0, 0.25, 0.25, 0.25);
//
//		//algorithm->OpenGLDraw();
//
//		theEnv->SetColor(_r, _g, _b, 0.5);
//		theEnv->OpenGLDraw(goal);
//	}
//
//	state current = loc;
//	state next;
//	
//	// Draw the cached moves
//  	for (unsigned int i=0; i<moves.size(); i++)
// 	{
//		theEnv->SetColor(1.0, 0.0, 0.0, 0.5);
// 		theEnv->OpenGLDraw(current);
// 		//theEnv->OpenGLDraw(current, moves[i]/*,1.0,0,0*/); // draw in red
// 		//theEnv->GetNextState(current, moves[i], next);
//		theEnv->ApplyAction(current, moves[i]);
// 	}
// 	
//}

template <class state, class action, class environment>
void GenericSearchUnit<state,action,environment>::LogStats(StatCollection *stats)
{
	if (((nodesExpanded == 0) && (nodesTouched != 0)) ||
			((nodesExpanded != 0) && (nodesTouched == 0)))
	{
		printf("Error; somehow nodes touched/expanded are inconsistent. t:%d e:%d\n",
					 nodesTouched, nodesExpanded);
	}

	if (nodesExpanded != 0)
		stats->AddStat("nodesExpanded", GetName(), (long)nodesExpanded);
	if (nodesTouched != 0)
		stats->AddStat("nodesTouched", GetName(), (long)nodesTouched);
	nodesExpanded = nodesTouched = 0;
}

template <class state, class action, class environment>
void GenericSearchUnit<state,action,environment>::LogFinalStats(StatCollection *stats)
{
	algorithm->LogFinalStats(stats);
}

/**
* Store path as a vector of actions. 
* ??? Rename moves --> actions? Moves only for maps?
*/
template<class state, class action, class environment>
void GenericSearchUnit<state,action,environment>::AddPathToCache(environment *theEnv, std::vector<state> &path)
{
 	for (unsigned int i=0; i<path.size()-1; i++)
 	{
 		moves.push_back(theEnv->GetAction(path[i], path[i+1]));
 	}
}

template<class state, class action, class environment>
bool GenericSearchUnit<state,action,environment>::getCachedMove(action &a)
{
	if (moves.size() > 0)
	{
		a = moves.front();
		moves.erase(moves.begin());
		//a = moves.back();
		//moves.pop_back();
		return true;
	}
	return false;
}

template<class state, class action, class environment>
void GenericSearchUnit<state,action,environment>::UpdateLocation(environment *, state &l, bool success, SimulationInfo<state,action,environment> *si)
{
	// Done in UnitSimulation instead
	//Update occupancy interface
	//env->GetOccupancyInterface()->SetStateOccupied(loc,false);
	//env->GetOccupancyInterface()->SetStateOccupied(l, true);
	
	if ((!success) || (l == loc))
	{
		moves.resize(0);
	}
	//lastLoc = loc;
	lastTime = si->GetSimulationTime();
	loc = l;
}

#endif
