/*
 *  $Id: SearchUnit.h
 *  hog2
 *
 *  Created by Nathan Sturtevant on 10/4/04.
 *  Modified by Nathan Sturtevant on 02/29/20.
 *
 * This file is part of HOG2. See https://github.com/nathansttt/hog2 for licensing information.
 *
 */

#include "Unit.h"
#include "Map.h"
#include "SearchAlgorithm.h"
#include "SpreadExecSearchAlgorithm.h"
#include "AbsMapUnit.h"

#ifndef SearchUnit_H
#define SearchUnit_H

/**
* A general unit which collects path information from a SearchAlgorithm and
* incrementally executes that path in the world
*/

class SearchUnit : public AbsMapUnit {
public:
	SearchUnit(int x, int y, AbsMapUnit *target, SearchAlgorithm *alg);
	virtual ~SearchUnit();
	virtual const char *GetName() { if (algorithm) return algorithm->GetName(); return "none"; }
	virtual SearchAlgorithm* getAlgorithm() { return algorithm; }
	//void setUnitSimulation(unitSimulation *_US) { US = _US; algorithm->setSimulationEnvironment(US); }
	virtual bool Done() { return onTarget; }

	void GetGoal(xyLoc &gs) { if (target) target->GetLocation(gs); else GetLocation(gs); }
	//xyLoc GetGoal() { return target->GetLocation(); }
	virtual void setTarget(AbsMapUnit *u) { target = u; }

	//using unit::makeMove;
	// this is where the World says you are  
	virtual bool MakeMove(AbsMapEnvironment *ame, OccupancyInterface<xyLoc,tDirection> *, AbsMapSimulationInfo *si, tDirection &dir)
		{ return makeMove(ame->GetMapAbstraction(), 0, si,dir); }
	virtual bool makeMove(MapProvider *, reservationProvider *, AbsMapSimulationInfo *simInfo, tDirection &dir); 
	
	void UpdateLocation(AbsMapEnvironment *, xyLoc &l, bool success, AbsMapSimulationInfo *si) { updateLocation(l.x, l.y, success, si); }
	virtual void updateLocation(int _x, int _y, bool, AbsMapSimulationInfo *);
	virtual void OpenGLDraw(const AbsMapEnvironment *, const AbsMapSimulationInfo *) const;
	//void printRoundStats(FILE *f);
	void LogStats(StatCollection *stats);
	void LogFinalStats(StatCollection *stats);
protected:
	virtual void addPathToCache(path *p);
	bool getCachedMove(tDirection &dir);
	int nodesExpanded;
	int nodesTouched;
	std::vector<tDirection> moves;
	//	path *p;
	SearchAlgorithm *algorithm;
	spreadExecSearchAlgorithm *s_algorithm;
	path *spread_cache;

	AbsMapUnit *target;

	double targetTime;
	bool onTarget;
};

#endif
