
#include "Unit.h"
#include "Map.h"
#include "SearchAlgorithm.h"
#include "SpreadExecSearchAlgorithm.h"
#include "AbsMapUnit.h"

#ifndef MySearchUnit_H
#define MySearchUnit_H

/**
* A general unit which collects path information from a SearchAlgorithm and
* incrementally executes that path in the world
*/

class MySearchUnit : public AbsMapUnit {
public:
	MySearchUnit( int x, int y, int targetUnit, SearchAlgorithm *alg);
	virtual ~MySearchUnit();
	virtual const char *GetName() { if (algorithm) return algorithm->GetName(); return "none"; }
	virtual SearchAlgorithm* getAlgorithm() { return algorithm; }
	//void setUnitSimulation(unitSimulation *_US) { US = _US; algorithm->setSimulationEnvironment(US); }
	virtual bool Done() { return onTarget; }

	void GetGoal(xyLoc &gs) { GetLocation(gs); }
	virtual void setTarget( int _targetUnit ) { targetUnit = _targetUnit; }

	//using unit::makeMove;
	// this is where the World says you are  
	virtual bool MakeMove(AbsMapEnvironment *ame, OccupancyInterface<xyLoc,tDirection> *, AbsMapSimulationInfo *si, tDirection &dir)
		{ return makeMove(ame->GetMapAbstraction(), 0, si,dir); }
	virtual bool makeMove(MapProvider *, reservationProvider *, AbsMapSimulationInfo *simInfo, tDirection &dir); 
	
	void UpdateLocation(AbsMapEnvironment *, xyLoc &l, bool success, AbsMapSimulationInfo *si) { updateLocation(l.x, l.y, success, si); }
	virtual void updateLocation(int _x, int _y, bool, AbsMapSimulationInfo *);
	virtual void OpenGLDraw(int window, AbsMapEnvironment *, AbsMapSimulationInfo *);
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

	int targetUnit;

	double targetTime;
	bool onTarget;
};

#endif
