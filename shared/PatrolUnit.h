/*
 *  $Id: patrolUnit.h
 *  hog2
 *
 *  Created by Nathan Sturtevant on 3/23/06.
 *  Modified by Nathan Sturtevant on 02/29/20.
 *
 * This file is part of HOG2. See https://github.com/nathansttt/hog2 for licensing information.
 *
 */

#include "AStar.h"
#include "PRAStar.h"
#include "Unit.h"

#ifndef PATROLUNIT_H
#define PATROLUNIT_H

class patrolUnit : public Unit {
public:
	patrolUnit(int x, int y);
//	patrolUnit(int _x, int _y, int numPLocations, unitSimulation* us);
	virtual const char *GetName() { return "patrolUnit"; }
	virtual tDirection makeMove(MapProvider *, reservationProvider *, SimulationInfo *simInfo);
	void OpenGLDraw(const MapProvider *, const SimulationInfo *) const;
	void addPatrolLocation(unit *);
	unit *GetGoal();
	void updateLocation(int _x, int _y, bool worked, SimulationInfo *)
	{ x = _x; y = _y; if (!worked) { moves.resize(0);	if (currTarget != -1) currTarget = 0; } }
	void LogStats(StatCollection *stats);
		void LogFinalStats(StatCollection *stats);
private:
	double goToLoc(MapAbstraction *aMap, int which);
	void addPathToCache(path *p);
	std::vector<tDirection> moves;
	std::vector<unit *> Locs;
	aStar a;
//	praStar a;
	int currTarget;
	uint64_t nodesExpanded;
	uint64_t nodesTouched;
};

#endif
