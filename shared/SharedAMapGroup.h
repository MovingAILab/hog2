/*
 *  $Id: SharedAMapGroup.h
 *  hog2
 *
 *  Created by Nathan Sturtevant on 12/16/04.
 *  Modified by Nathan Sturtevant on 02/29/20.
 *
 * This file is part of HOG2. See https://github.com/nathansttt/hog2 for licensing information.
 *
 */



#ifndef SHAREDAMAPGROUP_H
#define SHAREDAMAPGROUP_H

#include "AbsMapUnit.h"
#include "UnitGroup.h"
#include "AbsMapUnitGroup.h"
#include "MapProvider.h"

/**
 * A group which incrementally builds a map of the world as the units in the group
 * explore the Graph.
 */
class SharedAMapGroup : public AbsMapUnitGroup, public MapProvider {
public:
	SharedAMapGroup(MapProvider *);
	~SharedAMapGroup();
	virtual const char *GetName() { return "SharedAMapGroup"; }
	//virtual tDirection makeMove(unit *u, MapProvider *mp, reservationProvider *rp, AbsMapSimulationInfo *simInfo);
	virtual void OpenGLDraw(const AbsMapEnvironment *, const AbsMapSimulationInfo *) const;
	virtual Map *GetMap() const;
	virtual MapAbstraction *GetMapAbstraction();
	virtual int GetNewTileCount() { return newTileCountPerTrial; }
	
	/** reset the location of a given unit */
	void UpdateLocation(Unit<xyLoc, tDirection, AbsMapEnvironment> *u, AbsMapEnvironment *, xyLoc &loc, bool success, AbsMapSimulationInfo *);
	//virtual void updateLocation(BaseAbsMapUnit *, MapProvider *m, int _x, int _y, bool, AbsMapSimulationInfo *);
	/** Is the group done with their exploration? */
	virtual bool Done();
	/** Lets the unit group do what it needs to reset a trial */
	void StartNewTrial(StatCollection *stats);
	void LogStats(StatCollection *stats);
	
	void SetVisibilityRadius(int _visibility);
	int GetVisibilityRadius();
	bool Explored(int x, int y);
	bool Explored(unsigned int _node);
	//void printRoundStats(unit *, FILE *f);

	int GetNewTileCountPerStep() { return newTileCount; }
	bool SeenBefore(int x, int y) { return (seen->Get(y * map->GetMapWidth() + x)); }
	
protected:
	//void setUnitSimulation(unitSimulation *_us, Map *m);

	MapAbstraction *aMap;
	Map *map;
	BitVector *seen;
	int visRadius;
	bool sawNewLand;
	int newTileCount;
	int newTileCountPerTrial;
};

#endif
