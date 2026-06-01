/*
 *  RHRUnit.h
 *  hog2
 *
 *  Created by Nathan Sturtevant on 4/23/07.
 *  Copyright 2007 Nathan Sturtevant, University of Alberta. All rights reserved.
 *
 */

#include "MapUnit.h"
#include "Map2DEnvironment.h"

#ifndef RHRUNIT_H
#define RHRUNIT_H

/**
 * A simple unit that attempts to follow walls using the right hand rule.
 */
class RHRUnit : public MapUnit {
public:
	RHRUnit(int x, int y)
	:MapUnit(x, y)
	{ lastIndex = 0; initial = true; }
	const char *GetName() { return "RHRUnit"; }
	bool MakeMove(MapEnvironment *, OccupancyInterface<xyLoc, tDirection> *, MapSimulationInfo *, tDirection &);
	void UpdateLocation(MapEnvironment *, xyLoc &, bool, MapSimulationInfo *);
private:
	int lastIndex;
	bool initial;
};

#endif
