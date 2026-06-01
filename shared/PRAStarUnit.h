/*
 *  $Id: praStarUnit.h
 *  hog2
 *
 *  Created by Nathan Sturtevant on 1/16/05.
 *  Modified by Nathan Sturtevant on 02/29/20.
 *
 * This file is part of HOG2. See https://github.com/nathansttt/hog2 for licensing information.
 *
 */

#include "PRAStar.h"

#ifndef PRASTARUNIT_H
#define PRASTARUNIT_H

#include "SearchUnit.h"

/**
 * A unit which caches path information to speed pra*
 */

class praStarUnit : public SearchUnit {
public:
	praStarUnit(int _x, int _y, unit *_target, praStar *_alg);
//	praStarUnit(int _x, int _y, int _r, int _g, int _b, unit *_target, praStar *_alg);
	~praStarUnit() { delete cache; }
	const char *GetName() { sprintf(name, "c%s", algorithm->GetName()); return name; }
	tDirection makeMove(MapProvider *, reservationProvider *, SimulationInfo *simInfo); 
private:
	path *cache;
	praStar *algorithm;
char name[32];
};

#endif
