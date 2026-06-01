/*
 *  $Id: humanUnit.h
 *  hog2
 *
 *  Created by Nathan Sturtevant on 10/4/04.
 *  Modified by Nathan Sturtevant on 02/29/20.
 *
 * This file is part of HOG2. See https://github.com/nathansttt/hog2 for licensing information.
 *
 */

#ifndef HUMANUNIT_H
#define HUMANUNIT_H

#include "Unit.h"

class humanUnit : public unit {
public:
  humanUnit(int x, int y, unit *target);
	tDirection makeMove(MapProvider *mp, reservationProvider *rp, SimulationInfo *simInfo);
  //tDirection makeMove(MapProvider *, reservationProvider *, SimulationInfo *simInfo); // this is where the World says you are
  //	double getSpeed() { return .25; }
	void setNextDirection(tDirection dir) { nextDir = dir; }
private:
	tDirection nextDir;
};
	
#endif
