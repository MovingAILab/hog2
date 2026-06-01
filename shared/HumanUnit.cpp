/*
 *  $Id: humanUnit.cpp
 *  hog2
 *
 *  Created by Nathan Sturtevant on 10/4/04.
 *  Modified by Nathan Sturtevant on 02/29/20.
 *
 * This file is part of HOG2. See https://github.com/nathansttt/hog2 for licensing information.
 *
 */

#include "HumanUnit.h"

humanUnit::humanUnit(int _x, int _y, unit *_target)
:unit(_x, _y, _target)
{
	unitType = kWorldObject;
  r = 0.0; g = 0.0; b = 1.0;
}

tDirection humanUnit::makeMove(MapProvider *, reservationProvider *, SimulationInfo *)
{
  b = 1-b;
  tDirection res = nextDir;
	// uncomment this line if you don't want units to keep moving
	//nextDir = kStay;
  return (tDirection)res;
}
