/*
 *  $Id: aStar2.h
 *  hog2
 *
 *  Created by Nathan Sturtevant on 9/29/04.
 *  Modified by Nathan Sturtevant on 02/29/20.
 *
 * This file is part of HOG2. See https://github.com/nathansttt/hog2 for licensing information.
 *
 */

#ifndef ASTAR2_H
#define ASTAR2_H

#include "PRAStar.h"

/**
 * A implementation of A* which just re-uses code from PRA*.
 */

class aStar2 : public praStar {
public:
  aStar2() : praStar() {}
  path *GetPath(GraphAbstraction *aMap, node *from, node *to, reservationProvider *rp = 0);
};

#endif
