/*
 *  $Id: spreadExecSearchAlgorithm.cpp
 *  hog2
 *
 *  Created by Nathan Sturtevant on 6/27/05.
 *  Modified by Nathan Sturtevant on 02/29/20.
 *
 * This file is part of HOG2. See https://github.com/nathansttt/hog2 for licensing information.
 *
 */

#include "SpreadExecSearchAlgorithm.h"

path *spreadExecSearchAlgorithm::GetPath(GraphAbstraction *_aMap, node *from, node *to, reservationProvider *_rp)
{
	setTargets(_aMap, from, to, _rp);
	path *p;
	while ((p = think()) == 0) {}
	return p;
}

