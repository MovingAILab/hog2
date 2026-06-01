/*
 *  $Id: praStarUnit.cpp
 *  hog2
 *
 *  Created by Nathan Sturtevant on 1/16/05.
 *  Modified by Nathan Sturtevant on 02/29/20.
 *
 * This file is part of HOG2. See https://github.com/nathansttt/hog2 for licensing information.
 *
 */

#include "PRAStarUnit.h"

praStarUnit::praStarUnit(int _x, int _y, AbsMapUnit *_target, praStar *_alg)
:SearchUnit(_x, _y, _target, _alg), algorithm(_alg)
{ cache = 0; }

//praStarUnit::praStarUnit(int _x, int _y, int _r, int _g, int _b, unit *_target, praStar *_alg)
//:SearchUnit(_x, _y, _r, _g, _b, _target, _alg), algorithm(_alg)
//{ cache = 0; }


tDirection praStarUnit::makeMove(MapProvider *mp, reservationProvider *rp, SimulationInfo *simInfo)
{
	MapAbstraction *aMap = mp->GetMapAbstraction();

	algorithm->setExpandSearchRadius(false);
	if (moves.size() != 0)
	{
		return SearchUnit::makeMove(mp, rp, simInfo);
	}
	if (cache != 0)
	{
		node *n = aMap->GetNodeFromMap(loc.x, loc.y);
		node *l = aMap->GetNthParent(n, cache->n->GetLabelL(kAbstractionLevel));
		while (cache && (l != cache->n))
		{
			path *tmp = cache;
			cache = cache->next;
			tmp->next = 0;
			delete tmp;
		}
		if (cache)
			assert(l == cache->n);
		if (cache && (cache->next == 0))
		{
			delete cache;
			cache = 0;
		}
	}
	algorithm->setCache(&cache);
	//printf("--- cached PRA --- start ---\n");
	tDirection where =  SearchUnit::makeMove(mp, rp, simInfo);
	//printf("--- cached PRA --- finish ---\n");
	return where;
}
