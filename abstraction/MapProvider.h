/*
 *  $Id: MapProvider.h
 *  hog2
 *
 *  Created by Nathan Sturtevant on 5/10/05.
 *  Modified by Nathan Sturtevant on 02/29/20.
 *
 * This file is part of HOG2. See https://github.com/nathansttt/hog2 for licensing information.
 *
 */

#ifndef MAPPROVIDERINTERFACE_H
#define MAPPROVIDERINTERFACE_H

/**
 * An interface for any class that can provide a map & abstract map, as well as a
 * heuristic for that map.
 */

class Map;
class MapAbstraction;

class MapProvider {
public:
	virtual ~MapProvider() {};
	virtual Map *GetMap() const = 0;
	virtual MapAbstraction *GetMapAbstraction() = 0;
};

#include "Map.h"
#include "MapAbstraction.h"

#endif
