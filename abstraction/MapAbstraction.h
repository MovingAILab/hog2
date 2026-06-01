/*
 *  $Id: MapAbstraction.h
 *  hog2
 *
 *  Created by Nathan Sturtevant on 6/3/05.
 *  Modified by Nathan Sturtevant on 02/29/20.
 *
 * This file is part of HOG2. See https://github.com/nathansttt/hog2 for licensing information.
 *
 */

#include "Map.h"
#include "GraphAbstraction.h"
#include "GLUtil.h"
#include "MapProvider.h"
#include <stdlib.h>

#ifndef MAPABSTRACTION_H
#define MAPABSTRACTION_H

/**
* This class is designed as an interface to be added onto any type of GraphAbstraction
 * to support a few extra functionalities that mapabstractions should have.
 */

class MapAbstraction : public GraphAbstraction, public MapProvider {
public:
	MapAbstraction(Map *_m) :m(_m), levelDraw(0) {}
	virtual ~MapAbstraction();
	/** return a new abstraction map of the same type as this map abstraction */
	virtual MapAbstraction *Clone(Map *) = 0;
	node *GetNodeFromMap(int x, int y, tCorner c = kNone) { return abstractions[0]->GetNode(m->GetNodeNum(x, y, c)); }
	void GetTileFromNode(node *n, int &x, int &y);
	void GetRandomTileFromNode(node *n, int &x, int &y);

	/** Given a location (recVec) return the tile under that location */
	void GetTileUnderLoc(int &x, int &y, const recVec &);

	Map* GetMap() const { return m; }
	MapAbstraction *GetMapAbstraction() { return this; }

	virtual double h(node *a, node *b);
	
	double OctileDistance(double,double,double,double);
	
	// display functions
	virtual void OpenGLDraw() const;
	void ToggleDrawAbstraction(int which);
	void ClearMarkedNodes();
	recVec GetNodeLoc(node *n) const;
private:
		
	void DrawLevelConnections(node *n) const;
	void DrawGraph(Graph *g, bool levels = true) const;
	
	Map *m;
	unsigned long levelDraw;
};

Graph *GetMapGraph(Map *m);
void AddMapEdges(Map *m, Graph *g, int x, int y);

#endif
