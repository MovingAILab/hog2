/*
 *  $Id: aStar3.h
 *  hog2
 *
 *  Created by Nathan Sturtevant on 9/29/04.
 *  Modified by Nathan Sturtevant on 02/29/20.
 *
 * This file is part of HOG2. See https://github.com/nathansttt/hog2 for licensing information.
 *
 */

#ifndef ASTAROld_H
#define ASTAROld_H

#include "SearchAlgorithm.h"
#include "Heap.h"

// this is a "classic" implementation of A*
// it is not particularly optimized, it is more of an example of how an
// algorithm can be coded in this framework

/**
* A sample A* implementation.
 */

class aStarOld : public SearchAlgorithm {
	
public:
	aStarOld(double _w = 1.0, bool _doPathDraw = true);
	path *GetPath(GraphAbstraction *aMap, node *from, node *to, reservationProvider *rp = 0);
	virtual const char *GetName() { return aStarName; }
	void drawPath(bool _doPathDraw) { doPathDraw = _doPathDraw; }
	
private:
		void relaxEdge(Heap *nodeHeap, Graph *g, edge *e, int source, int nextNode, node *to);
	path *extractBestPath(Graph *g, unsigned int current);
	GraphAbstraction *map;
	double wh;
	char aStarName[128];
	bool doPathDraw;
};

#endif
