/*
 *  $Id: praStar.h
 *  hog2
 *
 *  Created by Nathan Sturtevant on 9/28/04.
 *  Modified by Nathan Sturtevant on 02/29/20.
 *
 * This file is part of HOG2. See https://github.com/nathansttt/hog2 for licensing information.
 *
 */

#ifndef PRASTAR_H
#define PRASTAR_H

#include <iostream>
#include "SearchAlgorithm.h"
#include "Heap.h"

/**
 * The pra* search algorithm which does partial pathfinding using abstraction.
 */

class praStar : public SearchAlgorithm {

public:
	praStar();
	virtual ~praStar() {}
	virtual path *GetPath(GraphAbstraction *aMap, node *from, node *to, reservationProvider *rp = 0);
	virtual const char *GetName() { return algName; } 
	void setPartialPathLimit(int limit) { partialLimit = limit; 
		sprintf(algName,"PRA*(%d)",partialLimit); }
	int getPartialPathLimit() { return partialLimit; }
	void setPlanFromMiddle(bool _planFromMiddle) { planFromMiddle = _planFromMiddle; }
	void setExpandSearchRadius(bool _expandSearchRadius) { expandSearchRadius = _expandSearchRadius; }
	void setUseSmoothing(bool _smoothing) { smoothing = _smoothing; }
	void setCache(path **p);
	/** Set a fixed level for abstraction in planning. -1 to return to dynamic level selection */
	void setFixedPlanLevel(int p) { fixedPlanLevel = p; }
	void getAbstractPathLengths(std::vector<int> &len) { len = lengths; }
protected:

  path *getAbstractPath(Graph *g, unsigned int source, unsigned int destParent,
			std::vector<unsigned int> &eligibleNodeParents, int LABEL,
			unsigned int dest);
  
  unsigned int astar(Graph *g, unsigned int source, unsigned int destParent,
		     std::vector<unsigned int> &eligibleNodeParents, int LABEL,
		     unsigned int dest);
  
  void relaxEdge(Heap *nodeHeap, Graph *g, edge *e, int source, int nextNode, int dest,
		 int LABEL);
	path *smoothPath(path *p);
	
	path **cache;
  int partialLimit;
	int fixedPlanLevel;
  char algName[30];
  GraphAbstraction *map;
	bool expandSearchRadius;
	bool planFromMiddle;
	bool smoothing;
	reservationProvider *rp;
	std::vector<int> lengths;
};


#endif
