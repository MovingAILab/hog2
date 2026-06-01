/*
 *  $Id: craStar.h
 *  hog2
 *
 *  Created by Nathan Sturtevant on 6/23/05.
 *  Modified by Nathan Sturtevant on 02/29/20.
 *
 * This file is part of HOG2. See https://github.com/nathansttt/hog2 for licensing information.
 *
 */

#ifndef CRASTAR_H
#define CRASTAR_H

#include "SearchAlgorithm.h"
#include "Heap.h"
#include "CorridorAStar.h"

/**
* The pra* search algorithm which does partial pathfinding using abstraction.
 */

class craStar : public SearchAlgorithm {
	
public:
	// From HPA* original code - smoothwizard.h
	typedef enum {
		NORTH,
		EAST,
		SOUTH,
		WEST,
		NE,
		SE,
		SW,
		NW} Direction;

	typedef enum {
		BEGIN,
		END,
		TWO_BACK} SmoothType;


	craStar();
	virtual ~craStar() {}
	virtual path *GetPath(GraphAbstraction *aMap, node *from, node *to, reservationProvider *rp = 0);
	virtual const char *GetName()
	{ return algName; } 
	void setPartialPathLimit(int limit)
	{ partialLimit = limit; sprintf(algName,"CRA*(%d)", partialLimit); }
	int getPartialPathLimit() { return partialLimit; }

	/**
	 * Set whether the path will be smoothed or not. Default is TRUE. 
	 */ 
	void setSmoothing(bool smooth) {smoothing = smooth;}

	/**
	 * Set the smoothing type.Default is BEGIN
	 *
	 * Whenever we update the path with a better subpath, 
	 * if s = BEGIN, smoothing is restarted from the beginning of this subpath
	 * if s = END, smoothing is restarted from the end of the new subpath
	 * if s = TWO_BACK, we go back two from the end of the new subpath
	 */
	void setSmoothType(SmoothType s) { smType = s; }

	/**
	 * set the abstract level 
	 *
	 * if the level is smaller than 1, the abstract level is chosen
	 * dynamically.
	 */
	void setAbstractLevel(int level) { absLevel = level; }

protected:
	void setupSearch(GraphAbstraction *aMap,
			 std::vector<node *> &fromChain, node *from,
			 std::vector<node *> &toChain, node *to);

	void findGoalNode(GraphAbstraction* aMap,node* n, std::vector<node *> &toChain);
	path *buildNextAbstractPath(GraphAbstraction *, path *lastPath,
				    std::vector<node *> &fromChain,
				    std::vector<node *> &toChain,
				    reservationProvider *);


	path *trimPath(path *lastPath, node *origDest);
	path* buildAbstractPath(GraphAbstraction *aMap, 
													std::vector<node *> &fromChain,
													std::vector<node *> &toChain,
													reservationProvider *rp);
	path* doRefinement(GraphAbstraction *aMap, path* absPath, 
										 std::vector<node*> &fromChain, 
										 std::vector<node*> &toChain);

	node* getNextNode(GraphAbstraction *aMap, node* currentLow, path* returnPath, path* apath, Graph* g, int abstractLevel);

	int partialLimit;
	bool expandSearchRadius;
	corridorAStar cAStar;
  char algName[30];

	path* smoothPath(GraphAbstraction* m,path* p);

	std::vector<node*> lookup;
	path* nextPathNode(GraphAbstraction* m,node* n, int dir);
	node* getNextNode(MapAbstraction* m,int x, int y, int dir);
	bool nextInLookup(int last, int curr, std::vector<node*> lookup);
	int backTwoNodes(int i, std::vector<node*> lookup);

	void findMinMax(path* p);

	SmoothType smType;
	bool smoothing;
	int absLevel;

	int minx;
	int maxx;
	int miny;
	int maxy;
};


#endif
