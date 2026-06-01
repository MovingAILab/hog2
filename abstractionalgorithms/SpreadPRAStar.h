/*
 *  $Id: spreadPRAStar.h
 *  hog2
 *
 *  Created by Nathan Sturtevant on 6/27/05.
 *  Modified by Nathan Sturtevant on 02/29/20.
 *
 * This file is part of HOG2. See https://github.com/nathansttt/hog2 for licensing information.
 *
 */

#include "SearchAlgorithm.h"
#include "Heap.h"
#include "CorridorAStar.h"

#ifndef SPREADPRASTAR_H
#define SPREADPRASTAR_H

#include "SpreadExecSearchAlgorithm.h"

class spreadPRAStar : public spreadExecSearchAlgorithm {
public:
	spreadPRAStar();
	virtual ~spreadPRAStar() {}
	void setPartialPathLimit(int limit)
	{ partialLimit = limit; sprintf(algName, "SpreadPRA*(%d)", partialLimit); }
	
	virtual path *GetPath(GraphAbstraction *aMap, node *from, node *to, reservationProvider *rp = 0);
	void setTargets(GraphAbstraction *, node *, node *, reservationProvider *);
	int getNumThinkSteps(); 
	path *think(); 
private:
	void setupSearch(GraphAbstraction *aMap,
									 std::vector<node *> &fromChain, node *from,
									 std::vector<node *> &toChain, node *to);
	path *buildNextAbstractPath(GraphAbstraction *, path *lastPath,
															std::vector<node *> &fromChain,
															std::vector<node *> &toChain,
															reservationProvider *);
	path *trimPath(path *lastPath, node *origDest);

	char algName[30];
	int partialLimit;
	bool expandSearchRadius;
	path *lastPath;
	std::vector<node *> startChain, endChain;
	corridorAStar cAStar;
};

#endif
