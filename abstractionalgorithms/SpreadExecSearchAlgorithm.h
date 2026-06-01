/*
 *  $Id: spreadExecSearchAlgorithm.h
 *  hog2
 *
 *  Created by Nathan Sturtevant on 6/27/05.
 *  Modified by Nathan Sturtevant on 02/29/20.
 *
 * This file is part of HOG2. See https://github.com/nathansttt/hog2 for licensing information.
 *
 */

#include "SearchAlgorithm.h"

#ifndef SPREADEXECSEARCHALGORITHM_H
#define SPREADEXECSEARCHALGORITHM_H

class spreadExecSearchAlgorithm : public SearchAlgorithm {
public:
	spreadExecSearchAlgorithm() { }
	virtual const char *GetName() { return "unnamed"; }
	virtual path *GetPath(GraphAbstraction *aMap, node *from, node *to, reservationProvider *rp = 0);

	virtual void setTargets(GraphAbstraction *_aMap, node *s, node *e, reservationProvider *_rp = 0)
	{ rp = _rp; start = s; end = e; aMap = _aMap; }
	/** how many times do we have to "think" to find the solution, return -1 if unknown */
	virtual int getNumThinkSteps() = 0; 
	/** do next processing for path, returns avaliability of path moves */
	virtual path *think() = 0; 
protected:
	node *start, *end;
	reservationProvider *rp;
	GraphAbstraction *aMap;
};

#endif
