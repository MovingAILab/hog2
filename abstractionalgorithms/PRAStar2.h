/*
 *  $Id: praStar2.h
 *  hog2
 *
 *  Created by Nathan Sturtevant on 6/23/05.
 *  Modified by Nathan Sturtevant on 02/29/20.
 *
 * This file is part of HOG2. See https://github.com/nathansttt/hog2 for licensing information.
 *
 * Skipping added by Renee Jansen on 5/15/06.
 *
 */

#ifndef PRASTAR2_H
#define PRASTAR2_H

#include "SearchAlgorithm.h"
#include "Heap.h"
#include "CorridorAStar.h"

/**
 * The pra* search algorithm which does partial pathfinding using abstraction.
 */

class praStar2 : public SearchAlgorithm {
	
public:
	praStar2();
	virtual ~praStar2() {}
	virtual path *GetPath(GraphAbstraction *aMap, node *from, node *to, reservationProvider *rp = 0);
	virtual const char *GetName()
	{ return algName; } 
	void setPartialPathLimit(int limit)
	{ partialLimit = limit; sprintf(algName,"PRA2*(%d)", partialLimit); }
	int getPartialPathLimit() { return partialLimit; }
	void setEnhancedAbstractPathing(bool enhance) { enhancedAbstractPathing = enhance; }
  
  /** Set whether we want to expand the corridor to search in. 
   * Set whether we want to expand the corridor from just the parents along the path to these parents and 
   * their neighbors
   */ 
  void setExpandSearchRadius(bool _expandSearchRadius) { expandSearchRadius = _expandSearchRadius; }

  /** Set whether we want to start planning in the middle between base and top level of abstraction.
   * If true, sets the top level for pathfinding to be the level in the middle between the top level and the base 
   * level of abstraction. If false, top level for pathfinding is the topmost abstraction if fixedPlanLevel=-1, and 
   * the fixedPlanLevel otherwise.
   */ 
  void setPlanFromMiddle(bool _planFromMiddle) { planFromMiddle = _planFromMiddle; } 
  
  /** Set the level of abstraction to start pathfinding at.
   * Setting the fixed plan level to -1 will do dynamic level selection if setPlanFromMiddle is true, and it 
   * will start at the highest possible level if setPlanFromMiddle is false. Default is -1.
   */
  void setFixedPlanLevel(int p) { fixedPlanLevel = p; }

  /** Set how many abstraction layers should be skipped at each step.
   * A skip of -1 will only do pathfinding at the top level of abstraction and at the base level. A skip of 0 is 
   * the default, which will do pathfinding at each level.
   */
  void setSkipParameter(int _skip) { skip = _skip; }

  int getTopLevel() { return topLevel; }
  int getNumLevelsUsed() { return numLevels; }

protected:
	void setupSearch(GraphAbstraction *aMap,
			 std::vector<node *> &fromChain, node *from,
			 std::vector<node *> &toChain, node *to);
	path *buildNextAbstractPath(GraphAbstraction *, path *lastPath,
				    std::vector<node *> &fromChain,
				    std::vector<node *> &toChain,
				    reservationProvider *);
	path *trimPath(path *lastPath, node *origDest);

  void selectTopAbstractionLevel(GraphAbstraction * aMap,
				 std::vector<node *> &fromChain,
				 std::vector<node *> & toChain);

	int partialLimit;
	bool enhancedAbstractPathing;
	bool expandSearchRadius;
	corridorAStar cAStar;
  char algName[30];

  bool planFromMiddle;
  int fixedPlanLevel;
  int skip; 
  int topLevel; //the highest level at which an abstract path is found
  int numLevels; // the number of levels at which an abstract path is found
};


#endif
