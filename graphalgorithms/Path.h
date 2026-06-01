/*
 *  $Id: path.h
 *  hog2
 *
 *  Created by Nathan Sturtevant on 9/28/04.
 *  Modified by Nathan Sturtevant on 02/29/20.
 *
 * This file is part of HOG2. See https://github.com/nathansttt/hog2 for licensing information.
 *
 */

#ifndef PATH_H
#define PATH_H

#include "Graph.h"

/**
 * A linked list of nodes which form a continuous path.
 */
class path {
public:
  node *n;
  path *next;
	
  path(node *_n, path *_next=0) : n(_n), next(_next) {}
  ~path() { if (next != NULL) delete next; }
	path *Clone() { return next?(new path(n, next->Clone())):new path(n, next);}
	path *tail() { if (next) return next->tail(); return this; }
	/** reverses path in place, and returns pointer to new head of path */
	path *reverse();
  /** returns the number of steps along the path */
  unsigned length(void); 
//  /** returns the distance covered by the path */
//  double distance(GraphAbstraction* aMap);
  unsigned degree();
  void Print(bool beginning=true);
};

#endif 
