/*
 *  $Id: RadiusAbstraction.h
 *  hog2
 *
 *  Created by Nathan Sturtevant on 6/3/05.
 *  Modified by Nathan Sturtevant on 02/29/20.
 *
 * This file is part of HOG2. See https://github.com/nathansttt/hog2 for licensing information.
 *
 */

#include "MapAbstraction.h"

#ifndef RADIUSABSTRACTION_H
#define RADIUSABSTRACTION_H

class RadiusAbstraction : public MapAbstraction {
public:
	RadiusAbstraction(Map *, int);
	~RadiusAbstraction();
	MapAbstraction *Clone(Map *_m) { return new RadiusAbstraction(_m, radius); }
	
	virtual bool Pathable(node *from, node *to);
	
	// utility functions
	/** verify that the hierarchy is consistent */
	virtual void VerifyHierarchy();
	
	// hierarchical modifications
	/** remove node from abstraction */
	virtual void RemoveNode(node *n);
	/** remove edge from abstraction */
  virtual void RemoveEdge(edge *e, unsigned int absLevel);
	/** add node to abstraction */
	virtual void AddNode(node *n);
	/** add edge to abstraction */
	virtual void AddEdge(edge *e, unsigned int absLevel);
	/** This must be called after any of the above add/remove operations. But the
		operations can be stacked followed by a single RepairAbstraction call. */
  virtual void RepairAbstraction();	
private:
	void buildAbstraction();
	void buildNodeIntoParent(node *n, node *parent);
	void abstractionBFS(node *which, node *parent, int depth);

	void addEdges(Graph *g);
	void addNodes(Graph *g);
	
	int radius;
};

#endif
