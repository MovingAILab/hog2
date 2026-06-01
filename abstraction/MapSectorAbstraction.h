/*
 *  $Id: MapSectorAbstraction.h
 *  hog2
 *
 *  Created by Nathan Sturtevant on 8/8/05.
 *  Modified by Nathan Sturtevant on 02/29/20.
 *
 * This file is part of HOG2. See https://github.com/nathansttt/hog2 for licensing information.
 *
 */

#include "MapAbstraction.h"

#ifndef MAPSectorABSTRACTION_H
#define MAPSectorABSTRACTION_H

class MapSectorAbstraction : public MapAbstraction {
public:
	/** Creat a SectorAbstraction of the map. The sector size must be greater than 1 */
	MapSectorAbstraction(Map *, int, int);
	MapSectorAbstraction(Map *, int);
	~MapSectorAbstraction();
	MapAbstraction *Clone(Map *_m) { return new MapSectorAbstraction(_m, sectorSize); }

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
	void abstractionBFS(node *which, node *parent, int quadrant);
	int getQuadrant(node *which);
	
	void addEdges(Graph *g);
	void addNodes(Graph *g);
	
	int sectorSize, sectorMultiplier;
};

#endif
