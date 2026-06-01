/*
 *  $Id: MapFlatAbstraction.h
 *  hog2
 *
 *  Created by Nathan Sturtevant on 6/10/05.
 *  Modified by Nathan Sturtevant on 02/29/20.
 *
 * This file is part of HOG2. See https://github.com/nathansttt/hog2 for licensing information.
 *
 */

#include "MapAbstraction.h"

#ifndef MAPFLATABSTRACTION_H
#define MAPFLATABSTRACTION_H

class MapFlatAbstraction : public MapAbstraction {
public:
	MapFlatAbstraction(Map *_m);
	~MapFlatAbstraction();
	/** return a new abstraction map of the same type as this map abstraction */
	virtual MapAbstraction *Clone(Map *_m) { return new MapFlatAbstraction(_m); }
	
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
		void buildConnectivityGroups();
		bool groupsValid;
		std::vector<int> groups;
};

#endif //  MAPFLATABSTRACTION_H
