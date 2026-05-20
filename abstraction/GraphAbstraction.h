/*
 *  $Id: GraphAbstraction.h
 *  hog2
 *
 *  Created by Nathan Sturtevant on 1/11/05.
 *  Modified by Nathan Sturtevant on 02/29/20.
 *
 * This file is part of HOG2. See https://github.com/nathansttt/hog2 for licensing information.
 *
 */

#include "Graph.h"
#include "Path.h"
#include "Graphics.h"
#include <stdlib.h>
#include <stdio.h>

#ifndef GRAPHABSTRACTION_H
#define GRAPHABSTRACTION_H
#include <vector>

namespace GraphAbstractionConstants {
	/** Definitions for node labels */
	enum {
		kAbstractionLevel = 0, // this is a LONG label
		kNumAbstractedNodes = 1, // nodes that we abstract; this is a LONG label
		kParent = 2, // node that abstracts us; this is a LONG label
		kNodeWidth = 3, // the maximum size object that can completely traverse this node; this is a LONG label
		kTemporaryLabel = 4, // for any temporary usage; this label can be LONG or FLOATING point
		kXCoordinate = 5, // cache for opengl drawing; this is a FLOATING POINT label
		kYCoordinate = 6,	// this is a FLOATING POINT label
		kZCoordinate = 7,	// this is a FLOATING POINT label
		kNodeBlocked = 8, // this is a LONG label
		kFirstData = 9
	};
	
	/** Definitions for edge labels */
	enum {
		kEdgeWidth = 1
	};
	
	
	/** kFirstData & beyond:
	 *
	 * in abstract Graph these are the node numbers of the abstraction (LONG labels)
	 *
	 * in 0th level abstraction they are the x, y location of the tile
	 * along with which side (type tCorner)  (all LONG labels)
	 */
	
	/** Definitions for edge labels */
	enum {
		kEdgeCapacity=2
	};
	
	const double kUnknownPosition = -50.0;
}

/**
 * A generic class for basic operations on Graph abstractions.
 */

class GraphAbstraction {
public:
	GraphAbstraction() :abstractions() {}
	virtual ~GraphAbstraction();
	
	// basic Graph functions
	/** is there a legal path between these 2 nodes? */
	virtual bool Pathable(node *from, node *to) = 0;
	/** given 2 nodes, find as much of their hierarchy that exists in the Graph */
	void GetNumAbstractGraphs(node *from, node *to, std::vector<node *> &fromChain, std::vector<node *> &toChain);
	/** return the abstract Graph at the given level */
	Graph* GetAbstractGraph(int level) { return abstractions[level]; }
	/** return the total number of graphs in the hierarchy */
	unsigned int getNumAbstractGraphs() { return (unsigned int)abstractions.size(); }
	/** heuristic cost between any two nodes */
	virtual double h(node *a, node *b) = 0;
	/** length in distance of a path */
	double distance(path *p);
	/** return nth level parent of which or null if it doesn't exist */
	node *GetNthParent(node *which, int n);
	/** return true if the first node is a parent of or is equal two the second node */
	bool IsParentOf(node *parent, node *child);
	
	inline node *GetParent(node *which) { return abstractions[GetAbstractionLevel(which)+1]->GetNode(which->GetLabelL(GraphAbstractionConstants::kParent)); }
	inline long GetNumChildren(node *which) { return which->GetLabelL(GraphAbstractionConstants::kNumAbstractedNodes); }
	inline node *GetNthChild(node *which, int n) { return abstractions[GetAbstractionLevel(which)-1]->GetNode(which->GetLabelL(GraphAbstractionConstants::kFirstData+n)); }
	
	node* GetRandomGroundNodeFromNode(node *n);
	
	inline long GetAbstractionLevel(node *which) { return which->GetLabelL(GraphAbstractionConstants::kAbstractionLevel); }
	inline Graph* GetAbstractGraph(node *which) { return abstractions[which->GetLabelL(GraphAbstractionConstants::kAbstractionLevel)]; }
	// utility functions
	/** verify that the hierarchy is consistent */
	virtual void VerifyHierarchy() = 0;
	/// rebuild hierarchy from original domain */
	// virtual void rebuild() = 0;
	/// get current revision of hierarchy -- indicates if changes have been made */
	//virtual int GetRevision() = 0;
	void ClearMarkedNodes();
	
	// hierarchical modifications
	/** remove node from abstraction */
	virtual void RemoveNode(node *n) = 0;
	/** remove edge from abstraction */
	virtual void RemoveEdge(edge *e, unsigned int absLevel) = 0;
	/** add node to abstraction */
	virtual void AddNode(node *n) = 0;
	/** add edge to abstraction */
	virtual void AddEdge(edge *e, unsigned int absLevel) = 0;
	/** This must be called after any of the above add/remove operations. But the
		operations can be stacked followed by a single RepairAbstraction call. */
	virtual void RepairAbstraction() = 0;
	virtual int MeasureRepairHits() { return 0; }
	void MeasureAbstractionValues(int level, double &n, double &n_dev, double &c, double &c_dev);
	double MeasureAverageNodeWidth(int level);
	
	virtual void OpenGLDraw() const {}
  virtual Graphics::point GetNodeLoc(node *) const { Graphics::point v; v.x = v.y = v.z = 0; return v; }
protected:
	std::vector<Graph *> abstractions;
private:
	int ComputeWidth(node *n);
	int WidthBFS(node *child, node *parent);
	double MeasureExpectedNodeWidth(node *n);
	int GetNumExternalEdges(node *n, node *p);
	int CountEdgesAtDistance(node *child, node *parent, std::vector<int> &dists);
};

// this class can be defined if you want to solve multiple domains...
//
// class domainAbstraction : public GraphAbstraction {
// 	void domainAbstraction(abstractableDomain *) = 0;
// };

#endif
