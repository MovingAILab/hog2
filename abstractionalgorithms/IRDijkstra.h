/*
 *  IRDijkstra.h
 *  hog2
 *
 *  Created by Nathan Sturtevant on 10/9/07.
 *  Copyright 2007 Nathan Sturtevant, University of Alberta. All rights reserved.
 *
 */

#ifndef IRDijkstra_H
#define IRDijkstra_H

#include "SearchAlgorithm.h"
#include "OpenClosedList.h"
#include "FPUtil.h"
#include "Graph.h"
#include "GraphAbstraction.h"

#include <unordered_map>

namespace IRDijkstraConstants {

	/** Definitions for node labels */
	enum {
		kAbstractionLevel = 0, // this is a LONG label
		kCorrespondingNode = 1, // this is a LONG label
		kGCost = 2, // this is a DOUBLE label
//		kHCost = 3, // this is a DOUBLE label
//		kOptimalFlag = 4, // this is a LONG label
//		kInOpenList = 5 // this is a LONG label
	};

	struct GNode {
		GNode(node *nn) :n(nn) {}
		GNode() :n(0) {}
		node *n;
	};

	struct NodeEqual {
		bool operator()(const GNode &i1, const GNode &i2) const
		{ return (i1.n->getUniqueID() == i2.n->getUniqueID()); }
	};

	struct NodeCompare {
		// return true if we prefer i2 over i1
		bool operator()(const GNode &i1, const GNode &i2) const
		{
			// return true if node1 has higher g-cost
			return (fgreater(i1.n->GetLabelF(kGCost),
											 i2.n->GetLabelF(kGCost)));
		}
	};

	struct NodeHash {
			size_t operator()(const GNode &x) const
			{ return (size_t)(x.n->getUniqueID()); }
	};

	typedef OpenClosedList<GNode, NodeHash,
		NodeEqual, NodeCompare> PQueue;
	
	typedef std::unordered_map<uint32_t, GNode> NodeLookupTable;
}


// variables starting with "a" are in the abstraction
// variables starting with "g" are in the defined search graph
class IRDijkstra : public SearchAlgorithm {
public:
	IRDijkstra();
	virtual ~IRDijkstra();
	virtual const char *GetName();
	virtual path *GetPath(GraphAbstraction *aMap, node *from, node *to, reservationProvider *rp = 0);
	path *DoOneSearchStep();
	bool InitializeSearch(GraphAbstraction *aMap, node *from, node *to);
	void OpenGLDraw() const;
	int GetNodesRefined() { return nodesRefined; }
private:
	node *FindTopLevelNode(node *one, node *two, GraphAbstraction *aMap);
	void SetInitialValues(node *gNewNode, node *aRealNode, node *gParent);
//	void UpdateNode(node *gNode);
//	void UpdateH(node *gNode);
//	void UpdateG(node *gNode);
//	void UpdateOptH(node *gNode);
//	void MakeNeighborsOpen(node *gNode);
	void RefineNode(node *gNode);
	node *GetRealNode(node *gNode) const;
	bool ShouldAddEdge(node *aLowerNode, node *aHigherNode);

	void GetAllSolutionNodes(node *goal, std::vector<node*> &nodes);
	void ExpandNeighbors(node *gNode);
	path *ExtractAndRefinePath();
	path *GetSolution(node *gNode);

	
	IRDijkstraConstants::PQueue q;
	IRDijkstraConstants::NodeLookupTable closedList;
	node *aStart, *aGoal;
	node *gStart, *gGoal;
	GraphAbstraction *absGraph;
	Graph *g;
	int nodesRefined;
	bool done;
};


#endif
