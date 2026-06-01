/*
 *  CFOptimalRefinement.h
 *  hog2
 *
 *  Created by Nathan Sturtevant on 5/22/07.
 *  Copyright 2007 __MyCompanyName__. All rights reserved.
 *
 */

#ifndef CFOPTIMALREFINEMENT_H
#define CFOPTIMALREFINEMENT_H

#include "SearchAlgorithm.h"
#include "OpenClosedList.h"
#include "FPUtil.h"
#include "Graph.h"
#include "GraphAbstraction.h"

#include <unordered_map>

namespace CFOptimalRefinementConstants {

	/** Definitions for node labels */
	enum {
		kAbstractionLevel = 0, // this is a LONG label
		kCorrespondingNode = 1, // this is a LONG label
		kGCost = 2, // this is a DOUBLE label
		kHCost = 3, // this is a DOUBLE label
		kOptimalFlag = 4, // this is a LONG label
		kInOpenList = 5 // this is a LONG label
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
			// if f-cost is tied
			if (fequal(i1.n->GetLabelF(kGCost)+i1.n->GetLabelF(kHCost),
								 i2.n->GetLabelF(kGCost)+i2.n->GetLabelF(kHCost)))
			{
				// if both in/not in open list
				if (i1.n->GetLabelL(kInOpenList) == i2.n->GetLabelL(kInOpenList))
				{
					if ((i1.n->GetLabelL(kInOpenList) == 0) && (i1.n->GetLabelL(kAbstractionLevel) == 0))
						return true;
					if ((i2.n->GetLabelL(kInOpenList) == 0) && (i2.n->GetLabelL(kAbstractionLevel) == 0))
						return false;

					return fgreater(std::min(i1.n->GetLabelF(kGCost), i1.n->GetLabelF(kHCost)),
													std::min(i2.n->GetLabelF(kGCost), i2.n->GetLabelF(kHCost)));
					
					// return true if node1 has lower g-cost
					//return (fless(i1.n->GetLabelF(kGCost), i2.n->GetLabelF(kGCost)));
				}
				// return true if node1 isn't in open list
				return (i2.n->GetLabelL(kInOpenList));
			}
			// return true if node1 has higher f-cost
			return (fgreater(i1.n->GetLabelF(kGCost)+i1.n->GetLabelF(kHCost),
											 i2.n->GetLabelF(kGCost)+i2.n->GetLabelF(kHCost)));
		}
	};

	struct NodeHash {
			size_t operator()(const GNode &x) const
			{ return (size_t)(x.n->getUniqueID()); }
	};
}

typedef OpenClosedList<CFOptimalRefinementConstants::GNode, CFOptimalRefinementConstants::NodeHash,
CFOptimalRefinementConstants::NodeEqual, CFOptimalRefinementConstants::NodeCompare> PQueue;

typedef std::unordered_map<uint32_t, CFOptimalRefinementConstants::GNode> NodeLookupTable;

// variables starting with "a" are in the abstraction
// variables starting with "g" are in the defined search graph
class CFOptimalRefinement : public SearchAlgorithm {
public:
	CFOptimalRefinement();
	virtual ~CFOptimalRefinement();
	virtual const char *GetName();
	virtual path *GetPath(GraphAbstraction *aMap, node *from, node *to, reservationProvider *rp = 0);
	path *DoOneSearchStep();
	bool InitializeSearch(GraphAbstraction *aMap, node *from, node *to);
	void OpenGLDraw() const;
private:
	node *FindTopLevelNode(node *one, node *two, GraphAbstraction *aMap);
	void SetInitialValues(node *gNewNode, node *aRealNode, node *gParent);
	void UpdateNode(node *gNode);
	void UpdateH(node *gNode);
	void UpdateG(node *gNode);
	void UpdateOptH(node *gNode);
	void MakeNeighborsOpen(node *gNode);
	void RefineNode(node *gNode);
	node *GetRealNode(node *gNode) const;
	bool ShouldAddEdge(node *aLowerNode, node *aHigherNode);

	PQueue q;
	node *aStart, *aGoal;
	node *gStart, *gGoal;
	GraphAbstraction *absGraph;
	Graph *g;
};


#endif
