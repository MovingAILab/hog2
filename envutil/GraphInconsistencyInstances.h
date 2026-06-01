//
//  GraphInconsistencyInstances.h
//  Inconsistency
//
//  Created by Nathan Sturtevant on 2/19/19.
//  Copyright © 2019 NS Software. All rights reserved.
//

#ifndef GraphInconsistencyInstances_h
#define GraphInconsistencyInstances_h

#include <stdio.h>
#include "GraphEnvironment.h"

namespace GraphInconsistencyExamples {

	const int kHeuristic = GraphSearchConstants::kTemporaryLabel;

	class GraphHeuristic : public Heuristic<graphState> {
	public:
		GraphHeuristic(Graph *g) :g(g) {}
		double HCost(const graphState &a, const graphState &b) const
		{
			return g->GetNode(a)->GetLabelL(kHeuristic);
		}
	private:
		Graph *g;
	};

	Graph *GetPolyGraph(int N);
	Graph *GetExpoGraphA(int N);
	Graph *GetExpoGraphB(int N);
	Graph *GetWeightedInconsistency(float w, int N);
}

#endif /* GraphInconsistencyInstances_h */
