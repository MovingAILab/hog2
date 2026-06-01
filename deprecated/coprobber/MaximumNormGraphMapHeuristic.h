#include "GraphEnvironment.h"
#include <stdlib.h>

#ifndef MAXIMUMNORMGRAPHMAPHEURISTIC_H
#define MAXIMUMNORMGRAPHMAPHEURISTIC_H

class MaximumNormGraphMapHeuristic : public GraphHeuristic {
public:
  MaximumNormGraphMapHeuristic(Graph *graph)
  :g(graph) {}
  double HCost(const graphState &state1, const graphState &state2)
  {
    int x1 = g->GetNode(state1)->GetLabelL(GraphSearchConstants::kMapX);
    int y1 = g->GetNode(state1)->GetLabelL(GraphSearchConstants::kMapY);
    int x2 = g->GetNode(state2)->GetLabelL(GraphSearchConstants::kMapX);
    int y2 = g->GetNode(state2)->GetLabelL(GraphSearchConstants::kMapY);

		return max( abs(x1-x2), abs(y1-y2) );
  }

	Graph *GetGraph() { return g; };
private:
  Graph *g;
};

#endif
