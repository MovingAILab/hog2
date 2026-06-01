#include "GraphAbstraction.h"
#include "GraphEnvironment.h"
#include <stdlib.h>
#include <cmath>

#ifndef ABSTRACTGRAPHMAPHEURISTIC_H
#define ABSTRACTGRAPHMAPHEURISTIC_H

class AbstractGraphMapHeuristic : public GraphHeuristic {
public:
  AbstractGraphMapHeuristic(Graph *graph, Map *map)
  :g(graph), m(map) {}
  double HCost(const graphState &state1, const graphState &state2)
  {
		double x1 = g->GetNode(state1)->GetLabelF(GraphAbstractionConstants::kXCoordinate);
		double y1 = g->GetNode(state1)->GetLabelF(GraphAbstractionConstants::kYCoordinate);
		double x2 = g->GetNode(state2)->GetLabelF(GraphAbstractionConstants::kXCoordinate);
		double y2 = g->GetNode(state2)->GetLabelF(GraphAbstractionConstants::kYCoordinate);

		double a = std::fabs(x1-x2); //((x1>x2)?(x1-x2):(x2-x1));
    double b = std::fabs(y1-y2); //((y1>y2)?(y1-y2):(y2-y1));
		double result = (a>b)?(b*ROOT_TWO+a-b):(a*ROOT_TWO+b-a);
		return( result * m->GetCoordinateScale() );
  };

	Graph *GetGraph() { return g; };
private:
  Graph *g;
	Map *m;
};

#endif
