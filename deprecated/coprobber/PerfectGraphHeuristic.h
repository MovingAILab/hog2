#include "GraphEnvironment.h"
#include <stdlib.h>

#ifndef PERFECTGRAPHHEURISTIC_H
#define PERFECTGRAPHHEURISTIC_H

class PerfectGraphHeuristic : public GraphHeuristic {
public:
	// this heuristic holds its own copy of the submitted graph because it will alter it
  PerfectGraphHeuristic(Graph *graph)
  :g(graph->cloneAll()) {
		edge_iterator eit = g->getEdgeIter();
		edge *e = g->edgeIterNext( eit );
		while( e ) {
			e->setWeight( 1. );
			e = g->edgeIterNext( eit );
		}
		FloydWarshall( g, distance_heuristic );
	
	};

	~PerfectGraphHeuristic() {
		delete g;
	};

  double HCost(const graphState &state1, const graphState &state2)
  {
		return distance_heuristic[state1][state2];
  };

	Graph *GetGraph() { return g; };

private:
  Graph *g;
	std::vector<std::vector<double> > distance_heuristic;
};

#endif
