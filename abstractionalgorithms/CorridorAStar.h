/*
 *  $Id: corridorAStar.h
 *  hog2
 *
 *  Created by Nathan Sturtevant on 6/22/05.
 *  Modified by Nathan Sturtevant on 02/29/20.
 *
 * This file is part of HOG2. See https://github.com/nathansttt/hog2 for licensing information.
 *
 */

#ifndef CORRIDORASTAR_H
#define CORRIDORASTAR_H

#include "SearchAlgorithm.h"
#include "Graph.h"
#include "Heap.h"

/** Corridor AStar builds a a* path between two nodes, restricting itself to
a particular corridor, if defined. The corridor must be set before every search
if it is to be used properly. After each GetPath call the corridor is reset. If
no corridor is defined, it will explore all nodes.
*/

class corridorAStar : public SearchAlgorithm {
public:
	corridorAStar();
	virtual ~corridorAStar() {}
	path *GetPath(GraphAbstraction *aMap, node *from, node *to, reservationProvider *rp = 0);
	/** get the best path from FROM to TO. Use hGoal as the heuristic goal. If TO is not on
		* the same level as from path will be returned that ends inside the child of TO. */
	path *getBestPath(GraphAbstraction *aMap, node *from, node *to, node *hGoal, reservationProvider *rp = 0);
	/** get the best path from aFROM to aTO. Use an insertion edge cost from the original from/to. */
	path *getBestPath(GraphAbstraction *aMap, node *afrom, node *ato, node *from, node *to, reservationProvider *rp = 0);
	void setCorridor(const std::vector<node *> *);
	virtual const char *GetName() { return "corridorAStar"; }
private:
		void relaxEdge(Heap *nodeHeap, Graph *g, GraphAbstraction *aMap,
									 edge *e, node *from, node *to, node *dest);
	void relaxFirstEdge(Heap *nodeHeap, Graph *g, GraphAbstraction *aMap,
											edge *e, node *from, node *afrom, node *ato, node *dest);

	void relaxFinalEdge(Heap *nodeHeap, Graph *g, GraphAbstraction *aMap,
											edge *e, node *from, node *to, node *realDest);
	path *extractBestPath(Graph *g, unsigned int current);
	const std::vector<node *> *corridor;
	std::vector<node *> emptyCorridor;
};

#endif
