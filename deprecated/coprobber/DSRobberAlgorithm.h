#include "DSCREnvironment.h"

#ifndef DSROBBERALGORITHM
#define DSROBBERALGORITHM

// This is an interface that will be used to compute a best response
// against a given robber algorithm.
// The algorithm cannot keep track of the history of the game since
// computation is bottom-up in the game tree.
// Furthermore, the robber runs at speed 1.

template<class state, class action>
class DSRobberAlgorithm {

	public:

	virtual ~DSRobberAlgorithm() {};

	virtual state MakeMove( state pos_robber, state pos_cop, unsigned int num_graph_nodes ) = 0;

};

#endif
