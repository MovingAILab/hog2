#ifndef BEAMNODE_H
#define BEAMNODE_H

#include "FPUtil.h"
struct beam_position {
	unsigned beam_num;
	unsigned beam_pos;
};

/**
A class for beam node information. Each beam node contains information regarding
the actual state, the various costs, the index of the parent in the beam in
which the parent exists, and the hash key of the node
**/
template <class state>
class BeamNode {
public:
	BeamNode() {
		cost = -1.0;
	}
	BeamNode(state &curr, double gCost, double hCost, double fCost, unsigned parent, uint64_t key) {
		my_state = curr;
		h_value = hCost;
		g_value = gCost;
		cost = fCost;
		my_key = key;
		parent_index = parent;
	}

	state my_state;
	double cost;
	double h_value;
	double g_value;
	uint64_t my_key;
	unsigned parent_index;


	/**
	Allows for comparison of BeamNodes for both sorting and merging BeamNode lists
	**/
	bool operator <(const BeamNode &second) const{
		if(fless(cost, second.cost) || (fequal(cost, second.cost) && fgreater(g_value, second.g_value)))
			return 1;
		return 0;
	}
};
#endif

