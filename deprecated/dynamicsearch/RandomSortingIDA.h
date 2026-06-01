#ifndef RandomSortingIDA_H
#define RandomSortingIDA_H
#include "SortingIDA.h"
#include <algorithm>

template <class state, class action, class environment>
class RandomSortingIDA: public SortingIDA<state, action, environment> {

	using SortingIDA<state, action, environment>::h_weight;
	using SortingIDA<state, action, environment>::g_weight;
	using SortingIDA<state, action, environment>::active_path;

public:
	RandomSortingIDA() {seed_for_depth = false;}

	virtual const char * GetName(){
		std::string name = "Random Sorting IDA*(H Weight = ";
		name += double_to_string(h_weight);
		name += ", G Weight = ";
		name += double_to_string(g_weight);
		name += ", Seed = ";
		name += int_to_string(seed);
		name += ")";
		return name.c_str();
	}

	void Change_Seed(unsigned new_seed){seed = new_seed;}

protected:
	virtual void sort_succ(environment *env, state &currState, state &goal, action &forbiddenAction, double parent_g, std::vector<action> &actions, std::vector<unsigned> &succ_order, std::vector<double> &succ_h_costs, std::vector<double> &succ_g_costs);

	unsigned seed;

	bool seed_for_depth;

	// initializes search parameters
	virtual void initialize_search(environment *, state, state);
};

template <class state, class action, class environment>
void  RandomSortingIDA<state, action, environment>::sort_succ(environment *env, state &currState, state &goal, action &forbiddenAction, double parent_g, std::vector<action> &actions, std::vector<unsigned> &succ_order, std::vector<double> &succ_h_costs, std::vector<double> &succ_edge_costs) {

	unsigned depth = active_path.size();

	for(unsigned i = 0; i < actions.size(); i++) {
		// prunes parent action from ordering
		if((depth != 0) && (forbiddenAction == actions[i])) {
			succ_h_costs.push_back(-1);
			succ_edge_costs.push_back(-1);
			continue;
		}

		action to_apply = actions[i];
		double edgeCost = env->GCost(currState, to_apply);

		env->ApplyAction(currState, to_apply);

		// gets heuristic of child
		double h;
		if(env->IsGoalStored()) {
			h = env->HCost(currState);
		}
		else {
			h = env->HCost(currState, goal);
		}

		// undoes action
		env->InvertAction(to_apply);
		env->ApplyAction(currState, to_apply);

		// pushes important info onto stacks
		succ_edge_costs.push_back(edgeCost);
		succ_h_costs.push_back(h);
		succ_order.push_back(i);
	}

	if(seed_for_depth)
		srand(seed + depth);

	for(unsigned i = 0; i < succ_order.size() - 1; i++) {
		int index = random() % (succ_order.size() - i);
		unsigned to_move = succ_order[i];
		succ_order[i] = succ_order[index + i];
		succ_order[index + i] = to_move;
	}

	assert(succ_edge_costs.size() == succ_h_costs.size());
	assert(actions.size() == succ_edge_costs.size());
	assert((depth == 0 && actions.size() == succ_order.size()) || (depth > 0 && actions.size() -1 == succ_order.size()));
}

template <class state, class action, class environment>
void RandomSortingIDA<state, action, environment>::initialize_search(environment *env, state start, state goal) {
	GeneralIDA<state, action, environment>::initialize_search(env, start, goal);

	if(!seed_for_depth)
		srand(seed);
}
#endif
