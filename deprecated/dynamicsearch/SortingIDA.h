#ifndef SortingIDA_H
#define SortingIDA_H
#include "GeneralIDA.h"
#include <algorithm>

typedef std::pair<unsigned, double> IndexAndCost;

bool IndexAndCostComp (IndexAndCost i,IndexAndCost j);

template <class state, class action, class environment>
class SortingIDA: public GeneralIDA<state, action, environment> {
	// variables and functions will be using
	using GeneralIDA<state, action, environment>::check_bounds;
	using GeneralIDA<state, action, environment>::g_stack;
	using GeneralIDA<state, action, environment>::expand_full_iter;
	using GeneralIDA<state, action, environment>::active_path;
	using GeneralIDA<state, action, environment>::sol_found;
	using GeneralIDA<state, action, environment>::best_path_cost;
	using GeneralIDA<state, action, environment>::to_expand;
	using GeneralIDA<state, action, environment>::nodes_ex_iter;
	using GeneralIDA<state, action, environment>::current_best_path;
	using GeneralIDA<state, action, environment>::h_stack;
	using GeneralIDA<state, action, environment>::nodes_gen_iter;
	using GeneralIDA<state, action, environment>::nodes_check_iter;
	using GeneralIDA<state, action, environment>::h_weight;
	using GeneralIDA<state, action, environment>::g_weight;
public:
	SortingIDA() {}

	virtual const char * GetName(){
		std::string name = "Sorting IDA*(H Weight = ";
		name += double_to_string(h_weight);
		name += ", G Weight = ";
		name += double_to_string(g_weight);
		name += ")";
		return name.c_str();
	}

protected:

	virtual int search_node(environment *env, state &currState, state &goal, action forbiddenAction, double edge_cost);

	virtual void sort_succ(environment *env, state &currState, state &goal, action &forbiddenAction, double parent_g, std::vector<action> &actions, std::vector<unsigned> &succ_order, std::vector<double> &succ_h_costs, std::vector<double> &succ_g_costs);

	/**
	Returns the cost for this state based on the g and h values.
	**/
	virtual double get_cost(double g, double h);

	/*
	virtual bool sort_succ(environment *env, state &currState, state &goal, state &forbiddenState, double parent_g, std::vector<state> &succs, std::vector<unsigned> &succ_order, std::vector<double> &succ_h_costs, std::vector<double> &succ_g_costs){
		if(active_state_path.size() == 0)
			return true;
		return false;}*/
};

template <class state, class action, class environment>
void  SortingIDA<state, action, environment>::sort_succ(environment *env, state &currState, state &goal, action &forbiddenAction, double parent_g, std::vector<action> &actions, std::vector<unsigned> &succ_order, std::vector<double> &succ_h_costs, std::vector<double> &succ_edge_costs) {

	unsigned depth = active_path.size();
	std::vector<IndexAndCost> to_sort;

	for(unsigned i = 0; i < actions.size(); i++) {
		// prunes parent action from ordering
		if((depth != 0) && (forbiddenAction == actions[i])) {
			succ_h_costs.push_back(-1);
			succ_edge_costs.push_back(-1);
			continue;
		}

		action to_apply = actions[i];
		double edgeCost = env->GCost(currState, to_apply);
		double g = parent_g + edgeCost;

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

		IndexAndCost succ_info(i, get_cost(g, h));
		to_sort.push_back(succ_info);

		// pushes important info onto stacks
		succ_edge_costs.push_back(edgeCost);
		succ_h_costs.push_back(h);
		//succ_order.push_back(i);
	}

	sort(to_sort.begin(), to_sort.end(), IndexAndCostComp);

	for(unsigned i = 0; i < to_sort.size(); i++){
		succ_order.push_back(to_sort[i].first);
	}
	assert(succ_edge_costs.size() == succ_h_costs.size());
	assert(actions.size() == succ_edge_costs.size());
	assert((depth == 0 && actions.size() == succ_order.size()) || (depth > 0 && actions.size() -1 == succ_order.size()));
}

template <class state, class action, class environment>
int SortingIDA<state, action, environment>::search_node(environment *env, state &currState, state &goal, action forbiddenAction, double edge_cost) {

	assert(h_stack.size() == active_path.size() + 1);
	assert(h_stack.size() == g_stack.size());

	int bound_check = check_bounds();
	if(bound_check != 1)
		return bound_check;

	// get node info that have already calculated
	double h = h_stack.back();
	double g = g_stack.back() + edge_cost;

	/* For Debugging
	for(unsigned i = 0; i < active_path.size(); i++)
		printf("\t");
	std::cout << currState << "       " << h << " " << g << std::endl;
	*/

	// use g + h to cut off areas of search where cannot find better solution
	if(expand_full_iter && sol_found && !fless(h + g, best_path_cost))
		return 0;

	if(!to_expand(g, h)) {
		return 0;
	}

	nodes_ex_iter++;

	// goal test
	if (fequal(h, 0.0) && env->GoalTest(currState, goal)) {
		current_best_path = active_path;
		best_path_cost = g;
		sol_found = true;
		return 1; // found goal
	}

	// generate applicable actions
	std::vector<action> actions;
	env->GetActions(currState, actions);

	nodes_gen_iter += actions.size();

	int my_status = 0;

	std::vector<unsigned> succ_order;
	std::vector<double> succ_h_costs;
	std::vector<double> succ_edge_costs;

	// sort successors
	sort_succ(env, currState, goal, forbiddenAction, g, actions, succ_order, succ_h_costs, succ_edge_costs);

	// check successors one at a time
	for (unsigned x = 0; x < succ_order.size(); x++)
	{
		// use g + h to cut off areas of search where cannot find better solution
		if(expand_full_iter && sol_found && !fless(h + g, best_path_cost))
			return my_status;

		nodes_check_iter++;

		active_path.push_back(actions[succ_order[x]]);

		env->ApplyAction(currState, actions[succ_order[x]]);
		env->InvertAction(actions[succ_order[x]]);

		// push information onto the stack
		h_stack.push_back(succ_h_costs[succ_order[x]]);
		g_stack.push_back(g);

		// recursively search child
		int status = search_node(env, currState, goal, actions[succ_order[x]], succ_edge_costs[succ_order[x]]);
		env->ApplyAction(currState, actions[succ_order[x]]);

		active_path.pop_back();
		g_stack.pop_back();
		h_stack.pop_back();

		// handle status
		if (status == 1) {
			if(expand_full_iter)
				my_status = 1;
			else {
				return 1;
			}
		}
		else if(status > 1) {
			return status;
		}
	}
	return my_status;
}

template <class state, class action, class environment>
double SortingIDA<state, action, environment>::get_cost(double g, double h) {
	return g_weight*g + h_weight*h;
}
#endif
