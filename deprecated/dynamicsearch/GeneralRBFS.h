#ifndef RBFS_H
#define RBFS_H

#include <iostream>
#include "FPUtil.h"
#include "StringUtils.h"
#include "GenericStepAlgorithm.h"
#include "assert.h"

template <class state, class action, class environment>
class GeneralRBFS : public GenericStepAlgorithm<state, action, environment> {
public:
	GeneralRBFS() {
		bound_expanded = false;
		bound_generated = false;
		bound_checked = false;

		h_weight = 1.0;
		g_weight = 1.0;

		best_path_cost = -1.0;
	}

	virtual ~GeneralRBFS() {}

	/**
	Finds a path from the node "from" to the node "to" in the search space using
	an IDA* like algorithm and stores the set of actions in thePath
	**/
	virtual int GetPath(environment *env, state from, state to,
	                    std::vector<action> &thePath);

	virtual int GetPath(environment *env, state from, state to,
	                    std::vector<state> &thePath) {return -1;}

	virtual const char * GetName(){
		std::string name = "WRBFS(H Weight = ";
		name += double_to_string(h_weight);
		name += ", G Weight = ";
		name += double_to_string(g_weight);
		name += ")";
		return name.c_str();
	}

	virtual void LogFinalStats(StatCollection *stats){}

	/** Get the number of nodes expanded (a node is expanded if the goal test is called) **/
	virtual uint64_t GetNodesExpanded() const { return nodesExpanded + nodes_ex_iter; }

	/** Get the number of nodes generated (if the successors or successor actions
	 of a node are successors, these are considered generated nodes)**/
	virtual uint64_t GetNodesTouched() const { return nodesGenerated + nodes_gen_iter; }

	/** Get the number of nodes checked to be generated (a node is checked if search_node
	 is called on it, in which case to_expand is necessarily called on it except in a few
	 situations regarding expanding an entire iteration) **/
	virtual uint64_t GetNodesChecked() {return nodesChecked + nodes_check_iter; }

	/**
	Returns the path of the last path found, unless a search is currently in progress, in
	which case it returns -1.0;
	**/
	double GetPathCost() {return best_path_cost;}

	/**
	Inherited methods for setting limits on th number of nodes expanded, generated, or checked
	**/
	virtual bool SetExpandedLimit(uint64_t limit);
	virtual bool SetTouchedLimit(uint64_t limit);
	virtual bool SetCheckedLimit(uint64_t limit);

	/**
	Prepares the algorithm for a step by step run.
	**/
	virtual bool Initialize(environment *env, state from, state to);

	/**
	Inherited methods for incrementing algorithm by one step.
	**/
	virtual int StepAlgorithm(std::vector<action> &thePath);
	virtual int StepAlgorithm(std::vector<state> &thePath) {return -1;}

	/**
	Checks if the algorithm is ready to be incremented one step
	**/
	bool Is_Step_Active() {return step_by_step_active;}

	/**
	Changes the weights.
	**/
	int Change_Weights(double h_g,double h_w) {
		if(!fless (h_g, 0.0) && !fless(h_w, 0.0)) {
			h_weight = h_w;
			g_weight = h_g;
			return 1;
		}
		return 0;
	}

protected:
	uint64_t nodesExpanded, nodesGenerated, nodesChecked; // counts nodes for the categories
	uint64_t nodes_ex_iter, nodes_gen_iter, nodes_check_iter; // counts nodes for the categories in the current iteration
	uint64_t expanded_limit, generated_limit, checked_limit; // the limits on the given categories

	bool bound_expanded, bound_generated, bound_checked; // whether the categories are bounded

	double h_weight, g_weight; // the appropriate weights
	double best_path_cost; // the cost of the last path found, -1.0 if none, or search is underway

	std::vector<action> current_best_path; // holds the last path found
	std::vector<action> active_path; // the current path being searched
	std::vector<action> inversion_stack; // holds all the inversions of the active path
	std::vector<double> g_stack; // holds the g values of nodes
	std::vector<double> h_stack; // holds the h values of nodes

	bool sol_found;

	double current_threshold; // stores the current threshold
	bool step_by_step_active;

	/**
	Finds the children of the given state, assigns the appropriate costs to them (based on the given lower
	bounds), sets the order array (lists the indices of the actions in ascending order of cost), and
	sets the vectors with other relevant information. Assumes the vectors are of the appropriate size.
	**/
	virtual unsigned assign_and_sort(environment *env, state &currState, state &goal,
	                                 double parent_cost, double lower_bound,
	                                 std::vector<action> &actions, std::vector<action> &inverted,
	                                 std::vector<unsigned> &order, std::vector<double> &actual_costs, std::vector<double> &costs, std::vector<double> &edge_costs, std::vector<double> &h_values);

	/**
	Performs the RBFS recursively.
	**/
	virtual int search_node(environment *env, state &currState, state &goal, double true_state_cost, double state_cost, double upper_bound, double &child_bound);

	/**
	Returns the cost for this state based on the g and h values.
	**/
	virtual double get_cost(double g, double h);

	/**
	Cleans out a set of variables prior to search
	**/
	virtual void prepare_vars_for_search();

	/**
	Updates the node counts at the conclusion of an iteration
	**/
	void update_node_counts();

	std::vector<uint64_t> iter_checked;
	std::vector<uint64_t> iter_generated;
	std::vector<uint64_t> iter_expanded;

	environment *my_env;
	state my_goal;
	state active_state;

	/** Stacks for the iterative step by step version of the algorithm **/
	std::vector<std::vector <action> > available_action_stack;
	std::vector<std::vector <action> > available_inversion_stack;
	std::vector<std::vector <unsigned> > action_order;
	std::vector <unsigned> num_children_stack;
	std::vector<std::vector <double> > actual_costs_stack;
	std::vector<std::vector <double> > child_costs_stack;
	std::vector<std::vector <double> > edge_costs_stack;
	std::vector<std::vector <double> > h_values_stack;

	std::vector <double> true_cost_stack;
	std::vector <double> state_cost_stack;
	std::vector <double> upper_bound_stack;

	void assert_stack_sizes_correct();
};

template <class state, class action, class environment>
void GeneralRBFS<state, action, environment>::prepare_vars_for_search() {
	nodes_ex_iter = 0;
	nodes_gen_iter = 0;
	nodes_check_iter = 0;

	nodesExpanded = 0;
	nodesGenerated = 0;
	nodesChecked = 0;

	current_best_path.resize(0);
	active_path.resize(0);
	inversion_stack.resize(0);

	h_stack.resize(0);
	g_stack.resize(0);

	iter_checked.resize(0);
	iter_generated.resize(0);
	iter_expanded.resize(0);

	best_path_cost = -1.0;
}

/** Updates the node counts by adding the nodes from the current iteration to the totals **/
template <class state, class action, class environment>
void GeneralRBFS<state, action, environment>::update_node_counts() {

	nodesExpanded += nodes_ex_iter;
	nodesGenerated += nodes_gen_iter;
	nodesChecked += nodes_check_iter;

	iter_checked.push_back(nodes_check_iter);
	iter_generated.push_back(nodes_gen_iter);
	iter_expanded.push_back(nodes_ex_iter);

	nodes_ex_iter = 0;
	nodes_gen_iter = 0;
	nodes_check_iter = 0;
}

template <class state, class action, class environment>
int GeneralRBFS<state, action, environment>::GetPath(environment *env, state from, state to, std::vector<action> &thePath) {

	prepare_vars_for_search();
	std::vector<action> act;
	env->GetActions(from, act);

	int status = 0;

	double h;

	if(env->IsGoalStored()) {
		h = env->HCost(from);
	}
	else {
		h = env->HCost(from, to);
	}

	h_stack.push_back(h);
	g_stack.push_back(0.0);

	double initial_cost = get_cost(0.0, h);

	double dummy = 0.0;
	step_by_step_active = false;
	sol_found = false;

	current_threshold = -1.0;
	status = search_node(env, from, to, initial_cost, initial_cost, -1.0, dummy);
	return status;
}

template <class state, class action, class environment>
bool GeneralRBFS<state, action, environment>::SetExpandedLimit(uint64_t limit) {
	if(limit > 0) {
		bound_expanded = true; expanded_limit = limit;
	}
	else {
		bound_expanded = false;
	}
	return true;
}

template <class state, class action, class environment>
bool GeneralRBFS<state, action, environment>::SetTouchedLimit(uint64_t limit) {
	if(limit > 0) {
		bound_generated = true; generated_limit = limit;
	}
	else {
		bound_generated = false;
	}
	return true;
}

template <class state, class action, class environment>
bool GeneralRBFS<state, action, environment>::SetCheckedLimit(uint64_t limit) {
	if(limit > 0) {
		bound_checked = true; checked_limit = limit;
	}
	else {
		bound_checked = false;
	}
	return true;
}

template <class state, class action, class environment>
double GeneralRBFS<state, action, environment>::get_cost(double g, double h) {
	return g_weight*g + h_weight*h;
}

/**
Assigns the correct costs to children of the currently expanding state and sorts the nodes based on this cost
parent_cost is the cost of the node being expanded
lower_bound is the lower bound on the cost of the children
actions are the available actions that create the children
inverted is the inverse of the action
order is contains indices indicating the sorted order of the children. For example, if 5 is in the first position
	of order, then the action in position 5 is the best action
actual_costs are the actual f-costs of the actions
costs are the assigned costs of the actions (sorting is done by this value)
edge_costs are the costs of each of the actions
h_values are the h values of the states resulting from the actions
**/
template <class state, class action, class environment>
unsigned GeneralRBFS<state, action, environment>::assign_and_sort (environment *env, state &currState, state &goal, double parent_cost, double lower_bound, std::vector<action> &actions, std::vector<action> &inverted, std::vector<unsigned> &order, std::vector<double> &actual_costs, std::vector<double> &costs, std::vector<double> &edge_costs, std::vector<double> &h_values) {

	unsigned num_invalid = 0;

	for(unsigned i = 0; i < actions.size(); i++) {

		// get inverted action
		inverted.push_back(actions[i]);
		env->InvertAction(inverted[i]);

		// is the forbidden action, so mark so it is never chosen, and put it at the end of the list
		if(active_path.size() > 0 && actions[i] == inversion_stack.back()) {
			order[i] = i;
			actual_costs[i] = -1.0;
			costs[i] = -1.0;
			edge_costs[i] = -1.0;
			h_values[i] = -1.0;
			num_invalid++;
			continue;
		}

		nodes_check_iter++;

		edge_costs[i] = env->GCost(currState, actions[i]);
		env->ApplyAction(currState, actions[i]); // change to child to calculate heuristic

		double h_value;
		if(env->IsGoalStored()) {
			h_value = env->HCost(currState);
		}
		else {
			h_value = env->HCost(currState, goal);
		}

		h_values[i] = h_value;

		double g_value = g_stack.back() + edge_costs[i];
		assert(!fless(h_value, 0.0));
		assert(!fless(g_value, 0.0));

		env->ApplyAction(currState, inverted[i]); // reset state to parent

		costs[i] = get_cost(g_value, h_value);
		assert(!fless(costs[i], 0.0));
		actual_costs[i] = costs[i];

		// if node has already expanded, set the cost as the lower bound if should
		if(!fequal(lower_bound, parent_cost) && fgreater(lower_bound, costs[i]))
			costs[i] = lower_bound;

		// am I using bubble sort? Yeesh
		int j = i - 1;
		for(; j >= 0; j--) {
			// if the costs in list is infinite or new child is better
			if(costs[order[j]] == -1.0 || fless(costs[i], costs[order[j]])) {
				order[j + 1] = order[j]; // push child back
			}
			else {
				break;
			}
		}
		order[j+1] = i; // assign place where new child is to go
	}

	return actions.size() - num_invalid; // return number of children
}

template <class state, class action, class environment>
int GeneralRBFS<state, action, environment>::search_node(environment *env, state &currState, state &goal, double true_state_cost, double state_cost, double upper_bound, double &child_bound) {

	assert(g_stack.back() >= 0.0);
	assert(true_state_cost >= 0.0);
	assert(state_cost >= 0.0);
	assert(g_stack.back() == 0.0 || upper_bound >= 0.0);

	// counts with different iterations
	if(!fequal(current_threshold, -1.0) && fless(current_threshold, upper_bound)) {
		update_node_counts();
		current_threshold = upper_bound;
	}
	else if(fequal(current_threshold, -1.0) && !fequal(upper_bound, -1.0)) {
		current_threshold = upper_bound;
	}

	// check if node bounds are applicable
	if(bound_expanded && nodesExpanded + nodes_ex_iter >= expanded_limit) {
		child_bound = state_cost;
		return EXPAND_MET;
	}
	if(bound_generated && nodesGenerated + nodes_gen_iter >= generated_limit) {
		child_bound = state_cost;
		return TOUCHED_MET;
	}
	if(bound_checked && nodesChecked + nodes_check_iter >= checked_limit){
		child_bound = state_cost;
		return CHECKED_MET;
	}

	nodes_ex_iter++; // a new node expanded

	if (env->GoalTest(currState, goal)) { // do goal test
		current_best_path = active_path;
		best_path_cost = g_stack.back();
		sol_found = true;
		child_bound = g_stack.back();
		return 1; // found goal
	}
	// generate applicable actions
	std::vector<action> actions;
	std::vector<action> inverted;
	env->GetActions(currState, actions);
	nodes_gen_iter += actions.size();

	std::vector<unsigned> order(actions.size());
	std::vector<double> actual_child_costs(actions.size());
	std::vector<double> child_costs(actions.size());
	std::vector<double> edge_costs(actions.size());
	std::vector<double> h_values(actions.size());

	unsigned num_children = assign_and_sort(env, currState, goal, true_state_cost, state_cost, actions, inverted, order, actual_child_costs, child_costs, edge_costs, h_values);
	if(num_children == 0) { // no children, have hit a dead-end, child_bound is set as infinity
		child_bound = -1.0;
		return 0;
	}

	double my_upper_bound = 0.0;
	double my_child_bound = 0.0; // bound of child

	// while the best child is not infinite, and has a cost less than the upper bound
	while(!fequal(child_costs[order[0]], -1.0) && (!fgreater(child_costs[order[0]], upper_bound) || fequal(upper_bound, -1.0))) {

		unsigned best_index = order[0];
		env->ApplyAction(currState, actions[best_index]);

		// select correct upper_bound if more than 1 child
		if(num_children > 1 && (upper_bound == -1.0 || (fgreater(child_costs[order[1]], 0.0) && fless(child_costs[order[1]], upper_bound)))) {
			my_upper_bound = child_costs[order[1]]; // select best sibling cost as upper bound
		}
		else
			my_upper_bound = upper_bound; // select upper bound given by parent
		assert(upper_bound == -1.0 || !fgreater(my_upper_bound, upper_bound));

		g_stack.push_back(g_stack.back() + edge_costs[best_index]);
		h_stack.push_back(h_values[best_index]);
		active_path.push_back(actions[best_index]);
		inversion_stack.push_back(inverted[best_index]);

		// recursively search child
		int my_status = search_node(env, currState, goal,
		                            actual_child_costs[best_index],
		                            child_costs[best_index], my_upper_bound, my_child_bound);

		child_costs[best_index] = my_child_bound;

		env->ApplyAction(currState, inverted[best_index]); // reset state to parent
		g_stack.pop_back();
		h_stack.pop_back();
		active_path.pop_back();
		inversion_stack.pop_back();

		if(my_status != 0) { // if search must stop
			child_bound = my_child_bound;
			return my_status;
		}

		// fix position of searched child in order
		unsigned j = 1;
		for(; j < num_children; j++) {
			if(!fless(child_costs[best_index], child_costs[order[j]]) ||
			   (fequal(child_costs[best_index], -1.0) && !fequal(child_costs[order[j]], -1.0))) {
				order[j-1] = order[j];
			}
			else
				break;
		}
		order[j-1] = best_index; // puts best_index back in

	}
	child_bound = child_costs[order[0]]; // return cost of best child
	return 0; // no solution found
}
template <class state, class action, class environment>
bool GeneralRBFS<state, action, environment>::Initialize(environment *env, state from, state to) {
	/*
	prepare_vars_for_search();

	available_action_stack.resize(0);
	available_inversion_stack.resize(0);
	action_order.resize(0);
	actual_costs_stack.resize(0);
	child_costs_stack.resize(0);
	edge_costs_stack.resize(0);
	h_values_stack.resize(0);
	num_children_stack.resize(0);
	true_cost_stack.resize(0);
	state_cost_stack.resize(0);
	upper_bound_stack.resize(0);

	step_by_step_active = true;
	sol_found = false;

	my_env = env;
	my_goal = to;
	active_state = from;

	double h = my_env->HCost(active_state, my_goal);

	h_stack.push_back(h);
	g_stack.push_back(0.0);

	double initial_cost = get_cost(0.0, h);

	current_threshold = -1.0;

	true_cost_stack.push_back(initial_cost);
	state_cost_stack.push_back(initial_cost);
	upper_bound_stack.push_back(-1.0);

	std::vector<action> act;
	my_env->GetActions(active_state, act);
	inversion_stack.push_back(act[0]);
	*/
	return true;
}

template <class state, class action, class environment>
void GeneralRBFS<state, action, environment>::assert_stack_sizes_correct() {

	assert(active_path.size() == available_action_stack.size() &&
	       active_path.size() == available_inversion_stack.size() &&
	       active_path.size() == action_order.size() &&
	       active_path.size() == num_children_stack.size() &&
	       active_path.size() == actual_costs_stack.size() &&
	       active_path.size() == child_costs_stack.size() &&
	       active_path.size() == edge_costs_stack.size() &&
	       active_path.size() == h_values_stack.size() &&
	       active_path.size() == inversion_stack.size());
	assert(active_path.size() + 1 == g_stack.size() &&
	       g_stack.size() == h_stack.size());
}

template <class state, class action, class environment>
int GeneralRBFS<state, action, environment>::StepAlgorithm(std::vector<action> &thePath) {
	/*
	assert_stack_sizes_correct();

	if(!step_by_step_active) // if can't use step by step at this time
		return -1;

	nodes_ex_iter++; // a new node expanded
	if (my_env->GoalTest(active_state, my_goal)) { // do goal test
		current_best_path = active_path;
		best_path_cost = g_stack.back();
		sol_found = true;
		return 1; // found goal
	}

	std::vector<action> actions;
	std::vector<action> inverted;
	my_env->GetActions(active_state, actions);
	nodes_gen_iter += actions.size();

	std::vector<unsigned> order(actions.size());
	std::vector<double> actual_child_costs(actions.size());
	std::vector<double> child_costs(actions.size());
	std::vector<double> edge_costs(actions.size());
	std::vector<double> h_values(actions.size());

	unsigned num_children = assign_and_sort(my_env, active_state, my_goal, true_cost_stack.back(), state_cost_stack.back(), actions, inverted, order, actual_child_costs, child_costs, edge_costs, h_values);

	num_children_stack.push_back(num_children);
	available_action_stack.push_back(actions);
	available_inversion_stack.push_back(inverted);
	action_order.push_back(order);
	actual_costs_stack.push_back(actual_child_costs);
	child_costs_stack.push_back(child_costs);
	edge_costs_stack.push_back(edge_costs);
	h_values_stack.push_back(h_values);

	// if this node has an expandable child
	if(num_children > 0 &&!fequal(child_costs[order[0]], -1.0) && (!fgreater(child_costs[order[0]], upper_bound_stack.back()) || fequal(upper_bound_stack.back(), -1.0))) {
		double my_upper_bound = 0.0;
		unsigned best_index = order[0];

		my_env->ApplyAction(active_state, actions[best_index]);

		// select correct upper_bound if more than 1 child
		if(num_children > 1 && (upper_bound_stack.back() == -1.0 || (fgreater(child_costs[order[1]], 0.0) && fless(child_costs[order[1]], upper_bound_stack.back())))) {
			my_upper_bound = child_costs[order[1]]; // select best sibling cost as upper bound
		}
		else
			my_upper_bound = upper_bound_stack.back(); // select upper bound given by parent

		assert(upper_bound_stack.back() == -1.0 || !fgreater(my_upper_bound, upper_bound_stack.back()));

		g_stack.push_back(g_stack.back() + edge_costs[best_index]);
		h_stack.push_back(h_values[best_index]);
		active_path.push_back(actions[best_index]);
		inversion_stack.push_back(inverted[best_index]);

		true_cost_stack.push_back(actual_child_costs[best_index]);
		state_cost_stack.push_back(child_costs[best_index]);
		upper_bound_stack.push_back(my_upper_bound);

		return 0;
	}



	do {

		order.resize(0);
		actual_child_costs.resize(0);
		child_costs.resize(0);
		edge_costs.resize(0);
		h_values.resize(0);

	}while(num_children == 0 || fequal(child_costs[order[0]], -1.0) || (fgreater(child_costs[order[0]], upper_bound_stack.back()) && !fequal(upper_bound_stack.back(), -1.0)));
*/
	return 0;
}
#endif
