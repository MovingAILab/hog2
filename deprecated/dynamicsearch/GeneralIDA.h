#ifndef IDASTAR_H
#define IDASTAR_H

#include <iostream>
#include <stdio.h>
#include "SearchEnvironment.h"
#include "FPUtil.h"
#include "GenericStepAlgorithm.h"
#include "StringUtils.h"

/*
Class for a general inheritable IDA star like algorithm. The
depth-first search component of the algorithm should not need
to be re-implemented. However, the to_expand algorithm is the
main part of the algorithm that may need to be altered, as
different IDA* like algorithms will determine whether a node
should or should not be expanded based on different criteria.

The GetPath algorithm is also expected to be re-implemented for
each variant as different updates will need to be performed
upon each iteration. The search_node algorithm can also be
re-implemented, but it is not suggested.

Note, the search can be modified in may ways. A bound can be
placed on the number of nodes expanded, generated, and checked.
As well, it is possible to force the search to expand the entire
iteration even after having found a solution.
*/
template <class state, class action, class environment>
class GeneralIDA : public GenericStepAlgorithm<state, action, environment> {
public:
	/**
	General constructor. Sets all values as some defaults
	**/
	GeneralIDA() {
		bound_expanded = false;
		bound_generated = false;
		bound_checked = false;
		expand_full_iter = false;
		step_by_step_active = false;
		h_weight = 1.0;
		g_weight = 1.0;
		reverse_order = false;
	}

	/**
	Constructor by which all values are set. Specifically the weights, bounds on the number of
	nodes to expand, generate, and check, and whether or not to expand the full iteration.
	**/
	GeneralIDA(double _g_weight, double _h_weight, bool _bound_expanded, bool _bound_generated,
	           bool _bound_checked, bool _expand_full_iter) {
		h_weight = _h_weight;
		g_weight = _g_weight;
		step_by_step_active = false;
		bound_expanded = _bound_expanded;
		bound_generated = _bound_generated;
		bound_checked = _bound_checked;
		expand_full_iter = _expand_full_iter;

		reverse_order = false;
	}

	virtual ~GeneralIDA() {}

	/**
	Finds a path from the node "from" to the node "to" in the search space using
	an IDA* like algorithm and stores the set of actions in thePath
	**/
	virtual int GetPath(environment *env, state from, state to,
	             std::vector<action> &thePath);

	virtual int GetPath(environment *env, state from, state to,
	                    std::vector<state> &thePath);

	virtual const char * GetName(){
		std::string name = "WIDA*(H Weight = ";
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

	/** returns the path cost of the last path found **/
	double GetPathCost() {return best_path_cost; }

	/** Sets a limit on the number of nodes to expand - input 0 for no limit **/
	virtual bool SetExpandedLimit(uint64_t limit);

	/** Sets a limit on the number of nodes to generate - input 0 for no limit **/
	virtual bool SetTouchedLimit(uint64_t limit);

	/** Sets a limit on the number of nodes to check - input 0 for no limit **/
	virtual bool SetCheckedLimit(uint64_t limit);

	/** Sets whether the entire iteration is to be expanded or not **/
	void SetExpandFullIteration(bool ex) {expand_full_iter = ex;}

	/** Initializes the search to perform in a step by step manner for the given
	 environment, starting at the given initial state and the given goal
	 **/
	bool Initialize(environment *env, state from, state to);

	unsigned GetNumIters() {return iter_checked.size();}
	/**
	Performs one step of the search. If a solution is found, it is stored in
	thePath. One step is defined as testing whether a node is to be expanded,
	and expanding if necessary. The return value defines the status.

	Returns 0 if no solution is found by the previous step.
	Returns 1 if a solution is found by that step and the search is to conclude.
	Returns 2 if ran out of nodes to expand
	Returns 3 if ran out of nodes to generate
	Returns 4 if ran out of nodes to check
	Returns -1 if a solution is found but the whole iteration has not finished searching.
	**/
	int StepAlgorithm(std::vector<action> &thePath);
	int StepAlgorithm(std::vector<state> &thePath) {
		fprintf(stderr, "ERROR: Step Algorithm returning states not implemented for GeneralIDA*\n");
		exit(1);}

	/**
	Returns a vector with the number of nodes checked, expanded, and generated at each iteration.
	**/
	std::vector<uint64_t> Get_Checked_Iters() {return iter_checked;}
	std::vector<uint64_t> Get_Expanded_Iters() {return iter_expanded;}
	std::vector<uint64_t> Get_Generated_Iters() {return iter_generated;}

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

	/**
	Returns the value of the applicable weight
	**/
	double Get_H_Weight() { return h_weight;}
	double Get_G_Weight() { return g_weight;}

	/**
	Whether or not the step by step version of the algorithm is currently active.
	**/
	bool Is_Step_Active() {return step_by_step_active;}

	/**
	Cleans up after the Step by step version of the algorithm.
	**/
	virtual void End_Step_By_Step();

	/**
	Reverses the order of the operators during the step by step version of the algorithm.
	By default, the step by step will use the opposite ordering of the actions given by
	the domain. The inputted value should be set to true in order to correct for this.
	**/
	void SetReverseOrder(bool setting) {reverse_order = setting;}

protected:
	/**
	Searches a node which involves testing whether the node should
	be expanded, performing the goal test, generating successors,
	and recursively calling the algorithm on successors.

	Returns 0 if no solution is found in subtree, 1 if a solution is
	found, and a number larger than 1 for other statuses
	ie.
	-2 for ran out of nodes to expand
	-3 for ran out of nodes to generate
	-4 for rant out of nodes to check
	**/
	virtual int search_node(environment *env, state &currState, state &goal, action forbiddenAction, double edge_cost);
	virtual int search_node(environment *env, state &currState, state &goal, state &parent);

	virtual int check_bounds();

	double nextBound;
	double currentBound;

	std::vector<action> current_best_path;
	std::vector<state> current_best_state_path;
	double best_path_cost;

	/**
	Checks if a node should be expanded based on its g and h value.
	Also responsible for updating nextBound if need be.
	**/
	virtual bool to_expand(double g, double h);

	// initializes search parameters
	virtual void initialize_search(environment *, state, state);

	// initializes node counts
	virtual void update_node_counts();

	// initializes bounds
	virtual void update_bounds();

	uint64_t nodesExpanded, nodesGenerated, nodesChecked;
	uint64_t nodes_ex_iter, nodes_gen_iter, nodes_check_iter;
	uint64_t expanded_limit, generated_limit, checked_limit;

	bool bound_expanded;
	bool bound_generated;
	bool bound_checked;

	bool expand_full_iter;
	bool sol_found;

	std::vector<action> active_path;
	std::vector<state> active_state_path;
	std::vector<action> inversion_stack;
	std::vector<double> g_stack;
	std::vector<double> h_stack;

	std::vector<std::vector <action> > depth_first_stack;

	state active_state;
	bool step_by_step_active;

	std::vector<uint64_t> iter_checked;
	std::vector<uint64_t> iter_generated;
	std::vector<uint64_t> iter_expanded;

	double h_weight;
	double g_weight;

	bool reverse_order;
	environment *my_env;
	state my_goal;
};

template <class state, class action, class environment>
void GeneralIDA<state, action, environment>::initialize_search(environment *env, state from, state to) {
	nodesExpanded = 0;
	nodesGenerated = 0;
	nodesChecked = 0;

	current_best_path.resize(0);
	current_best_state_path.resize(0);
	active_path.resize(0);
	active_state_path.resize(0);

	h_stack.resize(0);
	g_stack.resize(0);
	best_path_cost = -1.0;

	double h;
	if(env->IsGoalStored()) {
		h = env->HCost(from);
	}
	else {
		h = env->HCost(from, to);
	}

	// push on stats for initial state
	h_stack.push_back(h);
	g_stack.push_back(0.0);

	nextBound = h_weight*(h);
	currentBound = nextBound;

	sol_found = false;

	nodes_ex_iter = 0;
	nodes_gen_iter = 0;
	nodes_check_iter = 1;

	iter_checked.resize(0);
	iter_generated.resize(0);
	iter_expanded.resize(0);
}

template <class state, class action, class environment>
void GeneralIDA<state, action, environment>::update_node_counts() {
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
void GeneralIDA<state, action, environment>::update_bounds() {
	assert(currentBound != nextBound);
	currentBound = nextBound;
}

template <class state, class action, class environment>
int GeneralIDA<state, action, environment>::GetPath(environment *env,
                                     state from, state to,
                                     std::vector<action> &thePath)
{
	initialize_search(env, from, to);

	std::vector<action> act;
	env->GetActions(from, act);
	int status = 0;

	step_by_step_active = false;

	while (status == 0)
	{
		status = search_node(env, from, to, act[0], 0);
		update_node_counts();
		update_bounds();
	}
	thePath = current_best_path;

	return status;
}

template <class state, class action, class environment>
int GeneralIDA<state, action, environment>::GetPath(environment *env, state from, state to, std::vector<state> &thePath){
	initialize_search(env, from, to);

	std::vector<state> succs;
	env->GetSuccessors(from, succs);
	int status = 0;

	step_by_step_active = false;
	while (status == 0)
	{
		status = search_node(env, from, to, succs[0]);
		update_node_counts();
		update_bounds();
	}
	thePath = current_best_state_path;

	return status;

}

template <class state, class action, class environment>
int GeneralIDA<state, action, environment>::check_bounds() {
	// check if node bounds are applicable
	if(bound_expanded && nodesExpanded + nodes_ex_iter >= expanded_limit)
		return EXPAND_MET;
	if(bound_generated && nodesGenerated + nodes_gen_iter >= generated_limit)
		return TOUCHED_MET;
	if(bound_checked && nodesChecked + nodes_check_iter >= checked_limit)
		return CHECKED_MET;

	return 1;
}

template <class state, class action, class environment>
int GeneralIDA<state, action, environment>::search_node(environment *env, state &currState, state &goal, action forbiddenAction, double edge_cost) {

	int bound_check = check_bounds();
	if(bound_check != 1)
		return bound_check;

	double h;
	if(env->IsGoalStored()) {
		h = env->HCost(currState);
	}
	else {
		h = env->HCost(currState, goal);
	}

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

	if (fequal(h, 0.0) && env->GoalTest(currState, goal)) {
		current_best_path = active_path;
		best_path_cost = g;
		sol_found = true;
		return 1; // found goal
	}

	h_stack.push_back(h);
	g_stack.push_back(g);

	// generate applicable actions
	std::vector<action> actions;
	env->GetActions(currState, actions);

	nodes_gen_iter += actions.size();

	int my_status = 0;

	int depth = active_path.size();

	for (unsigned int x = 0; x < actions.size(); x++)
	{
		// if is inversion of last action
		if ((depth != 0) && (actions[x] == forbiddenAction))
			continue;

		// use g + h to cut off areas of search where cannot find better solution
		if(expand_full_iter && sol_found && !fless(h + g, best_path_cost))
			return my_status;

		nodes_check_iter++;

		active_path.push_back(actions[x]);

		double edgeCost = env->GCost(currState, actions[x]);
		env->ApplyAction(currState, actions[x]);
		env->InvertAction(actions[x]);

		// recursively search child
		int status = search_node(env, currState, goal, actions[x], edgeCost);
		env->ApplyAction(currState, actions[x]);
		active_path.pop_back();

		// handle status
		if (status == 1) {
			if(expand_full_iter)
				my_status = 1;
			else {
				h_stack.pop_back();
				g_stack.pop_back();
				return 1;
			}
		}
		else if(status > 1) {
			h_stack.pop_back();
			g_stack.pop_back();
			return status;
		}
	}

	h_stack.pop_back();
	g_stack.pop_back();
	return my_status;
}

template <class state, class action, class environment>
int GeneralIDA<state, action, environment>::search_node(environment *env, state &currState, state &goal, state &parent) {

	int bound_check = check_bounds();
	if(bound_check != 1)
		return bound_check;

	double g = g_stack.back();
	double h = h_stack.back();

	/* For Debugging
	for(unsigned i = 0; i < active_state_path.size(); i++)
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

	if (fequal(h, 0.0) && env->GoalTest(currState, goal)) {
		current_best_state_path = active_state_path;
		best_path_cost = g;
		sol_found = true;
		return 1; // found goal
	}

	// generate successors
	std::vector<state> succs;
	env->GetSuccessors(currState, succs);

	nodes_gen_iter += succs.size();

	int depth = active_state_path.size();

	int my_status = 0;
	for (unsigned int x = 0; x < succs.size(); x++)
	{
		// if is parent, prune
		if ((depth != 0) && (succs[x] == parent))
			continue;

		// use g + h to cut off areas of search where cannot find better solution
		if(expand_full_iter && sol_found && !fless(h + g, best_path_cost))
			return my_status;

		nodes_check_iter++;

		double edgeCost = env->GCost(currState, succs[x]);

		// get h_value of child
		double child_h;
		if(env->IsGoalStored()) {
			child_h = env->HCost(succs[x]);
		}
		else {
			child_h = env->HCost(succs[x], goal);
		}

		// push child info on stacks
		h_stack.push_back(child_h);
		g_stack.push_back(g + edgeCost);
		active_state_path.push_back(succs[x]);

		// recursively search child
		int status = search_node(env, succs[x], goal, currState);

		// remove child info from stacks
		active_state_path.pop_back();
		h_stack.pop_back();
		g_stack.pop_back();

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
	assert(h_stack.size() == g_stack.size());
	assert(h_stack.size() - 1 == active_state_path.size());
	return my_status;
}

template <class state, class action, class environment>
bool GeneralIDA<state, action, environment>::SetExpandedLimit(uint64_t limit) {
	if(limit > 0) {
		bound_expanded = true; expanded_limit = limit;
	}
	else {
		bound_expanded = false;
	}
	return true;
}

template <class state, class action, class environment>
bool GeneralIDA<state, action, environment>::SetTouchedLimit(uint64_t limit) {
	if(limit > 0) {
		bound_generated = true; generated_limit = limit;
	}
	else {
		bound_generated = false;
	}
	return true;
}

template <class state, class action, class environment>
bool GeneralIDA<state, action, environment>::SetCheckedLimit(uint64_t limit) {
	if(limit > 0) {
		bound_checked = true; checked_limit = limit;
	}
	else {
		bound_checked = false;
	}
	return true;
}

template <class state, class action, class environment>
bool GeneralIDA<state, action, environment>::to_expand(double g, double h) {

	if (fgreater(g_weight*g + h_weight*h, currentBound))
	{
		if(!fgreater(nextBound, currentBound) || fless(g_weight*g + h_weight*h, nextBound)) {
			nextBound = g_weight*g + h_weight*h;
		}
		return false;
	}

	return true;
}

template <class state, class action, class environment>
bool GeneralIDA<state, action, environment>::Initialize(environment *env, state from, state to) {
	step_by_step_active = true;

	my_env = env;
	my_goal = to;
	active_state = from;

	inversion_stack.resize(0);
	depth_first_stack.resize(0);

	initialize_search(env, from, to);

	std::vector<action> act;
	env->GetActions(from, act);

	nodes_gen_iter += act.size();
	nodes_ex_iter += 1; // since opening expansion done here

	if (reverse_order) {
		std::vector<action> stack;

		while (act.size() > 0){
			stack.push_back(act.back());
			act.pop_back();
		}

		depth_first_stack.push_back(stack);
	}
	else {
		depth_first_stack.push_back(act);
	}

	return true;
}

template <class state, class action, class environment>
int GeneralIDA<state, action, environment>::StepAlgorithm(std::vector<action> &thePath) {
	if(!step_by_step_active) // if can't use step by step at this time
		return STEP_NOT_ACTIVE;

	// check if node bounds are applicable
	if(bound_expanded && nodesExpanded + nodes_ex_iter >= expanded_limit) {
		update_node_counts();
		step_by_step_active = false;
		if(sol_found) {
			thePath = current_best_path;
		}
		return EXPAND_MET;
	}
	if(bound_generated && nodesGenerated + nodes_gen_iter >= generated_limit) {
		update_node_counts();
		step_by_step_active = false;
		if(sol_found) {
			thePath = current_best_path;
		}
		return TOUCHED_MET;
	}
	if(bound_checked && nodesChecked + nodes_check_iter >= checked_limit) {
		update_node_counts();
		step_by_step_active = false;
		if(sol_found) {
			thePath = current_best_path;
		}
		return CHECKED_MET;
	}

	// time to start new iteration
	if(depth_first_stack.size() == 0) {

		update_node_counts();
		update_bounds();

		if(sol_found) { // if have already found a goal
			step_by_step_active = false;
			thePath = current_best_path;
			return 1;
		}

		// otherwise are starting new iteration
		std::vector<action> act;
		my_env->GetActions(active_state, act);
		nodes_gen_iter += act.size();

		if (reverse_order) {
			std::vector<action> stack;

			while (act.size() > 0){
				stack.push_back(act.back());
				act.pop_back();
			}

			depth_first_stack.push_back(stack);
		}
		else {
			depth_first_stack.push_back(act);
		}

		nodes_ex_iter += 1; // since opening expansion done here
	}

	assert(g_stack.size() == depth_first_stack.size());
	assert(h_stack.size() == depth_first_stack.size());
	assert(inversion_stack.size() == depth_first_stack.size() - 1);
	assert(active_path.size() == depth_first_stack.size() - 1);

	// get next action
	action to_apply = (depth_first_stack.back()).back();
	active_path.push_back(to_apply);

	// get new state info
	double edgeCost = my_env->GCost(active_state, to_apply);
	double g = edgeCost + g_stack.back();

	my_env->ApplyAction(active_state, to_apply);

	double h;
	if(my_env->IsGoalStored()) {
		my_env->HCost(active_state);
	}
	else {
		my_env->HCost(active_state, my_goal);
	}

	nodes_check_iter++;

	// check if node should be expanded (or if should cut off due to better path found thus far
	if((!expand_full_iter || !sol_found || fless(g + h, best_path_cost))
	   && to_expand(g, h)) {
		nodes_ex_iter++;

		// test if node is a goal - assumes heuristic is admissible
		if (fequal(h, 0.0) &&  my_env->GoalTest(active_state, my_goal)) {
			current_best_path = active_path;
			best_path_cost = g;
			sol_found = true;

			if(!expand_full_iter) {
				depth_first_stack.resize(0);
				update_node_counts();
				step_by_step_active = false;
				thePath = current_best_path;
				return 1; // found goal
			}

			// if expanding full iteration
			// remove last action from path and stack
			active_path.pop_back();
			(depth_first_stack.back()).pop_back();

			// undo last action
			my_env->InvertAction(to_apply);
			my_env->ApplyAction(active_state, to_apply);
		}
		else {

			// get applicable actions
			std::vector<action> new_actions;
			my_env->GetActions(active_state, new_actions);

			nodes_gen_iter += new_actions.size();

			// re-order them
			if (reverse_order) {
				std::vector<action> stack;

				while (new_actions.size() > 0){
					stack.push_back(new_actions.back());
					new_actions.pop_back();
				}

				depth_first_stack.push_back(stack);
			}
			else {
				depth_first_stack.push_back(new_actions);
			}

			// update stacks
			g_stack.push_back(g);
			h_stack.push_back(h);

			my_env->InvertAction(to_apply);
			inversion_stack.push_back(to_apply);

		}
	}
	else {
		// remove last action from path and stack
		active_path.pop_back();
		(depth_first_stack.back()).pop_back();

		// undo last action
		my_env->InvertAction(to_apply);
		my_env->ApplyAction(active_state, to_apply);
	}

	// eliminate returning to previous state
	if(inversion_stack.size() > 0 &&
	   (depth_first_stack.back()).back() == inversion_stack.back()) {
		   (depth_first_stack.back()).pop_back();
	   }
		// if is not last first level of stack and top level is empty or no solution
	// can be found at the current depth with a better cost than has been found thus far
	while(depth_first_stack.size() > 1 &&
	      ((depth_first_stack.back()).size() == 0) ||
	      (expand_full_iter && sol_found &&
	       !fless(g_stack.back() + h_stack.back(), best_path_cost))) {
		depth_first_stack.pop_back(); // remove empty level
		(depth_first_stack.back()).pop_back(); // remove explored action

		// fix stacks
		active_path.pop_back();
		g_stack.pop_back();
		h_stack.pop_back();

		// undo action
		my_env->ApplyAction(active_state, inversion_stack.back());
		inversion_stack.pop_back();

		// eliminate returning to previous state
		if(inversion_stack.size() > 0 &&
		   (depth_first_stack.back()).back() == inversion_stack.back()) {
			   (depth_first_stack.back()).pop_back();
		   }
	}

	// if stack is finished
	if(depth_first_stack.size() == 1 && (depth_first_stack.back()).size() == 0) {
		depth_first_stack.pop_back();
	}
	if(sol_found) // have not expanded entire iteration, but have found a solution
		return 2;
	return 0;
}

template <class state, class action, class environment>
void GeneralIDA<state, action, environment>::End_Step_By_Step() {
	if(!step_by_step_active)
		return;

	step_by_step_active = false;
	update_node_counts();
}
#endif

