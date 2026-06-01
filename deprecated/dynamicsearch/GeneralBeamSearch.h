#ifndef GENBEAMSEARCH_H
#define GENBEAMSEARCH_H
#include "StatCollection.h"
#include "GenericStepAlgorithm.h"
#include <algorithm>
#include <unordered_map>
#include <iostream>
#include "BeamNode.h"
#include "StringUtils.h"
#include <assert.h>
#include <stdio.h>

#define NO_CHILDREN -100
#define LIM_HIT -200

/**
A class for performing regular beam search. The user sets a memory limit as a
number of nodes to hold in memory. If no goal is found by the time this limit
is reached, the search is said to fail.
**/
template <class state, class action, class environment>
class GeneralBeamSearch : public GenericStepAlgorithm<state, action, environment> {
public:
	GeneralBeamSearch() {
		beam_size = 1;
		memory_limit = 50;
		g_weight = 1.0; // weights don't matter if actions are constant cost
		h_weight = 1.0;
		step_by_step_active = false;

		best_path_cost = -1.0;

		prune_dups = false;
		full_check = false;
		debug = false;

		bound_expanded = false;
		bound_checked = false;
		bound_touched = false;
	}

	/** See GenericStepAlgorithm **/
	virtual int GetPath(environment *env, state from, state to, std::vector<state> &path);
	virtual int GetPath(environment *env, state from,
	            state to, std::vector<action> &path){return 0;}

	virtual const char * GetName();

	virtual uint64_t GetNodesExpanded() const { return nodes_expanded; }
	virtual uint64_t GetNodesTouched() const { return nodes_touched; }
	virtual uint64_t GetNodesChecked() {return nodes_checked; }

	virtual void LogFinalStats(StatCollection *stats){}

	virtual bool Initialize(environment *env, state start, state goal);

	virtual int StepAlgorithm(std::vector<action> &path){return 0;}
	virtual int StepAlgorithm(std::vector<state> &path);

	/** Changes the size of the beam. Default setting is 1. Returns true if succeeds, false otherwise. **/
	bool Change_Beam_Size(unsigned new_beam_size) {
		if(new_beam_size > 0) { beam_size = new_beam_size; return true; }
		return false;
	}
	/** Changes memory limit for beam search. That is, the number of nodes that can be in the beam
	 at one time. Default is set at 50.**/
	void Change_Memory_Limit(unsigned new_limit) {memory_limit = new_limit;}

	/** Sets a limit on the number of nodes to expand - input 0 for no limit **/
	bool SetExpandedLimit(uint64_t limit);

	/** Sets a limit on the number of nodes to generate - input 0 for no limit **/
	bool SetTouchedLimit(uint64_t limit);

	/** Sets a limit on the number of nodes to check - input 0 for no limit **/
	bool SetCheckedLimit(uint64_t limit);

	/**
	Checks if the algorithm is ready to be incremented one step
	**/
	bool Is_Step_Active() {return step_by_step_active;}

	/**
	Prints information on all items in the beams.
	**/
	virtual void print_beams();

	/** returns the path cost of the last path found **/
	double GetPathCost() {return best_path_cost; }

	/** Sets whether to remove duplicated nodes or not. If it is set to true, any node
	 that is in the beams cannot be added a second time to the latest beam. **/
	virtual void Select_Duplicate_Prune(bool to_prune) {prune_dups = to_prune;}

	/** If duplicates are being pruned, determines what type of check is performed
	 on the hash values. If to_check is set to true, the comparison of nodes
	 will not only compare hash values, but will also compare nodes that have
	 an identical hash value. If it is set to false, only hash values are compared.
	 to_check should only be set as true if there are expected to be hash
	 collisions.
	 **/
	void Select_Full_Check(bool to_check) {full_check = to_check;}

	/**
	Returns the value of the applicable weight
	**/
	double Get_H_Weight() { return h_weight;}
	double Get_G_Weight() { return g_weight;}

	/**
	Changes the weights. Note, if the problem is constant cost, changing the weights
	does nothing.
	**/
	int Change_Weights(double h_g,double h_w) {
		if(!fless (h_g, 0.0) && !fless(h_w, 0.0)) {
			h_weight = h_w;
			g_weight = h_g;
			return 1;
		}
		return 0;
	}

	bool Node_Stored_In_Beams(state &s, uint64_t hash_value);
	bool Node_Stored_In_Layer(state &s, uint64_t hash_value, std::vector<BeamNode<state> > &layer);

	void Add_To_Beam_Hash(BeamNode<state> &node, beam_position &position, std::unordered_map<uint64_t, std::vector<beam_position>, Hash64> &node_hash);

	void Add_To_Layer_Hash(BeamNode<state> &node, unsigned position, std::unordered_map<uint64_t, std::vector<unsigned>, Hash64> &node_hash);

	bool debug;

	virtual void count_new_nodes_expanded(unsigned new_nodes_ex){nodes_expanded+= new_nodes_ex;}
	virtual void count_new_nodes_touched(unsigned new_num){nodes_touched += new_num;};
	virtual void count_new_nodes_checked(unsigned new_nodes_ch){nodes_checked+= new_nodes_ch;};

	virtual bool hit_expanded_limit(){
		if(bound_expanded && nodes_expanded >= expanded_limit) {return true;}
		return false;
	}
	virtual bool hit_touched_limit() {
		if(bound_touched && nodes_touched >= touched_limit) {return true; }
		return false;
	}
	virtual bool hit_checked_limit(){
		if(bound_checked && nodes_checked >= checked_limit) {return true;}
		return false;
	}

protected:
	bool step_by_step_active;

	unsigned beam_size;
	unsigned memory_limit;
	uint64_t nodes_checked, nodes_expanded, nodes_touched;
	uint64_t expanded_limit, checked_limit, touched_limit;

	bool bound_expanded;
	bool bound_touched;
	bool bound_checked;

	bool prune_dups;

	/** Prepares the vars for search**/
	void prepare_vars_for_search();

	std::vector<std::vector <BeamNode<state> > > beams; // beam construct

	std::unordered_map<uint64_t, std::vector<unsigned>, Hash64> nodes_in_current_layer; // nodes in the current layer constructing
	std::unordered_map<uint64_t, std::vector<beam_position>, Hash64> nodes_in_beam; // nodes in beams not in current layer

	environment *my_env;
	state goal;

	/**
	Returns the cost for this state based on the g and h values.
	**/
	virtual double get_cost(double g, double h);

	/**
	Extracts the goal from the beams
	**/
	void extract_goal(std::vector<state> &state_path);

	double g_weight;
	double h_weight;

	unsigned beam_index;
	unsigned nodes_stored;

	double best_path_cost;

	int generate_all_successors(environment *env, std::vector<BeamNode<state> > &successors, state &goal);
	int expand_beam(environment *env, state &goal);
	void Merge_Beams(std::vector<BeamNode<state> > &new_nodes, std::vector<BeamNode<state> > &new_layer);

	bool full_check;

	void clear_memory_footprint();
};

template <class state, class action, class environment>
void GeneralBeamSearch<state, action, environment>::prepare_vars_for_search() {
	nodes_checked = 0;
	nodes_expanded = 0;
	nodes_touched = 0;

	nodes_in_current_layer.clear();
	beams.clear();
	nodes_in_beam.clear();
}

template <class state, class action, class environment>
void GeneralBeamSearch<state, action, environment>::clear_memory_footprint() {
	beams.clear();
	nodes_in_current_layer.clear();
	nodes_in_beam.clear();
}
/** Calculates weighted f-cost in standard way **/
template <class state, class action, class environment>
double GeneralBeamSearch<state, action, environment>::get_cost(double g, double h) {
	return g_weight*g + h_weight*h;
}

template <class state, class action, class environment>
bool GeneralBeamSearch<state, action, environment>::SetExpandedLimit(uint64_t limit) {
	if(limit > 0) {
		bound_expanded = true; expanded_limit = limit;
	}
	else {
		bound_expanded = false;
	}
	return true;
}

template <class state, class action, class environment>
bool GeneralBeamSearch<state, action, environment>::SetTouchedLimit(uint64_t limit) {
	if(limit > 0) {
		bound_touched = true; touched_limit = limit;
	}
	else {
		bound_touched = false;
	}
	return true;
}

template <class state, class action, class environment>
bool GeneralBeamSearch<state, action, environment>::SetCheckedLimit(uint64_t limit) {
	if(limit > 0) {
		bound_checked = true; checked_limit = limit;
	}
	else {
		bound_checked = false;
	}
	return true;
}

template <class state, class action, class environment>
int GeneralBeamSearch<state, action, environment>::GetPath(environment *env, state from, state to, std::vector<state> &path) {
	prepare_vars_for_search();

	double h_value;

	if(env->IsGoalStored()) {
		h_value = env->HCost(from);
	}
	else {
		h_value = env->HCost(from, to);
	}
	double initial_cost = get_cost(0.0, h_value);
	uint64_t initial_key = 0;
	if(prune_dups)
		initial_key = env->GetStateHash(from);

	// constructs initial beamnode
	BeamNode<state> start_node(from, 0.0, h_value, initial_cost, 0, initial_key);

	std::vector<BeamNode<state> > first_layer;
	first_layer.push_back(start_node);
	beams.push_back(first_layer); // puts first beam on the stack

	if(prune_dups) {
		beam_position starting;
		starting.beam_num = 0;
		starting.beam_pos = 0;

		Add_To_Beam_Hash(start_node, starting, nodes_in_beam);
	}

	int status = 0;

	nodes_stored = 1;
	count_new_nodes_touched(1);

	step_by_step_active = false;
	status = expand_beam(env, to);

	if(status == 1) {
		extract_goal(path);
	}
	clear_memory_footprint();
	return status;
}

template <class state, class action, class environment>
int GeneralBeamSearch<state, action, environment>::expand_beam(environment *env, state &goal) {

	if(debug) {
		print_beams();
	}
	std::vector<BeamNode<state> > successors;

	int status = generate_all_successors(env, successors, goal);

	if(status == 1) {
		// put goal on end of the beam stack
		assert(successors.size() == 1);
		beams.push_back(successors);
		return 1;
	}
	else if(status == LIM_HIT) { // hit node stored limit
		return 0;
	}
	else if(status != 0){ // next beam is empty
		return status;
	}

	// correct size of successors
	while(successors.size() > beam_size) {
		successors.pop_back();
	}

	assert(successors.size() <= beam_size);

	// if there are no children, search has failed
	if(successors.size() == 0) {
		return 0;
	}

	nodes_stored += successors.size();

	// put new successors on beam stack
	beams.push_back(successors);

	if(prune_dups) {
		unsigned succ_index = 0;
		while(succ_index < successors.size()) { // stores node if need be
			beam_position new_position;
			new_position.beam_num = beams.size() - 1;
			new_position.beam_pos = succ_index;
			Add_To_Beam_Hash(beams[beams.size() - 1][succ_index], new_position, nodes_in_beam);
			succ_index++;
		}
	}

	// recursively expand next beam
	status = expand_beam(env, goal);

	return status;
}


template <class state, class action, class environment>
bool GeneralBeamSearch<state, action, environment>::Initialize(environment *env, state from, state to) {

	/*
	prepare_vars_for_search();

	my_env = env;
	goal = to;

	double h_value;

	if(env->IsGoalStored()) {
		h_value = env->HCost(from);
	}
	else {
		h_value = env->HCost(from, goal);
	}
	double initial_cost = get_cost(0.0, h_value);

	uint64_t initial_key = 0;
	if(prune_dups)
		initial_key = my_env->GetStateHash(from);

	// constructs initial beamnode
	BeamNode<state> start_node(from, 0.0, h_value, initial_cost, 0,initial_key);

	std::vector<BeamNode<state> > first_layer;
	first_layer.push_back(start_node);
	beams.push_back(first_layer); // initiates beams

	beam_index = 0;
	step_by_step_active = true;

	nodes_stored = 1;
	nodes_in_beam.clear();

	if(prune_dups) {
		beam_position starting;
		starting.beam_num = 0;
		starting.beam_pos = 0;
		std::vector<beam_position> pos_vector;
		pos_vector.push_back(starting);
		nodes_in_beam[initial_key] = pos_vector;
	}
*/
	return true;
}

/*
template <class state, class action, class environment>
void GeneralBeamSearch<state, action, environment>::Merge_Beams(std::vector<BeamNode<state> > &new_nodes, std::vector<BeamNode<state> > &new_layer) {
	unsigned new_node_index = 0;
	unsigned old_beam_index = 0;

	nodes_stored -= beams[beams.size() -1].size(); // are to discard these nodes
	nodes_in_current_layer.clear();

	// performs a merge sort
	while(new_layer.size() < beam_size && (new_node_index < new_nodes.size() || old_beam_index < beams[beams.size() -1].size())) {

			// if new children layer is out of states
		if(new_node_index == new_nodes.size()) {
			new_layer.push_back(beams[beams.size() -1][old_beam_index]);
			old_beam_index++;
		}
			// if old beam is out of states
		else if(old_beam_index == beams[beams.size() -1].size()) {
			new_layer.push_back(new_nodes[new_node_index]);
			new_node_index++;
		}
		// if the next child in new children is best candidate
		else if(new_nodes[new_node_index] < beams[beams.size() -1][old_beam_index]) {
			new_layer.push_back(new_nodes[new_node_index]);
			new_node_index++;
		}
		else {
			new_layer.push_back(beams[beams.size() -1][old_beam_index]);
			old_beam_index++;
		}
	}

	if(prune_dups) {

		//unsigned position;
		//for(unsigned i = 0; i < beams[beams.size() -1].size(); i++) {
		//	position.beam_pos = i;
		//	Add_To_Layer_Hash(beams[beams.size() -1][i], position, nodes_in_current_layer);
		//}
	}

	nodes_stored += new_layer.size();
}*/

template <class state, class action, class environment>
int GeneralBeamSearch<state, action, environment>::StepAlgorithm(std::vector<state> &path) {
	/*

	if(!step_by_step_active) // if can't use step by step at this time
		return -1;

	unsigned layer_index;

	double max_fcost = -1.0;
	// figure out correct beam to select from, and set max_cost
	if(beam_index == 0) { // will need to create a new beam because are starting anew
		layer_index = beams.size() - 1;
		nodes_in_current_layer.clear();
	}
	else {
		layer_index = beams.size() - 2; // still constructing last beam
		if(beams[beams.size() - 1].size() == beam_size)
			max_fcost = beams[beams.size() - 1][beam_size - 1].cost;
	}

	BeamNode<state> *parent = &beams[layer_index][beam_index];
	std::vector<state> children;
	my_env->GetSuccessors(parent->my_state, children); // get successors of state

	nodes_expanded++;
	nodes_touched += children.size();

	if(bound_expanded && nodes_expanded >= expanded_limit) {
		step_by_step_active = false;
		return EXPAND_MET;
	}

	if(bound_touched && nodes_touched >= touched_limit) {
		step_by_step_active = false;
		return TOUCHED_MET;
	}

	std::vector<BeamNode<state> > children_beam; // new beam for children

	for(unsigned i = 0; i < children.size(); i++) {

		// prevents inversion of moves
		if(layer_index > 0 &&
		   children[i] == beams[layer_index - 1][parent->parent_index].my_state) {
			continue;
		}

		uint64_t child_key = 0;

		// if are to prune duplicates
		if(prune_dups) {
			child_key = my_env->GetStateHash(children[i]);

			if(Node_Stored_In_Beams(children[i], child_key))
				continue;
		}

		// if have found the goal, stop and return the answer
		if(my_env->GoalTest(children[i], goal)) {
			double child_g = parent->g_value + my_env->GCost(parent->my_state, children[i]);

			double child_cost = get_cost(child_g, 0);

			BeamNode<state> goal(children[i], child_g, 0, child_cost, beam_index, child_key);
			children_beam.clear(); // will be only node put on children_beam
			children_beam.push_back(goal);

			if(layer_index != beams.size() - 1)
				beams.pop_back();

			beams.push_back(children_beam); // first beam just holds start, last beam just holds goal
			extract_goal(path); // find the path

			best_path_cost = child_g;
			step_by_step_active = false;
			return 1; // have now found a goal
		}

		nodes_checked++;

		if(bound_checked && nodes_checked >= checked_limit) {
			step_by_step_active = false;
			return CHECKED_MET;
		}

		double child_g = parent->g_value + my_env->GCost(parent->my_state, children[i]);
		double child_h;

		// calculate h
		if(my_env->IsGoalStored())
			child_h = my_env->HCost(children[i]);
		else
			child_h = my_env->HCost(children[i], goal);

		double child_cost = get_cost(child_g, child_h);

		if(max_fcost >= 0 && child_cost >= max_fcost) // can skip child because cost is too big
			continue;

		// construct new BeamNode
		BeamNode<state> new_node(children[i], child_g, child_h, child_cost, beam_index, child_key);

		children_beam.push_back(new_node);


//printf("Candidate: ");
//	std::cout << new_node.my_state;
//	printf(" %.0f %.0f %.0f\n", new_node.g_value, new_node.h_value, new_node.cost);
	}

	// sorts children
	sort(children_beam.begin(), children_beam.end());

	if(beam_index == 0) { // add new layer to beams if is first of new beam
		while(children_beam.size() > beam_size) { // get rid of back of beam
			children_beam.pop_back();
		}
		beams.push_back(children_beam);

		nodes_stored += children_beam.size();

		if(nodes_stored >= memory_limit) {
			step_by_step_active = false;
			return NO_SOLUTION;
		}
	}
	else { // beam already full or partially full, need to merge lists
		std::vector<BeamNode<state> > new_layer;

		Merge_Beams(children_beam, new_layer);

		if(nodes_stored >= memory_limit) {
			step_by_step_active = false;
			return -5;
		}

		beams.pop_back(); // remove the old beam from the stack
		beams.push_back(new_layer); // add the fixed layer to the stack
	}

	// fix value of beam_index
	beam_index++;
	if(beam_index == beams[layer_index].size()) { // have expanded entire level
		beam_index = 0;

		// new beam is now officially to be added
		if(prune_dups) {
			for(unsigned i = 0; i < beams[beams.size()-1].size(); i++) {
				beam_position position;
				position.beam_num = beams.size() - 1;
				position.beam_pos = i;
				Add_To_Beam_Hash(beams[beams.size()-1][i], position, nodes_in_beam);
			}
		}
	}*/
	return 0;
}

template <class state, class action, class environment>
void GeneralBeamSearch<state, action, environment>::print_beams() {
	printf("\n\n");
	for(unsigned i = 0; i < beams.size(); i++) {
		printf("Beam %d\n", i);
		for(unsigned j = 0; j < beams[i].size(); j++) {

			if((j == 0 && beam_index == 0 && i == beams.size() - 1) ||
			   (beam_index != 0 && beam_index == j && i == beams.size() - 2)) {
				printf("Expanding Next: ");
			}
			std::cout << beams[i][j].my_state;
			printf(" G:%.0f H:%.0f Cost:%.0f\n", beams[i][j].g_value, beams[i][j].h_value, beams[i][j].cost);
		}
	}
}

template <class state, class action, class environment>
void GeneralBeamSearch<state, action, environment>::extract_goal(std::vector<state> &state_path) {
	state_path.resize(beams.size()); // properly resizes beam, last spot will have final node added elsewhere

	int beam_layer = beams.size() - 1;
	unsigned current_index = beams[beam_layer].size() - 1;

	while(beam_layer >= 0) { // extract states from beam
		state_path[beam_layer] = beams[beam_layer][current_index].my_state;
		current_index = beams[beam_layer][current_index].parent_index;
		beam_layer--;
	}
}

// assumes prune_dups check has been applied elsewhere
template <class state, class action, class environment>
bool GeneralBeamSearch<state, action, environment>::Node_Stored_In_Beams(state &s, uint64_t hash_value) {

	assert(prune_dups);

	// if is already have a state with that hash value
	if(nodes_in_beam.find(hash_value) != nodes_in_beam.end()) {

		if(full_check) { // not trusting hash function
			// iterate through all nodes with that hash value
			for(unsigned i = 0; i < nodes_in_beam[hash_value].size(); i++) {
				beam_position b = (nodes_in_beam[hash_value])[i];

				assert(b.beam_num < beams.size());
				assert(b.beam_pos < beams[b.beam_num].size());

				// if have identical state, node is stored
				if(beams[b.beam_num][b.beam_pos].my_state == s) {
					return true;
				}
			}

			// is a unique node
			return false;

		}
		// if are not to do full check, just basing it on hash value
		return true;
	}
	return false;
}

// assumes prune_dups check has been applied elsewhere
template <class state, class action, class environment>
bool GeneralBeamSearch<state, action, environment>::Node_Stored_In_Layer(state &s, uint64_t hash_value, std::vector<BeamNode<state> > &layer) {

	assert(prune_dups);
	if(nodes_in_current_layer.find(hash_value) != nodes_in_current_layer.end()) {

		if(full_check) { // not trusting hash function
			// iterate through all nodes with that hash value
			for(unsigned i = 0; i < nodes_in_current_layer[hash_value].size(); i++) {
				unsigned pos = (nodes_in_current_layer[hash_value])[i];

				// if have identical state, node is stored
				if(layer[pos].my_state == s) {
					return true;
				}
			}

			// is a unique node
			return false;

		}
		// if are not to do full check, just basing it on hash value
		return true;
	}
	return false;
}
/**
Adds a node to a hash. Assumes that this node is not already in the hash function
even if the hash value does exist there.
**/
template <class state, class action, class environment>
void GeneralBeamSearch<state, action, environment>::Add_To_Beam_Hash(BeamNode<state> &node, beam_position &position, std::unordered_map<uint64_t, std::vector<beam_position>, Hash64> &node_hash) {

	assert(prune_dups);
	if(full_check && node_hash.find(node.my_key) != node_hash.end()) {
		node_hash[node.my_key].push_back(position);
	}
	else {
		std::vector<beam_position> pos_vector;
		pos_vector.push_back(position);
		node_hash[node.my_key] = pos_vector;
	}
}

/**
Adds a node to a hash. If full_check is set to true
**/
template <class state, class action, class environment>
void GeneralBeamSearch<state, action, environment>::Add_To_Layer_Hash(BeamNode<state> &node, unsigned position, std::unordered_map<uint64_t, std::vector<unsigned>, Hash64> &node_hash) {

	assert(prune_dups);
	if(full_check && node_hash.find(node.my_key) != node_hash.end()) {
		node_hash[node.my_key].push_back(position);
	}
	else {
		std::vector<unsigned> pos_vector;
		pos_vector.push_back(position);
		node_hash[node.my_key] = pos_vector;
	}
}
/**
Generates all the successors of the nodes in the beam with deepest depth.
**/
template <class state, class action, class environment>
int GeneralBeamSearch<state, action, environment>::generate_all_successors(environment *env, std::vector<BeamNode<state> > &successors, state &goal) {

	nodes_in_current_layer.clear(); // nodes in current successors list
	successors.clear();

	std::vector<state> children;
	assert(beams.size() > 0);
	assert(beams.back().size() > 0);

	// iterate through states in last layer of beam
	for(unsigned i = 0 ; i < beams[beams.size() - 1].size(); i++) {
		count_new_nodes_expanded(1);
		// if hit expansion limit
		if(hit_expanded_limit()) {
			return EXPAND_MET;
		}

		children.clear();

		// generate children of current item in beam
		env->GetSuccessors(beams[beams.size() - 1][i].my_state, children);

		count_new_nodes_touched(children.size());

		// if hit generated limit
		if(hit_touched_limit()) {
			return TOUCHED_MET;
		}

		// iterate through generated child
		for(unsigned j = 0; j < children.size(); j++) {

			// Checks for inversion
			if(beams.size() > 1 && children[j] == beams[beams.size() - 2][beams[beams.size() - 1][i].parent_index].my_state)
				continue;

			uint64_t child_key = 0;

			if(prune_dups) {
				child_key = env->GetStateHash(children[j]);

				// if already have the node stored in beam or in successors
				if(Node_Stored_In_Beams(children[j], child_key) ||
				   Node_Stored_In_Layer(children[j], child_key, successors))
				   continue;
			}

			double child_g = beams[beams.size() - 1][i].g_value +
				env->GCost(beams[beams.size() - 1][i].my_state, children[j]);
			double child_h;

			if(env->IsGoalStored()) {
				child_h = env->HCost(children[j]);
			}
			else {
				child_h = env->HCost(children[j], goal);
			}

			double child_cost = get_cost(child_g, child_h);

			// construct new BeamNode
			BeamNode<state> new_node(children[j], child_g, child_h, child_cost, i, child_key);
			successors.push_back(new_node); // add to list of successors

			count_new_nodes_checked(1);

			// if hit nodes checked limit
			if(hit_checked_limit()) {
				return CHECKED_MET;
			}

			// if have found a goal
			if(env->GoalTest(children[j], goal)) {
				successors.clear();
				successors.push_back(new_node);
				best_path_cost = child_g;
				return 1;
			}


			if(nodes_stored + successors.size() >= memory_limit) {
				successors.clear();
				return LIM_HIT;
			}

			// label node as stored in current layer
			if(prune_dups) {
				unsigned position = successors.size() - 1;
				Add_To_Layer_Hash(new_node, position, nodes_in_current_layer);
			}
		}
	}

	// sorts children
	sort(successors.begin(), successors.end());
	if(debug) {
		printf("Successors: \n");
		for(unsigned i = 0; i < successors.size(); i++) {
			std::cout << successors[i].my_state;
			printf(" G:%.0f H:%.0f Cost:%.0f\n", successors[i].g_value, successors[i].h_value, successors[i].cost);
		}
	}
	return 0; // no goal found and no limits hit
}

template <class state, class action, class environment>
const char * GeneralBeamSearch<state, action, environment>::GetName(){
	std::string name = "Beam Search(Beam Width = ";
	name += int_to_string(beam_size);
	name += ", Memory Limit = ";
	name += int_to_string(memory_limit);
	name += ", Prune Duplicates = ";
	if(prune_dups)
		name += "true, Full Check = ";
	else
		name += "false, Full Check = ";
	if(full_check)
		name += "true)";
	else
		name += "false)";
	return name.c_str();
}
#endif
