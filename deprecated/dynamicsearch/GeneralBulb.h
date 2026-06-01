#ifndef BULB_H
#define BULB_H
#include "GeneralBeamSearch.h"

template <class state, class action, class environment>
class GeneralBulb: public GeneralBeamSearch<state, action, environment> {
	// variables will be using
	using GeneralBeamSearch<state, action, environment>::debug;
	using GeneralBeamSearch<state, action, environment>::beam_size;
	using GeneralBeamSearch<state, action, environment>::memory_limit;
	using GeneralBeamSearch<state, action, environment>::nodes_checked;
	using GeneralBeamSearch<state, action, environment>::nodes_expanded;
	using GeneralBeamSearch<state, action, environment>::nodes_touched;
	using GeneralBeamSearch<state, action, environment>::expanded_limit;
	using GeneralBeamSearch<state, action, environment>::checked_limit;
	using GeneralBeamSearch<state, action, environment>::touched_limit;
	using GeneralBeamSearch<state, action, environment>::bound_expanded;
	using GeneralBeamSearch<state, action, environment>::bound_touched;
	using GeneralBeamSearch<state, action, environment>::bound_checked;
	using GeneralBeamSearch<state, action, environment>::prune_dups;
	using GeneralBeamSearch<state, action, environment>::nodes_in_current_layer;
	using GeneralBeamSearch<state, action, environment>::nodes_in_beam;
	using GeneralBeamSearch<state, action, environment>::my_env;
	using GeneralBeamSearch<state, action, environment>::goal;
	using GeneralBeamSearch<state, action, environment>::full_check;
	using GeneralBeamSearch<state, action, environment>::beams;
	using GeneralBeamSearch<state, action, environment>::nodes_stored;
	using GeneralBeamSearch<state, action, environment>::get_cost;

	using GeneralBeamSearch<state, action, environment>::print_beams;
	using GeneralBeamSearch<state, action, environment>::generate_all_successors;
	using GeneralBeamSearch<state, action, environment>::clear_memory_footprint;
public:
	GeneralBulb() {
		initial_discrepancies = 0;
		discrepancies_increment = 1;
		bound_disc = false;
		max_disc = 0;
	}

	virtual const char * GetName();
	virtual void prepare_vars_for_search();
	virtual int GetPath(environment *env, state from, state to, std::vector<state> &path);

	virtual uint64_t GetNodesExpanded() const { return nodes_expanded + nodes_ex_iter; }
	virtual uint64_t GetNodesTouched() const { return nodes_touched + nodes_touch_iter; }
	virtual uint64_t GetNodesChecked() { return nodes_checked + nodes_check_iter; }

	void Set_Initial_Discrepancies(unsigned i_d) {
		if(i_d >= 0) initial_discrepancies = i_d;
		else fprintf(stderr, "Invalid Initial Discrepancies\n");
	}

	void Set_Discrepancies_Increment(unsigned d_i) {
		if(d_i > 0) discrepancies_increment = d_i;
		else fprintf(stderr, "Invalid Discrepancies Increment\n");
	}

	void Set_Max_Discrepancies(int max) {
		if(max >= 0 && (unsigned)max >= initial_discrepancies) {
			bound_disc = true;
			max_disc = (unsigned)max;
		}
		else {
			bound_disc = false;
		}

	}

protected:
	uint64_t nodes_ex_iter, nodes_touch_iter, nodes_check_iter;

	std::vector<uint64_t> iter_checked;
	std::vector<uint64_t> iter_touched;
	std::vector<uint64_t> iter_expanded;

	void update_node_counts();

	virtual void count_new_nodes_expanded(unsigned new_nodes_ex){nodes_ex_iter+= new_nodes_ex;}
	virtual void count_new_nodes_touched(unsigned new_num){nodes_touch_iter += new_num;};
	virtual void count_new_nodes_checked(unsigned new_nodes_ch){nodes_check_iter += new_nodes_ch;};

	virtual bool hit_expanded_limit(){
		if(bound_expanded && (nodes_expanded + nodes_ex_iter >= expanded_limit)) {return true;}
		return false;
	}
	virtual bool hit_touched_limit() {
		if(bound_touched && (nodes_touched + nodes_touch_iter >= touched_limit)) {return true; }
		return false;
	}
	virtual bool hit_checked_limit(){
		if(bound_checked && (nodes_checked + nodes_check_iter >= checked_limit)) {return true;}
		return false;
	}

	virtual void prepare_for_level_expansion();
	unsigned initial_discrepancies;
	unsigned discrepancies_increment;

	int BulbProbe(environment *env, unsigned discrepancies, state &goal);
	int next_slice(environment *env, std::vector<BeamNode<state> > &slice, unsigned &index, state &goal);
	unsigned max_depth; // maximum depth of the search in the last iteration

	void erase_last_beam_from_hash();

	bool bound_disc;
	unsigned max_disc;
};

template <class state, class action, class environment>
void GeneralBulb<state, action, environment>::prepare_vars_for_search() {
	// prepare regular beam search vars as well
	GeneralBeamSearch<state, action, environment>::prepare_vars_for_search();

	iter_expanded.clear();
	iter_touched.clear();
	iter_checked.clear();

	nodes_ex_iter = 0;
	nodes_touch_iter = 0;
	nodes_check_iter = 0;
}

template <class state, class action, class environment>
void GeneralBulb<state, action, environment>::update_node_counts() {
	nodes_expanded += nodes_ex_iter;
	nodes_touched += nodes_touch_iter;
	nodes_checked += nodes_check_iter;

	nodes_ex_iter = 0;
	nodes_touch_iter = 0;
	nodes_check_iter = 0;

	iter_checked.push_back(nodes_check_iter);
	iter_touched.push_back(nodes_touch_iter);
	iter_expanded.push_back(nodes_ex_iter);
}

template <class state, class action, class environment>
int GeneralBulb<state, action, environment>::GetPath(environment *env, state from, state to, std::vector<state> &path) {
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

	// construct first beam
	std::vector<BeamNode<state> > first_layer;
	first_layer.push_back(start_node);
	beams.push_back(first_layer);

	if(prune_dups) {
		beam_position start_position;
		start_position.beam_num = 0;
		start_position.beam_pos = 0;
		this->Add_To_Beam_Hash(start_node, start_position, nodes_in_beam);
	}

	unsigned discrepancies = initial_discrepancies;
	bool first = true;
	int status = 0;

	while(status == 0 && (!bound_disc || discrepancies <= max_disc)) {
		assert(beams.size() == 1);
		nodes_stored = 1;
		max_depth = 0;

		if(first){
			nodes_touch_iter = 1;
			first = false;
		}

		assert(beams.size() == 1);
		assert(nodes_in_beam.size() == 1);

		status = BulbProbe(env, discrepancies, to);

		update_node_counts();

		if(max_depth <= discrepancies) { // if have explored entire space of possible solutions
			status = NO_SOLUTION;
		}

		if(status == 0)
			discrepancies+= discrepancies_increment;

		if(debug)
			printf("Increasing Discrepancies\n");
	}

	if(status == 1) {
		this->extract_goal(path);
	}

	clear_memory_footprint();
	return status;

}
template <class state, class action, class environment>
void GeneralBulb<state, action, environment>::prepare_for_level_expansion() {
	if(beams.size() - 1 > max_depth) { // update max_depth
		max_depth = beams.size() - 1;
	}
}

template <class state, class action, class environment>
int GeneralBulb<state, action, environment>::BulbProbe(environment *env, unsigned discrepancies, state &goal) {

	prepare_for_level_expansion();

	std::vector<BeamNode<state> > slice; // slice to expand

	unsigned index = 0; // index in successors
	int status; // current status
	beam_position position;
	unsigned next_discrepancies; // number of discrepancies in recursive call

	if(discrepancies > 0) // if available discrepancies, start with one
		index = beam_size;

	do {
		if(debug) {
			print_beams();
			printf("discrepancies: %d, index: %d\n", discrepancies, index);
		}

		// get the next slice and index
		status = next_slice(env, slice, index, goal);

		if(status == NO_CHILDREN) { // if no children, time to backtrack
			slice.clear();
			status = 0; // indicate have failed search
			break;  // backtrack to previous level since no successors here
		}
		else if(status == 1) { // handle goal found
			assert(slice.size() == 1);
			beams.push_back(slice); //push slice onto beam
			slice.clear();
			return 1;
		}
		else if(status != 0) { // handle some status hit
			slice.clear();

			if(status == LIM_HIT){ // indicate must take another path
				if(debug)
					printf("Hit Memory Limit, Backtracking\n");
				return 0;
			}
			return status; // return this other status
		}

		if(prune_dups) {
			for(unsigned i = 0; i < slice.size(); i++) {
				position.beam_num = beams.size();
				position.beam_pos = i;
				this->Add_To_Beam_Hash(slice[i], position, nodes_in_beam);
			}
		}

		beams.push_back(slice); // add slice to beam
		nodes_stored += slice.size(); // have new nodes stored
		slice.clear();

		if(index != 0 && index <= beam_size) // first discrepancy is being expanded
			next_discrepancies = discrepancies;
		else
			next_discrepancies = discrepancies - 1;

		status = BulbProbe(env, next_discrepancies, goal); // recursive call

		if(status != 0) { // goal found or some limit hit
			break; // immediately return
		}

		if(prune_dups) {
			erase_last_beam_from_hash();
		}
		nodes_stored -= beams[beams.size() - 1].size();
		beams.pop_back(); // remove slice from beams
	}while(index == 0 || index > beam_size); // while still slices to check

	if(debug)
		printf("Backtracking\n");
	return status; // return the status
}

/**
If called with 0, will give first slice, and returned index will be greater than 1
but less than or equal to beam_size.
Otherwise returns slice starting at the given index, unless there is only a single
slice to be had (in which case, acts as if was called with 0).
Once the last slice is returned, the returned index will be 0.
**/
template <class state, class action, class environment>
int GeneralBulb<state, action, environment>::next_slice(environment *env, std::vector<BeamNode<state> > &slice, unsigned &index, state &goal) {
	slice.clear(); // holds the next slice, so clear

	std::vector<BeamNode<state> > successors;

	int status = generate_all_successors(env, successors, goal);

	if(status == 1) {// only return the goal
		slice.push_back(successors.back());
		successors.clear();
		return 1;
	}
	else if(status != 0) { // have hit some limit
		return status;
	}

	if(successors.size() == 0) // if there are no successors
		return NO_CHILDREN;

	unsigned succ_index = index;

	// if only one possible slice for children
	if(successors.size() <= beam_size)
		succ_index = 0;

	// build slice
	while(succ_index < successors.size() && slice.size() < beam_size) {
		slice.push_back(successors[succ_index]);
		succ_index++;
	}

	index = succ_index; // fix index
	// if this is last discrepancy and it is not the only slice, next slice should be first one

	if(succ_index == successors.size() && succ_index > beam_size)
		index = 0; // have taken all discrepancies

	successors.clear();
	return 0; // goal not found and no limits hit
}

template <class state, class action, class environment>
void GeneralBulb<state, action, environment>::erase_last_beam_from_hash() {
	for(int i = beams[beams.size() - 1].size() - 1; i >= 0; i--) {
		uint64_t hash_value = beams[beams.size() - 1][i].my_key;
				//need to erase key here
		assert(nodes_in_beam.find(hash_value) != nodes_in_beam.end());
		assert(nodes_in_beam[hash_value].back().beam_num == beams.size() - 1);
		assert(nodes_in_beam[hash_value].back().beam_pos == (unsigned) i);

		if(nodes_in_beam[hash_value].size() == 1)
			nodes_in_beam.erase(hash_value);
		else
			nodes_in_beam[hash_value].pop_back();
	}
}

template <class state, class action, class environment>
const char * GeneralBulb<state, action, environment>::GetName(){
	std::string name = "BULB(Beam Width = ";
	name += int_to_string(beam_size);
	name += ", Memory Limit = ";
	name += int_to_string(memory_limit);
	name += ", Prune Duplicates = ";
	if(prune_dups)
		name += "true, Full Check = ";
	else
		name += "false, Full Check = ";
	if(full_check)
		name += "true, Init Disc = ";
	else
		name += "false, Init Disc = ";
	name += int_to_string(initial_discrepancies);
	name += ", Disc Increment = ";
	name += int_to_string(discrepancies_increment);
	name += ", Max Disc = ";
	if(bound_disc)
		name += int_to_string(max_disc);
	else
		name += "false";
	name += ")";
	return name.c_str();
}
#endif
