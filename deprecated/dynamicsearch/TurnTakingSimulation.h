#ifndef TT_SIM_H
#define TT_SIM_H

#include "experiment_basics.h"

using namespace std;

class TurnTakingSimulation {

public:
	TurnTakingSimulation(){}
	TurnTakingSimulation(const char *input_file);

	static bool parse_input_files(const char *input_file,
	                         vector <string> &, vector<std::vector<double> > &,
	                         vector <vector<uint64_t> > &,
	                         vector <vector<uint64_t> > &,
	                         vector <vector<uint64_t> > &);

	static int output_turntaking_input_file(const char *filename, int num_probs, const char * output_file);

	virtual bool input_new_file(const char *input_file);
	void output_solver_names(vector<unsigned> &solvers_of_interest);

	void simulate(vector<unsigned> &puzzles_of_interest, vector<unsigned> &solvers_of_interest, vector<unsigned> &set_sizes, vector<unsigned> num_per_size, bool parallel, bool print_all_stats);

	int best_of_combo(vector<unsigned> &, vector<unsigned> &, unsigned);

	void turn_taking_on_weight_set(vector<unsigned> &puzzles_of_interest, vector<unsigned> &solvers_of_interest, vector<unsigned> &combo, uint64_t &total_combo_nodes, double &total_combo_cost, unsigned &combo_solved, bool parallel, bool print_histogram, bool print_all);

	void turn_taking_on_weight_set(vector<unsigned> &puzzles_of_interest, vector<unsigned> &solvers_of_interest, vector<unsigned> &combo, uint64_t &total_combo_nodes, double &total_combo_cost, unsigned &combo_solved, vector<uint64_t> &prob_expanded, vector<double> &sol_cost, bool parallel, bool print_histogram, bool print_all);

	void set_expand_bound(uint64_t _max);

protected:
	vector <string> solver_names;
	vector<vector<double> > costs;
	vector<vector<uint64_t> >nodes_expanded;
	vector<vector<uint64_t> >nodes_checked;
	vector<vector<uint64_t> >nodes_touched;

	unsigned num_solvers, num_problems;

	uint64_t max_expand;
};
#endif
