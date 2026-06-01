#include "TurnTakingSimulation.h"

TurnTakingSimulation::TurnTakingSimulation(const char *input_file)
{
	if(!TurnTakingSimulation::parse_input_files(input_file, solver_names, costs, nodes_expanded, nodes_checked, nodes_touched)) {
		fprintf(stderr, "Reading file for turn taking simulation failed\n");
		exit(1);
	}
	num_solvers = solver_names.size();
}

bool TurnTakingSimulation::parse_input_files(const char *input_file, vector <string> &solver_names, vector<vector<double> > &costs, vector <vector<uint64_t> > &nodes_expanded, vector <vector<uint64_t> > &nodes_checked, vector <vector<uint64_t> > &nodes_touched) {

	ifstream ifs(input_file);

	string s;
	vector<string> tokens;

	if(ifs.fail()) {
		printf("Input File not found\n");
		assert(false);
		return 1;
	}

	unsigned num_probs, num_solvers;

	// get the number of problems
	getline(ifs, s);

	if(s.find("NUMPROBS") == string::npos) {
		fprintf(stderr, "NUMPROBS line missing NUMPROBS\n");
		return false;
	}

	tokens = split(s, '\t');
	if(tokens.size() != 2) {
		fprintf(stderr, "NUMPROBS line have incorrect # of parameters\n");
		return false;
	}
	num_probs = atoi(tokens[1].c_str());

	if(num_probs == 0) {
		fprintf(stderr, "No Problems\n");
		return false;
	}

	// get the number of solvers
	getline(ifs, s);
	if(s.find("NUMSOLVERS") == string::npos) {
		fprintf(stderr, "NUMSOLVERS line missing NUMSOLVERS\n");
		return false;
	}

	tokens = split(s, '\t');

	if(tokens.size() != 2) {
		fprintf(stderr, "NUMSOLVERS line have incorrect # of parameters\n");
		return false;
	}
	num_solvers = atoi(tokens[1].c_str());
	if(num_solvers == 0) {
		fprintf(stderr, "No Solvers\n");
		return false;
	}

	// gets the solver names
	getline(ifs, s);

	while(s.find("SOLVER") != string::npos) {
		solver_names.push_back(s);
		getline(ifs, s);
	}

	// construct the information holders
	vector<uint64_t> temp_node_holder;
	vector<double> temp_costs_holder;
	temp_node_holder.resize(num_probs);
	temp_costs_holder.resize(num_probs);

	nodes_expanded.clear(); nodes_checked.clear(); nodes_touched.clear(); costs.clear();
	for(unsigned i = 0; i < num_solvers; i++) {
		nodes_expanded.push_back(temp_node_holder);
		nodes_checked.push_back(temp_node_holder);
		nodes_touched.push_back(temp_node_holder);

		costs.push_back(temp_costs_holder);
	}



	// gets the expanded nodes
	if(s.find("EXPANDED") == string::npos) {
		fprintf(stderr, "EXPANDED line missing\n");
		return false;
	}

	unsigned count = 0;
	getline(ifs, s);
	while(!ifs.eof() && count < num_probs) {
		tokens.clear();
		tokens = split(s, '\t');

		if(tokens.size() != num_solvers) {
			fprintf(stderr, "%d line after EXPANDED is invalid\n", count);
			return false;
		}

		for(unsigned i = 0; i < tokens.size(); i++) {
			nodes_expanded[i][count] = (uint64_t) atof(tokens[i].c_str());
		}
		count++;
		getline(ifs, s); // get next line
	}

	if(ifs.eof()) {
		fprintf(stderr, "Hit end of file while extracting expanded nodes\n");
		return false;
	}

	if(count != num_probs) {
		fprintf(stderr, "Only %d expanded problems. Not enough.\n", count);
		return false;
	}



	// get checked nodes
	if(s.find("CHECKED") == string::npos) {
		fprintf(stderr, "CHECKED line missing\n");
		return false;
	}

	count = 0;
	getline(ifs, s);
	while(!ifs.eof() && count < num_probs) {
		tokens.clear();
		tokens = split(s, '\t');

		if(tokens.size() != num_solvers) {
			fprintf(stderr, "%d line after CHECKED is invalid\n", count);
			return false;
		}

		for(unsigned i = 0; i < tokens.size(); i++) {
			nodes_checked[i][count] = (uint64_t) atof(tokens[i].c_str());
		}
		count++;
		getline(ifs, s); // get next line
	}

	if(ifs.eof()) {
		fprintf(stderr, "Hit end of file while extracting checked nodes\n");
		return false;
	}

	if(count != num_probs) {
		fprintf(stderr, "Only %d checked problems. Not enough.\n", count);
		return false;
	}



	// get touched nodes
	if(s.find("TOUCHED") == string::npos) {
		fprintf(stderr, "TOUCHED line missing\n");
		return false;
	}

	count = 0;
	getline(ifs, s);
	while(!ifs.eof() && count < num_probs) {
		tokens.clear();
		tokens = split(s, '\t');

		if(tokens.size() != num_solvers) {
			fprintf(stderr, "%d line after TOUCHED is invalid\n", count);
			return false;
		}

		for(unsigned i = 0; i < tokens.size(); i++) {
			nodes_touched[i][count] = (uint64_t) atof(tokens[i].c_str());
		}
		count++;
		getline(ifs, s); // get next line
	}

	if(ifs.eof()) {
		fprintf(stderr, "Hit end of file while extracting touched nodes\n");
		return false;
	}

	if(count != num_probs) {
		fprintf(stderr, "Only %d touched problems. Not enough.\n", count);
		return false;
	}



	// get solution costs
	if(s.find("COSTS") == string::npos) {
		fprintf(stderr, "COSTS line missing\n");
		return false;
	}

	count = 0;
	getline(ifs, s);
	while(!ifs.eof()) {
		tokens.clear();
		tokens = split(s, '\t');

		if(tokens.size() != num_solvers) {
			fprintf(stderr, "%d line after COSTS is invalid\n", count);
			return false;
		}

		for(unsigned i = 0; i < tokens.size(); i++) {
			costs[i][count] = atof(tokens[i].c_str());
		}
		count++;
		getline(ifs, s); // get next line
	}

	if(count != num_probs) {
		fprintf(stderr, "Only %d costs problems. Not enough.\n", count);
		return false;
	}

	ifs.close();

	return true;
}

bool TurnTakingSimulation::input_new_file(const char *input_file) {
	costs.clear();
	nodes_expanded.clear();
	nodes_checked.clear();
	nodes_touched.clear();
	solver_names.clear();

	return TurnTakingSimulation::parse_input_files(input_file, solver_names, costs, nodes_expanded, nodes_checked, nodes_touched);
}

void TurnTakingSimulation::output_solver_names(vector<unsigned> &solvers_of_interest) {
	for(unsigned i = 0; i < solvers_of_interest.size(); i++) {
		assert(solvers_of_interest[i] < num_solvers);
	}

	for(unsigned i = 0; i < solvers_of_interest.size(); i++) {
		printf("%u %s\n", i, solver_names[solvers_of_interest[i]].c_str());
	}
}

int TurnTakingSimulation::best_of_combo(vector<unsigned> &solvers_of_interest, vector<unsigned> &combo, unsigned problem_num) {
	int best_index = -1;
	for(unsigned i = 0; i < combo.size(); i++) {

		//std::cout << nodes_expanded[solvers_of_interest[combo[i]]][problem_num] << "\n";
		if(nodes_expanded[solvers_of_interest[combo[i]]][problem_num] > 0 && // problem was solved
		   (best_index < 0 || nodes_expanded[solvers_of_interest[combo[i]]][problem_num] <
		    nodes_expanded[solvers_of_interest[combo[best_index]]][problem_num])) { // if best needs updating
			    best_index = i;
		    }
	}
	//std::cout << "\n";
	return best_index;
}

void TurnTakingSimulation::turn_taking_on_weight_set(vector<unsigned> &puzzles_of_interest, vector<unsigned> &solvers_of_interest, vector<unsigned> &combo, uint64_t &total_combo_nodes, double &total_combo_cost, unsigned &combo_solved, vector<uint64_t> &prob_expanded, vector<double> &prob_cost, bool parallel, bool print_histogram, bool print_all){

	prob_expanded.clear();
	prob_cost.clear();

	total_combo_nodes = 0;
	total_combo_cost = 0.0;
	combo_solved = 0;

	unsigned histogram[combo.size()];

	for(unsigned i = 0; i < combo.size(); i++) {
		histogram[i] = 0;
	}

	// iterate through all problems
	for(unsigned i = 0; i < puzzles_of_interest.size(); i++) {

		int best_index = best_of_combo(solvers_of_interest, combo, puzzles_of_interest[i]);

		if(best_index < 0) { // turn-taking did not solve this problem
			prob_expanded.push_back(0);
			prob_cost.push_back(0);
			continue;
		}

		combo_solved++;

		unsigned best_solver_index = solvers_of_interest[combo[best_index]];

		uint64_t current_nodes = nodes_expanded[best_solver_index][puzzles_of_interest[i]];
		double current_cost = costs[best_solver_index][puzzles_of_interest[i]];

		assert(current_nodes > 0);

		// gets number of nodes expanded
		uint64_t problem_nodes;

		if(parallel)
			problem_nodes = current_nodes;
		else
			problem_nodes = (current_nodes - 1)*combo.size() + best_index + 1;

		if(max_expand > 0 && problem_nodes > max_expand) {
			prob_expanded.push_back(0);
			prob_cost.push_back(0);
			continue;
		}

		prob_expanded.push_back(problem_nodes);
		prob_cost.push_back(current_cost);

		histogram[best_index]++;

		if(print_all) {
			printf("%.0f\t", current_cost);
			std::cout << problem_nodes << "\n";
		}
		total_combo_nodes += problem_nodes; // increment problem
		total_combo_cost += current_cost;
	}

	if(print_histogram) {
		for(unsigned i = 0; i < combo.size(); i++) {
			printf("%u\t%u\n", solvers_of_interest[combo[i]], histogram[i]);
		}
	}
}

void TurnTakingSimulation::turn_taking_on_weight_set(vector<unsigned> &puzzles_of_interest, vector<unsigned> &solvers_of_interest, vector<unsigned> &combo, uint64_t &total_combo_nodes, double &total_combo_cost, unsigned &combo_solved, bool parallel, bool print_histogram, bool print_all) {

	vector<uint64_t> prob_expanded;
	vector<double> prob_cost;

	turn_taking_on_weight_set(puzzles_of_interest, solvers_of_interest, combo, total_combo_nodes, total_combo_cost, combo_solved, prob_expanded, prob_cost, parallel, print_histogram, print_all);
}

void TurnTakingSimulation::simulate(vector<unsigned> &puzzles_of_interest, vector<unsigned> &solvers_of_interest, vector<unsigned> &set_sizes, vector<unsigned> num_per_size, bool parallel, bool print_all_stats) {

	num_solvers = solver_names.size();
	num_problems = nodes_expanded[0].size();

	// asserts values are fine
	for(unsigned i = 0; i < puzzles_of_interest.size(); i++) {
		assert(puzzles_of_interest[i] < num_problems);
	}

	for(unsigned i = 0; i < solvers_of_interest.size(); i++) {
		assert(solvers_of_interest[i] < num_solvers);
	}

	for(unsigned i = 0; i < set_sizes.size(); i++) {
		assert(set_sizes[i] <= solvers_of_interest.size());
	}

	assert(set_sizes.size() == num_per_size.size());

	double all_bests[set_sizes.size()][2]; // holds costs and nodes expanded
	double all_worsts[set_sizes.size()][2];
	double all_ratios[set_sizes.size()][3]; // holds best, av, worst
	double all_avs[set_sizes.size()][3];

	// holds total nodes for each individual parameter setting
	double total_nodes[solvers_of_interest.size()];
	double total_costs[solvers_of_interest.size()];
	unsigned total_solved[solvers_of_interest.size()];
	unsigned count = 0;

	for(unsigned i = 0; i < solvers_of_interest.size(); i++) {
		total_nodes[i] = 0.0;
		total_solved[i]= 0;
		total_costs[i] = 0.0;
	}

	// calculate total nodes
	for(unsigned i = 0; i < puzzles_of_interest.size(); i++) {
		for(unsigned j = 0; j < solvers_of_interest.size(); j++) {
			if(nodes_expanded[solvers_of_interest[j]][puzzles_of_interest[i]] > 0) {
				total_nodes[j] += nodes_expanded[solvers_of_interest[j]][puzzles_of_interest[i]];
				total_costs[j] += costs[solvers_of_interest[j]][puzzles_of_interest[i]];
				total_solved[j]++;
			}
		}
	}

	for(unsigned i = 0; i < solvers_of_interest.size(); i++) {
		printf("%u\t%.0f\t%.0f\n", i, total_costs[i], total_nodes[i]);
	}

	vector<unsigned> combo;
	vector<vector <unsigned> > best_combos;
	vector<vector <unsigned> > worst_combos;
	map<string, string> combo_map;

	vector<unsigned> all_best_ratio_solved, all_worst_ratio_solved, all_best_solved, all_worst_solved;
	unsigned combo_solved, best_nodes_index, current_nodes_index, best_ratio_solved = 0, worst_ratio_solved = 0, num_solved_best = 0, num_solved_worst = 0;
	uint64_t total_combo_nodes;
	double total_combo_cost;
	double best_ratio;
	double node_size_av = 0, cost_size_av = 0, ratio_size_av = 0, num_solved_av = 0;
	double node_size_best = 0, cost_size_best = 0, ratio_size_best = 0;
	double node_size_worst = 0, cost_size_worst = 0, ratio_size_worst = 0;

	unsigned size_count = 0;

	// do each desired size
	for(unsigned counter = 0; counter < set_sizes.size(); counter++) {
		combo_map.clear();

		unsigned size = set_sizes[counter];
		count = 0; // number of problems
		node_size_av = 0.0;
		cost_size_av = 0.0;
		ratio_size_av = 0.0;
		num_solved_av = 0.0;

		// for total per size
		while(true) {

			// get them by iterating through all possible combinations
			if(num_per_size[counter] == 0) {
				if(count == 0) {
					combo.resize(size);
					get_next_combo(combo, solvers_of_interest.size() - 1, true);
				}
				else if (!get_next_combo(combo, solvers_of_interest.size() - 1, false))
					break;

			}
			else { // get combinations randomly
				get_rand_permutation(combo, 0, solvers_of_interest.size() - 1, size);

				sort(combo.begin(), combo.end());
				string combo_key = "";

				// get combo hash value
				for (unsigned i = 0; i < size; i++) {
					combo_key.append(int_to_string(combo[i]));
					combo_key.push_back('_');
				}
				// if have already seen this combo
				if (combo_map.find(combo_key) != combo_map.end()) {
					continue;
				}

				combo_map[combo_key] = combo_key;
			}

			// print solvers in this combo
			if(print_all_stats) {
				for(unsigned i = 0; i < size; i++) {
					printf("%u ", solvers_of_interest[combo[i]]);
				}
			}
			best_nodes_index = combo[0];

			// find best_nodes and print combo
			for(unsigned i = 1; i < size; i++) {
				// get index of nodes for this combo
				current_nodes_index = combo[i];

				// if solved more problems or as many in fewer nodes
				if(total_solved[best_nodes_index] < total_solved[current_nodes_index] ||
				   (total_solved[best_nodes_index] == total_solved[current_nodes_index] &&
				    total_nodes[current_nodes_index] < total_nodes[best_nodes_index]))
					best_nodes_index = current_nodes_index;
			}
			turn_taking_on_weight_set(puzzles_of_interest, solvers_of_interest, combo, total_combo_nodes, total_combo_cost, combo_solved, parallel, false, false);

			best_ratio = (total_combo_nodes / total_nodes[best_nodes_index])*
				(total_solved[best_nodes_index] / combo_solved);

			if(print_all_stats) {
				printf(": %.0f\t", total_combo_cost);
				std::cout << total_combo_nodes << "\t";
				printf("%.4f\t%d\n", best_ratio, combo_solved);
				//printf(": %.0f\t%.0f\t%.4f\t%d\n", total_combo_cost, total_combo_nodes, best_ratio, combo_solved);
			}

			node_size_av += total_combo_nodes;
			cost_size_av += total_combo_cost;
			ratio_size_av += best_ratio;
			num_solved_av += combo_solved;

			// if is first combo
			if(count == 0) {
				node_size_best = total_combo_nodes;
				cost_size_best = total_combo_cost;
				ratio_size_best = best_ratio;

				node_size_worst = total_combo_nodes;
				cost_size_worst = total_combo_cost;
				ratio_size_worst = best_ratio;

				num_solved_best = combo_solved;
				num_solved_worst = combo_solved;
				best_ratio_solved = combo_solved;
				worst_ratio_solved = combo_solved;

				best_combos.push_back(combo);
				worst_combos.push_back(combo);
			}
			else { // is not first combo

				if(num_solved_best < combo_solved ||
				   (num_solved_best == combo_solved && node_size_best > total_combo_nodes)) {
					   node_size_best = total_combo_nodes;
					   cost_size_best = total_combo_cost;
					   num_solved_best = combo_solved;

					   best_combos.pop_back();
					   best_combos.push_back(combo);
				   }
				if(num_solved_best > combo_solved ||
				   (num_solved_best == combo_solved && node_size_worst < total_combo_nodes)) {
					   node_size_worst = total_combo_nodes;
					   cost_size_worst = total_combo_cost;
					   num_solved_worst = combo_solved;

					   worst_combos.pop_back();
					   worst_combos.push_back(combo);
				   }
				if(combo_solved > best_ratio_solved ||
				   (combo_solved == best_ratio_solved && ratio_size_best > best_ratio))
					ratio_size_best = best_ratio;

				if(combo_solved < best_ratio_solved ||
				   (combo_solved == worst_ratio_solved && ratio_size_worst < best_ratio))
					ratio_size_worst = best_ratio;
			}
			count++;
			if(num_per_size[counter] > 0 && count >= num_per_size[counter])
				break;
		}

		num_per_size[counter] = count;

		cost_size_av /= num_per_size[counter];
		node_size_av /= num_per_size[counter];
		ratio_size_av /= num_per_size[counter];
		num_solved_av /= num_per_size[counter];

		all_bests[size_count][0] = cost_size_best;
		all_bests[size_count][1] = node_size_best;

		all_worsts[size_count][0] = cost_size_worst;
		all_worsts[size_count][1] = node_size_worst;

		all_ratios[size_count][0] = ratio_size_best;
		all_ratios[size_count][1] = ratio_size_av;
		all_ratios[size_count][2] = ratio_size_worst;

		all_avs[size_count][0] = cost_size_av;
		all_avs[size_count][1] = node_size_av;
		all_avs[size_count][2] = num_solved_av;
		size_count++;

		all_best_ratio_solved.push_back(best_ratio_solved);
		all_worst_ratio_solved.push_back(worst_ratio_solved);
		all_best_solved.push_back(num_solved_best);
		all_worst_solved.push_back(num_solved_worst);
	}

	printf("Num Per Size\tCost of Best\tNodes of Best\tBest Solved\t");
	printf("Cost of Worst\tNodes of Worst\tWorst Solved\t");
	printf("Av Cost\tAv Nodes\tAv Solved\t");
	printf("Smallest Best Ratio\tAverage Ratio\tWorst Best Ratio\t\tBest Combo\t\tWorst Combo\n");
	for(unsigned i = 0; i < size_count; i++) {
		printf("%d\t%.0f\t%.0f\t%d\t", num_per_size[i], all_bests[i][0], all_bests[i][1], all_best_solved[i]);
		printf("%.0f\t%.0f\t%d\t", all_worsts[i][0], all_worsts[i][1], all_worst_solved[i]);
		printf("%.0f\t%.0f\t%.0f\t", all_avs[i][0], all_avs[i][1], all_avs[i][2]);
		printf("%.4f\t%.4f\t%.4f\t\t", all_ratios[i][0], all_ratios[i][1], all_ratios[i][2]);

		/*
		for(unsigned j = 0; j < best_combos[i].size() - 1; j++) {
			printf("%u, ", best_combos[i][j]);
		}
		printf("%u\t\t", best_combos[i][best_combos[i].size() - 1]);

		for(unsigned j = 0; j < worst_combos[i].size() - 1; j++) {
			printf("%u, ", worst_combos[i][j]);
		}
		printf("%u\n", worst_combos[i][worst_combos[i].size() - 1]);
		*/
		printf("\n");
	}
}

int TurnTakingSimulation::output_turntaking_input_file(const char *filename, int num_probs, const char * output_file) {

	// assorted files to output to
	FILE * of;
	of = fopen (output_file,"w");

	ifstream ifs(filename);

	if(ifs.fail()) {
		printf("output_exp_info failed in file read\n");
		return 1;
	}

	string s, temp, current_solver_info;
	vector<string> tokens;
	vector<string> solver_names;

	// table of nodes
	vector<vector<double> > ex_nodes;
	vector<vector<double> > ch_nodes;
	vector<vector<double> > t_nodes;
	vector<vector<double> > costs;

	// vector for current solver extracting info for
	vector<double> current_ex_nodes;
	vector<double> current_ch_nodes;
	vector<double> current_t_nodes;
	vector<double> current_costs;

	// total values for all solvers
	vector<double> total_ex_nodes;
	vector<double> total_ch_nodes;
	vector<double> total_t_nodes;


	vector<double> total_cost;
	vector<unsigned> total_solved;

	int prob_count = 0;
	int solver_count = 0;
	unsigned prob_solved_count = 0;
	int all_solve_count = 0;

	int min_solver = 0;
	int max_solver = 0;
	double all_ex_nodes = 0;
	double all_ch_nodes = 0;
	double all_t_nodes = 0;
	double all_cost = 0;

	// while still available lines to read
	while(!ifs.eof()) {
		getline(ifs, s);

		// new solver instance
		if(s.find("SOLVER") != string::npos) {
			//reset everything
			current_solver_info = s;
			prob_count = 0;
			current_ex_nodes.clear();
			current_ch_nodes.clear();
			current_t_nodes.clear();
			current_costs.clear();
			prob_solved_count = 0;
			continue;
		} // if is the TOTALS line for the solver
		else if(s.find("TOTALS") != string::npos) {

			// make sure have enough problems extracted
			assert(prob_count == num_probs);

			solver_names.push_back(current_solver_info);

			tokens.resize(0);
			tokens = split(s, '\t');

			assert(tokens.size() >= 3); // must be at least TOTALS\tCOST\tNODES_EXPANDED

			assert(tokens[0].find("TOTALS") != string::npos);
			total_cost.push_back(atof(tokens[1].c_str()));
			total_ex_nodes.push_back(atof(tokens[2].c_str()));

			// if also has nodes checked
			if(tokens.size() >= 4)
				total_ch_nodes.push_back(atof(tokens[3].c_str()));
			else
				total_ch_nodes.push_back(0);

			// if also have nodes touched
			if(tokens.size() >= 5)
				total_t_nodes.push_back(atof(tokens[4].c_str()));
			else
				total_t_nodes.push_back(0);

			// if is least num of nodes expanded so far
			if(solver_count == 0 || total_ex_nodes.back() < total_ex_nodes[min_solver])
				min_solver = solver_count;

			// if is amx num of nodes expanded so far
			if(total_ex_nodes.back() > total_ex_nodes[max_solver])
				max_solver = solver_count;

			// store number of probs solved
			total_solved.push_back(prob_solved_count);

			// totals
			all_ex_nodes += total_ex_nodes.back();
			all_ch_nodes += total_ch_nodes.back();
			all_t_nodes += total_t_nodes.back();
			all_cost += total_cost.back();
			all_solve_count += total_solved.back();

			// increment number of solvers
			solver_count++;

			// put current vectors on vector stack
			ex_nodes.push_back(current_ex_nodes);
			ch_nodes.push_back(current_ch_nodes);
			t_nodes.push_back(current_t_nodes);
			costs.push_back(current_costs);
			continue;
		}

		tokens.resize(0);

		tokens = split(s, '\t');

		if(tokens.size() == 0) // empty line
			continue;

		// has some info
		assert(prob_count < num_probs);
		prob_count++;
		if(tokens.size() == 1) { // problem wasn't solved
			assert(tokens[0].find("-1") != string::npos);

			current_ex_nodes.push_back(0);
			current_ch_nodes.push_back(0);
			current_t_nodes.push_back(0);
			current_costs.push_back(0);
			continue;
		}

		// if info to extract on this line
		assert(tokens.size() >= 2);
		current_costs.push_back(atof(tokens[0].c_str()));
		current_ex_nodes.push_back(atof(tokens[1].c_str()));

		// if line includes checked info
		if(tokens.size() >= 3)
			current_ch_nodes.push_back(atof(tokens[2].c_str()));
		else
			current_ch_nodes.push_back(0);

		// if line includes touched info
		if(tokens.size() >= 4)
			current_t_nodes.push_back(atof(tokens[3].c_str()));
		else
			current_t_nodes.push_back(0);

		prob_solved_count++; // increment number of problems solved

	}
	ifs.close();

	// output all information
	fprintf(of, "NUMPROBS\t%d\n", num_probs);
	fprintf(of, "NUMSOLVERS\t%u\n", solver_names.size());

	// output solver info
	for(unsigned i = 0; i < solver_names.size(); i++) {
		fprintf(of, "%s\n", solver_names[i].c_str());
		printf("%d %s\n", i, solver_names[i].c_str());
	}


	// write expanded info to file
	fprintf(of, "EXPANDED\n");
	for(int i = 0; i < num_probs; i++) {
		for(unsigned j = 0; j < ex_nodes.size(); j++) {
			fprintf(of, "%.0f\t", ex_nodes[j][i]);
		}
		fprintf(of, "\n");
	}

	// write checked info to file
	fprintf(of, "CHECKED\n");
	for(int i = 0; i < num_probs; i++) {
		for(unsigned j = 0; j < ch_nodes.size(); j++) {
			fprintf(of, "%.0f\t", ch_nodes[j][i]);
		}
		fprintf(of, "\n");
	}

	// write touched info to file
	fprintf(of, "TOUCHED\n");
	for(int i = 0; i < num_probs; i++) {
		for(unsigned j = 0; j < t_nodes.size(); j++) {
			fprintf(of, "%.0f\t", t_nodes[j][i]);
		}
		fprintf(of, "\n");
	}

	// write cost info to file
	fprintf(of, "COSTS\n");
	for(int i = 0; i < num_probs; i++) {
		for(unsigned j = 0; j < costs.size(); j++) {
			fprintf(of, "%.0f\t", costs[j][i]);
		}
		fprintf(of,"\n");
	}

	for(unsigned i = 0; i < total_ex_nodes.size(); i++) {
		printf("%.0f\t", total_cost[i]);
		printf("%.0f\t", total_ex_nodes[i]);
		printf("%.0f\t", total_ch_nodes[i]);
		printf("%.0f\t", total_t_nodes[i]);
		printf("%d\n", total_solved[i]);
	}

	printf("\nINDEX MIN NODES EX newline INDEX MAX NODES EX newline AVERAGES\n");
	printf("%u\t%.0f\t%.0f\t%.0f\t%.0f\t%d\n", min_solver, total_cost[min_solver], total_ex_nodes[min_solver], total_ch_nodes[min_solver], total_t_nodes[min_solver], total_solved[min_solver]);
	printf("%u\t%.0f\t%.0f\t%.0f\t%.0f\t%d\n", max_solver, total_cost[max_solver], total_ex_nodes[max_solver], total_ch_nodes[max_solver], total_t_nodes[max_solver], total_solved[max_solver]);
	printf("%.0f\t%.0f\t%.0f\t%.0f\t%d\n", all_cost / total_cost.size(), all_ex_nodes / total_cost.size(), all_ch_nodes / total_cost.size(), all_t_nodes / total_cost.size(), all_solve_count / total_cost.size());

	fclose(of);
	return 0;
}

void TurnTakingSimulation::set_expand_bound(uint64_t _max){
	max_expand = _max;
}
