#include <vector>
#include <stdio.h>
#include <sstream>
#include "Common.h"
#include "dynsearch.h"
#include "MNPuzzle.h"
#include "GeneralIDA.h"
#include "SortingIDA.h"
#include "RandomSortingIDA.h"
#include "GeneralRBFS.h"
#include "GeneralBeamSearch.h"
#include "GeneralBulb.h"
#include "experiment_basics.h"
#include "TurnTakingSimulation.h"
//#include "TurnTakingUCT.h"
#include "PancakePuzzle.h"
//#include "GenericTemplateAStar.h"
#include <sys/resource.h>
#include <errno.h>

using namespace std;

void handle_warnings() {
	MNPuzzleState start(4, 4);
	if(start == start) {
		cout << kUp;
		cout << start;
	}
}

int main(int argc, char** argv)
{

	struct rlimit limit;
	if (getrlimit(RLIMIT_STACK, &limit) != 0) {
		printf("getrlimit() failed with errno=%d\n", errno);
		exit(1);
	}
	//printf("The soft limit is %llu\n", limit.rlim_cur);
	//printf("The hard limit is %llu\n", limit.rlim_max);


	limit.rlim_cur = 83886080;
	limit.rlim_max = 83886080;
	if (setrlimit(RLIMIT_STACK, &limit) != 0) {
		printf("setrlimit() failed with errno=%d\n", errno);
		exit(1);
	}

	/************ Turn-taking experiments **************/

	// creates turn-taking test file

	//TurnTakingSimulation::output_turntaking_input_file("../../apps/dynamicsearch/input/rbfs/5x5_wrbfs_1000", 1000, "../../apps/dynamicsearch/input/rbfs/5x5_wrbfs_1000_tt");


	// puzzles to test on
	vector<unsigned> desired_puzzles;

	// Ruml Experiments
	/*
	desired_puzzles.push_back(26);
	desired_puzzles.push_back(43);
	desired_puzzles.push_back(44);
	desired_puzzles.push_back(9);
	desired_puzzles.push_back(1);
	desired_puzzles.push_back(27);
	desired_puzzles.push_back(5);
	desired_puzzles.push_back(32);
	desired_puzzles.push_back(31);
	desired_puzzles.push_back(17);
	desired_puzzles.push_back(12);
	desired_puzzles.push_back(14);
	desired_puzzles.push_back(37);
	desired_puzzles.push_back(41);
	desired_puzzles.push_back(36);
	desired_puzzles.push_back(2);
	desired_puzzles.push_back(18);
	desired_puzzles.push_back(38);
	desired_puzzles.push_back(7);
	desired_puzzles.push_back(11);
	desired_puzzles.push_back(42);
	desired_puzzles.push_back(3);
	desired_puzzles.push_back(22);
	desired_puzzles.push_back(29);
	desired_puzzles.push_back(19);
	desired_puzzles.push_back(39);
	desired_puzzles.push_back(35);
	desired_puzzles.push_back(23);
	desired_puzzles.push_back(16);
	desired_puzzles.push_back(12);
	desired_puzzles.push_back(34);
	desired_puzzles.push_back(25);
	desired_puzzles.push_back(0);
	desired_puzzles.push_back(24);
	desired_puzzles.push_back(15);
	desired_puzzles.push_back(13);
	desired_puzzles.push_back(21);
	desired_puzzles.push_back(8);
	desired_puzzles.push_back(6);
	desired_puzzles.push_back(20);
	desired_puzzles.push_back(28);
	desired_puzzles.push_back(4);
	*/

	for(unsigned i = 0; i < 1000; i++) {
		desired_puzzles.push_back(i);
	}

	unsigned total_solvers = 9;
	unsigned start = 0;
	unsigned inc = 1;
	vector<unsigned> desired_solvers;

	// solvers to test on
	for(unsigned i = start; i < start + inc*(total_solvers - 1) + 1; i+= inc) {
		desired_solvers.push_back(i);
	}

	// candidate set sizes to test
	vector<unsigned> set_sizes;
	for(unsigned i = 2; i <= total_solvers; i++) {
	//for(unsigned i = 2; i <= 2; i++) {
		//for(unsigned i = total_solvers; i <= total_solvers; i++) {
		set_sizes.push_back(i);
	}

	// num per candidate set size to test
	vector<unsigned> num_per_size;


	for(unsigned i = 2; i <= total_solvers; i++) {
	//for(unsigned i = 2; i <= 2; i++) {
		num_per_size.push_back(0);
	}

	// for size 24
	/*
	num_per_size.push_back(0);
	num_per_size.push_back(0);

	for(unsigned i = 4; i <= 20; i++) {
		num_per_size.push_back(10000);
	}
	num_per_size.push_back(0);
	num_per_size.push_back(0);
	num_per_size.push_back(0);
	num_per_size.push_back(0);
	*/

	/** Dovetailing Simulations **/

	// many simulations


	TurnTakingSimulation simulator( "../../apps/dynamicsearch/input/rbfs/5x5_wrbfs_1000_tt");
	//simulator.output_solver_names(desired_solvers);
	//simulator.simulate(desired_puzzles, desired_solvers, set_sizes, num_per_size, false, false);

	// single candidate set size

	vector<unsigned> combo;
	uint64_t total_combo_nodes;
	double total_combo_cost;
	unsigned combo_solved;
	for(unsigned i = 0; i < total_solvers; i++) {
		combo.push_back(i);
	}
	vector<uint64_t> prob_expanded;
	vector<double> prob_cost;
	simulator.turn_taking_on_weight_set(desired_puzzles, desired_solvers, combo, total_combo_nodes, total_combo_cost, combo_solved, prob_expanded, prob_cost, true, false, false);



	/** UCT Dovetailing **/
	/*
	// many simulations
	TurnTakingUCT tt_uct("../../apps/dynamicsearch/input/ida/5x5_ida_1000_weight_stats_tt");
	tt_uct.output_solver_names(desired_solvers);
	tt_uct.simulate_uct(desired_puzzles, desired_solvers, set_sizes, num_per_size, 0.01, false, false);

	// single size simulations
	//double total_cost, total_nodes;
	//unsigned num_solved;
	//tt_uct.uct_run(desired_puzzles, desired_solvers, 5, total_cost, total_nodes, num_solved, 10, true, false);
	//printf("%.0f\t%.0f\n", total_cost, total_nodes);
	*/

	/******************** MN PUZZLE EXPERIMENTS ********************/
	vector<MNPuzzleState> temp_mn_puzzles;
	//get_big_4x4_test_set(temp_mn_puzzles, 1000);
	//get_standard_test_set(temp_mn_puzzles, 100);
	//get_6x6_test_set(puzzles, 1);
	get_5x5_test_set(temp_mn_puzzles, 1000);

	unsigned num_cols = 5;
	unsigned num_rows = 5;

	std::vector<slideDir> my_op_order;
	// defines order to use
	my_op_order.push_back(kUp); my_op_order.push_back(kLeft); my_op_order.push_back(kRight); my_op_order.push_back(kDown);

	MNPuzzleState goal(num_cols, num_rows);

	// creates puzzle environment and stores the goal
	MNPuzzle mnp(num_cols, num_rows, my_op_order);
	mnp.StoreGoal(goal);

	// checks to make sure problems are solvable
	mnp.Validate_Problems(temp_mn_puzzles);
	for(unsigned i = 0; i < temp_mn_puzzles.size(); i++) {
		if(MNPuzzle::GetParity(temp_mn_puzzles[i]) != MNPuzzle::GetParity(goal)) {
			printf("Bad Puzzle %d\n", i);
			assert(false);
		}
	}

	// takes puzzles you want to solve
	unsigned begin_index = 0;
	unsigned end_index = 999;
	vector<MNPuzzleState> mn_puzzles;
	for(unsigned i = begin_index; i <= end_index; i++) {
		mn_puzzles.push_back(temp_mn_puzzles[i]);
	}

	// creates puzzle paths, may be needed by different algorithms
	vector<slideDir> path;
	vector<MNPuzzleState> state_path;

	/** MN Puzzle Solvers **/
	GeneralIDA<MNPuzzleState, slideDir, MNPuzzle> mn_ida;
	GeneralRBFS<MNPuzzleState, slideDir, MNPuzzle> mn_rbfs;
	GeneralBeamSearch<MNPuzzleState, slideDir, MNPuzzle> mn_beam_search;
	GeneralBulb<MNPuzzleState, slideDir, MNPuzzle> mn_bulb;
	//GenericTemplateAStar<MNPuzzleState, slideDir, MNPuzzle> mn_astar;

	/** For Ordering Experiments **/
	std::vector<std::vector<slideDir> > mnp_op_orders;
	std::vector<uint64_t> nodes_expanded;
	for(unsigned i = 0; i < 24; i++) {
		std::vector<slideDir> op_order = MNPuzzle::Get_Op_Order_From_Hash(i);
		mnp_op_orders.push_back(op_order);
	}

	/** Tested Beam Widths **/

	vector<unsigned> beam_sizes;
	beam_sizes.push_back(2);
	beam_sizes.push_back(3);
	beam_sizes.push_back(4);
	beam_sizes.push_back(5);
	beam_sizes.push_back(6);
	beam_sizes.push_back(7);
	beam_sizes.push_back(8);
	beam_sizes.push_back(9);
	beam_sizes.push_back(10);
	beam_sizes.push_back(12);
	beam_sizes.push_back(15);
	beam_sizes.push_back(20);
	beam_sizes.push_back(25);
	beam_sizes.push_back(30);
	beam_sizes.push_back(40);
	beam_sizes.push_back(50);
	beam_sizes.push_back(60);
	beam_sizes.push_back(75);
	beam_sizes.push_back(100);
	beam_sizes.push_back(125);
	beam_sizes.push_back(150);
	beam_sizes.push_back(175);
	beam_sizes.push_back(200);
	beam_sizes.push_back(250);
	beam_sizes.push_back(300);
	beam_sizes.push_back(400);
	beam_sizes.push_back(500);
	beam_sizes.push_back(750);
	beam_sizes.push_back(1000);

	/**
	IDA* MN Puzzle Weight Experiments
	**/
	/*
	for(double weight = 3.0; weight <= 3.0; weight+= 1) {
		//mn_ida.SetExpandedLimit(15);
		mn_ida.Change_Weights(1.0, weight);
		general_batch(&mnp, &mn_ida, mn_puzzles, true, ACTION_PATH);
		cout << "\n";
	}*/


	/** Random Sorting Order Experiments **/
	/*
	RandomSortingIDA<MNPuzzleState, slideDir, MNPuzzle> mn_rsort_ida;
	mn_rsort_ida.Change_Weights(1.0, 3.0);
	for(int seed = 10000; seed <= 10040; seed+= 21) {
		mn_rsort_ida.Change_Seed(seed);
		general_batch(&mnp, &mn_rsort_ida, mn_puzzles, true, ACTION_PATH);
		cout << "\n";
	}
	*/

	/**
	IDA* MN Order Experiments
	**/
	/*
	for(double weight = 3.0; weight <= 3.0; weight ++) {
		mn_ida.Change_Weights(1.0, weight);
		general_batch_orders(&mnp, &mn_ida, mnp_op_orders, mn_puzzles, true, ACTION_PATH, nodes_expanded);
		cout << "\n";
	}*/

	/**
	RBFS MN Puzzle Experiments
	**/

	for(double weight = 2.0; weight <= 2.0; weight+= 1) {

		mn_rbfs.Change_Weights(1.0, weight);
		//general_batch(&mnp, &mn_rbfs, mn_puzzles, true, ACTION_PATH);
		general_batch_prob_expand_limits(&mnp, &mn_rbfs, mn_puzzles, prob_expanded, true, ACTION_PATH);
		cout << "\n";
	}

	/**
	RBFS MN Order Experiments
	**/
	/*
	for(double weight = 3.0; weight <= 3.0; weight ++) {
		mn_rbfs.Change_Weights(1.0, weight);
		general_batch_orders(&mnp, &mn_rbfs, mnp_op_orders, mn_puzzles, true, ACTION_PATH, nodes_expanded);
		cout << "\n";
	}*/

	/**
	WA* MN Puzzle Weight Experiments
	**/
	/*
	mn_astar.SetMemoryLimit(1000000);
	for(double weight = 3.0; weight <= 6.0; weight+= 1) {
		mn_astar.SetWeight(weight);
		general_batch(&mnp, &mn_astar, mn_puzzles, true, STATE_PATH);
		cout << "\n";
	}*/


	/**
	WA MN Order Experiments
	**/

	/*
	mn_astar.SetMemoryLimit(1000000);
	for(double weight = 1.4; weight <= 1.4; weight ++) {
		mn_astar.SetWeight(weight);
		general_batch_orders(&mnp, &mn_astar, mnp_op_orders, mn_puzzles, true, STATE_PATH, nodes_expanded);
		cout << "\n";
	}*/

	/** BEAM/BULB MN Puzzle settings **/

	unsigned mn_beam_memory_limit = 1000000;
	bool mn_beam_prune_dups = false;
	bool mn_beam_full_check = true;

	unsigned mn_initial_discrepancies = 0;
	unsigned mn_disc_inc = 1;
	int mn_max_disc = -1;

	mn_beam_search.Change_Memory_Limit(mn_beam_memory_limit);
	mn_bulb.Change_Memory_Limit(mn_beam_memory_limit);

	mn_beam_search.Select_Full_Check(mn_beam_full_check);
	mn_bulb.Select_Full_Check(mn_beam_full_check);

	mn_beam_search.Select_Duplicate_Prune(mn_beam_prune_dups);
	mn_bulb.Select_Duplicate_Prune(mn_beam_prune_dups);

	mn_bulb.Set_Initial_Discrepancies(mn_initial_discrepancies);
	mn_bulb.Set_Discrepancies_Increment(mn_disc_inc);
	mn_bulb.Set_Max_Discrepancies(mn_max_disc);


	/**
	BEAM/BULB MN Puzzle Experiments
	**/


	// MN Puzzle Beam Search Experiments
	/*
	for(unsigned i = 0; i < 3; i++) {
		mn_beam_search.Change_Beam_Size(beam_sizes[i]);
		general_batch(&mnp, &mn_beam_search, mn_puzzles, true, STATE_PATH);
		cout << "\n";
	}*/

	/*
	// MN Puzzle BULB Experiments
	for(unsigned i = 0; i < beam_sizes.size(); i++) {
		mn_bulb.Change_Beam_Size(beam_sizes[i]);
		general_batch(&mnp, &mn_bulb, mn_puzzles, true, STATE_PATH);
		cout << "\n";
	}*/

	/**
	BEAM/BULB MN Puzzle Order Experiments
	**/

	// MN Puzzle Beam Search Experiments
	/*
	for(unsigned i = 0; i < 1; i++) {
		mn_beam_search.Change_Beam_Size(beam_sizes[i]);
		general_batch_orders(&mnp, &mn_beam_search, mnp_op_orders, mn_puzzles, true, STATE_PATH, nodes_expanded);
		cout << "\n";
	}*/
	/*
	// MN Puzzle BULB Experiments
	for(unsigned i = 0; i < beam_sizes.size(); i++) {
		mn_bulb.Change_Beam_Size(beam_sizes[i]);
		general_batch_orders(&mnp, &mn_bulb, mnp_op_orders, mn_puzzles, true, STATE_PATH, nodes_expanded);
		cout << "\n";
	}*/


	/*
	// For Timing
	Timer t, t2;
	t.StartTimer();
	t.EndTimer();
	cout << "Time:" << t.GetElapsedTime() << endl;
	*/


	/******************** PANCAKE PUZZLE EXPERIMENTS ********************/
	vector<PancakePuzzleState> pan_puzzles;
	unsigned pan_size = 20;

	vector<PancakePuzzleState> temp_pancake_puzzles;
	get_20pancake_test_set(temp_pancake_puzzles, 1000);

	PancakePuzzle pancake_env(pan_size);
	PancakePuzzleState pan_goal(pan_size);
	pancake_env.StoreGoal(pan_goal);

	pancake_env.Validate_Problems(temp_pancake_puzzles);

	// takes puzzles you want to solve
	begin_index = 0;
	end_index = 5;
	vector<PancakePuzzleState> pancake_puzzles;
	for(unsigned i = begin_index; i <= end_index; i++) {
		pancake_puzzles.push_back(temp_pancake_puzzles[i]);
	}

	/** Build Pancake PDB **/
	/*
	vector<int> distinct;
	distinct.push_back(7);
	distinct.push_back(8);
	distinct.push_back(9);
	distinct.push_back(10);
	distinct.push_back(11);
	distinct.push_back(12);
	distinct.push_back(13);

	//pancake_puzz.Build_Regular_PDB(s, distinct, "../../apps/dynamicsearch/input/14panc_pdb_7_8_9_10_11_12_13_distinct");
	*/

	/** Build Pancake Test Set **/
	/*
	pancake_puzzles.clear();
	PancakePuzzle::Create_Random_Pancake_Puzzles(pancake_puzzles, 200, 1000);
	pancake_puzz.Output_Puzzles(pancake_puzzles, false);
	*/

	/** Load Pattern Databases **/
	//pancake_puzz.Load_Regular_PDB("../../apps/dynamicsearch/input/14panc_pdb_0_1_2_3_4_5_6_distinct", pan_goal, false);
	//pancake_puzz.Load_Regular_PDB("../../apps/dynamicsearch/input/14panc_pdb_7_8_9_10_11_12_13_distinct", pan_goal, false);

	/** Get Operator Orderings **/
	vector<vector<unsigned> > pan_op_orders;
	vector<unsigned> test_ops;
	vector<unsigned> rand_perm;

	for(unsigned i = 2; i <= pan_size; i ++) {
		test_ops.clear();

		if(i ==2) {
			for(unsigned j = 2; j <= pan_size; j++)
				test_ops.push_back(j);
		}
		else if (i == pan_size) {
			for(unsigned j = pan_size; j >= 2; j--)
				test_ops.push_back(j);
		}
		else {
			test_ops.push_back(i);
			get_rand_permutation(rand_perm, 2, pan_size -1, pan_size-2);
			for(unsigned j = 0; j < rand_perm.size(); j++) {
				if(rand_perm[j] == i)
					test_ops.push_back(pan_size);
				else
					test_ops.push_back(rand_perm[j]);
			}
		}
		pan_op_orders.push_back(test_ops);
	}

	/** Pancake Solvers **/
	GeneralIDA<PancakePuzzleState, unsigned, PancakePuzzle> pan_ida;
	GeneralRBFS<PancakePuzzleState, unsigned, PancakePuzzle> pan_rbfs;
	GeneralBeamSearch<PancakePuzzleState, unsigned, PancakePuzzle> pan_beam_search;
	GeneralBulb<PancakePuzzleState, unsigned, PancakePuzzle> pan_bulb;
	//GenericTemplateAStar<PancakePuzzleState, unsigned, PancakePuzzle> pan_astar;

	/**
	IDA* Pancake Weight Experiments
	**/
	/*
	for(double weight = 3.0; weight <= 6.0; weight+= 1) {
		pan_ida.Change_Weights(1.0, weight);
		general_batch(&pancake_env, &pan_ida, pancake_puzzles, true, ACTION_PATH);
		cout << "\n";
	}*/

	/**
	IDA* Pancake Order Experiments
	**/
	/*
	for(double weight = 3.0; weight <= 3.0; weight ++) {
		pan_ida.Change_Weights(1.0, weight);
		general_batch_orders(&pancake_env, &pan_ida, pan_op_orders, pancake_puzzles, true, ACTION_PATH, nodes_expanded);
		cout << "\n";
	}*/

	/**
	RBFS Pancake Weight Experiments
	**/
	/*
	for(double weight = 1.0; weight <= 6.0; weight+= 1) {
		pan_rbfs.Change_Weights(1.0, weight);
		general_batch(&pancake_env, &pan_rbfs, pancake_puzzles, true, ACTION_PATH);
		cout << "\n";
	}*/

	/**
	RBFS Pancake Order Experiments
	**/
	/*
	for(double weight = 3.0; weight <= 3.0; weight ++) {
		pan_rbfs.Change_Weights(1.0, weight);
		general_batch_orders(&pancake_env, &pan_rbfs, pan_op_orders, pancake_puzzles, true, ACTION_PATH, nodes_expanded);
		cout << "\n";
	}
	*/

	/**
	WA* Pancake Weight Experiments
	**/

	/*
	pan_astar.SetMemoryLimit(1000000);
	for(double weight = 4.0; weight <= 6.0; weight+= 1) {
		pan_astar.SetWeight(weight);
		general_batch(&pancake_env, &pan_astar, pancake_puzzles, true, STATE_PATH);
		cout << "\n";
	}
	*/

	/**
	WA* Pancake Order Experiments
	**/

	/*
	pan_astar.SetMemoryLimit(1000000);
	for(double weight = 3.0; weight <= 3.0; weight ++) {
		pan_astar.SetWeight(weight);
		general_batch_orders(&pancake_env, &pan_astar, pan_op_orders, pancake_puzzles, true, STATE_PATH, nodes_expanded);
		cout << "\n";
	}*/


	/** BEAM/BULB Pancake Puzzle Settings **/

	unsigned pan_beam_memory_limit = 50000;
	bool pan_beam_prune_dups = false;
	bool pan_beam_full_check = false;

	unsigned pan_initial_discrepancies = 0;
	unsigned pan_disc_inc = 1;
	int pan_max_disc = -1;

	pan_beam_search.Change_Memory_Limit(pan_beam_memory_limit);
	pan_bulb.Change_Memory_Limit(pan_beam_memory_limit);

	pan_beam_search.Select_Full_Check(pan_beam_full_check);
	pan_bulb.Select_Full_Check(pan_beam_full_check);

	pan_beam_search.Select_Duplicate_Prune(pan_beam_prune_dups);
	pan_bulb.Select_Duplicate_Prune(pan_beam_prune_dups);

	pan_bulb.Set_Initial_Discrepancies(pan_initial_discrepancies);
	pan_bulb.Set_Discrepancies_Increment(pan_disc_inc);
	pan_bulb.Set_Max_Discrepancies(pan_max_disc);

	/**
	BEAM / BULB Pancake Puzzle Beam Size Experiments
	**/
	/*
	// Pancake Puzzle Beam Search Experiments
	for(unsigned i = 5; i < 11; i++) {
		pan_beam_search.Change_Beam_Size(beam_sizes[i]);
		general_batch(&pancake_env, &pan_beam_search, pancake_puzzles, true, STATE_PATH);
		cout << "\n";
	}*/

	/*
	// Pancake Puzzle BULB Experiments
	for(unsigned i = 5; i < 11; i++) {
		pan_bulb.Change_Beam_Size(beam_sizes[i]);
		general_batch(&pancake_env, &pan_bulb, pancake_puzzles, true, STATE_PATH);
		cout << "\n";
	}*/

	/**
	BEAM / BULB Pancake Puzzle Order Experiments
	**/

	// MN Puzzle Beam Search Experiments
	/*
	for(unsigned i = 0; i < 1; i++) {
		pan_beam_search.Change_Beam_Size(beam_sizes[i]);
		general_batch_orders(&pancake_env, &pan_beam_search, pan_op_orders, pancake_puzzles, true, STATE_PATH, nodes_expanded);
		cout << "\n";
	}

	for(unsigned i = 0; i < 1; i++) {
		pan_bulb.Change_Beam_Size(beam_sizes[i]);
		general_batch_orders(&pancake_env, &pan_bulb, pan_op_orders, pancake_puzzles, true, STATE_PATH, nodes_expanded);
		cout << "\n";
	}*/

	return 0;
}
