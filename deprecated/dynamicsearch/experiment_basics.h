#ifndef EXP_BASICS
#define EXP_BASICS

#include <vector>
#include <cmath>
#include <map>
#include <algorithm>
#include <sstream>
#include <stdio.h>
#include "Common.h"
#include <iostream>
#include "MNPuzzle.h"
#include "PancakePuzzle.h"
#include "GeneralIDA.h"
#include "GenericStepAlgorithm.h"
#include <stdlib.h>
#include "StringUtils.h"

#define STATE_PATH 1
#define ACTION_PATH 2

void warnings();

typedef std::pair<double, double> Puzzle_Info;

/** Returns a description for a batch run, based on the descriptions of the
	environment and solver.
 **/
template <class state, class action, class environment>
std::string batch_header(environment *env, GenericStepAlgorithm<state, action, environment> *solver) {
	std::stringstream header;
	header << "SOLVER ";
	header <<  solver->GetName();
	header << "; DOMAIN: " ;
	header << env->GetName();

	return header.str();
}

/** Perfoms a batch of experiments with given search environment, given solver and set of puzzles.
	If print_all_stats is true, will print the value of every single puzzle solved. If the puzzle
	solving failed, prints -1. The type selects what kind of solutions are returned during problem
	solving. Is used because not all methods are defined for all solvers.

	Note: assumes that the goal is stored in the environment.
 **/
template <class state, class action, class environment>
uint64_t general_batch(environment *env, GenericStepAlgorithm<state, action, environment> *solver, std::vector<state> &puzzles, bool print_all_stats, unsigned type){
	uint64_t total_checked = 0;
	uint64_t total_touched = 0;
	uint64_t total_expanded = 0;

	double total_cost = 0.0;

	std::vector<action> action_path;
	std::vector<state> state_path;

	if(!env->IsGoalStored()) {
		std::cerr << "ERROR: general_batch called with environment that does not have the goal stored.\n";
		exit(1);
	}

	state goal = env->Get_Goal();
	unsigned solved_problems = 0;

	// print header for batch solving
	std::cout << batch_header(env, solver) << std::endl;
	for(unsigned i = 0; i < puzzles.size(); i++) {

		int status = 0;
		// tries to solve the problem
		if(type == ACTION_PATH) {
			status = solver->GetPath(env, puzzles[i], goal, action_path);
		}
		else if(type == STATE_PATH) {
			status = solver->GetPath(env, puzzles[i], goal, state_path);
		}

		if(status != 1) { // if no solution was found
			if(print_all_stats) {
				printf("-1\n");
				fflush(stdout);
			}
			continue;
		}

		solved_problems++;

		total_cost += solver->GetPathCost();
		total_checked += solver->GetNodesChecked();
		total_touched += solver->GetNodesTouched();
		total_expanded += solver->GetNodesExpanded();

		if(print_all_stats) {
			printf("%4.f\t", solver->GetPathCost());
			std::cout << solver->GetNodesExpanded() << "\t";
			std::cout << solver->GetNodesChecked() << "\t";
			std::cout << solver->GetNodesTouched() << "\n";
		}
	}
	std::cout << "TOTALS\t";
	printf("%4.f\t", total_cost);
	std::cout << total_expanded << "\t" << total_checked << "\t" << total_touched << "\t" << solved_problems << "\n";
	printf("\n");
	return (uint64_t) total_checked;
}

/** Perfoms a batch of experiments with given search environment, given solver and set of puzzles.
	If print_all_stats is true, will print the value of every single puzzle solved. If the puzzle
	solving failed, prints -1. The type selects what kind of solutions are returned during problem
	solving. Is used because not all methods are defined for all solvers.

	Note: assumes that the goal is stored in the environment.
 **/
template <class state, class action, class environment>
uint64_t general_batch_prob_expand_limits(environment *env, GenericStepAlgorithm<state, action, environment> *solver, std::vector<state> &puzzles, std::vector<uint64_t> &exp_limit, bool print_all_stats, unsigned type){

	if(puzzles.size() != exp_limit.size()) {
		std::cerr << "ERROR: general_batch_prob_expand_limits called with unequal length problem and limit lists.\n";
		exit(1);
	}

	uint64_t total_checked = 0;
	uint64_t total_touched = 0;
	uint64_t total_expanded = 0;

	double total_cost = 0.0;

	std::vector<action> action_path;
	std::vector<state> state_path;

	if(!env->IsGoalStored()) {
		std::cerr << "ERROR: general_batch called with environment that does not have the goal stored.\n";
		exit(1);
	}

	state goal = env->Get_Goal();
	unsigned solved_problems = 0;

	// print header for batch solving
	std::cout << batch_header(env, solver) << std::endl;
	for(unsigned i = 0; i < puzzles.size(); i++) {

		int status = 0;
		solver->SetExpandedLimit(exp_limit[i]);
		// tries to solve the problem
		if(type == ACTION_PATH) {
			status = solver->GetPath(env, puzzles[i], goal, action_path);
		}
		else if(type == STATE_PATH) {
			status = solver->GetPath(env, puzzles[i], goal, state_path);
		}

		if(status != 1) { // if no solution was found
			if(print_all_stats) {
				printf("-1\n");
				fflush(stdout);
			}
			continue;
		}

		solved_problems++;

		total_cost += solver->GetPathCost();
		total_checked += solver->GetNodesChecked();
		total_touched += solver->GetNodesTouched();
		total_expanded += solver->GetNodesExpanded();

		if(print_all_stats) {
			printf("%4.f\t", solver->GetPathCost());
			std::cout << solver->GetNodesExpanded() << "\t";
			std::cout << solver->GetNodesChecked() << "\t";
			std::cout << solver->GetNodesTouched() << "\n";
		}
	}
	std::cout << "TOTALS\t";
	printf("%4.f\t", total_cost);
	std::cout << total_expanded << "\t" << total_checked << "\t" << total_touched << "\t" << solved_problems << "\n";
	printf("\n");
	return (uint64_t) total_checked;
}

template <class state, class action, class environment>
void general_batch_orders(environment *env, GenericStepAlgorithm<state, action, environment> *solver, std::vector<std::vector<action> >&puzz_orders, std::vector<state> &puzzles, bool print_all_stats, unsigned type, std::vector<uint64_t> &nodes_expanded){

	nodes_expanded.clear();
	for(unsigned i = 0; i < puzz_orders.size(); i++) {
		env->Change_Op_Order(puzz_orders[i]);
		general_batch(env, solver, puzzles, print_all_stats, type);
		std::cout << "\n";
	}
}

/** This is the korf test set. It is of 4x4 puzle problems.
	num cannot exceed 100.
 **/
void get_standard_test_set(std::vector<MNPuzzleState> &puzzles, unsigned num);

/** This is a set of 1000 randomly generated but solvable 4x4 problems.
	num is the number of problems you want appended to puzzles.
	num cannot exceed 1000.
 **/
void get_big_4x4_test_set(std::vector<MNPuzzleState> &puzzles, unsigned num);

/** These return various test sets of the type num_cols x num_rows.
	Each test set contains a 1000 problems. num is the number of problems
	you want appended to puzzles. num cannot exceed 1000.
 **/
void get_4x5_test_set(std::vector<MNPuzzleState> &puzzles, unsigned num);
void get_5x4_test_set(std::vector<MNPuzzleState> &puzzles, unsigned num);
void get_3x6_test_set(std::vector<MNPuzzleState> &puzzles, unsigned num);
void get_6x3_test_set(std::vector<MNPuzzleState> &puzzles, unsigned num);
void get_5x5_test_set(std::vector<MNPuzzleState> &puzzles, unsigned num);
void get_6x6_test_set(std::vector<MNPuzzleState> &puzzles, unsigned num);
void get_7x7_test_set(std::vector<MNPuzzleState> &puzzles, unsigned num);

/** These return various pancake puzzle test sets. Each test set contains a 1000 problems.
	num is the number of problems you want appended to puzzles. num cannot exceed 1000.
 **/
void get_12pancake_test_set(std::vector<PancakePuzzleState> &puzzles, unsigned num);
void get_13pancake_test_set(std::vector<PancakePuzzleState> &puzzles, unsigned num);
void get_14pancake_test_set(std::vector<PancakePuzzleState> &puzzles, unsigned num);
void get_16pancake_test_set(std::vector<PancakePuzzleState> &puzzles, unsigned num);
void get_18pancake_test_set(std::vector<PancakePuzzleState> &puzzles, unsigned num);
void get_20pancake_test_set(std::vector<PancakePuzzleState> &puzzles, unsigned num);
void get_25pancake_test_set(std::vector<PancakePuzzleState> &puzzles, unsigned num);
void get_30pancake_test_set(std::vector<PancakePuzzleState> &puzzles, unsigned num);
void get_50pancake_test_set(std::vector<PancakePuzzleState> &puzzles, unsigned num);
void get_75pancake_test_set(std::vector<PancakePuzzleState> &puzzles, unsigned num);
void get_100pancake_test_set(std::vector<PancakePuzzleState> &puzzles, unsigned num);
void get_150pancake_test_set(std::vector<PancakePuzzleState> &puzzles, unsigned num);
void get_200pancake_test_set(std::vector<PancakePuzzleState> &puzzles, unsigned num);

/** For n elements, combinations of size k can be ordered. This method returns the next
	combination in the ordering. The values in the combination are the unsigned integer
	values from 0 to max_num, inclusive. The value returned is true until the end of
	the ordering is reached. If the first combination in the ordering is wanted,
	then set_as_smallest should be set to true.

	Note, the combination is always returned sorted in ascending order. The input
	for current_combination is expected to be the same.
**/
bool get_next_combo(std::vector<unsigned> &current_combination, unsigned max_num, bool reset);

/** Prints the elements in the given vector **/
void print_combo(std::vector<unsigned> &current_combination);

/** Constructs a random permutation of length size. The elements that may appear
	in the permutation are the integers in the range from start_index to
	end_index, inclusive.
**/
void get_rand_permutation(std::vector<unsigned> &perm, unsigned start_index, unsigned end_index, unsigned size);
#endif
