#include "experiment_basics.h"

void warnings() {
	MNPuzzleState start(4, 4);
	if(start == start) {
		std::cout << kUp;
		std::cout << start;
	}
}

void get_standard_test_set(std::vector<MNPuzzleState> &puzzles, unsigned num) {
	MNPuzzle::read_in_mn_puzzles("../../apps/dynamicsearch/input/4x4_100", false, 4, 4, num, puzzles);
}

void get_big_4x4_test_set(std::vector<MNPuzzleState> &puzzles, unsigned num) {
	MNPuzzle::read_in_mn_puzzles("../../apps/dynamicsearch/input/4x4_1000", true, 4, 4, num, puzzles);
}

void get_4x5_test_set(std::vector<MNPuzzleState> &puzzles, unsigned num) {
	MNPuzzle::read_in_mn_puzzles("../../apps/dynamicsearch/input/4x5_1000", false, 4, 5, num, puzzles);
}

void get_5x4_test_set(std::vector<MNPuzzleState> &puzzles, unsigned num) {
	MNPuzzle::read_in_mn_puzzles("../../apps/dynamicsearch/input/5x4_1000", false, 5, 4, num, puzzles);
}

void get_3x6_test_set(std::vector<MNPuzzleState> &puzzles, unsigned num) {
	MNPuzzle::read_in_mn_puzzles("../../apps/dynamicsearch/input/3x6_1000", false, 3, 6, num, puzzles);
}

void get_6x3_test_set(std::vector<MNPuzzleState> &puzzles, unsigned num) {
	if(MNPuzzle::read_in_mn_puzzles("../../apps/dynamicsearch/input/6x3_1000", false, 6, 3, num, puzzles)) {
		std::cerr << "File Reading Failed\n";
	}
}

void get_5x5_test_set(std::vector<MNPuzzleState> &puzzles, unsigned num) {
	if(MNPuzzle::read_in_mn_puzzles("../../apps/dynamicsearch/input/5x5_1000", false, 5, 5, num, puzzles)) {
		std::cerr << "File Reading Failed\n";
	}
}

void get_6x6_test_set(std::vector<MNPuzzleState> &puzzles, unsigned num) {
	if(MNPuzzle::read_in_mn_puzzles("../../apps/dynamicsearch/input/6x6_1000", false, 6, 6, num, puzzles)) {
		std::cerr << "File Reading Failed\n";
	}
}

void get_7x7_test_set(std::vector<MNPuzzleState> &puzzles, unsigned num) {
	if(MNPuzzle::read_in_mn_puzzles("../../apps/dynamicsearch/input/7x7_1000", false, 7, 7, num, puzzles)) {
		std::cerr << "File Reading Failed\n";
	}
}

bool get_next_combo(std::vector<unsigned> &current_combination, unsigned max_num, unsigned pos, bool reset_rest) {
	assert(max_num >= current_combination.size() - 1);

	// if are to set all positions from here on out as minimum possible
	if(reset_rest) {

		// if is the first position, reset the entire combination recursively
		if(pos == 0) {
			current_combination[0] = 0;
			return get_next_combo(current_combination, max_num, pos + 1, true);
		}

		// make sure there are enough positions for remaining pieces (since in ascending order)
		assert(current_combination[pos - 1] + current_combination.size() - pos <= max_num);

		// assign as smallest possible
		current_combination[pos] = current_combination[pos - 1] + 1;

		// if have hit end of combination
		if(pos == current_combination.size() - 1)
			return true;

		return get_next_combo(current_combination, max_num, pos + 1, true);
	}

	// if have hit a number that is too high, must backtrack and replace
	if(current_combination[pos] + current_combination.size() - pos - 1 == max_num) {
		return false;
	}
	// have successfully constructed new combination
	else if(pos == current_combination.size() - 1) {
		current_combination[pos]++;
		return true;
	}

	// if have run out of changes below, change this position
	if(!get_next_combo(current_combination, max_num, pos + 1, false)) {
		current_combination[pos]++;
		return get_next_combo(current_combination, max_num, pos + 1, true);
	}

	return true;
}

bool get_next_combo(std::vector<unsigned> &current_combination, unsigned max_num, bool reset){
	get_next_combo(current_combination, max_num, 0, reset);
}

void print_combo(std::vector<unsigned> &current_combination) {
	for(unsigned i = 0; i < current_combination.size(); i++) {
		printf("%u ", current_combination[i]);
	}
	printf("\n");
}

void get_rand_permutation(std::vector<unsigned> &new_perm, unsigned start_index, unsigned end_index, unsigned size) {
	std::vector<unsigned> solver_index;
	solver_index.clear();
	new_perm.clear();

	// initialize solver_index
	for(unsigned i = start_index; i <= end_index; i++) {
		solver_index.push_back(i);
	}

	// construct new permutation
	for(unsigned i = 0; i < size; i++) {
		int r = random();
		int index = r % solver_index.size();
		new_perm.push_back(solver_index[index]);

		solver_index[index] = solver_index.back();
		solver_index.pop_back();
	}
}

void get_12pancake_test_set(std::vector<PancakePuzzleState> &puzzles, unsigned num){
	if(PancakePuzzle::read_in_pancake_puzzles("../../apps/dynamicsearch/input/12pancake_1000", false, 12, num, puzzles)) {
		std::cerr << "File Reading Failed\n";
	}
}
void get_13pancake_test_set(std::vector<PancakePuzzleState> &puzzles, unsigned num){
	if(PancakePuzzle::read_in_pancake_puzzles("../../apps/dynamicsearch/input/13pancake_1000", false, 13, num, puzzles)) {
		std::cerr << "File Reading Failed\n";
	}
}
void get_14pancake_test_set(std::vector<PancakePuzzleState> &puzzles, unsigned num){
	if(PancakePuzzle::read_in_pancake_puzzles("../../apps/dynamicsearch/input/14pancake_1000", false, 14, num, puzzles)) {
		std::cerr << "File Reading Failed\n";
	}
}

void get_easy_14pancake_test_set(std::vector<PancakePuzzleState> &puzzles, unsigned num){
	if(PancakePuzzle::read_in_pancake_puzzles("../../apps/dynamicsearch/input/14panc_50_easy", false, 14, num, puzzles)) {
		std::cerr << "File Reading Failed\n";
	}
}

void get_16pancake_test_set(std::vector<PancakePuzzleState> &puzzles, unsigned num){
	if(PancakePuzzle::read_in_pancake_puzzles("../../apps/dynamicsearch/input/16pancake_1000", false, 16, num, puzzles)) {
		std::cerr << "File Reading Failed\n";
	}
}

void get_18pancake_test_set(std::vector<PancakePuzzleState> &puzzles, unsigned num){
	if(PancakePuzzle::read_in_pancake_puzzles("../../apps/dynamicsearch/input/18pancake_1000", false, 18, num, puzzles)) {
		std::cerr << "File Reading Failed\n";
	}
}

void get_20pancake_test_set(std::vector<PancakePuzzleState> &puzzles, unsigned num){
	if(PancakePuzzle::read_in_pancake_puzzles("../../apps/dynamicsearch/input/20pancake_1000", false, 20, num, puzzles)) {
		std::cerr << "File Reading Failed\n";
	}
}

void get_25pancake_test_set(std::vector<PancakePuzzleState> &puzzles, unsigned num){
	if(PancakePuzzle::read_in_pancake_puzzles("../../apps/dynamicsearch/input/25pancake_1000", false, 25, num, puzzles)) {
		std::cerr << "File Reading Failed\n";
	}
}

void get_30pancake_test_set(std::vector<PancakePuzzleState> &puzzles, unsigned num){
	if(PancakePuzzle::read_in_pancake_puzzles("../../apps/dynamicsearch/input/30pancake_1000", false, 30, num, puzzles)) {
		std::cerr << "File Reading Failed\n";
	}
}

void get_50pancake_test_set(std::vector<PancakePuzzleState> &puzzles, unsigned num){
	if(PancakePuzzle::read_in_pancake_puzzles("../../apps/dynamicsearch/input/50pancake_1000", false, 50, num, puzzles)) {
		std::cerr << "File Reading Failed\n";
	}
}

void get_75pancake_test_set(std::vector<PancakePuzzleState> &puzzles, unsigned num){
	if(PancakePuzzle::read_in_pancake_puzzles("../../apps/dynamicsearch/input/75pancake_1000", false, 75, num, puzzles)) {
		std::cerr << "File Reading Failed\n";
	}
}

void get_100pancake_test_set(std::vector<PancakePuzzleState> &puzzles, unsigned num){
	if(PancakePuzzle::read_in_pancake_puzzles("../../apps/dynamicsearch/input/100pancake_1000", false, 100, num, puzzles)) {
		std::cerr << "File Reading Failed\n";
	}
}

void get_150pancake_test_set(std::vector<PancakePuzzleState> &puzzles, unsigned num){
	if(PancakePuzzle::read_in_pancake_puzzles("../../apps/dynamicsearch/input/150pancake_1000", false, 150, num, puzzles)) {
		std::cerr << "File Reading Failed\n";
	}
}

void get_200pancake_test_set(std::vector<PancakePuzzleState> &puzzles, unsigned num){
	if(PancakePuzzle::read_in_pancake_puzzles("../../apps/dynamicsearch/input/200pancake_1000", false, 200, num, puzzles)) {
		std::cerr << "File Reading Failed\n";
	}
}
