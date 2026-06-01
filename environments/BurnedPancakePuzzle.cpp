#include "BurnedPancakePuzzle.h"
#include <cstdlib>

BurnedPancakePuzzle::BurnedPancakePuzzle(unsigned s)
{
	assert(s >= 2);
	size = s;

	// assign the default operator ordering
	for (unsigned i = s; i >= 2; i--)
		operators.push_back(i);

	goal_stored = false;
	use_memory_free = true;
}

BurnedPancakePuzzle::BurnedPancakePuzzle(unsigned s, const std::vector<unsigned> op_order)
{
	size = s;

	Change_Op_Order(op_order);

	goal_stored = false;
	use_memory_free = false;
}

BurnedPancakePuzzle::~BurnedPancakePuzzle()
{
	ClearGoal();
}

//const std::string BurnedPancakePuzzle::GetName()
//{
//	std::stringstream name;
//	name << size;
//	name << " BurnedPancake Puzzle";
//
//	if (PDB_distincts.size() > 0)
//	{
//		name << ", PDBS:";
//		for (unsigned i = 0; i < PDB_distincts.size(); i++)
//		{
//			name << " <";
//			for (unsigned j = 0; j < PDB_distincts[i].size() - 1; j++)
//			{
//				name << PDB_distincts[i][j];
//				name << ", ";
//			}
//			name << PDB_distincts[i].back();
//			name << ">";
//		}
//		if (use_memory_free)
//		name << ", Memory-Free Heuristic";
//	}
//	else if (use_memory_free)
//	{
//		name << ", Memory-Free Heuristic";
//	}
//	else {
//		name << ",No Heuristic";
//	}
//
//	name << ", Op Order: ";
//	for (unsigned op_num = 0; op_num < operators.size() - 1; op_num++)
//	{
//		name << operators[op_num];
//		name << ", ";
//	}
//	name << operators.back();
//	return name.str();
//}

void BurnedPancakePuzzle::GetSuccessors(const BurnedPancakePuzzleState &parent,
                             std::vector<BurnedPancakePuzzleState> &children) const
{
	children.resize(0);

	// all operators are applicable in all states
	for (unsigned i = 0; i < operators.size(); i++)
	{
		children.push_back(parent); // adds a copy of the state to the stack
		ApplyAction(children.back(), operators[i]);
	}
}

void BurnedPancakePuzzle::GetActions(const BurnedPancakePuzzleState &, std::vector<unsigned> &actions) const
{
	actions.resize(0);

	// all operators are applicable in all states
	for (unsigned i = 0; i < operators.size(); i++)
	{
		actions.push_back(operators[i]);
	}
}

unsigned BurnedPancakePuzzle::GetAction(const BurnedPancakePuzzleState &parent, const BurnedPancakePuzzleState &child) const
{
	unsigned current_action;
	bool are_equal = false;

	assert(child.puzzle.size() == size);
	assert(parent.puzzle.size() == size);
	BurnedPancakePuzzleState parentCopy = parent;
	for (unsigned i = 0; i < operators.size(); i++)
	{
		current_action = operators[i];
		ApplyAction(parentCopy, current_action);
		if (parentCopy == child)
			are_equal = true;
		InvertAction(current_action);
		ApplyAction(parentCopy, current_action);

		if (are_equal)
			return operators[i];
	}
	fprintf(stderr, "ERROR: GetAction called with non-adjacent states\n");
	exit(1);
	return 0;
}

void BurnedPancakePuzzle::ApplyAction(BurnedPancakePuzzleState &s, unsigned action) const
{
	assert(s.puzzle.size() == size);
	assert(action > 1 && action <= size);

	int upper = 0;
	int lower = action - 1;
	int temp;
	// performs pancake flipping -- burned side switches
	while(upper < lower)
	{
		temp = s.puzzle[upper];
		s.puzzle[upper] = -s.puzzle[lower];
		s.puzzle[lower] = -temp;
		upper++;
		lower--;
	}
	if (upper == lower)
		s.puzzle[lower] = -s.puzzle[lower];
}

bool BurnedPancakePuzzle::InvertAction(unsigned &a) const
{
	// ever action is self-inverse
	assert(a > 1 && a <= size);
	return true;
}

double BurnedPancakePuzzle::HCost(const BurnedPancakePuzzleState &state) const
{
	if (!goal_stored)
	{
		fprintf(stderr, "ERROR: HCost called with a single state and goal is not stored.\n");
		exit(1);
	}
	if (state.puzzle.size() != size)
	{
		fprintf(stderr, "ERROR: HCost called with a single state with wrong size.\n");
		exit(1);
	}
	double h_cost = 0;

//	// use PDB heuristic
//	if (PDB.size() > 0)
//	{
//		if (PDB.size() != PDB_distincts.size())
//		{
//			fprintf(stderr, "ERROR: HCost called with a single state, no use of memory free heuristic, and invalid setup of pattern databases.\n");
//			exit(1);
//		}
//		h_cost = std::max(Regular_PDB_Lookup(state), h_cost);
//	}

	// use memory-free heuristic
	if (use_memory_free)
	{
		h_cost =  std::max(Memory_Free_HCost(state, goal_locations), h_cost);
	}
	// if no heuristic
	else //if (PDB.size()==0)
	{
		if (goal == state)
			return 0;
		else
			return 1;
	}

	return h_cost;
}

double BurnedPancakePuzzle::HCost(const BurnedPancakePuzzleState &state, const BurnedPancakePuzzleState &goal_state) const
{
	if (state.puzzle.size() != size)
	{
		fprintf(stderr, "ERROR: HCost called with state with wrong size.\n");
		exit(1);
	}
	if (goal_state.puzzle.size() != size)
	{
		fprintf(stderr, "ERROR: HCost called with goal with wrong size.\n");
		exit(1);
	}

	if (use_memory_free)
	{
		std::vector<int> goal_locs(size);
		for (unsigned i = 0; i < size; i++)
		{
			int theSign = abs(goal_state.puzzle[i])/goal_state.puzzle[i];
			goal_locs[abs(goal_state.puzzle[i])-1] = (i+1)*theSign;
//			std::cout << "goal array: ";
//			for (unsigned int x = 0; x < size; x++)
//				std::cout << goal_locs[x] << " ";
//			std::cout << std::endl;
		}
		return Memory_Free_HCost(state, goal_locs);
	}

	if (state == goal_state)
		return 0.0;
	return 1.0;
}
//#define GetDualEntry(s, r, i) (r[abs(s.puzzle[i])-1])
//#define GetDualEntry(s, r, i) (s.puzzle[r[i]-1])
int GetDualEntryLoc(const BurnedPancakePuzzleState &state, const std::vector<int> &goal_locs, int index)
{
	int theSign = abs(state.puzzle[index])/state.puzzle[index];
	int loc = goal_locs[abs(state.puzzle[index])-1];
	return loc*theSign;
}

double BurnedPancakePuzzle::Memory_Free_HCost(const BurnedPancakePuzzleState &state, const std::vector<int> &goal_locs) const
{
	if (state.puzzle.size() != size)
	{
		fprintf(stderr, "ERROR: HCost called with state with wrong size.\n");
		exit(1);
	}

	double h_count = 0.0;
	unsigned i = 0;
	for (; i < size - 1; i++)
	{
//		int diff = abs(goal_locs[abs(state.puzzle[i])-1]) - abs(goal_locs[abs(state.puzzle[i+1])-1]);
		int curr = GetDualEntryLoc(state, goal_locs, i);
		int next = GetDualEntryLoc(state, goal_locs, i+1);
		int diff = (abs(abs(curr)-abs(next)));
		if (diff > 1 || diff < - 1)
		{
			h_count++;
		}
		else if ((curr < 0 && next > 0) ||// sign is different
				 (curr > 0 && next < 0))
		{
			h_count+=2;
		}
	}
	int last = GetDualEntryLoc(state, goal_locs, i);
	//if ((unsigned) goal_locs[abs(state.puzzle[i])-1] != size -1)
	if (last != (int)size)
		h_count++;
	else if (-last == (int)size)
		h_count+=2;
//	else if (state.puzzle[i] < 0) // in place but must switch out
//		h_count+=2;

//	std::cout << "goal array: ";
//	for (unsigned int x = 0; x < size; x++)
//		std::cout << goal_locs[x] << " ";
//	std::cout << std::endl;
	return h_count;
}

bool BurnedPancakePuzzle::GoalTest(const BurnedPancakePuzzleState &state, const BurnedPancakePuzzleState &theGoal) const
{
	return (state == theGoal);
}

bool BurnedPancakePuzzle::GoalTest(const BurnedPancakePuzzleState &s) const
{
	if (!goal_stored)
	{
		fprintf(stderr, "ERROR: GoalTest called with a single state and goal is not stored.\n");
		exit(1);
	}
	if (s.puzzle.size() != size)
	{
		fprintf(stderr, "ERROR: GoalTest called with a single state with wrong size.\n");
		exit(1);
	}
	return (s == goal);
}

uint64_t BurnedPancakePuzzle::GetActionHash(unsigned act) const
{
	return (uint64_t) act;
}

void BurnedPancakePuzzle::StoreGoal(BurnedPancakePuzzleState &g)
{
	assert(g.puzzle.size() == size);

	goal = g;
	goal_stored = true;

	goal_locations.resize(size);
	for (unsigned i = 0; i < size; i++)
	{
		goal_locations[goal.puzzle[i]] = i;
	}
}

void BurnedPancakePuzzle::Change_Op_Order(const std::vector<unsigned> op_order)
{
	operators.clear();

	if (op_order.size() != size - 1)
	{
		fprintf(stderr, "ERROR: Not enough operators in operator sequence for construction of BurnedPancakePuzzle\n");
		exit(1);
	}

	bool all_ops[op_order.size()];

	for (unsigned i = 0; i < size; i++)
	{
		all_ops[i] = false;
	}

	for (unsigned i = 0; i < op_order.size(); i++)
	{
		if (op_order[i] < 2 || op_order[i] > size)
		{
			fprintf(stderr, "ERROR: Invalid operator included in construction of BurnedPancakePuzzle\n");
			exit(1);
		}
		all_ops[op_order[i] - 2] = true;
	}

	for (unsigned i = 0; i < op_order.size(); i++)
	{
		assert(false);
		// code here was originally problematic
		// (!all_ops[i])
		if (all_ops[i] == false)
		{
			fprintf(stderr, "ERROR: Missing operator %u in construction of BurnedPancakePuzzle\n", i+2);
			exit(1);
		}
	}
	// assign the default operator ordering
	for (unsigned i = 0; i < op_order.size(); i++)
		operators.push_back(op_order[i]);
}


//void BurnedPancakePuzzle::Create_Random_BurnedPancake_Puzzles(std::vector<BurnedPancakePuzzleState> &puzzle_vector, unsigned size, unsigned num_puzzles)
//{
//	std::map<uint64_t, uint64_t> puzzle_map; // used to ensure uniqueness
//
//	BurnedPancakePuzzle my_puzz(size);
//
//	unsigned count = 0;
//
//	std::vector<int> perm;
//	BurnedPancakePuzzleState potential_puzz(size);
//	while (count < num_puzzles)
//	{
//		perm = Get_Random_Permutation(size);
//
//		// construct puzzle
//		for (unsigned i = 0; i < size; i++)
//		{
//			potential_puzz.puzzle[i] = perm[i];
//		}
//
//		uint64_t next_hash = my_puzz.GetStateHash(potential_puzz);
//
//		// make sure is not a duplicate
//		if (puzzle_map.find(next_hash) != puzzle_map.end())
//		{
//			continue;
//		}
//
//		puzzle_map[next_hash] = next_hash;
//		puzzle_vector.push_back(potential_puzz);
//		count++;
//	}
//}

//int BurnedPancakePuzzle::read_in_pancake_puzzles(const char *filename, bool puzz_num_start, unsigned size, unsigned max_puzzles, std::vector<BurnedPancakePuzzleState> &puzzles)
//{
//	std::vector<std::vector<int> > permutations;
//	Read_In_Permutations(filename, size, max_puzzles, permutations, puzz_num_start);
//
//	// convert permutations into BurnedPancakePuzzleStates
//	for (unsigned i = 0; i < permutations.size(); i++)
//	{
//		BurnedPancakePuzzleState new_state(size);
//
//		for (unsigned j = 0; j < size; j++)
//		{
//			new_state.puzzle[j] = permutations[i][j];
//		}
//		puzzles.push_back(new_state);
//	}
//	return 0;
//}
//
//bool BurnedPancakePuzzle::Path_Check(BurnedPancakePuzzleState start, BurnedPancakePuzzleState theGoal, std::vector<unsigned> &actions)
//{
//	if (start.puzzle.size() != size || theGoal.puzzle.size() != size)
//		return false;
//
//	for (unsigned i = 0; i < actions.size(); i++)
//	{
//		if (actions[i] < 2 || actions[i] > size)
//			return false;
//		ApplyAction(start, actions[i]);
//	}
//
//	if (start == theGoal)
//		return true;
//
//	return false;
//}
//
//std::vector<unsigned> BurnedPancakePuzzle::Get_Puzzle_Order(int64_t order_num, unsigned num_pancakes)
//{
//	std::vector<unsigned> ops;
//	assert(order_num >= 0);
//	assert(num_pancakes > 0);
//
//	std::vector<int64_t> op_nums(num_pancakes -1);
//
//	int64_t num_left = 1;
//	for (int64_t x = num_pancakes - 2; x >= 0; x--)
//	{
//		op_nums[x] = order_num % num_left;
//		order_num /= num_left;
//		num_left++;
//
//		for (int64_t y = x+1; y < num_pancakes-1; y++)
//		{
//			if (op_nums[y] >= op_nums[x])
//			{
//				op_nums[y]++;
//			}
//		}
//	}
//
//	std::vector<bool> actions;
//
//	for (unsigned i = 0; i < num_pancakes - 1; i++)
//	{
//		actions.push_back(false);
//	}
//
//	for (unsigned i = 0; i < num_pancakes - 1; i++)
//	{
//		ops.push_back(op_nums[i] + 2);
//		actions[op_nums[i]] = true;
//	}
//
//	for (unsigned i = 0; i < num_pancakes - 1; i++)
//	{
//		assert(actions[i]);
//	}
//	return ops;
//}

uint64_t BurnedPancakePuzzle::GetStateHash(const BurnedPancakePuzzleState &s) const
{
	std::vector<int> puzzle = s.puzzle;
	uint64_t hashVal = 0;
	int numEntriesLeft = (int)s.puzzle.size();
	for (unsigned int x = 0; x < s.puzzle.size(); x++)
	{
		hashVal += puzzle[x]*Factorial(numEntriesLeft-1);
		numEntriesLeft--;
		for (unsigned y = x; y < puzzle.size(); y++)
		{
			if (puzzle[y] > puzzle[x])
				puzzle[y]--;
		}
	}
	return hashVal;
}
