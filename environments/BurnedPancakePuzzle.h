#ifndef BURNEDPANCAKE_H
#define BURNEDPANCAKE_H

#include <stdint.h>
#include <iostream>
#include "SearchEnvironment.h"
#include <sstream>

class BurnedPancakePuzzleState {
public:
	BurnedPancakePuzzleState() { puzzle.clear(); }

	BurnedPancakePuzzleState(unsigned int puzzle_size) {
		puzzle.resize(puzzle_size);
		for (unsigned int x = 0; x < puzzle.size(); x++)
			puzzle[x] = x+1;
	}
	std::vector<int> puzzle;
};

static std::ostream& operator <<(std::ostream & out, const BurnedPancakePuzzleState &loc)
{
	for (unsigned int x = 0; x < loc.puzzle.size(); x++)
	{
		out << loc.puzzle[x] << " ";
	}
	return out;
}

static bool operator==(const BurnedPancakePuzzleState &l1, const BurnedPancakePuzzleState &l2)
{
	if (l1.puzzle.size() != l2.puzzle.size())
		return false;

	for (unsigned int x = 0; x < l1.puzzle.size(); x++)
		if (l1.puzzle[x] != l2.puzzle[x])
			return false;
	return true;
}

class BurnedPancakePuzzle : public SearchEnvironment<BurnedPancakePuzzleState, unsigned> {
public:
	BurnedPancakePuzzle(unsigned s);
	BurnedPancakePuzzle(unsigned size, const std::vector<unsigned> op_order); // used to set action order

	~BurnedPancakePuzzle();
	void GetSuccessors(const BurnedPancakePuzzleState &state, std::vector<BurnedPancakePuzzleState> &neighbors) const;
	void GetActions(const BurnedPancakePuzzleState &state, std::vector<unsigned> &actions) const;
	unsigned GetAction(const BurnedPancakePuzzleState &s1, const BurnedPancakePuzzleState &s2) const;
	void ApplyAction(BurnedPancakePuzzleState &s, unsigned a) const;
	bool InvertAction(unsigned &a) const;

	virtual uint64_t GetStateHash(const BurnedPancakePuzzleState &s) const;

	double HCost(const BurnedPancakePuzzleState &state1, const BurnedPancakePuzzleState &state2) const;
	double Memory_Free_HCost(const BurnedPancakePuzzleState &state1, const std::vector<int> &goal_locs) const;
	double HCost(const BurnedPancakePuzzleState &state1) const;

	double GCost(const BurnedPancakePuzzleState &, const BurnedPancakePuzzleState &) const {return 1.0;}
	double GCost(const BurnedPancakePuzzleState &, const unsigned &) const { return 1.0; }

	bool GoalTest(const BurnedPancakePuzzleState &state, const BurnedPancakePuzzleState &goal) const;

	bool GoalTest(const BurnedPancakePuzzleState &s) const;

	uint64_t GetActionHash(unsigned act) const;
	void StoreGoal(BurnedPancakePuzzleState &); // stores the locations for the given goal state

//	virtual const std::string GetName();
	std::vector<unsigned> Get_Op_Order(){return operators;}

	/** Returns stored goal state if it is stored.**/
	BurnedPancakePuzzleState Get_Goal(){
		if (!goal_stored) {
			fprintf(stderr, "ERROR: Call to Get_Goal when no goal stored\n");
			exit(1);
		}
		return goal;
	}

	void ClearGoal(){} // clears the current stored information of the goal

	bool IsGoalStored() const {return goal_stored;} // returns if a goal is stored or not

	/**
	Changes the ordering of operators to the new inputted order
	**/
	void Change_Op_Order(const std::vector<unsigned> op_order);

	// currently not drawing anything
//	void OpenGLDraw() const{}
//	void OpenGLDraw(const BurnedPancakePuzzleState &) const {}
//	void OpenGLDraw(const BurnedPancakePuzzleState &, const unsigned &) const {}
//	void OpenGLDraw(const BurnedPancakePuzzleState&, const BurnedPancakePuzzleState&, float) const {}

	/**
	**/
//	static void Create_Random_BurnedPancake_Puzzles(std::vector<BurnedPancakePuzzleState> &puzzle_vector, unsigned size, unsigned num_puzzles);
//
//	static int read_in_pancake_puzzles(const char *filename, bool first_counter, unsigned size, unsigned max_puzzles, std::vector<BurnedPancakePuzzleState> &puzzle_vector);
//
//	bool State_Check(const BurnedPancakePuzzleState &to_check) {
//		if (to_check.puzzle.size() != size)
//			return false;
//
//		return true;
//	}
//
//	bool Path_Check(BurnedPancakePuzzleState start, BurnedPancakePuzzleState goal, std::vector<unsigned> &actions);

	/**
	Returns a possible ordering of the operators. The orders are in a "lexicographic"
	with the original ordering being 2, 3, ..., num_pancakes. This is therefore the order
	returned with a call of order_num=0. The default ordering used when a BurnedPancakePuzzle
	environment is created is num_pancakes, ..., 2 which is returned with a call of
	num_pancakes! -1.
	**/
	static std::vector<unsigned> Get_Puzzle_Order(int64_t order_num, unsigned num_pancakes);

	void Set_Use_Memory_Free_Heuristic(bool to_use){use_memory_free = to_use;}

private:

	std::vector<unsigned> operators;
	bool goal_stored; // whether a goal is stored or not
	bool use_memory_free;

	BurnedPancakePuzzleState goal;
	std::vector<int> goal_locations;
	unsigned size;


	uint64_t Factorial(int val) const
	{
		static uint64_t table[21] =
		{ 1ll, 1ll, 2ll, 6ll, 24ll, 120ll, 720ll, 5040ll, 40320ll, 362880ll, 3628800ll, 39916800ll, 479001600ll,
			6227020800ll, 87178291200ll, 1307674368000ll, 20922789888000ll, 355687428096000ll,
			6402373705728000ll, 121645100408832000ll, 2432902008176640000ll };
		if (val > 20)
			return (uint64_t)-1;
		return table[val];
	}
};

//typedef UnitSimulation<BurnedPancakePuzzleState, unsigned, BurnedPancake> BurnedPancakeSimulation;
#endif
