#ifndef PANCAKE_H
#define PANCAKE_H

#include <stdint.h>
#include <iostream>
#include "SearchEnvironment.h"
#include "PermutationPuzzleEnvironment.h"
#include <sstream>
#include "Permutations.h"

typedef unsigned PancakePuzzleAction;

template <int N>
class PancakePuzzleState {
public:
	PancakePuzzleState() {
		Reset();
	}
	size_t size() const { return N; }
	void FinishUnranking() {}
	void Reset()
	{
		for (unsigned int x = 0; x < N; x++)
			puzzle[x] = x;
	}
	int puzzle[N];
};

template <int N>
static std::ostream& operator <<(std::ostream & out, const PancakePuzzleState<N> &loc)
{
	for (unsigned int x = 0; x < loc.size(); x++)
		out << +loc.puzzle[x] << " ";
	return out;
}

template <int N>
static bool operator==(const PancakePuzzleState<N> &l1, const PancakePuzzleState<N> &l2)
{
	for (unsigned int x = 0; x < l1.size(); x++)
		if (l1.puzzle[x] != l2.puzzle[x])
			return false;
	return true;
}

template <int N>
static bool operator!=(const PancakePuzzleState<N> &l1, const PancakePuzzleState<N> &l2)
{
	return !(l1==l2);
}

template <int N>
class PancakePuzzle : public PermutationPuzzle::PermutationPuzzleEnvironment<PancakePuzzleState<N>, PancakePuzzleAction> {
public:
	PancakePuzzle(int gap = 0);
	PancakePuzzle(const std::vector<unsigned> op_order); // used to set action order

	~PancakePuzzle();
	void GetSuccessors(const PancakePuzzleState<N> &state, std::vector<PancakePuzzleState<N>> &neighbors) const;
	void GetActions(const PancakePuzzleState<N> &state, std::vector<unsigned> &actions) const;
	PancakePuzzleAction GetAction(const PancakePuzzleState<N> &s1, const PancakePuzzleState<N> &s2) const;
	PancakePuzzleAction GetAction(const PancakePuzzleState<N> &l1, point3d p) const;
	void ApplyAction(PancakePuzzleState<N> &s, PancakePuzzleAction a) const;
	bool InvertAction(PancakePuzzleAction &a) const;

	virtual uint64_t GetMaxHash() const;
	virtual uint64_t GetStateHash(const PancakePuzzleState<N> &node);
	virtual void GetStateFromHash(uint64_t parent, PancakePuzzleState<N> &s) const;

	double HCost(const PancakePuzzleState<N> &state1, const PancakePuzzleState<N> &state2) const;
	double DefaultH(const PancakePuzzleState<N> &state1) const;
	double DefaultH(const PancakePuzzleState<N> &state1, const std::vector<int> &goal_locs) const;
	double HCost(const PancakePuzzleState<N> &state1) const;

	double GCost(const PancakePuzzleState<N> &s1, const PancakePuzzleState<N> &s2) const
	{
		if (!real) return 1.0;
		PancakePuzzleAction a;
		a = GetAction(s1, s2);
		return GCost(s1, a);
	}
	double GCost(const PancakePuzzleState<N> &, const PancakePuzzleAction &a) const
	{
		if (!real) return 1.0;
		else // bigger a is more pancakes - more expensive
			return 1.0+0.1*(static_cast<double>(a)/static_cast<double>(N));
	}

	bool GoalTest(const PancakePuzzleState<N> &state, const PancakePuzzleState<N> &goal) const;

	bool GoalTest(const PancakePuzzleState<N> &s) const;

	uint64_t GetActionHash(PancakePuzzleAction act) const;
	void StoreGoal(PancakePuzzleState<N> &); // stores the locations for the given goal state

	virtual const std::string GetName();
	std::vector<PancakePuzzleAction> Get_Op_Order(){return operators;}

	/** Returns stored goal state if it is stored.**/
	PancakePuzzleState<N> Get_Goal(){
		if (!goal_stored) {
			fprintf(stderr, "ERROR: Call to Get_Goal when no goal stored\n");
			exit(1);
		}
		return goal;
	}

	void ClearGoal(){} // clears the current stored information of the goal

	bool IsGoalStored()const {return goal_stored;} // returns if a goal is stored or not

	/**
	Changes the ordering of operators to the new inputted order
	**/
	void Change_Op_Order(const std::vector<PancakePuzzleAction> op_order);

	// currently not drawing anything
//	void OpenGLDraw() const{}
//	void OpenGLDraw(const PancakePuzzleState<N> &) const;
//	void OpenGLDraw(const PancakePuzzleState<N> &, const PancakePuzzleAction &) const {}
//	void OpenGLDraw(const PancakePuzzleState<N>&, const PancakePuzzleState<N>&, float) const {}

	void Draw(Graphics::Display &display) const;
	void Draw(Graphics::Display &display, const PancakePuzzleState<N>&) const;
	void Draw(Graphics::Display &display, const PancakePuzzleAction&) const;
	void Draw(Graphics::Display &display,
			  const PancakePuzzleState<N> &from,
			  const PancakePuzzleState<N> &to, float p) const;

	/**
	**/
	static void Create_Random_Pancake_Puzzles(std::vector<PancakePuzzleState<N>> &puzzle_vector, unsigned num_puzzles);

	static int read_in_pancake_puzzles(const char *filename, bool first_counter, unsigned max_puzzles, std::vector<PancakePuzzleState<N>> &puzzle_vector);

	bool State_Check(const PancakePuzzleState<N> &to_check)
	{
		if (to_check.size() != N)
			return false;

		return true;
	}

	bool Path_Check(PancakePuzzleState<N> start, PancakePuzzleState<N> goal, std::vector<PancakePuzzleAction> &actions);

	/**
	Returns a possible ordering of the operators. The orders are in a "lexicographic"
	with the original ordering being 2, 3, ..., num_pancakes. This is therefore the order
	returned with a call of order_num=0. The default ordering used when a PancakePuzzle
	environment is created is num_pancakes, ..., 2 which is returned with a call of
	num_pancakes! -1.
	**/
	static std::vector<PancakePuzzleAction> Get_Puzzle_Order(int64_t order_num, unsigned num_pancakes);

	void Set_Use_Memory_Free_Heuristic(bool to_use){use_memory_free = to_use;}
	void Set_Use_Dual_Lookup( bool to_use ) { use_dual_lookup = to_use; };
	void SetUseRealValueEdges(bool use) { real = use; }
	bool pruneActions;
private:
	bool real = false;
	std::vector<PancakePuzzleAction> operators;
	mutable std::vector<PancakePuzzleAction> actCache;
	bool goal_stored; // whether a goal is stored or not
	bool use_memory_free;
	bool use_dual_lookup;
	int gap;
	PancakePuzzleState<N> goal;
	std::vector<int> goal_locations;
	//unsigned size;
};


namespace std {
	
	template <int N>
	struct hash<PancakePuzzleState<N>>
	{
		std::size_t operator()(const PancakePuzzleState<N>& p) const
		{
			return PancakePuzzle<N>::Hash(p);
		}
	};
	
}


template <int N>
PancakePuzzle<N>::PancakePuzzle(int gap)
:gap(gap)
{
	assert(N >= 2);
	
	// assign the default operator ordering
	for (unsigned i = N; i >= 2; i--)
		operators.push_back(i);
	
	goal_stored = false;
	use_memory_free = true;
	use_dual_lookup = true;
	pruneActions = false;
}

template <int N>
PancakePuzzle<N>::PancakePuzzle(const std::vector<PancakePuzzleAction> op_order)
:gap(0)
{
	Change_Op_Order(op_order);
	
	goal_stored = false;
	use_memory_free = false;
	use_dual_lookup = true;
	pruneActions = false;
}

template <int N>
PancakePuzzle<N>::~PancakePuzzle()
{
	ClearGoal();
}

template <int N>
const std::string PancakePuzzle<N>::GetName(){
	std::string s = "Pancake("+std::to_string(N)+")";
	return s;
//	std::stringstream name;
//	name << std::to_string(N);
//	name << "Pancake(" ;
	
//	if (PDB_distincts.size() > 0)
//	{
//		name << ", PDBS:";
//		for (unsigned i = 0; i < PDB_distincts.size(); i++)
//		{
//			name << " <";
//			for (unsigned j = 0; j < PDB_distincts[i].size() - 1; j++) {
//				name << PDB_distincts[i][j];
//				name << ", ";
//			}
//			name << PDB_distincts[i].back();
//			name << ">";
//		}
//		if (use_memory_free)
//			name << ", Memory-Free Heuristic";
//	}
//	if (use_memory_free)
//	{
//		name << ", Memory-Free Heuristic";
//	}
//	else {
//		name << ",No Heuristic";
//	}
	
//	name << ", Op Order: ";
//	for (unsigned op_num = 0; op_num < operators.size() - 1; op_num++){
//		name << operators[op_num];
//		name << ", ";
//	}
//	name << operators.back();
//	return name.str();
}

template <int N>
void PancakePuzzle<N>::GetSuccessors(const PancakePuzzleState<N> &parent,
								  std::vector<PancakePuzzleState<N>> &children) const
{
	children.resize(0);
	if (!pruneActions)
	{
		// all operators are applicable in all states
		for (unsigned i = 0; i < operators.size(); i++)
		{
			children.push_back(parent); // adds a copy of the state to the stack
			ApplyAction(children.back(), operators[i]);
		}
	}
	else {
		GetActions(parent, actCache);
		for (unsigned i = 0; i < actCache.size(); i++)
		{
			children.push_back(parent); // adds a copy of the state to the stack
			ApplyAction(children.back(), actCache[i]);
		}
	}
}

template <int N>
void PancakePuzzle<N>::GetActions(const PancakePuzzleState<N> &s, std::vector<PancakePuzzleAction> &actions) const
{
	actions.resize(0);
	if (!pruneActions)
	{
		// all operators are applicable in all states
		for (unsigned i = 0; i < operators.size(); i++)
		{
			actions.push_back(operators[i]);
		}
	}
	else {
		bool skip = true;
		for (unsigned i = N; i >= 2; i--)
		{
			if (s.puzzle[i-1] == i-1 && skip)
				continue;
			skip = false;
			actions.push_back(i);
		}
	}
}

template <int N>
PancakePuzzleAction PancakePuzzle<N>::GetAction(const PancakePuzzleState<N> &parent, const PancakePuzzleState<N> &child) const
{
	PancakePuzzleAction current_action;
	bool are_equal = false;
	
	assert(child.size() == N);
	assert(parent.size() == N);
	PancakePuzzleState<N> parentCopy = parent;
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

template <int N>
void PancakePuzzle<N>::ApplyAction(PancakePuzzleState<N> &s, PancakePuzzleAction action) const
{
	assert(s.size() == N);
	assert(action > 1 && action <= N);
	
	int upper = 0;
	int lower = action - 1;
	int temp;
	// performs pancake flipping
	while(upper < lower)
	{
		temp = s.puzzle[upper];
		s.puzzle[upper] = s.puzzle[lower];
		s.puzzle[lower] = temp;
		upper++;
		lower--;
	}
}

template <int N>
bool PancakePuzzle<N>::InvertAction(PancakePuzzleAction &a) const
{
	// ever action is self-inverse
	assert(a > 1 && a <= N);
	return true;
}

template <int N>
double PancakePuzzle<N>::HCost(const PancakePuzzleState<N> &state) const
{
	if (!goal_stored)
	{
		fprintf(stderr, "ERROR: HCost called with a single state and goal is not stored.\n");
		exit(1);
	}
//	if (state.size() != N)
//	{
//		fprintf(stderr, "ERROR: HCost called with a single state with wrong size.\n");
//		exit(1);
//	}
	double h_cost = 0;
	
	//	// use PDB heuristic
	//	if (PDB.size() > 0) {
	//		if (PDB.size() != PDB_distincts.size()) {
	//			fprintf(stderr, "ERROR: HCost called with a single state, no use of memory free heuristic, and invalid setup of pattern databases.\n");
	//			exit(1);
	//		}
	//		h_cost = std::max(PDB_Lookup(state), h_cost);
	//	}
	
	// use memory-free heuristic
	if (use_memory_free)
	{
		h_cost =  std::max(DefaultH(state, goal_locations), h_cost);
	}
	//	// if no heuristic
	//	else if (PDB.size()==0) {
	//		if (goal == state)
	//			return 0;
	//		else
	//			return 1;
	//	}
	
	return h_cost;
}

template <int N>
double PancakePuzzle<N>::HCost(const PancakePuzzleState<N> &state, const PancakePuzzleState<N> &goal_state) const
{
//	if (state.size() != N)
//	{
//		fprintf(stderr, "ERROR: HCost called with state with wrong size.\n");
//		exit(1);
//	}
//	if (goal_state.size() != N)
//	{
//		fprintf(stderr, "ERROR: HCost called with goal with wrong size.\n");
//		exit(1);
//	}
	
	//	if ( use_dual_lookup ) {
	//		// Note: This lookup only works if all PDB's have the same goal
	//		//   (or alternatively there is only one PDB)
	//
	//		// sanity check
	//		assert( IsGoalStored() );
	//
	//		// beta is a remapping of the elements to the goal
	//		std::vector<int> beta;
	//		beta.resize( size );
	//		for ( unsigned int i = 0; i < size; i++ )
	//			beta[goal_state.puzzle[i]] = goal.puzzle[i];
	//
	//		PancakePuzzleState<N> t = state;
	//		// remap t with respect to beta
	//		for ( unsigned int i = 0; i < size; i++ )
	//			t.puzzle[i] = beta[state.puzzle[i]];
	//
	//		return PDB_Lookup( t );
	//	}
	
	if (use_memory_free)
	{
		//assert(!"This code allocates a vector; re-write to be more efficient");
		static std::vector<int> goal_locs(N);
		for (unsigned i = 0; i < N; i++)
		{
			goal_locs[goal_state.puzzle[i]] = i;
		}
		return DefaultH(state, goal_locs);
	}
	
	if (state == goal_state)
		return 0.0;
	return 1.0;
}

template <int N>
double PancakePuzzle<N>::DefaultH(const PancakePuzzleState<N> &state) const
{
	return DefaultH(state, goal_locations);
}

template <int N>
double PancakePuzzle<N>::DefaultH(const PancakePuzzleState<N> &state, const std::vector<int> &goal_locs) const
{
//	if (state.size() != N)
//	{
//		fprintf(stderr, "ERROR: HCost called with state with wrong size.\n");
//		exit(1);
//	}
	
	double h_count = 0.0;
	unsigned i = 0;
	for (; i < N - 1; i++)
	{
		if ((goal_locs[state.puzzle[i]] < gap) || (goal_locs[state.puzzle[i+1]] < gap))
			continue;
		int diff = goal_locs[state.puzzle[i]] - goal_locs[state.puzzle[i+1]];
		if (diff > 1 || diff < -1)
			h_count++;
	}
	if ((unsigned) goal_locs[state.puzzle[i]]!= N -1)
		h_count++;
	
	return h_count;
}

template <int N>
bool PancakePuzzle<N>::GoalTest(const PancakePuzzleState<N> &state, const PancakePuzzleState<N> &theGoal) const
{
	return (state == theGoal);
}

template <int N>
bool PancakePuzzle<N>::GoalTest(const PancakePuzzleState<N> &s) const
{
	if (!goal_stored)
	{
		fprintf(stderr, "ERROR: GoalTest called with a single state and goal is not stored.\n");
		exit(1);
	}
	if (s.size() != N)
	{
		fprintf(stderr, "ERROR: GoalTest called with a single state with wrong size.\n");
		exit(1);
	}
	return (s == goal);
}

template <int N>
uint64_t PancakePuzzle<N>::GetActionHash(PancakePuzzleAction act) const
{
	return (uint64_t) act;
}

template <int N>
void PancakePuzzle<N>::StoreGoal(PancakePuzzleState<N> &g)
{
	//assert(g.puzzle.size() == size);
	
	goal = g;
	goal_stored = true;
	
	goal_locations.resize(N);
	for (unsigned i = 0; i < N; i++)
	{
		goal_locations[goal.puzzle[i]] = i;
	}
}

template <int N>
void PancakePuzzle<N>::Change_Op_Order(const std::vector<PancakePuzzleAction> op_order)
{
	operators.clear();
	
	if (op_order.size() != N - 1)
	{
		fprintf(stderr, "ERROR: Not enough operators in operator sequence for construction of PancakePuzzle\n");
		exit(1);
	}
	
	bool all_ops[op_order.size()];
	
	for (unsigned i = 0; i < N; i++)
	{
		all_ops[i] = false;
	}
	
	for (unsigned i = 0; i < op_order.size(); i++)
	{
		if (op_order[i] < 2 || op_order[i] > N)
		{
			fprintf(stderr, "ERROR: Invalid operator included in construction of PancakePuzzle\n");
			exit(1);
		}
		all_ops[op_order[i] - 2] = true;
	}
	
	for (unsigned i = 0; i < op_order.size(); i++)
	{
		if (!all_ops[i])
		{
			fprintf(stderr, "ERROR: Missing operator %u in construction of PancakePuzzle\n", i+2);
			exit(1);
		}
	}
	// assign the default operator ordering
	for (unsigned i = 0; i < op_order.size(); i++)
		operators.push_back(op_order[i]);
}


template <int N>
void PancakePuzzle<N>::Create_Random_Pancake_Puzzles(std::vector<PancakePuzzleState<N>> &puzzle_vector, unsigned num_puzzles)
{
	
	std::map<uint64_t, uint64_t> puzzle_map; // used to ensure uniqueness
	
	PancakePuzzle my_puzz(N);
	
	unsigned count = 0;
	
	std::vector<int> perm;
	PancakePuzzleState<N> potential_puzz(N);
	while (count < num_puzzles)
	{
		perm = PermutationPuzzle::PermutationPuzzleEnvironment<PancakePuzzleState<N>, PancakePuzzleAction>::Get_Random_Permutation(N);
		
		// construct puzzle
		for (unsigned i = 0; i < N; i++)
		{
			potential_puzz.puzzle[i] = perm[i];
		}
		
		uint64_t next_hash = my_puzz.GetStateHash(potential_puzz);
		
		
		// make sure is not a duplicate
		if (puzzle_map.find(next_hash) != puzzle_map.end())
		{
			continue;
		}
		
		puzzle_map[next_hash] = next_hash;
		puzzle_vector.push_back(potential_puzz);
		count++;
	}
	
}

template <int N>
int PancakePuzzle<N>::read_in_pancake_puzzles(const char *filename, bool puzz_num_start, unsigned max_puzzles, std::vector<PancakePuzzleState<N>> &puzzles)
{
	
	std::vector<std::vector<int> > permutations;
	PermutationPuzzle::PermutationPuzzleEnvironment<PancakePuzzleState<N>, PancakePuzzleAction>::Read_In_Permutations(filename, N, max_puzzles, permutations, puzz_num_start);
	
	// convert permutations into PancakePuzzleState<N>s
	for (unsigned i = 0; i < permutations.size(); i++)
	{
		PancakePuzzleState<N> new_state(N);
		
		for (unsigned j = 0; j < N; j++)
		{
			new_state.puzzle[j] = permutations[i][j];
		}
		puzzles.push_back(new_state);
	}
	return 0;
}

template <int N>
bool PancakePuzzle<N>::Path_Check(PancakePuzzleState<N> start, PancakePuzzleState<N> theGoal, std::vector<PancakePuzzleAction> &actions)
{
	if (start.size() != N || theGoal.size() != N)
		return false;
	
	for (unsigned i = 0; i < actions.size(); i++)
	{
		if (actions[i] < 2 || actions[i] > N)
			return false;
		ApplyAction(start, actions[i]);
	}
	
	if (start == theGoal)
		return true;
	
	return false;
}

template <int N>
std::vector<unsigned> PancakePuzzle<N>::Get_Puzzle_Order(int64_t order_num, unsigned num_pancakes)
{
	std::vector<unsigned> ops;
	assert(order_num >= 0);
	assert(num_pancakes > 0);
	
	
	std::vector<int64_t> op_nums(num_pancakes -1);
	
	int64_t num_left = 1;
	for (int64_t x = num_pancakes - 2; x >= 0; x--)
	{
		op_nums[x] = order_num % num_left;
		order_num /= num_left;
		num_left++;
		
		for (int64_t y = x+1; y < num_pancakes-1; y++)
		{
			if (op_nums[y] >= op_nums[x])
			{
				op_nums[y]++;
			}
		}
	}
	
	std::vector<bool> actions;
	
	for (unsigned i = 0; i < num_pancakes - 1; i++)
	{
		actions.push_back(false);
	}
	
	for (unsigned i = 0; i < num_pancakes - 1; i++)
	{
		ops.push_back(op_nums[i] + 2);
		actions[op_nums[i]] = true;
	}
	
	for (unsigned i = 0; i < num_pancakes - 1; i++)
	{
		assert(actions[i]);
	}
	return ops;
}

template <int N>
uint64_t PancakePuzzle<N>::GetMaxHash() const
{
	Permutations<N> c;
	return c.MaxRank();
}

template <int N>
uint64_t PancakePuzzle<N>::GetStateHash(const PancakePuzzleState<N> &node)
{
	Permutations<N> c;
	return c.Rank(node.puzzle);
}

template <int N>
void PancakePuzzle<N>::GetStateFromHash(uint64_t parent, PancakePuzzleState<N> &s) const
{
	Permutations<N> c;
	return c.Unrank(parent, s.puzzle);
}

//template <int N>
//void PancakePuzzle<N>::OpenGLDraw(const PancakePuzzleState<N> &pps) const
//{
//	double count = pps.size();
//	double widthUnit = 1.5/count;
//	
//	for (size_t y = 0; y < pps.size(); y++)
//	{
//		for (int x = 0; x <= pps.puzzle[y]; x++)
//		{
//			glColor3f(pps.puzzle[y]/count, 0, 1-pps.puzzle[y]/count);
//			DrawBox(-pps.puzzle[y]*widthUnit/4+x*widthUnit/2,
//					-1+widthUnit*y+widthUnit/2, 0,
//					widthUnit/2);
//		}
//	}
//}

template <int N>
PancakePuzzleAction PancakePuzzle<N>::GetAction(const PancakePuzzleState<N> &s, point3d p) const
{
//	if (p.y > -0.5 && p.y < 0.5)
//	{
//		return N-(1-p.y-0.5)*N+1;
//	}
//	return 0;
//
	for (int x = 0; x < N; x++)
	{
		float t = 0.6*s.puzzle[x]/(N-1.0f)+0.1;
		float v = 1.0f/(N-1.0f);
		Graphics::rect r(-t, -0.5f+(x)*v, t, -0.5f+(x+1)*v);
		if (PointInRect(p, r))
		{
			return x+1;
		}
//		else {
//			std::cout << "[" << x << "]" << p << " missed [" << r << "]\n";
//		}
	}
	return 0;
}

template <int N>
void PancakePuzzle<N>::Draw(Graphics::Display &display) const
{
	// no baseline drawing
	float v = 1.0f/(N-1.0f);
	display.FillRect(Graphics::rect(-1, -0.5f+(N)*v, 1, -0.5f+(N+1)*v), Colors::darkgray);
}

template <int N>
void PancakePuzzle<N>::Draw(Graphics::Display &display, const PancakePuzzleState<N>&s) const
{
	float v = 1.0f/(N-1.0f);
	for (int x = 0; x < N; x++)
	{
		float t = 0.6*s.puzzle[x]/(N-1.0f)+0.1;
		display.FillRect(Graphics::rect(-t, -0.5f+(x)*v, t, -0.5f+(x+1)*v), rgbColor(1, t, 0));
		display.FrameRect(Graphics::rect(-t, -0.5f+(x)*v, t, -0.5f+(x+1)*v), Colors::black, v*0.1f);
	}
}

template <int N>
void PancakePuzzle<N>::Draw(Graphics::Display &display, const PancakePuzzleAction &a) const
{
	float v = 1.0f/(N-1.0f);
	Graphics::point p1(0, -0.5+v/2);
	Graphics::point p2(0, -0.5f+(a)*v-v/2);
//		display.FillRect(Graphics::rect(-t, -0.5f+(x+1)*v, t, -0.5f+x*v), rgbColor(1, t, 0));
//		display.FrameRect(Graphics::rect(-t, -0.5f+(x+1)*v, t, -0.5f+x*v), Colors::black, v*0.1f);
	display.DrawArrow(p2, p1, v*0.2, Colors::blue);
	Graphics::point p3(-0.7, -0.5f+(a)*v);
	Graphics::point p4(0.7, -0.5f+(a)*v);
	Graphics::point p5(0.7+0.3, -0.5f+(a)*v-0.3);
	display.DrawLine(p3, p4, v*0.3, Colors::black);
	display.DrawLine(p4, p5, v*0.3, Colors::black);
}

template <int N>
void PancakePuzzle<N>::Draw(Graphics::Display &display,
							const PancakePuzzleState<N> &from,
							const PancakePuzzleState<N> &to, float p) const
{
	Graphics::rect fromRect[N];
	Graphics::rect toRect[N];

	float v = 1.0f/(N-1.0f);
	for (int x = 0; x < N; x++)
	{
		float t = 0.6*from.puzzle[x]/(N-1.0f)+0.1;
		Graphics::rect r(-t, -0.5f+(x)*v, t, -0.5f+(x+1)*v);
		fromRect[from.puzzle[x]] = r;

		t = 0.6*to.puzzle[x]/(N-1.0f)+0.1;
		v = 1.0f/(N-1.0f);
		r = Graphics::rect(-t, -0.5f+(x)*v, t, -0.5f+(x+1)*v);
		toRect[to.puzzle[x]] = r;
	}
	for (int x = 0; x < N; x++)
	{
		float t = 0.6*x/(N-1.0f)+0.1;
		fromRect[x].lerp(toRect[x], p);
		display.FillRect(fromRect[x], rgbColor(1, t, 0));
		display.FrameRect(fromRect[x], Colors::black, v*0.1f);
	}
}


#endif
