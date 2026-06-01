/*
 *  MNPuzzle.h
 *  hog2
 *
 *  Created by Nathan Sturtevant on 5/9/07.
 *  Copyright 2007 Nathan Sturtevant, University of Alberta. All rights reserved.
 *
 */

#ifndef MNPUZZLE_H
#define MNPUZZLE_H

#include <stdint.h>
#include <iostream>
#include "SearchEnvironment.h"
#include "PermutationPuzzleEnvironment.h"
#include "UnitSimulation.h"
#include "GraphEnvironment.h"
#include "Graph.h"
#include "GraphEnvironment.h"
#include <sstream>
#include <array>
#include "LexPermutationPDB.h"

enum puzzleWeight {
    kDWSTP,
    kUnitWeight,
    kSquared,
    kSquareRoot,
    kSquarePlusOneRoot,
    kUnitPlusFrac
};

template <int width, int height>
class MNPuzzleState {
public:
	MNPuzzleState()
	{
		Reset();
	}
	void Reset()
	{
		for (size_t x = 0; x < size(); x++)
			puzzle[x] = x;
		blank = 0;
	}
	size_t size() const { return width*height; }
	void FinishUnranking()
	{
		for (size_t x = 0; x < size(); x++)
		{
			if (puzzle[x] == 0)
			{
				blank = x;
				return;
			}
		}
	}
	bool operator<(const MNPuzzleState &b)
	{
		for (size_t x = 0; x < size(); x++)
			if (puzzle[x] != b.puzzle[x])
				return puzzle[x] < b.puzzle[x];
		return false;
	}

	unsigned int blank;
	std::array<int, width*height> puzzle;

	puzzleWeight weight;
};

namespace std {
	
	template <int w, int h>
	struct hash<MNPuzzleState<w, h>>
	{
		std::size_t operator()(const MNPuzzleState<w, h>& k) const
		{
			size_t hash = 0;
			for (int x = 0; x < w*h; x++)
				hash = hash*w*h + k.puzzle[x];
			return hash;
		}
	};
}


/**
 * Note, direction "kLeft" indicates that the blank is being moved to the left.
 * That is, a tile is being slid right into the blank position. The other
 * actions are defined similarly.
 */
enum slideDir {
	kLeft, kUp, kDown, kRight, kNoSlide
};

template <int width, int height>
static std::ostream& operator <<(std::ostream & out, const MNPuzzleState<width, height> &loc)
{
	out << "(" << width << "x" << height << ")";
	for (size_t x = 0; x < loc.size(); x++)
		out << loc.puzzle[x] << " ";
	return out;
}

static std::ostream& operator <<(std::ostream & out, const slideDir &loc)
{
	switch (loc)
	{
		case kLeft: out << "Left"; break;
		case kRight: out << "Right"; break;
		case kUp: out << "Up"; break;
		case kDown: out << "Down"; break;
        case kNoSlide: out << "NoSlide"; break;
	}
	return out;
}


template <int width, int height>
static bool operator==(const MNPuzzleState<width, height> &l1, const MNPuzzleState<width, height> &l2)
{
	for (unsigned int x = 0; x < l1.size(); x++)
	{
		if (l1.puzzle[x] > 0 || l2.puzzle[x] > 0) // don't have to check the blank
			if (l1.puzzle[x] != l2.puzzle[x])
				return false;
	}
	return true;
}

template <int width, int height>
static bool operator!=(const MNPuzzleState<width, height> &l1, const MNPuzzleState<width, height> &l2)
{
	return !(l1 == l2);
}

template <int width, int height>
class MNPuzzle : public PermutationPuzzle::PermutationPuzzleEnvironment<MNPuzzleState<width, height>, slideDir> {
public:
	MNPuzzle();
	MNPuzzle(const std::vector<slideDir> op_order); // used to set action order
	~MNPuzzle();
	void SetMiddleState(MNPuzzleState<width, height> &s);
    void SetPuzzleWeight(int puzzleW);
    void SetTerrainSize(double s);
    void SetInputWeight(double w);
	void SetWeighted(puzzleWeight w) { weight = w; }
	puzzleWeight GetWeighted() const { return weight; }
	void GetSuccessors(const MNPuzzleState<width, height> &stateID, std::vector<MNPuzzleState<width, height>> &neighbors) const;
	void GetActions(const MNPuzzleState<width, height> &stateID, std::vector<slideDir> &actions) const;
	void GetActions(const MNPuzzleState<width, height> &nodeID, std::vector<slideDir> &actions, const slideDir &lastAction) const;
	slideDir GetAction(const MNPuzzleState<width, height> &s1, const MNPuzzleState<width, height> &s2) const;
	slideDir GetAction(const MNPuzzleState<width, height> &l1, point3d p);
	void ApplyAction(MNPuzzleState<width, height> &s, slideDir a) const;
	bool InvertAction(slideDir &a) const;
	static unsigned GetParity(const MNPuzzleState<width, height> &state);

	OccupancyInterface<MNPuzzleState<width, height>, slideDir> *GetOccupancyInfo() { return 0; }
	double HCost(const MNPuzzleState<width, height> &state1, const MNPuzzleState<width, height> &state2) const;
	double HCost(const MNPuzzleState<width, height> &state1) const;
	double DefaultH(const MNPuzzleState<width, height> &s) const;
	double DCost(const MNPuzzleState<width, height> &state1, const MNPuzzleState<width, height> &state2) const;
	double DCost(const MNPuzzleState<width, height> &state1) const;

	double GCost(const MNPuzzleState<width, height> &state1, const MNPuzzleState<width, height> &state2) const;
	double GCost(const MNPuzzleState<width, height> &, const slideDir &) const;
	bool GoalTest(const MNPuzzleState<width, height> &state, const MNPuzzleState<width, height> &goal) const;

	bool GoalTest(const MNPuzzleState<width, height> &s) const;

	double AdditiveGCost(const MNPuzzleState<width, height> &, const slideDir &) const;
	bool InPattern(int tile) const;
	void SetPattern(const std::vector<int> &pattern);
	void GetStateFromPDBHash(uint64_t hash, MNPuzzleState<width, height> &s,
							 int count, const std::vector<int> &pattern,
							 std::vector<int> &dual);

	uint64_t GetActionHash(slideDir act) const;
//	void OpenGLDraw() const;
//	void OpenGLDraw(const MNPuzzleState<width, height> &s) const;
//	void OpenGLDraw(const MNPuzzleState<width, height> &l1, const MNPuzzleState<width, height> &l2, float v) const;
//	void OpenGLDraw(const MNPuzzleState<width, height> &, const slideDir &) const { /* currently not drawing moves */ }
	void Draw(Graphics::Display &display, const MNPuzzleState<width, height>&) const;
	void Draw(Graphics::Display &display, const MNPuzzleState<width, height> &l1, const MNPuzzleState<width, height> &l2, float v) const;

	
	void StoreGoal(MNPuzzleState<width, height> &); // stores the locations for the given goal state

	/** Returns stored goal state if it is stored.**/
	MNPuzzleState<width, height> Get_Goal(){
		if (!goal_stored) {
			fprintf(stderr, "ERROR: Call to Get_Goal when no goal stored\n");
			exit(1);
		}
		return goal;
	}

	virtual const std::string GetName();

	void ClearGoal(); // clears the current stored information of the goal

	bool IsGoalStored() const {return goal_stored;} // returns if a goal is stored or not
	Graph *GetGraph();

	/**
	Changes the ordering of operators to the new inputted order
	**/
	void Change_Op_Order(const std::vector<slideDir> op_order);

	/**
	Creates num_puzzles random MN puzzles of the specified size and stores them in
	puzzle-vector. All random puzzles are unique and solvable for the standard goal
	of 0, 1, 2, ..., MN - 1. The method assumes that num_puzzles is no bigger than
	the total number of solvable problems for that size.
	**/
	static void Create_Random_MN_Puzzles(MNPuzzleState<width, height> &goal, std::vector<MNPuzzleState<width, height>> &puzzle_vector, unsigned num_puzzles);

	/**
	Reads in the the desired number of puzzles from the given filename with the
	given dimensions and stores them in puzzle_vector. Only the first max_puzzles
	are stored. first_counter should be set to true if the first element on every
	row is the index of the puzzle. The input format is the entries in each of
	the puzzle positions separated by a space, with the optional index entry
	described above.
	**/
	static int read_in_mn_puzzles(const char *filename, bool first_counter, unsigned max_puzzles, std::vector<MNPuzzleState<width, height>> &puzzle_vector);

	/**
	Returns a possible ordering of the operators. The orders are in a "lexicographic"
	with the original ordering being Up, Left, Right, Down. This is therefore the order
	returned with a call of order_num=0. This initial ordering is that used by Korf in
	his original IDA* experiments. The ordering originally used in HOG is returned with a
	call of order_num=15.
	**/
	static std::vector<slideDir> Get_Op_Order_From_Hash(int order_num);
	std::vector<slideDir> Get_Op_Order(){return ops_in_order;}

	static MNPuzzleState<width, height> Generate_Random_Puzzle();

	virtual void GetStateFromHash(MNPuzzleState<width, height> &s, uint64_t hash) const;
	uint64_t GetStateHash(const MNPuzzleState<width, height> &s) const;
	uint64_t GetMaxStateHash() const;
	
	bool State_Check(const MNPuzzleState<width, height> &to_check);

	unsigned Get_Num_Of_Columns(){return width;}
	unsigned Get_Num_Of_Rows(){return height;}

	void Set_Use_Manhattan_Heuristic(bool to_use){use_manhattan = to_use;}
private:
	bool pattern[width*height];
	std::vector<std::vector<slideDir> > operators; // stores the operators applicable at each blank position
	std::vector<slideDir> ops_in_order;
	bool goal_stored; // whether a goal is stored or not
	bool use_manhattan;
	puzzleWeight weight;
	
	// stores the heuristic value of each tile-position pair indexed by the tile value (0th index is empty)
	std::vector<std::vector<unsigned> > h_increment;
	MNPuzzleState<width, height> goal;

	// stores a random state on the WA* solution path, to create a swamp area on the path using heuristic
    MNPuzzleState<width, height> middleState;

    // stores the terrain size of the swamped area.
    double swampedTerrainSize;

    // stores the input weight of the search
    double inputWeight;
};

template <int width, int height>
class GraphPuzzleDistanceHeuristic : public GraphDistanceHeuristic {
public:
	GraphPuzzleDistanceHeuristic(MNPuzzle<width, height> &mnp, Graph *graph, int count);
	double HCost(const graphState &state1, const graphState &state2) const;
private:
	MNPuzzle<width, height> puzzle;
};

/*
This function sets the puzzleWeight.
*/
template <int width, int height>
void MNPuzzle<width, height>::SetPuzzleWeight(int puzzleW){
    switch (puzzleW)
    {
    case 0:
        weight = kUnitWeight;
        break;
    case 1:
        weight = kSquareRoot;
        break;
    case 2:
        weight = kSquared;
        break;
    case 3:
        weight = kUnitPlusFrac;
        break;
    case 4:
        weight = kSquarePlusOneRoot;
        break;
    case 5:
        weight = kDWSTP;
        break;
    
    default:
        break;
    }
}

/*
sets the middleState to the random state s on the WA* solution path.
middleState is used to create a swamp area.
*/
template <int width, int height>
void MNPuzzle<width, height>::SetMiddleState(MNPuzzleState<width, height> &s)
{
    for(int i=0; i<width*height; i++) middleState.puzzle[i]=s.puzzle[i];
    middleState.blank = s.blank;
    middleState.weight = s.weight;
}

/*
sets the swampedTerrainSize to be used later.
*/
template <int width, int height>
void MNPuzzle<width, height>::SetTerrainSize(double s)
{
    swampedTerrainSize=s;
}

/*
sets the input weight of the search.
*/
template <int width, int height>
void MNPuzzle<width, height>::SetInputWeight(double w)
{
    inputWeight=w;
}


template <int width, int height>
class STPLexPDB : public LexPermutationPDB<MNPuzzleState<width, height>, slideDir, MNPuzzle<width, height>, 8> {
public:
	STPLexPDB(MNPuzzle<width, height> *e, const MNPuzzleState<width, height> &s, const std::vector<int> &distincts)
	:LexPermutationPDB<MNPuzzleState<width, height>, slideDir, MNPuzzle<width, height>, 8>(e, s, distincts)
	{}
	virtual uint64_t GetPDBHash(const MNPuzzleState<width, height> &s, int threadID = 0) const
	{
		int locs[width*height];
		int dual[width*height];
		
		// find item locations
		for (unsigned int x = 0; x < s.size(); x++)
		{
			if (s.puzzle[x] != -1)
				dual[s.puzzle[x]] = x;
		}
		for (size_t x = 0; x < this->distinct.size(); x++)
		{
			locs[x] = dual[this->distinct[x]];
		}
		
		uint64_t hashVal = 0;
		int numEntriesLeft = (int)s.size();
		
		for (unsigned int x = 0; x < distinct.size(); x++)
		{
			hashVal += locs[x]*FactorialUpperK(numEntriesLeft-1, s.size()-distinct.size());
			numEntriesLeft--;
			
			// decrement locations of remaining items
			for (unsigned y = x; y < distinct.size(); y++)
			{
				if (locs[y] > locs[x])
					locs[y]--;
			}
		}
		return hashVal;
	}
protected:
	using PermutationPDB<MNPuzzleState<width, height>, slideDir, MNPuzzle<width, height>, 8>::distinct;
	using LexPermutationPDB<MNPuzzleState<width, height>, slideDir, MNPuzzle<width, height>, 8>::Factorial;
	using LexPermutationPDB<MNPuzzleState<width, height>, slideDir, MNPuzzle<width, height>, 8>::FactorialUpperK;
};


template <int width, int height>
MNPuzzle<width, height>::MNPuzzle()
{
	weight = kUnitWeight;
	// stores applicable operators at each of the width*height positions
	Change_Op_Order(Get_Op_Order_From_Hash(15)); // Right, Left, Down, Up is default operator ordering
	goal_stored = false;
	use_manhattan = true;
}

template <int width, int height>
MNPuzzle<width, height>::MNPuzzle(const std::vector<slideDir> op_order)
{
	Change_Op_Order(op_order);
	goal_stored = false;
	use_manhattan = true;
	weight = kUnitWeight;
}

template <int width, int height>
MNPuzzle<width, height>::~MNPuzzle()
{
	ClearGoal();
}

template <int width, int height>
void MNPuzzle<width, height>::Change_Op_Order(std::vector<slideDir> op_order)
{
	operators.clear();
	ops_in_order.clear();
	
	bool up_act = false;
	bool down_act = false;
	bool left_act = false;
	bool right_act = false;
	
	if (op_order.size() != 4)
	{
		fprintf(stderr, "ERROR: Not enough operators in operator sequence for construction of MNPuzzle\n");
		exit(1);
	}
	
	for (unsigned int op_num = 0; op_num < 4; op_num++)
	{
		if (op_order[op_num] == kUp)
		{
			up_act = true;
		}
		else if (op_order[op_num] == kDown)
		{
			down_act = true;
		}
		else if (op_order[op_num] == kLeft)
		{
			left_act = true;
		}
		else if (op_order[op_num] == kRight)
		{
			right_act = true;
		}
	}
	
	if (!up_act || !down_act || !left_act || !right_act)
	{
		fprintf(stderr, "ERROR: Invalid operator sequence for construction of MNPuzzle\n");
		exit(1);
	}
	
	for (unsigned i = 0; i < op_order.size(); i++)
	{
		ops_in_order.push_back(op_order[i]);
	}
	
	// stores applicable operators at each of the width*height positions
	std::vector<slideDir> ops(4);
	for (unsigned int blank = 0; blank < width*height; blank++) {
		ops.resize(0);
		
		for (unsigned int op_num = 0; op_num < 4; op_num++)
		{
			if (op_order[op_num] == kUp && blank > width - 1)
			{
				ops.push_back(kUp);
			}
			if (op_order[op_num] == kLeft && blank % width > 0)
			{
				ops.push_back(kLeft);
			}
			if (op_order[op_num] == kRight && (blank % width) < width - 1)
			{
				ops.push_back(kRight);
			}
			if (op_order[op_num] == kDown && blank < width * height - width)
			{
				ops.push_back(kDown);
			}
		}
		
		operators.push_back(ops);
	}
}

template <int width, int height>
const std::string MNPuzzle<width, height>::GetName(){
	std::string s = "STP(";
	s += std::to_string(width);
	s += ",";
	s += std::to_string(height);
	s += ")";
	return s;
}

template <int width, int height>
Graph *MNPuzzle<width, height>::GetGraph()
{
	Graph *g = new Graph();
	
	for (int x = 0; x < 362880; x++)
	{
		node *n;
		g->AddNode(n = new node(""));
		n->SetLabelF(GraphSearchConstants::kXCoordinate, (32768.0-random()%65536)/32768.0);
		n->SetLabelF(GraphSearchConstants::kYCoordinate, (32768.0-random()%65536)/32768.0);
		n->SetLabelF(GraphSearchConstants::kZCoordinate, (32768.0-random()%65536)/32768.0);
	}
	for (int x = 0; x < 362880; x++)
	{
		MNPuzzleState<width, height> t;
		GetStateFromHash(t, x);
		std::vector<slideDir> moves;
		GetActions(t, moves);
		g->GetNode(x)->SetLabelF(GraphSearchConstants::kZCoordinate, GetParity(t));
		for (unsigned int y = 0; y < moves.size(); y++)
		{
			ApplyAction(t, moves[y]);
			uint64_t hash = GetStateHash(t);
			InvertAction(moves[y]);
			ApplyAction(t, moves[y]);
			if (!g->FindEdge(x, hash))
				g->AddEdge(new edge(x, hash, 1));
		}
	}
	return g;
}

template <int width, int height>
void MNPuzzle<width, height>::StoreGoal(MNPuzzleState<width, height> &s)
{
	// makes sure goal contains all legal
	bool goal_tester[width*height];
	for (unsigned int i = 0; i < width*height; i++)
	{
		goal_tester[i] = false;
	}
	
	for (unsigned int i = 0; i < width*height; i++)
	{
		goal_tester[s.puzzle[i]] = true;
	}
	
	for (unsigned int i = 0; i < width*height; i++)
	{
		if (!goal_tester[i])
		{
			printf("INVALID GOAL\n");
			assert(goal_tester[i]);
		}
	}
	
	h_increment.resize(width*height);
	
	for (unsigned int i = 1; i < width*height; i++)
	{
		h_increment[i].resize(width*height);
	}
	
	for (unsigned goal_pos = 0; goal_pos < width*height; goal_pos++)
	{
		unsigned tile = s.puzzle[goal_pos];
		if (tile == 0)
			continue;
		
		for (unsigned pos = 0; pos < width*height; pos++)
		{
			h_increment[tile][pos] = abs((int) (goal_pos % width) - (int)(pos % width)); // difference in column
			h_increment[tile][pos] += abs((int) (goal_pos / width) - (int)(pos / width)); // difference in row
		}
	}
	
	goal_stored = true;
	
	goal = s;
}

template <int width, int height>
bool MNPuzzle<width, height>::GoalTest(const MNPuzzleState<width, height> &s) const
{
	return (s == goal);
}

template <int width, int height>
void MNPuzzle<width, height>::ClearGoal()
{
	if (goal_stored)
	{
		goal_stored = false;
	}
}

template <int width, int height>
void MNPuzzle<width, height>::GetSuccessors(const MNPuzzleState<width, height> &stateID,
							 std::vector<MNPuzzleState<width, height>> &neighbors) const
{
	neighbors.resize(0);
	
	for (unsigned int i = 0; i < operators[stateID.blank].size(); i++)
	{
		neighbors.push_back(stateID);
		ApplyAction(neighbors.back(), operators[stateID.blank][i]);
	}
}

template <int width, int height>
void MNPuzzle<width, height>::GetActions(const MNPuzzleState<width, height> &stateID,
										 std::vector<slideDir> &actions, const slideDir &lastAction) const
{
	slideDir d = lastAction;
	bool result = InvertAction(d);
	assert(result);
	actions.resize(0);
	for (unsigned int i = 0; i < operators[stateID.blank].size(); i++)
	{
		if (!(operators[stateID.blank][i] == d))
			actions.push_back(operators[stateID.blank][i]);
	}
}


template <int width, int height>
void MNPuzzle<width, height>::GetActions(const MNPuzzleState<width, height> &stateID, std::vector<slideDir> &actions) const
{
	actions.resize(0);
	for (unsigned int i = 0; i < operators[stateID.blank].size(); i++)
	{
		actions.push_back(operators[stateID.blank][i]);
	}
}

template <int width, int height>
slideDir MNPuzzle<width, height>::GetAction(const MNPuzzleState<width, height> &a, const MNPuzzleState<width, height> &b) const
{
	int row1 = a.blank%width;
	int col1 = a.blank/height;
	int row2 = b.blank%width;
	int col2 = b.blank/height;
	if (row1 == row2)
	{
		if (col1 > col2)
			return kUp;
		return kDown;
	}
	if (row1 > row2)
		return kLeft;
	return kRight;
}

template <int width, int height>
void MNPuzzle<width, height>::ApplyAction(MNPuzzleState<width, height> &s, slideDir a) const
{
	// we actually do the swap to maintain consistency when using abstract states
	// (these contain -1 in some positions, including possibly the blank position.)
	switch (a)
	{
		case kUp:
			if (s.blank >= width)
			{
				int tmp = s.puzzle[s.blank];
				s.puzzle[s.blank] = s.puzzle[s.blank-width];
				s.blank -= width;
				s.puzzle[s.blank] = tmp;
			}
			else {
				printf("Invalid up operator\n");
				assert(false);
				exit(0);
			}
			break;
		case kDown:
			if (s.blank < s.size() - width)
			{
				int tmp = s.puzzle[s.blank];
				s.puzzle[s.blank] = s.puzzle[s.blank+width];
				s.blank += width;
				s.puzzle[s.blank] = tmp;
			}
			else {
				printf("Invalid down operator\n");
				assert(false);
				exit(0);
			}
			break;
		case kRight:
			if ((s.blank%width) < width-1)
			{
				int tmp = s.puzzle[s.blank];
				s.puzzle[s.blank] = s.puzzle[s.blank+1];
				s.blank += 1;
				s.puzzle[s.blank] = tmp;
			}
			else {
				printf("Invalid right operator\n");
				assert(false);
				exit(0);
			}
			break;
		case kLeft:
			if ((s.blank%width) > 0)
			{
				int tmp = s.puzzle[s.blank];
				s.puzzle[s.blank] = s.puzzle[s.blank-1];
				s.blank -= 1;
				s.puzzle[s.blank] = tmp;
			}
			else {
				printf("Invalid left operator\n");
				assert(false);
				exit(0);
			}
			break;
        case kNoSlide:
            break;
	}
}

template <int width, int height>
bool MNPuzzle<width, height>::InvertAction(slideDir &a) const
{
	switch (a)
	{
		case kLeft: a = kRight; break;
		case kUp: a = kDown; break;
		case kDown: a = kUp; break;
		case kRight: a = kLeft; break;
		case kNoSlide: a = kNoSlide; break;
	}
	return true;
}

template <int width, int height>
double MNPuzzle<width, height>::HCost(const MNPuzzleState<width, height> &state1) const
{
	return DefaultH(state1);
}

template <int width, int height>
double MNPuzzle<width, height>::HCost(const MNPuzzleState<width, height> &state1, const MNPuzzleState<width, height> &state2) const
{
	if (goal_stored)
		return HCost(state1);
	
	double hval = 0;
	
	if (use_manhattan)
	{
		double man_dist = 0;
		int xloc[width*height];
		int yloc[width*height];
		
		for (unsigned int x = 0; x < width; x++)
		{
			for (unsigned int y = 0; y < height; y++)
			{
				xloc[state2.puzzle[x + y*width]] = x;
				yloc[state2.puzzle[x + y*width]] = y;
			}
		}
		for (unsigned int x = 0; x < width; x++)
		{
			for (unsigned int y = 0; y < height; y++)
			{
				if (state1.puzzle[x + y*width] != 0)
				{
					double absDist = (abs((int)(xloc[state1.puzzle[x + y*width]] - x))
									 + abs((int)(yloc[state1.puzzle[x + y*width]] - y)));
					double movingTile = state1.puzzle[x + y*width];
					switch (weight)
					{
						case kDWSTP: man_dist += absDist; break;
						case kUnitWeight: man_dist += absDist; break;
						case kUnitPlusFrac: man_dist += absDist*(1.0+1.0/(1.0+movingTile)); break;
						case kSquared: man_dist += absDist*(movingTile)*(movingTile); break;
						case kSquareRoot: man_dist += absDist*sqrt(movingTile); break;
						case kSquarePlusOneRoot:
						{
							double tmp = movingTile;
							tmp = sqrt(tmp*tmp+1);
							man_dist += tmp*absDist;
						}
					}
				}
			}
		}
		hval = std::max(hval, man_dist);
	}
	
	return hval;
}

template <int width, int height>
double MNPuzzle<width, height>::DefaultH(const MNPuzzleState<width, height> &state) const
{
	double man_dist = 0;
	// increments amound for each tile location
	// this calculates the Manhattan distance
	for (unsigned loc = 0; loc < width*height; loc++)
	{
		if (state.puzzle[loc] > 0)
			man_dist += h_increment[state.puzzle[loc]][loc];
	}
	return man_dist;
}

template <int width, int height>
double MNPuzzle<width, height>::DCost(const MNPuzzleState<width, height> &state1,
									  const MNPuzzleState<width, height> &state2) const
{
	double hval = 0;
	double man_dist = 0;
	int xloc[width*height];
	int yloc[width*height];
	
	for (unsigned int x = 0; x < width; x++)
	{
		for (unsigned int y = 0; y < height; y++)
		{
			xloc[state2.puzzle[x + y*width]] = x;
			yloc[state2.puzzle[x + y*width]] = y;
		}
	}
	for (unsigned int x = 0; x < width; x++)
	{
		for (unsigned int y = 0; y < height; y++)
		{
			if (state1.puzzle[x + y*width] != 0)
			{
				man_dist += (abs((int)(xloc[state1.puzzle[x + y*width]] - x))
								 + abs((int)(yloc[state1.puzzle[x + y*width]] - y)));
			}
		}
	}
	hval = std::max(hval, man_dist);
}

template <int width, int height>
double MNPuzzle<width, height>::DCost(const MNPuzzleState<width, height> &state1) const
{
	return DefaultH(state1);
}



static int costs[25] =
{
	3, 5, 4, 7, 10, 5, 3, 3, 8, 9, 2, 10, 10, 1, 2, 1, 1, 4, 7, 9, 6, 10, 2, 8, 8
};


template <int width, int height>
double MNPuzzle<width, height>::GCost(const MNPuzzleState<width, height> &a, const MNPuzzleState<width, height> &b) const
{
	// Options:
	// * tile squared
	// square root of tile
	// tile itself
	switch (weight)
	{
		case kDWSTP:
        {
            double tmpH = HCost(middleState, b);
            if(flesseq(tmpH, swampedTerrainSize+5) && fgreatereq(tmpH, swampedTerrainSize-5)){
                return 2.0*inputWeight-1.0;
            }
            else{
                return 1.0;
            }
        }
		case kUnitWeight: return 1;
		case kUnitPlusFrac: return (1.0+1.0/(1.0+a.puzzle[b.blank]));
		case kSquared: return a.puzzle[b.blank]*a.puzzle[b.blank];
		case kSquareRoot: return sqrt(a.puzzle[b.blank]);
		case kSquarePlusOneRoot: return sqrt(1+a.puzzle[b.blank]*a.puzzle[b.blank]);
	}
	return 1;
}

template <int width, int height>
void MNPuzzle<width, height>::SetPattern(const std::vector<int> &pattern)
{
	for (int x = 0; x < width*height; x++)
		this->pattern[x] = false;
	for (int i : pattern)
		this->pattern[i] = true;
}

template <int width, int height>
bool MNPuzzle<width, height>::InPattern(int tile) const
{
	if (tile == -1)
		return false;
	return pattern[tile] == true;
}

template <int width, int height>
double MNPuzzle<width, height>::AdditiveGCost(const MNPuzzleState<width, height> &s, const slideDir &d) const
{
	switch (d)
	{
		case kLeft: return InPattern(s.puzzle[s.blank-1])?GCost(s,d):0;
		case kUp: return InPattern(s.puzzle[s.blank-width])?GCost(s,d):0;
		case kDown: return InPattern(s.puzzle[s.blank+width])?GCost(s,d):0;
		case kRight: return InPattern(s.puzzle[s.blank+1])?GCost(s,d):0;
		case kNoSlide: assert(false);
	}
	return 0;
}

template <int width, int height>
double MNPuzzle<width, height>::GCost(const MNPuzzleState<width, height> &s, const slideDir &d) const
{
	switch (weight)
	{
		// case kDWSTP: //TODO: DWSTP, we need new state b, to find its h(mid,b).
        // {
        //     return -1;
        // }
		case kUnitWeight: return 1;
		case kUnitPlusFrac:
		{
			switch (d)
			{
				case kLeft: return 1.0+1.0/(1.0+s.puzzle[s.blank-1]);
				case kUp: return 1.0+1.0/(1.0+s.puzzle[s.blank-width]);
				case kDown: return 1.0+1.0/(1.0+s.puzzle[s.blank+width]);
				case kRight: return 1.0+1.0/(1.0+s.puzzle[s.blank+1]);
                default: assert(!"Illegal move"); break;
			}
		}
		case kSquared:
		{
			switch (d)
			{
				case kLeft: return s.puzzle[s.blank-1]*s.puzzle[s.blank-1];
				case kUp: return s.puzzle[s.blank-width]*s.puzzle[s.blank-width];
				case kDown: return s.puzzle[s.blank+width]*s.puzzle[s.blank+width];
				case kRight: return s.puzzle[s.blank+1]*s.puzzle[s.blank+1];
                default: assert(!"Illegal move"); break;
			}
		}
		case kSquareRoot:
		{
			switch (d)
			{
				case kLeft: return sqrt(s.puzzle[s.blank-1]);
				case kUp: return sqrt(s.puzzle[s.blank-width]);
				case kDown: return sqrt(s.puzzle[s.blank+width]);
				case kRight: return sqrt(s.puzzle[s.blank+1]);
                default: assert(!"Illegal move"); break;
			}
		}
		case kSquarePlusOneRoot:
		{
			switch (d)
			{
				case kLeft: return sqrt(1+s.puzzle[s.blank-1]*s.puzzle[s.blank-1]);
				case kUp: return sqrt(1+s.puzzle[s.blank-width]*s.puzzle[s.blank-width]);
				case kDown: return sqrt(1+s.puzzle[s.blank+width]*s.puzzle[s.blank+width]);
				case kRight: return sqrt(1+s.puzzle[s.blank+1]*s.puzzle[s.blank+1]);
                default: assert(!"Illegal move"); break;
			}
		}
	}

	return 1;
}

template <int width, int height>
bool MNPuzzle<width, height>::GoalTest(const MNPuzzleState<width, height> &state, const MNPuzzleState<width, height> &theGoal) const
{
	return (state == theGoal);
}

template <int width, int height>
uint64_t MNPuzzle<width, height>::GetActionHash(slideDir act) const
{
	switch (act)
	{
		case kUp: return 0;
		case kDown: return 1;
		case kRight: return 2;
		case kLeft: return 3;
        default: return 4;
	}
}

template <int width, int height>
void MNPuzzle<width, height>::GetStateFromPDBHash(uint64_t hash, MNPuzzleState<width, height> &s,
								   int count, const std::vector<int> &pattern,
								   std::vector<int> &dual)
{
	uint64_t hashVal = hash;
	dual.resize(pattern.size());
	
	int numEntriesLeft = count-pattern.size()+1;
	for (int x = pattern.size()-1; x >= 0; x--)
	{
		dual[x] = hashVal%numEntriesLeft;
		hashVal /= numEntriesLeft;
		numEntriesLeft++;
		for (int y = x+1; y < pattern.size(); y++)
		{
			if (dual[y] >= dual[x])
				dual[y]++;
		}
	}
	s.puzzle.resize(count);
	std::fill(s.puzzle.begin(), s.puzzle.end(), -1);
	for (int x = 0; x < dual.size(); x++)
	{
		s.puzzle[dual[x]] = pattern[x];
		if (pattern[x] == 0)
			s.blank = dual[x];
	}
}


//template <int width, int height>
//void MNPuzzle<width, height>::OpenGLDraw() const
//{
//}

//void DrawTile(float x, float y, char c1, char c2, int w, int h);
//void DrawFrame(int w, int h);

template <int width, int height>
void MNPuzzle<width, height>::Draw(Graphics::Display &display, const MNPuzzleState<width, height>&s) const
{
	float squareSize = std::max(width, height)+1;
	squareSize = 2.0f/squareSize;
	float xOrigin = (0-((float)width)/2.0f)*squareSize;
	float yOrigin = (0-((float)height)/2.0f)*squareSize;
	char txt[10];
	display.FillRect({xOrigin, yOrigin, xOrigin+(width)*squareSize, yOrigin+(height)*squareSize}, Colors::gray);
	// background stiple
	{
		float t = 0;
		while (fless(t, (width)*squareSize+(height)*squareSize))
		{
			float xLoc1 = xOrigin+t;
			float yLoc1 = yOrigin;
			if (t > (width)*squareSize)
			{
				yLoc1 += t - (width)*squareSize;
				xLoc1 = xOrigin + (width)*squareSize;
			}
			float xLoc2 = xOrigin;
			float yLoc2 = yOrigin+t;
			if (t > (height)*squareSize)
			{
				xLoc2 += t - (height)*squareSize;
				yLoc2 = yOrigin + (height)*squareSize;
			}
			display.DrawLine({xLoc1, yLoc1}, {xLoc2, yLoc2}, 0.04f*squareSize, Colors::darkgray);
			t+=0.15f*squareSize;
		}
	}
	display.FrameRect({xOrigin, yOrigin, xOrigin+(width)*squareSize, yOrigin+(height)*squareSize}, Colors::lightgray, 0.04*squareSize);
	for (unsigned int y = 0; y < height; y++)
	{
		for (unsigned int x = 0; x < width; x++)
		{
			if (s.puzzle[x+y*width] != 0)
			{
				sprintf(txt, "%d", s.puzzle[x+y*width]);
				Graphics::rect bound(xOrigin+x*squareSize, yOrigin+y*squareSize, xOrigin+(x+1)*squareSize, yOrigin+(y+1)*squareSize);
				display.FillRect(bound, Colors::white);
				display.FrameRect(bound, Colors::darkblue, 0.01f*squareSize);
				bound = bound.inset(0.02f*squareSize);
				display.FrameRect(bound, Colors::lightgray, 0.02f*squareSize);
				if (s.puzzle[x+y*width] > 0)
				{
					display.DrawText(txt,
									 {xOrigin+x*squareSize+1.03f*squareSize/2.f, yOrigin+y*squareSize+1.03f*squareSize/2.f},
									 Colors::lightblue, squareSize/2.0f, Graphics::textAlignCenter, Graphics::textBaselineMiddle);
					display.DrawText(txt,
									 {xOrigin+x*squareSize+squareSize/2.f, yOrigin+y*squareSize+squareSize/2.f},
									 Colors::blue, squareSize/2.0f, Graphics::textAlignCenter, Graphics::textBaselineMiddle);
				}
			}
		}
	}
}

template <int width, int height>
slideDir MNPuzzle<width, height>::GetAction(const MNPuzzleState<width, height> &s, point3d p)
{
	int hitx=-1, hity=-1;
	// Find which location was hit
	{
		float squareSize = std::max(width, height)+1;
		squareSize = 2.0f/squareSize;
		float xOrigin = (0-((float)width)/2.0f)*squareSize;
		float yOrigin = (0-((float)height)/2.0f)*squareSize;
		char txt[10];
		for (unsigned int y = 0; y < height; y++)
		{
			for (unsigned int x = 0; x < width; x++)
			{
				if (s.puzzle[x+y*width] != 0)
				{
					Graphics::rect r = {xOrigin+x*squareSize, yOrigin+y*squareSize, xOrigin+(x+1)*squareSize, yOrigin+(y+1)*squareSize};
					if (Graphics::PointInRect(p, r))
					{
						hitx = x;
						hity = y;
						break;
					}
				}
			}
		}
	}
	if (hitx == -1 || hity == -1)
		return kNoSlide;

	int x = hitx, y = hity;
	if (s.puzzle[x+y*width] == 0)
		return kNoSlide;
	std::vector<slideDir> acts;
	GetActions(s, acts);
	auto tmp = s;
	for (auto a : acts)
	{
		ApplyAction(tmp, a);
		// If an action makes the place we clicked to be blank, that is the action we want
		if (tmp.puzzle[x+y*width] == 0)
		{
			return a;
		}
		this->UndoAction(tmp, a);
	}
	return kNoSlide;
}


template <int width, int height>
void MNPuzzle<width, height>::Draw(Graphics::Display &display, const MNPuzzleState<width, height> &s1, const MNPuzzleState<width, height> &s2, float v) const
{
	float squareSize = std::max(width, height)+1;
	squareSize = 2.0f/squareSize;
	float xOrigin = (0-((float)width)/2.0f)*squareSize;
	float yOrigin = (0-((float)height)/2.0f)*squareSize;
	char txt[10];
	display.FillRect({xOrigin, yOrigin, xOrigin+(width)*squareSize, yOrigin+(height)*squareSize}, Colors::gray);
	// background stiple
	{
		float t = 0;
		while (fless(t, (width)*squareSize+(height)*squareSize))
		{
			float xLoc1 = xOrigin+t;
			float yLoc1 = yOrigin;
			if (t > (width)*squareSize)
			{
				yLoc1 += t - (width)*squareSize;
				xLoc1 = xOrigin + (width)*squareSize;
			}
			float xLoc2 = xOrigin;
			float yLoc2 = yOrigin+t;
			if (t > (height)*squareSize)
			{
				xLoc2 += t - (height)*squareSize;
				yLoc2 = yOrigin + (height)*squareSize;
			}
			display.DrawLine({xLoc1, yLoc1}, {xLoc2, yLoc2}, 0.04f*squareSize, Colors::darkgray);
			t+=0.15f*squareSize;
		}
	}
	display.FrameRect({xOrigin, yOrigin, xOrigin+(width)*squareSize, yOrigin+(height)*squareSize}, Colors::lightgray, 0.04f*squareSize);
	for (unsigned int y = 0; y < height; y++)
	{
		for (unsigned int x = 0; x < width; x++)
		{
			if (s1.puzzle[x+y*width] == s2.puzzle[x+y*width])
			{
				if (s1.puzzle[x+y*width] != 0)
				{
					sprintf(txt, "%d", s1.puzzle[x+y*width]);
					Graphics::rect bound(xOrigin+x*squareSize, yOrigin+y*squareSize, xOrigin+(x+1)*squareSize, yOrigin+(y+1)*squareSize);
					display.FillRect(bound, Colors::white);
					display.FrameRect(bound, Colors::darkblue, 0.01f*squareSize);
					bound = bound.inset(0.02f*squareSize);
					display.FrameRect(bound, Colors::lightgray, 0.02f*squareSize);
					if (s1.puzzle[x+y*width] > 0)
					{
						display.DrawText(txt, {xOrigin+x*squareSize+1.03f*squareSize/2.f, yOrigin+y*squareSize+1.03f*squareSize/2.f}, Colors::lightblue, squareSize/2.0f, Graphics::textAlignCenter, Graphics::textBaselineMiddle);
						display.DrawText(txt, {xOrigin+x*squareSize+squareSize/2.f, yOrigin+y*squareSize+squareSize/2.f}, Colors::blue, squareSize/2.0f, Graphics::textAlignCenter, Graphics::textBaselineMiddle);
					}
				}
			}
			else if (s1.puzzle[x+y*width] != 0)
			{
				Graphics::point p1 = {xOrigin+x*squareSize, yOrigin+y*squareSize};
				Graphics::point p2;
				switch (GetAction(s1, s2))
				{
					case kUp:
						p2  = {xOrigin+x*squareSize, yOrigin+(y+1)*squareSize};
						break;
					case kDown:
						p2  = {xOrigin+x*squareSize, yOrigin+(y-1)*squareSize};
						break;
					case kLeft:
						p2  = {xOrigin+(x+1)*squareSize, yOrigin+(y)*squareSize};
						break;
					case kRight:
						p2  = {xOrigin+(x-1)*squareSize, yOrigin+(y)*squareSize};
						break;

					default: assert(!"action not found");
				}
				sprintf(txt, "%d", s1.puzzle[x+y*width]);
				Graphics::rect bound(p1.x*v+p2.x*(1-v), p1.y*v+p2.y*(1-v), p1.x*v+p2.x*(1-v)+squareSize, p1.y*v+p2.y*(1-v)+squareSize);
				display.FillRect(bound, Colors::white);
				display.FrameRect(bound, Colors::darkblue, 0.01f*squareSize);
				bound = bound.inset(0.02f*squareSize);
				display.FrameRect(bound, Colors::lightgray, 0.02f*squareSize);

				if (s1.puzzle[x+y*width] > 0)
				{
					display.DrawText(txt, {p1.x*v+p2.x*(1-v)+1.03f*squareSize/2.f, p1.y*v+p2.y*(1-v)+1.03f*squareSize/2.f}, Colors::lightblue, squareSize/2.0f, Graphics::textAlignCenter, Graphics::textBaselineMiddle);
					display.DrawText(txt, {p1.x*v+p2.x*(1-v)+squareSize/2.f, p1.y*v+p2.y*(1-v)+squareSize/2.f}, Colors::blue, squareSize/2.0f, Graphics::textAlignCenter, Graphics::textBaselineMiddle);
				}
			}
		}
	}
}


//template <int width, int height>
//void MNPuzzle<width, height>::OpenGLDraw(const MNPuzzleState<width, height> &s) const
//{
//	glEnable(GL_LINE_SMOOTH);
//	for (unsigned int y = 0; y < height; y++)
//	{
//		for (unsigned int x = 0; x < width; x++)
//		{
//			char c1=0, c2=0;
//			if (s.puzzle[x+y*width] > 9)
//				c1 = '0'+(((s.puzzle[x+y*width])/10)%10);
//			if (s.puzzle[x+y*width] > 0)
//				c2 = '0'+((s.puzzle[x+y*width])%10);
//			if (s.puzzle[x+y*width] == -1)
//				c1 = ' ';
//			DrawTile(x, y, c1, c2, width, height);
//		}
//	}
//	DrawFrame(width, height);
//}
//
//template <int width, int height>
//void MNPuzzle<width, height>::OpenGLDraw(const MNPuzzleState<width, height> &s1, const MNPuzzleState<width, height> &s2, float v) const
//{
//	glEnable(GL_LINE_SMOOTH);
//	for (unsigned int y = 0; y < height; y++)
//	{
//		for (unsigned int x = 0; x < width; x++)
//		{
//			char c1=0, c2=0;
//			if (s2.puzzle[x+y*width] > 9)
//				c1 = '0'+(((s2.puzzle[x+y*width])/10)%10);
//			if (s2.puzzle[x+y*width] > 0)
//				c2 = '0'+((s2.puzzle[x+y*width])%10);
//			if (s2.puzzle[x+y*width] == -1)
//				c1 = ' ';
//
//			if (s1.puzzle[x+y*width] == s2.puzzle[x+y*width])
//			{
//				DrawTile(x, y, c1, c2, width, height);
//			}
//			else {
//				switch (GetAction(s1, s2))
//				{
//					case kUp: DrawTile(x, (y-1)*v + (y)*(1-v), c1, c2, width, height); break;
//					case kDown: DrawTile(x, (y+1)*v + (y)*(1-v), c1, c2, width, height); break;
//					case kLeft: DrawTile((x)*(1-v)+(x-1)*v, y, c1, c2, width, height); break;
//					case kRight: DrawTile((x+1)*v+(x)*(1-v), y, c1, c2, width, height); break;
//					default: assert(!"action not found");
//				}
//			}
//		}
//	}
//	DrawFrame(width, height);
//}

/**
 Reads in MNPuzzle states from the given filename. Each line of the
 input file contains a puzzle state of the given width and height.
 The state is written in the form of listing the tile in each puzzle
 position from 1 to width*height. The blank is represented by the 0
 character.
 
 If any line is found to not be legitimate, either because it does
 not have the proper number of tiles, or the proper tiles, it is
 simply not added to the list of puzzles.
 
 The function will return 1 if reading from the given filename failed,
 and 0 otherwise.
 
 puzz_num_start - should be set as true if the first non-whitespace
 element on each line is the puzzle number, otherwise should be set
 to false.
 
 max_puzzles - the maximum number of puzzles that will be added to
 the puzzle list.
 **/
template <int width, int height>
int MNPuzzle<width, height>::read_in_mn_puzzles(const char *filename, bool puzz_num_start, unsigned int max_puzzles,
												std::vector<MNPuzzleState<width, height>> &puzzles)
{
	
	std::vector<std::vector<int> > permutations;
	PermutationPuzzle::PermutationPuzzleEnvironment<MNPuzzleState<width, height>, slideDir>::Read_In_Permutations(filename, width*height, max_puzzles, permutations, puzz_num_start);
	
	// convert permutations into MNPuzzleStates
	for (unsigned i = 0; i < permutations.size(); i++)
	{
		MNPuzzleState<width, height> new_state;
		
		for (unsigned j = 0; j < width*height; j++)
		{
			new_state.puzzle[j] = permutations[i][j];
			if (new_state.puzzle[j] == 0)
				new_state.blank = j;
		}
		puzzles.push_back(new_state);
	}
	return 0;
}

template <int width, int height>
GraphPuzzleDistanceHeuristic<width, height>::GraphPuzzleDistanceHeuristic(MNPuzzle<width, height> &mnp, Graph *graph, int count)
:GraphDistanceHeuristic(graph), puzzle(mnp)
{
	for (int x = 0; x < count /*10*/; x++)
	{
		AddHeuristic();
	}
}

template <int width, int height>
double GraphPuzzleDistanceHeuristic<width, height>::HCost(const graphState &state1, const graphState &state2) const
{
	MNPuzzleState<3, 3> a, b;
	puzzle.GetStateFromHash(a, state1);
	puzzle.GetStateFromHash(b, state2);
	double val = puzzle.HCost(a, b);
	
	for (unsigned int i=0; i < heuristics.size(); i++)
	{
		double hval = heuristics[i][state1]-heuristics[i][state2];
		if (hval < 0)
			hval = -hval;
		if (fgreater(hval,val))
			val = hval;
	}
	
	return val;
}


/**
 Randomly generates a puzzle of the specified dimensions
 and returns that puzzle.
 **/
template <int width, int height>
MNPuzzleState<width, height> MNPuzzle<width, height>::Generate_Random_Puzzle()
{
	MNPuzzleState<width, height> new_puzz;
	std::vector<int> permutation = PermutationPuzzle::PermutationPuzzleEnvironment<MNPuzzleState<width, height>, slideDir>::Get_Random_Permutation(width*height);
	
	for (unsigned i = 0; i < permutation.size(); i++)
	{
		new_puzz.puzzle[i] = permutation[i];
		if (permutation[i] == 0)
			new_puzz.blank = i;
	}
	
	return new_puzz;
}

template <int width, int height>
void MNPuzzle<width, height>::GetStateFromHash(MNPuzzleState<width, height> &s, uint64_t hash) const
{
	int count = width*height;
	int countm2 = width*height-2;
	uint64_t hashVal = hash;
	std::vector<int> dual(width*height-2);
	
	// unrank the locations of the first 10 tiles
	int numEntriesLeft = count-countm2+1;
	for (int x = countm2-1; x >= 0; x--)
	{
		dual[x] = hashVal%numEntriesLeft;
		hashVal /= numEntriesLeft;
		numEntriesLeft++;
		for (int y = x+1; y < countm2; y++)
		{
			if (dual[y] >= dual[x])
				dual[y]++;
		}
	}
	// clear puzzle locations
	for (int x = 0; x < count; x++)
	{
		s.puzzle[x] = -1;
	}
	// revert locations of tiles into positions in the puzzle
	for (int x = 0; x < countm2; x++)
	{
		s.puzzle[dual[x]] = x;
	}
	// reset the cache of the blanks location
	s.blank = dual[0];
	
	// now find the two -1's and assign them
	// to ensure the right parity
	int x = 0;
	int loc1 = -1, loc2 = -1;
	for (; x < count; x++)
	{
		if (s.puzzle[x] == -1)
		{
			loc1 = x;
			x++;
			break;
		}
	}
	for (; x < count; x++)
	{
		if (s.puzzle[x] == -1)
		{
			loc2 = x;
			break;
		}
	}
	assert(loc1 != -1 && loc2 != -1);
	// Choose an arbitrary ordering and then
	// check the parity. If it's wrong, we just
	// swap them and are guaranteed to get the right
	// parity.
	s.puzzle[loc1] = countm2;
	s.puzzle[loc2] = countm2+1;
	if (IsGoalStored())
	{
		if (GetParity(s) != MNPuzzle<width, height>::GetParity(goal))
		{
			s.puzzle[loc1] = countm2+1;
			s.puzzle[loc2] = countm2;
		}
	}
	else if (GetParity(s) == 1)
	{
		s.puzzle[loc1] = countm2+1;
		s.puzzle[loc2] = countm2;
	}
}

template <int width, int height>
uint64_t MNPuzzle<width, height>::GetStateHash(const MNPuzzleState<width, height> &s) const
{
	std::array<int, width*height-2> locs;// We only rank n-2 of n items; last two are fixed by the parity
	std::array<int, width*height> dual;
	
	// build the representation containing the item locations
	for (unsigned int x = 0; x < width*height; x++)
	{
		dual[s.puzzle[x]] = x;
	}
	// build an array with the locations of the first 10 items
	for (int x = 0; x < width*height-2; x++)
	{
		locs[x] = dual[x];
	}
	
	uint64_t hashVal = 0;
	int numEntriesLeft = width*height;
	
	// compute the lexographical ranking of the locations
	// of the first 10 tiles
	for (unsigned int x = 0; x < width*height-2; x++)
	{
		hashVal += locs[x]*PermutationPuzzle::PermutationPuzzleEnvironment<MNPuzzleState<width, height>, slideDir>::Factorial(numEntriesLeft-1)/2;
		numEntriesLeft--;
		
		// decrement locations of remaining items
		// to keep the numbering compact
		for (unsigned y = x; y < width*height-2; y++)
		{
			if (locs[y] > locs[x])
				locs[y]--;
		}
	}
	return hashVal;
}

template <int width, int height>
uint64_t MNPuzzle<width, height>::GetMaxStateHash() const
{
	int val = width*height;
	return PermutationPuzzle::PermutationPuzzleEnvironment<MNPuzzleState<width, height>, slideDir>::Factorial(val)/2;
}


//	if q is odd, the invariant is the parity (odd or even) of the number of pairs of pieces in reverse order (the parity of the permutation). For the order of the 15 pieces consider line 2 after line 1, etc., like words on a page.
//	if q is even, the invariant is the parity of the number of pairs of pieces in reverse order plus the row number of the empty square counting from bottom and starting from 0.
template <int width, int height>
unsigned MNPuzzle<width, height>::GetParity(const MNPuzzleState<width, height> &state)
{
	unsigned swaps = 0; // counts number of swaps
	for (unsigned x = 0; x < width*height; x++)
	{
		if (state.puzzle[x] == 0) // skip blank
			continue;
		for (unsigned y = x + 1; y < width*height; y++)
		{
			if (state.puzzle[y] == 0) // skip blank
				continue;
			if (state.puzzle[y] < state.puzzle[x])
				swaps++;
		}
	}
	// if odd num of columns
	if ((width % 2) == 1)
	{
		return swaps % 2;
	}
	
	// if even num of columns
	return (swaps + (width*height-state.blank-1)/width)%2;
}

template <int width, int height>
void MNPuzzle<width, height>::Create_Random_MN_Puzzles(MNPuzzleState<width, height> &goal, std::vector<MNPuzzleState<width, height>> &puzzle_vector, unsigned num_puzzles)
{
	std::map<uint64_t, uint64_t> puzzle_map; // used to ensure uniqueness
	
	MNPuzzle<width, height> my_puzz;
	
	unsigned count = 0;
	unsigned goal_parity = GetParity(goal);
	
	while (count < num_puzzles)
	{
		MNPuzzleState<width, height> next = MNPuzzle<width, height>::Generate_Random_Puzzle();
		uint64_t next_hash = my_puzz.GetStateHash(next);
		
		if (puzzle_map.find(next_hash) != puzzle_map.end())
		{
			continue;
		}
		
		// checks parity to make sure problem is solvable
		if (GetParity(next) == goal_parity)
		{
			puzzle_map[next_hash] = next_hash;
			puzzle_vector.push_back(next);
			count++;
		}
		
	}
}

template <int width, int height>
bool MNPuzzle<width, height>::State_Check(const MNPuzzleState<width, height> &to_check)
{
	if (to_check.blank >= width*height)
		return false;
	
	if (to_check.puzzle[to_check.blank] != 0)
		return false;
	
	return true;
}

template <int width, int height>
std::vector<slideDir> MNPuzzle<width, height>::Get_Op_Order_From_Hash(int order_num)
{
	std::vector<slideDir> ops;
	assert(order_num <= 23);
	assert(order_num >= 0);
	
	std::vector<int> op_nums(4);
	
	int num_left = 1;
	for (int x = 3; x >= 0; x--)
	{
		op_nums[x] = order_num % num_left;
		order_num /= num_left;
		num_left++;
		
		for (int y = x+1; y < 4; y++)
		{
			if (op_nums[y] >= op_nums[x])
			{
				op_nums[y]++;
			}
		}
	}
	
	bool up_act = false;
	bool down_act = false;
	bool left_act = false;
	bool right_act = false;

	for (unsigned i = 0; i < 4; i++)
	{
		if (op_nums[i] == 0)
		{
			ops.push_back(kUp);
			up_act = true;
		}
		else if (op_nums[i] == 1)
		{
			ops.push_back(kLeft);
			left_act = true;
		}
		else if (op_nums[i] == 2)
		{
			ops.push_back(kRight);
			right_act = true;
		}
		else if (op_nums[i] == 3)
		{
			ops.push_back(kDown);
			down_act = true;
		}
	}
	printf("\n");

	assert(up_act);
	assert(left_act);
	assert(right_act);
	assert(down_act);
	
	return ops;
}

#endif
