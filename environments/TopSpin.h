/*
 *  TopSpin.h
 *  hog2
 *
 *  Created by Nathan Sturtevant on 9/4/14.
 *  Copyright 2014 Nathan Sturtevant, University of Denver. All rights reserved.
 *
 */

#ifndef TopSpin_H
#define TopSpin_H

#include <stdint.h>
#include <iostream>
#include "SearchEnvironment.h"
#include "PermutationPuzzleEnvironment.h"
#include "GraphEnvironment.h"
#include "Graph.h"
#include <sstream>
#include <unordered_map>

template <int N>
class TopSpinState {
public:
	TopSpinState()
	{
//		puzzle.resize(N);
		for (unsigned int x = 0; x < N; x++)
			puzzle[x] = x;
	}
	void Reset()
	{
		for (unsigned int x = 0; x < N; x++)
			puzzle[x] = x;
	}
	size_t size() const { return N; }
	void FinishUnranking()
	{ }
//	std::vector<int> puzzle;
	int puzzle[N];
};

/**
 * Actions are the first tile that gets swapped.
 */
typedef int TopSpinAction;

template <int N>
static std::ostream& operator <<(std::ostream & out, const TopSpinState<N> &loc)
{
	for (unsigned int x = 0; x < N; x++)
		out << loc.puzzle[x] << " ";
	return out;
}

template <int N>
static bool operator==(const TopSpinState<N> &l1, const TopSpinState<N> &l2)
{
//	if (l1.puzzle.size() != l2.puzzle.size())
//		return false;
	for (unsigned int x = 0; x < N; x++)
	{
		if (l1.puzzle[(x)%N] != l2.puzzle[(x)%N])
			return false;
	}
	return true;
}

template <int N>
static bool operator!=(const TopSpinState<N> &l1, const TopSpinState<N> &l2)
{
	return !(l1 == l2);
}

template <int N, int k>
class TopSpin : public PermutationPuzzle::PermutationPuzzleEnvironment<TopSpinState<N>, TopSpinAction> {
public:
	TopSpin();
	~TopSpin();
	void SetWeighted(bool w) { weighted = w; }
	bool GetWeighted() { return weighted; }
	void SetPruneSuccessors(bool val)
	{ if (val) ComputeMovePruning(); pruneSuccessors = val; history.resize(0); }
	void GetSuccessors(const TopSpinState<N> &stateID, std::vector<TopSpinState<N>> &neighbors) const;
	void GetActions(const TopSpinState<N> &stateID, std::vector<TopSpinAction> &actions) const;
	void ApplyAction(TopSpinState<N> &s, TopSpinAction a) const;
	void UndoAction(TopSpinState<N> &s, TopSpinAction a) const;
	bool InvertAction(TopSpinAction &a) const;
	static unsigned GetParity(TopSpinState<N> &state);
	void SetPattern(const std::vector<int> &pattern);
	double AdditiveGCost(const TopSpinState<N> &s, const TopSpinAction &d);

	double HCost(const TopSpinState<N> &state1, const TopSpinState<N> &state2) const;

	double GCost(const TopSpinState<N> &state1, const TopSpinState<N> &state2) const;
	double GCost(const TopSpinState<N> &, const TopSpinAction &) const;
	bool GoalTest(const TopSpinState<N> &state, const TopSpinState<N> &goal) const;
	bool GoalTest(const TopSpinState<N> &s) const;
	void GetGoals(std::vector<TopSpinState<N>> &goals)
	{
		goals.clear();
		TopSpinState<N> s;
		for (int x = 0; x < N; x++)
		{
			for (int y = 0; y < N; y++)
				s.puzzle[y] = (y+x)%N;
			goals.push_back(s);
		}
	}
	
	//void LoadPDB(char *fname, const std::vector<int> &tiles, bool additive);

	uint64_t GetActionHash(TopSpinAction act) const;
//	void OpenGLDraw() const;
//	void OpenGLDraw(const TopSpinState<N> &s) const;
//	void OpenGLDraw(const TopSpinState<N> &l1, const TopSpinState<N> &l2, float v) const;
//	void OpenGLDraw(const TopSpinState<N> &, const TopSpinAction &) const { /* currently not drawing moves */ }
	void StoreGoal(TopSpinState<N> &); // stores the locations for the given goal state

	/** Returns stored goal state if it is stored.**/
	TopSpinState<N> Get_Goal(){
		return goal;
	}

	virtual const std::string GetName();

	void ClearGoal() { } // clears the current stored information of the goal

	bool IsGoalStored() const {return true;} // returns if a goal is stored or not

	bool State_Check(const TopSpinState<N> &to_check) { return true; }

	void PrintHStats()
	{
		printf("-\t");
		for (int x = 0; x < hDist.size(); x++)
			printf("%d\t", x);
		printf("\n");
		for (int y = 0; y <= 12; y++)
		{
			printf("%d\t", y);
			for (int x = 0; x < hDist.size(); x++)
			{
				if (y >= hDist[x].size())
					printf("0\t");
				else
					printf("%d\t", hDist[x][y]);
			}
			printf("\n");
		}
	}
private:
	void ComputeMovePruning();
	void RecursiveMovePruning(int depth, TopSpinState<N> &state);

	std::unordered_map<uint64_t,bool> pruningMap;
	std::unordered_map<uint64_t,int> pruningCostMap;
	
	//unsigned int numTiles, swapDiameter;
	std::vector<TopSpinAction> operators;
	std::vector<bool> movePrune;
	
	// stores the heuristic value of each tile-position pair indexed by the tile value (0th index is empty)
	unsigned **h_increment;
	TopSpinState<N> goal;
	std::vector<std::vector<int> > hDist;
	bool pattern[N];

	mutable std::vector<TopSpinAction> history;
	bool pruneSuccessors;
	bool weighted;
	bool additive;
};

//typedef UnitSimulation<TopSpinState<N>, TopSpinAction, TopSpin> TopSpinSimulation;



template <int N, int k>
TopSpin<N, k>::TopSpin()
:PermutationPuzzle::PermutationPuzzleEnvironment<TopSpinState<N>, TopSpinAction>()
{
	weighted = false;
	additive = false;
	pruneSuccessors = false;
	for (int x = 0; x < N; x++)
		operators.push_back(x);
}

template <int N, int k>
TopSpin<N, k>::~TopSpin()
{
	ClearGoal();
}

template <int N, int k>
const std::string TopSpin<N, k>::GetName(){
	std::stringstream name;
	name << "TopSpin(" << N << ", " << k << ")";
	
	return name.str();
}

template <int N, int k>
void TopSpin<N, k>::SetPattern(const std::vector<int> &pattern)
{
	if (pattern.size() == 0)
		additive = false;
	else
		additive = true;
	for (int x = 0; x < N; x++)
		this->pattern[x] = false;
	for (int i : pattern)
		this->pattern[i] = true;
}

template <int N, int k>
double TopSpin<N, k>::AdditiveGCost(const TopSpinState<N> &s, const TopSpinAction &a)
{
	double cnt = 0;
	for (int x = 0; x < k; x++)
	{
		if (pattern[s.puzzle[(a+x)%N]])
			cnt++;
	}
	return cnt;
}


template <int N, int k>
void TopSpin<N, k>::ComputeMovePruning()
{
	TopSpinState<N> start;
	movePrune.resize(N*N*N);
	//movePrune.resize(N*N);
	history.resize(0);
	RecursiveMovePruning(3, start);
	//RecursiveMovePruning(2, start);
	pruningMap.clear();
	pruningCostMap.clear();
}

template <int N, int k>
void TopSpin<N, k>::RecursiveMovePruning(int depth, TopSpinState<N> &state)
{
	if (depth == 0)
	{
		if (pruningMap.find(this->GetStateHash(state)) != pruningMap.end())
		{
			int last = history.size()-1;
			//assert(last >= 1);
			assert(last >= 2);
			//movePrune[history[last]*N+history[last-1]] = true; // prune this
			movePrune[history[last]*N*N+history[last-1]*N+history[last-2]] = true; // prune this
//			printf("Pruning the sequence %d %d %d\n", history[last-2], history[last-1], history[last]);
		}
		pruningMap[this->GetStateHash(state)] = true;
		return;
	}
	
	for (int x = 0; x < N; x++)
	{
		ApplyAction(state, x);
		history.push_back(x);
		RecursiveMovePruning(depth-1, state);
		history.pop_back();
		UndoAction(state, x);
	}
}

template <int N, int k>
void TopSpin<N, k>::StoreGoal(TopSpinState<N> &s)
{
}

template <int N, int k>
bool TopSpin<N, k>::GoalTest(const TopSpinState<N> &s) const
{
	for (int x = 0; x < N; x++)
		if ((s.puzzle[x]+1)%N != s.puzzle[(x+1)%N])
			return false;
	return true;
}

template <int N, int k>
void TopSpin<N, k>::GetSuccessors(const TopSpinState<N> &stateID,
							std::vector<TopSpinState<N>> &neighbors) const
{
	neighbors.resize(0);
	for (unsigned int i = 0; i < N; i++)
	{
		TopSpinState<N> s(stateID);
		ApplyAction(s, i);
		neighbors.push_back(s);
	}
}

template <int N, int k>
void TopSpin<N, k>::GetActions(const TopSpinState<N> &stateID, std::vector<TopSpinAction> &actions) const
{
	if (!pruneSuccessors || history.size() < 3)
		//if (!pruneSuccessors || history.size() < 2)
	{
		actions = operators;
	}
	else {
		actions.resize(0);
		
		for (int x = 0; x < N; x++)
		{
//			if (!movePrune[x*N+history.back()])
			if (!movePrune[x*N*N+history.back()*N+history[history.size()-2]])
				actions.push_back(x);
		}
	}
}

template <int N, int k>
void TopSpin<N, k>::ApplyAction(TopSpinState<N> &s, TopSpinAction a) const
{
	for (int x = 0; x < k/2; x++)
	{
		std::swap(s.puzzle[(a+x)%N], s.puzzle[(a+x+k-1-2*x)%N]);
	}
	if (pruneSuccessors)
		history.push_back(a);
}

template <int N, int k>
void TopSpin<N, k>::UndoAction(TopSpinState<N> &s, TopSpinAction a) const
{
	if (pruneSuccessors && history.size() > 0)
	{
		assert(history.back() == a);
		history.pop_back();
	}
	for (int x = 0; x < k/2; x++)
	{
		std::swap(s.puzzle[(a+x)%N], s.puzzle[(a+x+k-1-2*x)%N]);
	}
}

template <int N, int k>
bool TopSpin<N, k>::InvertAction(TopSpinAction &a) const
{
	return true;
}

// TODO Remove PDB heuristic from this heuristic evaluator.
template <int N, int k>
double TopSpin<N, k>::HCost(const TopSpinState<N> &state1, const TopSpinState<N> &state2) const
{
//	return PermutationPuzzleEnvironment<TopSpinState<N>, TopSpinAction>::HCost(state1);
	return 0;
}

//static const int tsCostCount = 16;
//static int tscosts[tsCostCount] = {5,4,3,3,10,2,6,8,4,8,6,2,10,6,5,1}; // 1...10
//static const int tsCostCount = 16;
//static int tscosts[tsCostCount] = {16,1,8,15,22,29,17,14,9,20,26,28,3,16,12,18}; // 1...29
static const int tsCostCount = 21;
static int tscosts[tsCostCount] = {47,45,52,41,56,43,50,46,49,51,48,53,59,58,55,40,57,60,44,42,54}; // [40...60]
//static const int tsCostCount = 12;
//static const int tscosts[tsCostCount] = {191,467,63,266,691,697,916,534,486,619,581,766}; // duplicating results run in PSVN
template <int N, int k>
double TopSpin<N, k>::GCost(const TopSpinState<N> &node, const TopSpinAction &act) const
{
	if (additive)
	{
		return k;
	}
	else if (!weighted)
	{
		return 1;
	}
	else {
		assert(N <= tsCostCount);
		return tscosts[act];
	}
}

template <int N, int k>
double TopSpin<N, k>::GCost(const TopSpinState<N> &s1, const TopSpinState<N> &s2) const
{
//	return 1;
	TopSpinAction a = this->GetAction(s1, s2);
	return GCost(s1, a);
}

template <int N, int k>
bool TopSpin<N, k>::GoalTest(const TopSpinState<N> &state, const TopSpinState<N> &theGoal) const
{
	return GoalTest(state);
//	return (state == theGoal);
}

template <int N, int k>
uint64_t TopSpin<N, k>::GetActionHash(TopSpinAction act) const
{
	return act;
}

//template <int N, int k>
//void TopSpin<N, k>::OpenGLDraw() const
//{
//}

//static void LocalDrawCircle(float x, float y, float r1, float r2, int segments)
//{
//	float angleStep = TWOPI/segments;
//	//	glBegin(GL_LINE_LOOP);
//	glBegin(GL_QUAD_STRIP);
//	for (float angle = 0; angle < TWOPI; angle += angleStep)
//	{
//		glVertex3f(x+cos(angle)*r1, y+sin(angle)*r1, 0);
//		glVertex3f(x+cos(angle)*r2, y+sin(angle)*r2, 0);
//	}
//	glVertex3f(x+r1, y, 0);
//	glVertex3f(x+r2, y, 0);
//	glEnd();
//}

//static void DrawTSTile(float x, float y, char c1, char c2, int w, int h)
//{
//	glLineWidth(1.0);
//	int textWidth = 0;
//	//int textHeight = 0;
//	//	if (c1 != 0)
//	//	textWidth += glutStrokeWidth(GLUT_STROKE_ROMAN, c1);
//	//if (c2 != 0)
//	//textWidth += glutStrokeWidth(GLUT_STROKE_ROMAN, c2);
//	//printf("%d\n", textWidth);
//	glPushMatrix();
//	glTranslatef(x, y, -0.001);
//	glScalef(1.0/(w*120.0), 1.0/(h*120.0), 1);
//	glRotatef(180, 0.0, 0.0, 1.0);
//	glRotatef(180, 0.0, 1.0, 0.0);
//	glTranslatef(-textWidth/2, -60, 0);
//	//if (c1 != 0)
//	//	glutStrokeCharacter(GLUT_STROKE_ROMAN, c1);
//	//if (c2 != 0)
//	//	glutStrokeCharacter(GLUT_STROKE_ROMAN, c2);
//	glPopMatrix();
//	
//}

template <int N, int k>
static void DrawFrame(int w, int h)
{
}

//template <int N, int k>
//void TopSpin<N, k>::OpenGLDraw(const TopSpinState<N> &s) const
//{
//	glEnable(GL_LINE_SMOOTH);
//	char c1=0, c2=0;
//	double diam = 2.0;
//	diam /= N;
//	for (int x = 0; x < N; x++)
//	{
//		if (s.puzzle[x] > 9)
//			c1 = '0'+(((s.puzzle[x])/10)%10);
//		if (s.puzzle[x] >= 0)
//			c2 = '0'+((s.puzzle[x])%10);
//		
//		glColor3f(1.0, 1.0, 1.0);
//		DrawTSTile(-1+x*diam + diam/2, 0.0f, c1, c2, N, N);
//		glLineWidth(2.0);
//		glColor3f(0.0, 0.3, 0.8);
//		LocalDrawCircle(-1+x*diam + diam/2, 0, diam/2-diam/20, diam/2+diam/20, 50);
//	}
//}

//template <int N, int k>
//void TopSpin<N, k>::OpenGLDraw(const TopSpinState<N> &s1, const TopSpinState<N> &s2, float v) const
//{
//}


#endif
