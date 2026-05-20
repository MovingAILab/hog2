//
//  RubikEdge.h
//  hog2 glut
//
//  Created by Nathan Sturtevant on 1/4/13.
//  Copyright (c) 2013 University of Denver. All rights reserved.
//

#ifndef __hog2_glut__RubikEdge__
#define __hog2_glut__RubikEdge__

#include <iostream>
#include <stdint.h>
#include <vector>
#include "SearchEnvironment.h"
#include "PDBHeuristic.h"
#include "MR1Permutation.h"

class RubikEdgeStateBits
{
public:
	RubikEdgeStateBits();
	void Reset();
	void Clear();
	void GetDual(RubikEdgeStateBits &s) const;
	int GetCubeInLoc(int whichLoc) const;
	void SetCubeInLoc(int whichLoc, int cube);
	bool GetCubeOrientation(int whichLoc) const;
	void SetCubeOrientation(int whichLoc, bool flip);
	void FlipCubeOrientation(int whichLoc);
	// 60 bits
	// 12 bits of flips
	// 48 bits of pieces
	uint64_t state;
};

class RubikEdgeStateArray
{
public:
	RubikEdgeStateArray();
	void Reset();
	void Clear();
	void GetDual(RubikEdgeStateArray &s) const;
	int GetCubeInLoc(int whichLoc) const;
	void SetCubeInLoc(int whichLoc, int cube);
	bool GetCubeOrientation(int whichLoc) const;
	void SetCubeOrientation(int whichLoc, bool flip);
	void FlipCubeOrientation(int whichLoc);
	// 60 bits
	// 12 bits of flips
	// 48 bits of pieces
	uint8_t state[24];
};

//typedef RubikEdgeStateBits RubikEdgeState;
typedef RubikEdgeStateArray RubikEdgeState;

RubikEdgeState RotateRubikEdgeClockwise(const RubikEdgeState &in, int step);

static bool operator==(const RubikEdgeStateBits &l1, const RubikEdgeStateBits &l2)
{
	return l1.state == l2.state;
}

static bool operator==(const RubikEdgeStateArray &l1, const RubikEdgeStateArray &l2)
{
	for (int x = 0; x < 24; x++)
		if (l1.state[x] != l2.state[x])
			return false;
	return true;
}


static std::ostream& operator <<(std::ostream & out, const RubikEdgeStateArray &s)
{
	for (int x = 0; x < 12; x++)
	{
		out << s.GetCubeInLoc(x) << " [" << s.GetCubeOrientation(s.GetCubeInLoc(x)) << "] ";
	}
	return out;
}

static std::ostream& operator <<(std::ostream & out, const RubikEdgeStateBits &s)
{
	for (int x = 0; x < 12; x++)
	{
		out << s.GetCubeInLoc(x) << " [" << s.GetCubeOrientation(s.GetCubeInLoc(x)) << "] ";
	}
	return out;
}

namespace std {
	template <> struct hash<RubikEdgeStateBits>
	{
		size_t operator()(const RubikEdgeStateBits &s) const
		{
			return s.state;
		}
	};
	
	template <> struct hash<RubikEdgeStateArray>
	{
		size_t operator()(const RubikEdgeStateArray &s) const
		{
			size_t result = 0;
			for (int x = 0; x < 12; x++)
				result ^= (s.state[x])<<(x);
			for (int x = 12; x < 24; x++)
				result ^= (s.state[x])<<((x-12)*4+12);
			return result;
		}
	};
}


typedef int RubikEdgeAction;

class RubikEdgeMove {
public:
	RubikEdgeAction act;
	RubikEdgeMove *next;
	int length() { if (next == 0) return 1; return 1+next->length(); }
};

class RubikEdge  : public SearchEnvironment<RubikEdgeState, RubikEdgeAction>
{
public:
	RubikEdge()
	{
		piecesToRank = 12;
		for (int x = 0; x < 18; x++)
		{
			moves[x].act = x;
			if (x != 17)
				moves[x].next = &moves[x+1];
		} moves[17].next = 0;
	}
	~RubikEdge() {}
	virtual void GetSuccessors(const RubikEdgeState &nodeID, std::vector<RubikEdgeState> &neighbors) const;
	virtual void GetActions(const RubikEdgeState &nodeID, std::vector<RubikEdgeAction> &actions) const;
	virtual RubikEdgeAction GetAction(const RubikEdgeState &s1, const RubikEdgeState &s2) const;
	virtual void ApplyAction(RubikEdgeStateArray &s, RubikEdgeAction a) const;
	virtual void UndoAction(RubikEdgeStateArray &s, RubikEdgeAction a) const;
	virtual void ApplyAction(RubikEdgeStateBits &s, RubikEdgeAction a) const;
	virtual void UndoAction(RubikEdgeStateBits &s, RubikEdgeAction a) const;

	
	/** Heuristic value between two arbitrary nodes. **/
	virtual double HCost(const RubikEdgeState &node1, const RubikEdgeState &node2) const { return 1; }
	virtual double GCost(const RubikEdgeState &node1, const RubikEdgeState &node2) const { return 1; }
	virtual double GCost(const RubikEdgeState &node, const RubikEdgeAction &act) const { return 1; }
	virtual bool GoalTest(const RubikEdgeState &node, const RubikEdgeState &goal) const { return GoalTest(node); }
	bool GoalTest(const RubikEdgeStateBits &) const;
	bool GoalTest(const RubikEdgeStateArray &) const;
	
	void ApplyMove(RubikEdgeState &s, RubikEdgeMove *a);
	void UndoMove(RubikEdgeState &s, RubikEdgeMove *a);
	void unrankPlayer(uint64_t d, RubikEdgeState & s, int who)
	{
		GetStateFromHash(d, s);
	}
	//
	int64_t getMaxSinglePlayerRank() const;
	int64_t getMaxSinglePlayerRank2();
	int64_t getMaxSinglePlayerRank2(int64_t firstIndex);
	
	RubikEdgeMove *getMoves(RubikEdgeState &) { return &moves[0]; }
	void freeMove(RubikEdgeMove *m) {}
	virtual void GetNextState(const RubikEdgeState &, RubikEdgeAction , RubikEdgeState &) const;
	
	virtual bool InvertAction(RubikEdgeAction &a) const;
	
	int64_t rankPlayer(RubikEdgeState &s, int who)
	{ return GetStateHash(s); }
	void rankPlayerFirstTwo(const RubikEdgeState &s, int who, int64_t &rank);
	void rankPlayerRemaining(const RubikEdgeState &s, int who, int64_t &rank);
	void rankPlayer(const RubikEdgeState &s, int who, int64_t &index1, int64_t &index2)
	{ rankPlayerFirstTwo(s, 0, index1); rankPlayerRemaining(s, 0, index2); }
	
	virtual uint64_t GetStateHash(const RubikEdgeState &node) const;
	virtual uint64_t GetActionHash(RubikEdgeAction act) const { return 0; }
	virtual void GetStateFromHash(uint64_t hash, RubikEdgeState &node) const;
	
	virtual void Draw(Graphics::Display &display, const RubikEdgeState &s) const;

	
//	virtual void OpenGLDraw() const;
//	virtual void OpenGLDraw(const RubikEdgeState&) const;
//	/** Draw the transition at some percentage 0...1 between two states */
//	virtual void OpenGLDraw(const RubikEdgeState&, const RubikEdgeState&, float) const;
//	virtual void OpenGLDraw(const RubikEdgeState&, const RubikEdgeAction&) const;
//	void OpenGLDrawCube(const RubikEdgeState &s, int cube) const;
	static void MRUnrank(int n, uint64_t r, uint64_t &perm);
	static void MRUnrank2(int n, uint64_t r, uint64_t &perm);
	static uint64_t MRRank(int n, uint64_t perm, uint64_t dual);
	static uint64_t MRRank2(int n, uint64_t perm, uint64_t dual);
	void GetTriangles(const RubikEdgeState &s, int cube) const;

	struct triangleColor {
		Graphics::triangle t;
		int color;
	};
private:
	int piecesToRank;
	mutable std::vector<triangleColor> triangles;

	int GetCubeColor(int which, bool face, const RubikEdgeState&) const;
	void SetCubeColor(int which, bool face, const RubikEdgeState&) const;
	RubikEdgeMove moves[18];
};

class RubikEdgePDB : public PDBHeuristic<RubikEdgeState, RubikEdgeAction, RubikEdge, RubikEdgeState, 4> {
public:
	RubikEdgePDB(RubikEdge *e, const RubikEdgeState &s, std::vector<int> &distinctEdges);
	static uint64_t GetStateSpaceSize();
	static uint64_t GetStateHash(const RubikEdgeState &s);
	static void GetStateFromHash(RubikEdgeState &s, uint64_t hash);
	uint64_t GetPDBSize() const;
	uint64_t GetPDBHash(const RubikEdgeState &s, int threadID = 0) const;
	virtual uint64_t GetAbstractHash(const RubikEdgeState &s, int threadID = 0) const { return GetPDBHash(s); }
	void GetStateFromPDBHash(uint64_t hash, RubikEdgeState &s, int threadID = 0) const;
	RubikEdgeState GetStateFromAbstractState(RubikEdgeState &s) const { return s; }

	bool Load(const char *prefix);
	void Save(const char *prefix);
	bool Load(FILE *f);
	void Save(FILE *f);
	std::string GetFileName(const char *prefix);
private:
	static uint64_t Factorial(int val);
	static uint64_t FactorialUpperK(int n, int k);
	std::vector<int> edges;
	size_t puzzleSize;
	uint64_t pdbSize;
	MR1KPermutation mr1;
	// cache for computing ranking/unranking
	mutable std::vector<std::vector<int> > puzzles;
//	mutable std::vector<std::vector<int> > dual;
//	mutable std::vector<std::vector<int> > locs;
};

class RubikEdgeOrientationPDB : public PDBHeuristic<RubikEdgeState, RubikEdgeAction, RubikEdge, RubikEdgeState, 4> {
public:
	RubikEdgeOrientationPDB(RubikEdge *e, const RubikEdgeState &s);
	static uint64_t GetStateSpaceSize();
	static uint64_t GetStateHash(const RubikEdgeState &s);
	static void GetStateFromHash(RubikEdgeState &s, uint64_t hash);
	uint64_t GetPDBSize() const;
	uint64_t GetPDBHash(const RubikEdgeState &s, int threadID = 0) const;
	virtual uint64_t GetAbstractHash(const RubikEdgeState &s, int threadID = 0) const { return GetPDBHash(s); }
	void GetStateFromPDBHash(uint64_t hash, RubikEdgeState &s, int threadID = 0) const;
	RubikEdgeState GetStateFromAbstractState(RubikEdgeState &s) const { return s; }
	
	bool Load(const char *prefix);
	void Save(const char *prefix);
	bool Load(FILE *f);
	void Save(FILE *f);
	std::string GetFileName(const char *prefix);
};


#endif /* defined(__hog2_glut__RubikEdge__) */
