//
//  Rubik7Edge.h
//  hog2 glut
//
//  Created by Nathan Sturtevant on 1/4/13.
//  Copyright (c) 2013 University of Denver. All rights reserved.
//

#ifndef __hog2_glut__Rubik7Edge__
#define __hog2_glut__Rubik7Edge__

#include <iostream>
#include <stdint.h>
#include <vector>
#include "SearchEnvironment.h"

const int pieces = 10;

class Rubik7EdgeState
{
public:
	Rubik7EdgeState()
	{
		Reset();
	}
	void Reset()
	{
		state = 0;
		for (int x = 0; x < 12; x++)
			SetCubeInLoc(x, x);
	}
	void GetDual(Rubik7EdgeState &s) const;
	int GetCubeInLoc(int whichLoc) const
	{
		return (state>>(12+4*whichLoc))&0xF;
	}
	void SetCubeInLoc(int whichLoc, int cube)
	{
		uint64_t blank = 0xF;
		uint64_t value = cube&0xF;
		state = state&(~(blank<<(12+4*whichLoc)));
		state |= (value<<(12+4*whichLoc));
	}
	bool GetCubeOrientation(int whichCube) const
	{
		return state&(0x1<<whichCube);
	}
	void SetCubeOrientation(int whichCube, bool flip)
	{
		uint64_t blank = 0x1;
		if (flip)
			state |= (0x1<<whichCube);
		else
			state = state&(~(blank<<whichCube));
	}
	void FlipCubeOrientation(int whichCube)
	{
//		if (whichCube == 0xF)
//			return;
		//		printf("Was: 0x%X [flip %d] -- ", state, whichCube);
		state = state^(0x1<<whichCube);
		//		printf("Now: 0x%X \n", state);
	}
	uint64_t state;
};

static bool operator==(const Rubik7EdgeState &l1, const Rubik7EdgeState &l2)
{
	for (int x = 0; x < 12; x++)
	{
		if (l1.GetCubeInLoc(x) != l2.GetCubeInLoc(x))
			return false;
		//if (l1.GetCubeInLoc(x) != 15)
		{
			if (l1.GetCubeOrientation(l1.GetCubeInLoc(x)) != l2.GetCubeOrientation(l1.GetCubeInLoc(x)))
				return false;
		}
	}
	return true;
	//return l1.state == l2.state;
}

static std::ostream& operator <<(std::ostream & out, const Rubik7EdgeState &s)
{
	for (int x = 0; x < 12; x++)
	{
		out << s.GetCubeInLoc(x) << "-" << (s.GetCubeOrientation(s.GetCubeInLoc(x))?1:0) << " ";
	}
	return out;
}


typedef int Rubik7EdgeAction;

class Rubik7EdgeMove {
public:
	Rubik7EdgeAction act;
	Rubik7EdgeMove *next;
	int length() { if (next == 0) return 1; return 1+next->length(); }
};

class Rubik7Edge : public SearchEnvironment<Rubik7EdgeState, Rubik7EdgeAction>
{
public:
	Rubik7Edge()
	{
		for (int x = 0; x < 18; x++)
		{
			moves[x].act = x;
			if (x != 17)
				moves[x].next = &moves[x+1];
		} moves[17].next = 0;
	}
	~Rubik7Edge() {}
	virtual void GetSuccessors(const Rubik7EdgeState &nodeID, std::vector<Rubik7EdgeState> &neighbors) const;
	virtual void GetActions(const Rubik7EdgeState &nodeID, std::vector<Rubik7EdgeAction> &actions) const;
	virtual Rubik7EdgeAction GetAction(const Rubik7EdgeState &s1, const Rubik7EdgeState &s2) const;
	virtual void ApplyAction(Rubik7EdgeState &s, Rubik7EdgeAction a) const;

	
	virtual double HCost(const Rubik7EdgeState &node1, const Rubik7EdgeState &node2) const { return 0; }
	
	virtual double GCost(const Rubik7EdgeState &node1, const Rubik7EdgeState &node2) const { return 1; }
	virtual double GCost(const Rubik7EdgeState &node, const Rubik7EdgeAction &act) const { return 1; }
	virtual bool GoalTest(const Rubik7EdgeState &node, const Rubik7EdgeState &goal) const { return node == goal; }
	

	void ApplyMove(Rubik7EdgeState &s, Rubik7EdgeMove *a);
	void UndoMove(Rubik7EdgeState &s, Rubik7EdgeMove *a);
	void unrankPlayer(uint64_t d, Rubik7EdgeState & s, int who)
	{
		GetStateFromHash(d, s);
	}
	//
	int64_t getMaxSinglePlayerRank() const;
	int64_t getMaxSinglePlayerRank2();
	int64_t getMaxSinglePlayerRank2(int64_t firstIndex);
	
	Rubik7EdgeMove *getMoves(Rubik7EdgeState &) { return &moves[0]; }
	void freeMove(Rubik7EdgeMove *m) {}
	virtual void GetNextState(const Rubik7EdgeState &, Rubik7EdgeAction , Rubik7EdgeState &) const;
	
	virtual bool InvertAction(Rubik7EdgeAction &a) const
	{ 	if (2 == a%3)
		return true;
		if (1 == a%3)
		{
			a -= 1;
			return true;
		}
		a += 1;
		return true;
	}
	
	int64_t rankPlayer(Rubik7EdgeState &s, int who)
	{ return GetStateHash(s); }
	void rankPlayerFirstTwo(const Rubik7EdgeState &s, int who, int64_t &rank);
	void rankPlayerRemaining(const Rubik7EdgeState &s, int who, int64_t &rank);
	void rankPlayer(const Rubik7EdgeState &s, int who, int64_t &index1, int64_t &index2)
	{ rankPlayerFirstTwo(s, 0, index1); rankPlayerRemaining(s, 0, index2); }
	
	virtual uint64_t GetStateHash(const Rubik7EdgeState &node) const;
	virtual uint64_t GetActionHash(Rubik7EdgeAction act) const { return 0; }
	virtual void GetStateFromHash(uint64_t hash, Rubik7EdgeState &node) const;
	
//	virtual void OpenGLDraw() const;
//	virtual void OpenGLDraw(const Rubik7EdgeState&) const;
//	/** Draw the transition at some percentage 0...1 between two states */
//	virtual void OpenGLDraw(const Rubik7EdgeState&, const Rubik7EdgeState&, float) const;
//	virtual void OpenGLDraw(const Rubik7EdgeState&, const Rubik7EdgeAction&) const;
private:
	void MRUnrank(int n, uint64_t r, uint64_t &perm) const;
	void MRUnrank2(int n, uint64_t r, uint64_t &perm) const;
	
	void SetCubeColor(int which, bool face, const Rubik7EdgeState&) const;
	Rubik7EdgeMove moves[18];
};

#endif /* defined(__hog2_glut__Rubik7Edge__) */
