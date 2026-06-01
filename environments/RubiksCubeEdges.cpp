//
//  RubikEdge.cpp
//  hog2 glut
//
//  Created by Nathan Sturtevant on 1/4/13.
//  Copyright (c) 2013 University of Denver. All rights reserved.
//

#include "RubiksCubeEdges.h"
#include "Constants.h"
#include <cassert>
#include <string>
#include <thread>

/**
 *
 * Implementation details:
 *
 * This just implements the 12 Edges. Corners are labeled from 0...11 as follows
 *
 *
 *  +---6---+
 *  |\       \
 *  | 0       4
 *  7  \___2___\
 *  |  |       |
 *  \  1       3
 *   8 |       |
 *    \|___9___|
 *
 *
 *  +---6---+
 *  |       |\
 *  |       | 4
 *  7       5  \
 *  |___11__|  |
 *  \       \  3
 *   8      10 |
 *    \___9___\|
 *
 */

RubikEdgeState RotateRubikEdgeClockwise(const RubikEdgeState &in, int step)
{
	RubikEdgeState e = in;
	int rotations[] = {8, 9, 1, 2, 0, 6, 7, 11, 10, 3, 4, 5};
	for (int x = 0; x < 12; x++)
	{
		e.SetCubeInLoc(x, in.GetCubeInLoc(rotations[x]));
		e.SetCubeOrientation(in.GetCubeInLoc(rotations[x]), in.GetCubeOrientation(in.GetCubeInLoc(rotations[x])));
	}
	e.FlipCubeOrientation(e.GetCubeInLoc(1));
	e.FlipCubeOrientation(e.GetCubeInLoc(3));
	e.FlipCubeOrientation(e.GetCubeInLoc(4));
	e.FlipCubeOrientation(e.GetCubeInLoc(8));
	e.FlipCubeOrientation(e.GetCubeInLoc(7));
	e.FlipCubeOrientation(e.GetCubeInLoc(11));
	return e;
}


RubikEdgeStateBits::RubikEdgeStateBits()
{
	Reset();
}
void RubikEdgeStateBits::Reset()
{
	state = 0;
	for (int x = 0; x < 12; x++)
		SetCubeInLoc(x, x);
}
void RubikEdgeStateBits::Clear()
{
	state = 0;
}
int RubikEdgeStateBits::GetCubeInLoc(int whichLoc) const
{
	return (state>>(12+4*whichLoc))&0xF;
}
void RubikEdgeStateBits::SetCubeInLoc(int whichLoc, int cube)
{
	if (whichLoc >= 12)
		return;
	uint64_t blank = 0xF;
	uint64_t value = cube&0xF;
	state = state&(~(blank<<(12+4*whichLoc)));
	state |= (value<<(12+4*whichLoc));
}
bool RubikEdgeStateBits::GetCubeOrientation(int whichCube) const
{
	return state&(0x1<<whichCube);
}
void RubikEdgeStateBits::SetCubeOrientation(int whichCube, bool flip)
{
	if (whichCube >= 12)
		return;
	uint64_t blank = 0x1;
	if (flip)
		state |= (0x1<<whichCube);
	else
		state = state&(~(blank<<whichCube));
}
void RubikEdgeStateBits::FlipCubeOrientation(int whichCube)
{
	if (whichCube >= 12)
		return;
	state = state^(0x1<<whichCube);
}

void RubikEdgeStateBits::GetDual(RubikEdgeStateBits &s) const
{
	for (int x = 0; x < 12; x++)
	{
		s.SetCubeInLoc(GetCubeInLoc(x), x);
		s.SetCubeOrientation(x, GetCubeOrientation(GetCubeInLoc(x)));
	}
}


RubikEdgeStateArray::RubikEdgeStateArray()
{
	Reset();
}
void RubikEdgeStateArray::Reset()
{
	for (int x = 0; x < 12; x++)
	{
		state[x] = 0;
		SetCubeInLoc(x, x);
	}
}
void RubikEdgeStateArray::Clear()
{
	Reset();
}
int RubikEdgeStateArray::GetCubeInLoc(int whichLoc) const
{
	return state[12+whichLoc];
	//return (state>>(12+4*whichLoc))&0xF;
}
void RubikEdgeStateArray::SetCubeInLoc(int whichLoc, int cube)
{
	if (whichLoc >= 12)
		return;
	state[whichLoc+12] = cube;
//	uint64_t blank = 0xF;
//	uint64_t value = cube&0xF;
//	state = state&(~(blank<<(12+4*whichLoc)));
//	state |= (value<<(12+4*whichLoc));
}
bool RubikEdgeStateArray::GetCubeOrientation(int whichCube) const
{
	return state[whichCube];
	//return state&(0x1<<whichLoc);
}
void RubikEdgeStateArray::SetCubeOrientation(int whichCube, bool flip)
{
	if (whichCube >= 12)
		return;
	state[whichCube] = flip;
//	uint64_t blank = 0x1;
//	if (flip)
//		state |= (0x1<<whichLoc);
//	else
//		state = state&(~(blank<<whichLoc));
}
void RubikEdgeStateArray::FlipCubeOrientation(int whichCube)
{
	if (whichCube >= 12)
		return;
	state[whichCube] = !state[whichCube];
	//state = state^(0x1<<whichLoc);
}

void RubikEdgeStateArray::GetDual(RubikEdgeStateArray &s) const
{
	for (int x = 0; x < 12; x++)
	{
		s.SetCubeInLoc(GetCubeInLoc(x), x);
		s.SetCubeOrientation(x, GetCubeOrientation(GetCubeInLoc(x)));
	}
}


void RubikEdge::GetSuccessors(const RubikEdgeState &nodeID, std::vector<RubikEdgeState> &neighbors) const
{
	RubikEdgeState s;
	for (int x = 0; x < 18; x++)
	{
		GetNextState(nodeID, x, s);
		neighbors.push_back(s);
	}
}

void RubikEdge::GetActions(const RubikEdgeState &nodeID, std::vector<RubikEdgeAction> &actions) const
{
	actions.resize(0);
	for (int x = 0; x < 18; x++)
	{
		actions.push_back(x);
	}
}

RubikEdgeAction RubikEdge::GetAction(const RubikEdgeState &s1, const RubikEdgeState &s2) const
{
	assert(false);
	RubikEdgeAction a;
	return a;
}

void RubikEdge::ApplyMove(RubikEdgeState &s, RubikEdgeMove *a)
{
	ApplyAction(s, a->act);
}

void RubikEdge::UndoMove(RubikEdgeState &s, RubikEdgeMove *a)
{
	RubikEdgeAction todo = a->act;
	if (0 == todo%3)
	{
		todo += 1;
	}
	else if (1 == todo%3)
	{
		todo -= 1;
	}
	ApplyAction(s, todo);
}

void RubikEdge::UndoAction(RubikEdgeStateBits &s, RubikEdgeAction a) const
{
	RubikEdgeAction todo = a;
	if (0 == todo%3)
	{
		todo += 1;
	}
	else if (1 == todo%3)
	{
		todo -= 1;
	}
	ApplyAction(s, todo);
}

void RubikEdge::UndoAction(RubikEdgeStateArray &s, RubikEdgeAction a) const
{
	RubikEdgeAction todo = a;
	if (0 == todo%3)
	{
		todo += 1;
	}
	else if (1 == todo%3)
	{
		todo -= 1;
	}
	ApplyAction(s, todo);
}

template <typename t>
void LocalApplyAction(t &s, RubikEdgeAction a)
{
	switch (a)
	{
		case 0: // face 0
		{
			int a = s.GetCubeInLoc(1-1);
			int b = s.GetCubeInLoc(3-1);
			int c = s.GetCubeInLoc(5-1);
			int d = s.GetCubeInLoc(7-1);
			s.SetCubeInLoc(1-1, d);
			s.SetCubeInLoc(3-1, a);
			s.SetCubeInLoc(5-1, b);
			s.SetCubeInLoc(7-1, c);
		}
			break;
		case 1:
		{
			int a = s.GetCubeInLoc(1-1);
			int b = s.GetCubeInLoc(3-1);
			int c = s.GetCubeInLoc(5-1);
			int d = s.GetCubeInLoc(7-1);
			s.SetCubeInLoc(1-1, b);
			s.SetCubeInLoc(3-1, c);
			s.SetCubeInLoc(5-1, d);
			s.SetCubeInLoc(7-1, a);
		}
			break;
		case 2:
		{
			int a = s.GetCubeInLoc(1-1);
			int b = s.GetCubeInLoc(3-1);
			int c = s.GetCubeInLoc(5-1);
			int d = s.GetCubeInLoc(7-1);
			s.SetCubeInLoc(1-1, c);
			s.SetCubeInLoc(3-1, d);
			s.SetCubeInLoc(5-1, a);
			s.SetCubeInLoc(7-1, b);
		}
			break;
		case 3: // face 5
		{
			int a = s.GetCubeInLoc(9-1);
			int b = s.GetCubeInLoc(10-1);
			int c = s.GetCubeInLoc(11-1);
			int d = s.GetCubeInLoc(12-1);
			s.SetCubeInLoc(9-1, d);
			s.SetCubeInLoc(10-1, a);
			s.SetCubeInLoc(11-1, b);
			s.SetCubeInLoc(12-1, c);
		}
			break;
		case 4:
		{
			int a = s.GetCubeInLoc(9-1);
			int b = s.GetCubeInLoc(10-1);
			int c = s.GetCubeInLoc(11-1);
			int d = s.GetCubeInLoc(12-1);
			s.SetCubeInLoc(9-1, b);
			s.SetCubeInLoc(10-1, c);
			s.SetCubeInLoc(11-1, d);
			s.SetCubeInLoc(12-1, a);
		}
			break;
		case 5:
		{
			int a = s.GetCubeInLoc(9-1);
			int b = s.GetCubeInLoc(10-1);
			int c = s.GetCubeInLoc(11-1);
			int d = s.GetCubeInLoc(12-1);
			s.SetCubeInLoc(9-1, c);
			s.SetCubeInLoc(10-1, d);
			s.SetCubeInLoc(11-1, a);
			s.SetCubeInLoc(12-1, b);
		}
			break;
			
		case 6: // face 2
		{
			int a = s.GetCubeInLoc(2-1);
			int b = s.GetCubeInLoc(3-1);
			int c = s.GetCubeInLoc(4-1);
			int d = s.GetCubeInLoc(10-1);
			s.SetCubeInLoc(2-1, d);
			s.SetCubeInLoc(3-1, a);
			s.SetCubeInLoc(4-1, b);
			s.SetCubeInLoc(10-1, c);
			s.FlipCubeOrientation(b);
			s.FlipCubeOrientation(d);
		}
			break;
		case 7:
		{
			int a = s.GetCubeInLoc(2-1);
			int b = s.GetCubeInLoc(3-1);
			int c = s.GetCubeInLoc(4-1);
			int d = s.GetCubeInLoc(10-1);
			s.SetCubeInLoc(2-1, b);
			s.SetCubeInLoc(3-1, c);
			s.SetCubeInLoc(4-1, d);
			s.SetCubeInLoc(10-1, a);
			s.FlipCubeOrientation(a);
			s.FlipCubeOrientation(c);
		}
			break;
		case 8:
		{
			int a = s.GetCubeInLoc(2-1);
			int b = s.GetCubeInLoc(3-1);
			int c = s.GetCubeInLoc(4-1);
			int d = s.GetCubeInLoc(10-1);
			s.SetCubeInLoc(2-1, c);
			s.SetCubeInLoc(3-1, d);
			s.SetCubeInLoc(4-1, a);
			s.SetCubeInLoc(10-1, b);
			s.FlipCubeOrientation(a);
			s.FlipCubeOrientation(b);
			s.FlipCubeOrientation(c);
			s.FlipCubeOrientation(d);
		}
			break;
			
		case 9: // face 4
		{
			int a = s.GetCubeInLoc(6-1);
			int b = s.GetCubeInLoc(7-1);
			int c = s.GetCubeInLoc(8-1);
			int d = s.GetCubeInLoc(12-1);
			s.SetCubeInLoc(6-1, d);
			s.SetCubeInLoc(7-1, a);
			s.SetCubeInLoc(8-1, b);
			s.SetCubeInLoc(12-1, c);
			s.FlipCubeOrientation(c);
			s.FlipCubeOrientation(d);
		}
			break;
		case 10:
		{
			int a = s.GetCubeInLoc(6-1);
			int b = s.GetCubeInLoc(7-1);
			int c = s.GetCubeInLoc(8-1);
			int d = s.GetCubeInLoc(12-1);
			s.SetCubeInLoc(6-1, b);
			s.SetCubeInLoc(7-1, c);
			s.SetCubeInLoc(8-1, d);
			s.SetCubeInLoc(12-1, a);
			s.FlipCubeOrientation(a);
			s.FlipCubeOrientation(d);
		}
			break;
		case 11:
		{
			int a = s.GetCubeInLoc(6-1);
			int b = s.GetCubeInLoc(7-1);
			int c = s.GetCubeInLoc(8-1);
			int d = s.GetCubeInLoc(12-1);
			s.SetCubeInLoc(6-1, c);
			s.SetCubeInLoc(7-1, d);
			s.SetCubeInLoc(8-1, a);
			s.SetCubeInLoc(12-1, b);
			s.FlipCubeOrientation(b);
			s.FlipCubeOrientation(d);
		}
			break;
			
		case 12: // face 1
		{
			int a = s.GetCubeInLoc(1-1);
			int b = s.GetCubeInLoc(2-1);
			int c = s.GetCubeInLoc(9-1);
			int d = s.GetCubeInLoc(8-1);
			s.SetCubeInLoc(1-1, d);
			s.SetCubeInLoc(2-1, a);
			s.SetCubeInLoc(9-1, b);
			s.SetCubeInLoc(8-1, c);
			s.FlipCubeOrientation(a);
			s.FlipCubeOrientation(d);
		}
			break;
		case 13:
		{
			int a = s.GetCubeInLoc(1-1);
			int b = s.GetCubeInLoc(2-1);
			int c = s.GetCubeInLoc(9-1);
			int d = s.GetCubeInLoc(8-1);
			s.SetCubeInLoc(1-1, b);
			s.SetCubeInLoc(2-1, c);
			s.SetCubeInLoc(9-1, d);
			s.SetCubeInLoc(8-1, a);
			s.FlipCubeOrientation(a);
			s.FlipCubeOrientation(b);
		}
			break;
		case 14:
		{
			int a = s.GetCubeInLoc(1-1);
			int b = s.GetCubeInLoc(2-1);
			int c = s.GetCubeInLoc(9-1);
			int d = s.GetCubeInLoc(8-1);
			s.SetCubeInLoc(1-1, c);
			s.SetCubeInLoc(2-1, d);
			s.SetCubeInLoc(9-1, a);
			s.SetCubeInLoc(8-1, b);
			s.FlipCubeOrientation(a);
			s.FlipCubeOrientation(c);
		}
			break;
		case 15: // face 3
		{
			int a = s.GetCubeInLoc(4-1);
			int b = s.GetCubeInLoc(5-1);
			int c = s.GetCubeInLoc(6-1);
			int d = s.GetCubeInLoc(11-1);
			s.SetCubeInLoc(4-1, d);
			s.SetCubeInLoc(5-1, a);
			s.SetCubeInLoc(6-1, b);
			s.SetCubeInLoc(11-1, c);
			s.FlipCubeOrientation(b);
			s.FlipCubeOrientation(d);
		}
			break;
		case 16:
		{
			int a = s.GetCubeInLoc(4-1);
			int b = s.GetCubeInLoc(5-1);
			int c = s.GetCubeInLoc(6-1);
			int d = s.GetCubeInLoc(11-1);
			s.SetCubeInLoc(4-1, b);
			s.SetCubeInLoc(5-1, c);
			s.SetCubeInLoc(6-1, d);
			s.SetCubeInLoc(11-1, a);
			s.FlipCubeOrientation(a);
			s.FlipCubeOrientation(c);
		}
			break;
		case 17:
		{
			int a = s.GetCubeInLoc(4-1);
			int b = s.GetCubeInLoc(5-1);
			int c = s.GetCubeInLoc(6-1);
			int d = s.GetCubeInLoc(11-1);
			s.SetCubeInLoc(4-1, c);
			s.SetCubeInLoc(5-1, d);
			s.SetCubeInLoc(6-1, a);
			s.SetCubeInLoc(11-1, b);
			s.FlipCubeOrientation(a);
			s.FlipCubeOrientation(b);
			s.FlipCubeOrientation(c);
			s.FlipCubeOrientation(d);
		}
			break;
		default:
			break;
	}
	
}

void RubikEdge::ApplyAction(RubikEdgeStateArray &s, RubikEdgeAction a) const
{
	LocalApplyAction<RubikEdgeStateArray>(s, a);
}

void RubikEdge::ApplyAction(RubikEdgeStateBits &s, RubikEdgeAction a) const
{
	LocalApplyAction<RubikEdgeStateBits>(s, a);
}


void RubikEdge::GetNextState(const RubikEdgeState &s0, RubikEdgeAction a, RubikEdgeState &s1) const
{
	s1 = s0;
	ApplyAction(s1, a);
}

bool RubikEdge::InvertAction(RubikEdgeAction &a) const
{
	if (2 == a%3)
		return true;
	if (1 == a%3)
	{
		a -= 1;
		return true;
	}
	a += 1;
	return true;

}

bool RubikEdge::GoalTest(const RubikEdgeStateBits &b) const
{
	assert(!"Code not called");
	return b.state == 0;
}

bool RubikEdge::GoalTest(const RubikEdgeStateArray &a) const
{
	assert(!"Code not called");
}


inline uint64_t Factorial(int n)
{
	static uint64_t Factorial[21] =
	{ 1ll, 1ll, 2ll, 6ll, 24ll, 120ll, 720ll, 5040ll, 40320ll, 362880ll, 3628800ll, 39916800ll, 479001600ll,
		6227020800ll, 87178291200ll, 1307674368000ll, 20922789888000ll, 355687428096000ll,
		6402373705728000ll, 121645100408832000ll, 2432902008176640000ll };
	
	return Factorial[n];
}

inline int get(uint64_t state, int whichLoc)
{
	return (state>>((whichLoc<<2)))&0xF;
}

inline void set(uint64_t &state, int whichLoc, int cube)
{
	const uint64_t blank = 0xF;
	uint64_t value = cube;//&0xF;
	state = state&(~(blank<<((whichLoc<<2))));
	state |= (value<<((whichLoc<<2)));
}

inline void swap(uint64_t &state, int loc1, int loc2)
{
	loc1<<=2;
	loc2<<=2;
	uint64_t val1 = (state>>(loc1));
	uint64_t val2 = (state>>(loc2));

	uint64_t xord = (val1 ^ val2)&0xF;
	xord = (xord << loc1) | (xord << loc2);
	state = state^xord;
//	const uint64_t blank = 0xF;
//	uint64_t mask = (blank<<(loc1))|(blank<<(loc2));
//	state = state&(~mask);
//	state = state|(val1<<(loc2))|(val2<<(loc1));
}

int64_t RubikEdge::getMaxSinglePlayerRank() const
{
	static uint64_t Factorial[21] =
	{ 1ll, 1ll, 2ll, 6ll, 24ll, 120ll, 720ll, 5040ll, 40320ll, 362880ll, 3628800ll, 39916800ll, 479001600ll,
		6227020800ll, 87178291200ll, 1307674368000ll, 20922789888000ll, 355687428096000ll,
		6402373705728000ll, 121645100408832000ll, 2432902008176640000ll };
	
	return Factorial[12]*(0x1<<11);
	//	return 980995276800ll;
}

int64_t RubikEdge::getMaxSinglePlayerRank2()
{
	return 128;
}

int64_t RubikEdge::getMaxSinglePlayerRank2(int64_t firstIndex)
{
	return (Factorial(12)*(0x1<<4));
}

void RubikEdge::rankPlayerFirstTwo(const RubikEdgeState &s, int, int64_t &rank)
{
	rank = (s.GetCubeOrientation(11)<<6)|(s.GetCubeOrientation(10)<<5)|
	(s.GetCubeOrientation(9)<<4)|(s.GetCubeOrientation(8)<<3)|
	(s.GetCubeOrientation(7)<<2)|(s.GetCubeOrientation(6)<<1)|
	s.GetCubeOrientation(5);
	//	rank = s.GetCubeInLoc(0);
}

void RubikEdge::rankPlayerRemaining(const RubikEdgeState &node, int, int64_t &rank)
{
	uint64_t perm = 0, dual = 0;
	for (int x = 0; x < 12; x++)
	{
		set(perm, x, node.GetCubeInLoc(x));
		set(dual, node.GetCubeInLoc(x), x);
	}
	
	uint64_t hashVal = 0;
	for (int x = 7; x < 11; x++)
	{
		hashVal = (hashVal<<1)+node.GetCubeOrientation(11-x);
	}
	hashVal = hashVal*Factorial(12)+MRRank(12, perm, dual);
	rank = hashVal;
}


uint64_t RubikEdge::GetStateHash(const RubikEdgeState &node) const
{
	uint64_t perm = 0, dual = 0;
	for (int x = 0; x < 12; x++)
	{
		int val = node.GetCubeInLoc(x);
		set(perm, x, val);
		set(dual, val, x);
	}
	
	uint64_t hashVal = 0;
	for (int x = 0; x < 11; x++)
	{
		hashVal = (hashVal<<1)+node.GetCubeOrientation(11-x);
	}
	//	return (MRRank(12, perm, dual)<<11)|hashVal;
	hashVal = hashVal*Factorial(12)+MRRank(12, perm, dual);
	return hashVal;
}

// 38.522
uint64_t RubikEdge::MRRank(int n, uint64_t perm, uint64_t dual)
{
	int ss[12];
	int ssLoc = 0;
	//	static std::vector<int> ss;
	//	ss.resize(0);
	int s;
	for (int i = n; i > 1; i--)
	{
		s = get(perm, i-1);
		ss[ssLoc] = s;
		ssLoc++;
		//		ss.push_back(s);
		swap(perm, i-1, get(dual, i-1));
		swap(dual, s, i-1);
	}
	uint64_t result = 0;
	int cnt = 2;
	//	printf("0");
	for (int i = (int)ssLoc-1; i >= 0; i--)
	{
		//		printf(")*%d + %d ", cnt, ss[i]);
		result *= cnt;
		result += ss[i];
		cnt++;
	}
	//	printf("\n");
	return result;
	//	if (n == 1)
	//		return 0;
	//	int s = get(perm, n-1);
	//	swap(perm, n-1, get(dual, n-1));
	//	swap(dual, s, n-1);
	//	return s+n*MRRank(n-1, perm, dual);
}

// 53.5% time
uint64_t RubikEdge::MRRank2(int n, uint64_t perm, uint64_t dual)
{
	if (n == 1)
		return 0;
	int s = get(perm, n-1);
	swap(perm, n-1, get(dual, n-1));
	swap(dual, s, n-1);
	return s+n*MRRank2(n-1, perm, dual);
}

void RubikEdge::GetStateFromHash(uint64_t hash, RubikEdgeState &node) const
{
	int cnt = 0;
	uint64_t bits = hash/Factorial(12);
	uint64_t hVal = hash%Factorial(12);
	for (int x = 10; x >= 0; x--)
	{
		node.SetCubeOrientation(11-x, bits&0x1);
		cnt += bits & 0x1;
		bits >>= 1;
	}
	if (1 == cnt%2)
	{
		node.SetCubeOrientation(11-11, true);
	}
	else {
		node.SetCubeOrientation(11-11, false);
	}
	
	uint64_t val = 0;
	for (int x = 0; x < 12; x++)
		set(val, x, x);
	MRUnrank2(12, hVal, val);
	for (int x = 0; x < 12; x++)
	{
		node.SetCubeInLoc(x, get(val, x));
	}
}

// 38.982 sec elapsed
// 3.5 % of time in function
// Overall locality: 936842 / 1800000 = 0.520468
void RubikEdge::MRUnrank(int n, uint64_t r, uint64_t &perm)
{
	if (n > 0)
	{
		//		printf("Swap %d %" PRId64 "\n", n-1, r%n);
		swap(perm, n-1, r%n);
		MRUnrank(n-1, r/n, perm);
	}
}

//38.152 sec elapsed
void RubikEdge::MRUnrank2(int n, uint64_t r, uint64_t &perm)
{
	for (int i = n; i > 0; i--)
	{
		//		printf("Swap %d %" PRId64 "\n", i-1, r%i);
		swap(perm, i-1, r%i);
		r = r/i;
	}
}


// 50.970 sec elapsed
// 73.9% of time inside this function
// Overall locality: 1044342 / 1800000 = 0.580190
//uint64_t RubikEdge::GetStateHash(const RubikEdgeState &node) const
//{
//	static uint64_t Factorial[21] =
//	{ 1ll, 1ll, 2ll, 6ll, 24ll, 120ll, 720ll, 5040ll, 40320ll, 362880ll, 3628800ll, 39916800ll, 479001600ll,
//		6227020800ll, 87178291200ll, 1307674368000ll, 20922789888000ll, 355687428096000ll,
//		6402373705728000ll, 121645100408832000ll, 2432902008176640000ll };
//
//	int puzzle[12];
//	for (int x = 0; x < 12; x++)
//		puzzle[x] = node.GetCubeInLoc(x);
//
//	uint64_t hashVal = 0;
//	int numEntriesLeft = 12;
//	for (unsigned int x = 0; x < 12; x++)
//	{
//		hashVal += puzzle[x]*Factorial[numEntriesLeft-1];
//		numEntriesLeft--;
//		for (unsigned y = x; y < 12; y++)
//		{
//			if (puzzle[y] > puzzle[x])
//				puzzle[y]--;
//		}
//	}
//	for (int x = 0; x < 11; x++)
//	{
//		hashVal = (hashVal<<1)+node.GetCubeOrientation(x);
//	}
//	return hashVal;
//}
//
//void RubikEdge::GetStateFromHash(uint64_t hash, RubikEdgeState &node) const
//{
//	int puzzle[12];
//	uint64_t hashVal = hash;
//
//	int cnt = 0;
//	for (int x = 10; x >= 0; x--)
//	{
//		node.SetCubeOrientation(x, hashVal&0x1);
//		cnt += hashVal & 0x1;
//		hashVal >>= 1;
//	}
//	if (1 == cnt%2)
//	{
//		node.SetCubeOrientation(11, true);
//	}
//	int numEntriesLeft = 1;
//	for (int x = 12-1; x >= 0; x--)
//	{
//		puzzle[x] = hashVal%numEntriesLeft;
//		hashVal /= numEntriesLeft;
//		numEntriesLeft++;
//		for (int y = x+1; y < 12; y++)
//		{
//			if (puzzle[y] >= puzzle[x])
//				puzzle[y]++;
//		}
//	}
//	for (int x = 0; x < 12; x++)
//		node.SetCubeInLoc(x, puzzle[x]);
//}

static Graphics::point transform(const Graphics::point &p)
{
	const float cosr = cos(-1.11f);
	const float sinr = sin(-1.11f);
	return Graphics::point(p.x+0.5*p.z*cosr, p.y+0.5*p.z*sinr);
}

static Graphics::triangle transform(const Graphics::triangle &t)
{
	return Graphics::triangle(transform(t.p1), transform(t.p2), transform(t.p3));
}

static bool operator<(const RubikEdge::triangleColor &s1, const RubikEdge::triangleColor &s2)
{
	float z1 = s1.t.p1.z+s1.t.p2.z+s1.t.p3.z;
	float z2 = s2.t.p1.z+s2.t.p2.z+s2.t.p3.z;
	if (z1 == z2)
	{
		float y1 = (s1.t.p1.y+s1.t.p2.y+s1.t.p3.y);
		float y2 = (s2.t.p1.y+s2.t.p2.y+s2.t.p3.y);
		if (y1 == y2)
		{
			float x1 = (s1.t.p1.x+s1.t.p2.x+s1.t.p3.x);
			float x2 = (s2.t.p1.x+s2.t.p2.x+s2.t.p3.x);
			return x1 > x2;
		}
		return y1 > y2;
	}
	return z1 > z2;
//	)	return  > ;
}

void RubikEdge::Draw(Graphics::Display &display, const RubikEdgeState &s) const
{
	rgbColor colors[7] = {{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}, {1.0, 1.0, 0.0}, {1.0, 0.75, 0.0},{1.0, 1.0, 1.0},};
	triangles.resize(0);
	// Get triangles
	
	
	for (int x = 1; x <= 7; x+=2)
		GetTriangles(s, x);
	for (int x = 9; x <= 25; x+=2)
		GetTriangles(s, x);

	// Do transform
	// Sort by z
//	for (int x = 0; x < triangles.size(); x++)
//		triangles[x].t = transform(triangles[x].t);
	std::sort(triangles.begin(),triangles.end());

	// Transform and draw into display
	for (auto &t : triangles)
	{
		display.FillTriangle(transform(t.t), colors[1+t.color]);
//		display.FillTriangle(transform(t.t.p1),
//							 transform(t.t.p2),
//							 transform(t.t.p3),
//							 colors[1+t.color]);
	}

}

//void RubikEdge::OpenGLDraw() const
//{
//	
//}
//
//void RubikEdge::OpenGLDraw(const RubikEdgeState&s) const
//{
////	glPushMatrix();
////	glRotatef(45, 0, 1, 0);
//	glBegin(GL_QUADS);
//	for (int x = 1; x <= 7; x+=2)
//		OpenGLDrawCube(s, x);
////	glEnd();
////	glPopMatrix();
////	
////	glBegin(GL_QUADS);
//	for (int x = 9; x <= 25; x+=2)
//		OpenGLDrawCube(s, x);
//	glEnd();
//}
///** Draw the transition at some percentage 0...1 between two states */
//
//void RubikEdge::OpenGLDraw(const RubikEdgeState&, const RubikEdgeState&, float) const
//{
//	
//}
//
//void RubikEdge::OpenGLDraw(const RubikEdgeState&, const RubikEdgeAction&) const
//{
//	
//}

void RubikEdge::GetTriangles(const RubikEdgeState &s, int cube) const
{
	const float scale = 0.3;
	const float offset = 0.95*2.0*scale/3.0;
	const float offset2 = 2.0*scale/3.0;
	const float epsilon = 0.002;
	Graphics::triangle t1, t2;
	int color;
	switch(cube)
	{
		case 1: // cube 1
		{
			// Face 0 - cube 1
			color = GetCubeColor(3, false, s);
			t1 = {{-offset/2.0f, -scale, -scale}, {-offset/2.0f, -scale, -scale+offset}, {offset/2.0f, -scale, -scale+offset}};
			t2 = {{offset/2.0f, -scale, -scale+offset}, {offset/2.0f, -scale, -scale}, {-offset/2.0f, -scale, -scale}};
			triangles.push_back({t1, color});
			triangles.push_back({t2, color});

			color = GetCubeColor(-1, false, s);
			t1 = {{-offset2/2.0f, -scale+epsilon, -scale},
				{-offset2/2.0f, -scale+epsilon, -scale+offset2},
				{offset2/2.0f, -scale+epsilon, -scale+offset2}};
			t2 = {{offset2/2.0f, -scale+epsilon, -scale+offset2},
				{offset2/2.0f, -scale+epsilon, -scale},
				{-offset2/2.0f, -scale+epsilon, -scale}};
//			triangles.push_back({t1, color});
//			triangles.push_back({t2, color});
			
			// Face 2 - cube 1
			color = GetCubeColor(3, true, s);
			t1 = {{-offset/2.0f, -scale+offset, -scale},{offset/2.0f, -scale+offset, -scale},{offset/2.0f, -scale, -scale}};
			t2 = {{offset/2.0f, -scale, -scale},{-offset/2.0f, -scale, -scale},{-offset/2.0f, -scale+offset, -scale}};
			triangles.push_back({t1, color});
			triangles.push_back({t2, color});

			color = GetCubeColor(-1, true, s);
			t1 = {{-offset2/2.0f, -scale+offset2, -scale+epsilon},{offset2/2.0f, -scale+offset2, -scale+epsilon},{offset2/2.0f, -scale, -scale+epsilon}};
			t2 = {{offset2/2.0f, -scale, -scale+epsilon},{-offset2/2.0f, -scale, -scale+epsilon},{-offset2/2.0f, -scale+offset2, -scale+epsilon}};
//			triangles.push_back({t1, color});
//			triangles.push_back({t2, color});

		} break;
		case 3: // cube 3
		{
			// Face 0 - cube 3
			color = GetCubeColor(1, false, s);
			t1 = {{-scale, -scale, offset/2.0f},{-scale, -scale, -offset/2.0f},{-scale+offset, -scale, -offset/2.0f}};
			t2 = {{-scale+offset, -scale, -offset/2.0f},{-scale+offset, -scale, offset/2.0f},{-scale, -scale, offset/2.0f}};
			triangles.push_back({t1, color});
			triangles.push_back({t2, color});

			color = GetCubeColor(-1, false, s);
			t1 = {{-scale, -scale+epsilon, offset2/2.0f},{-scale, -scale+epsilon, -offset2/2.0f},{-scale+offset2, -scale+epsilon, -offset2/2.0f}};
			t2 = {{-scale+offset2, -scale+epsilon, -offset2/2.0f},{-scale+offset2, -scale+epsilon, offset2/2.0f},{-scale, -scale+epsilon, offset2/2.0f}};
//			triangles.push_back({t1, color});
//			triangles.push_back({t2, color});

			// Face 1 - cube 3
			color = GetCubeColor(1, true, s);
			t1 = {{-scale, -scale+offset, -offset/2.0f},{-scale, -scale+offset, offset/2.0f},{-scale, -scale, offset/2.0f}};
			t2 = {{-scale, -scale, offset/2.0f},{-scale, -scale, -offset/2.0f},{-scale, -scale+offset, -offset/2.0f}};
			triangles.push_back({t1, color});
			triangles.push_back({t2, color});

			color = GetCubeColor(-1, true, s);
			t1 = {{-scale+epsilon, -scale+offset2, -offset2/2.0f},{-scale+epsilon, -scale+offset2, offset2/2.0f},{-scale+epsilon, -scale, offset2/2.0f}};
			t2 = {{-scale+epsilon, -scale, offset2/2.0f},{-scale+epsilon, -scale, -offset2/2.0f},{-scale+epsilon, -scale+offset2, -offset2/2.0f}};
//			triangles.push_back({t1, color});
//			triangles.push_back({t2, color});
		} break;
		case 5: // cube 5
		{
			// Face 0 - cube 5
			color = GetCubeColor(5, false, s);
			t1 = {{scale, -scale, offset/2.0f},{scale, -scale, -offset/2.0f},{scale-offset, -scale, -offset/2.0f}};
			t2 = {{scale-offset, -scale, -offset/2.0f},{scale-offset, -scale, offset/2.0f},{scale, -scale, offset/2.0f}};
			triangles.push_back({t1, color});
			triangles.push_back({t2, color});

			color = GetCubeColor(-1, false, s);
			t1 = {{scale, -scale+epsilon, offset2/2.0f},{scale, -scale+epsilon, -offset2/2.0f},{scale-offset2, -scale+epsilon, -offset2/2.0f}};
			t2 = {{scale-offset2, -scale+epsilon, -offset2/2.0f},{scale-offset2, -scale+epsilon, offset2/2.0f},{scale, -scale+epsilon, offset2/2.0f}};
//			triangles.push_back({t1, color});
//			triangles.push_back({t2, color});

			// Face 3 - cube 5
			color = GetCubeColor(5, true, s);
			t1 = {{scale, -scale+offset, -offset/2.0f},{scale, -scale+offset, offset/2.0f},{scale, -scale, offset/2.0f}};
			t2 = {{scale, -scale, offset/2.0f},{scale, -scale, -offset/2.0f},{scale, -scale+offset, -offset/2.0f}};
			triangles.push_back({t1, color});
			triangles.push_back({t2, color});

			color = GetCubeColor(-1, true, s);
			t1 = {{scale-epsilon, -scale+offset2, -offset2/2.0f},{scale-epsilon, -scale+offset2, offset2/2.0f},{scale-epsilon, -scale, offset2/2.0f}};
			t2 = {{scale-epsilon, -scale, offset2/2.0f},{scale-epsilon, -scale, -offset2/2.0f},{scale-epsilon, -scale+offset2, -offset2/2.0f}};
//			triangles.push_back({t1, color});
//			triangles.push_back({t2, color});
		} break;
		case 7: // cube 7
		{
			// Face 0 - cube 7
			color = GetCubeColor(7, false, s);
			t1 = {{-offset/2.0f, -scale, scale},{-offset/2.0f, -scale, scale-offset},{offset/2.0f, -scale, scale-offset}};
			t2 = {{offset/2.0f, -scale, scale-offset},{offset/2.0f, -scale, scale},{-offset/2.0f, -scale, scale}};
			triangles.push_back({t1, color});
			triangles.push_back({t2, color});

			color = GetCubeColor(-1, false, s);
			t1 = {{-offset2/2.0f, -scale+epsilon, scale},{-offset2/2.0f, -scale+epsilon, scale-offset2},{offset2/2.0f, -scale+epsilon, scale-offset2}};
			t2 = {{offset2/2.0f, -scale+epsilon, scale-offset2},{offset2/2.0f, -scale+epsilon, scale},{-offset2/2.0f, -scale+epsilon, scale}};
//			triangles.push_back({t1, color});
//			triangles.push_back({t2, color});

			// Face 4 - cube 7
			color = GetCubeColor(7, true, s);
			t1 = {{-offset/2.0f, -scale+offset, scale},{offset/2.0f, -scale+offset, scale},{offset/2.0f, -scale, scale}};
			t2 = {{offset/2.0f, -scale, scale},{-offset/2.0f, -scale, scale},{-offset/2.0f, -scale+offset, scale}};
			triangles.push_back({t1, color});
			triangles.push_back({t2, color});

			color = GetCubeColor(-1, true, s);
			t1 = {{-offset2/2.0f, -scale+offset2, scale-epsilon},{offset2/2.0f, -scale+offset2, scale-epsilon},{offset2/2.0f, -scale, scale-epsilon}};
			t2 = {{offset2/2.0f, -scale, scale-epsilon},{-offset2/2.0f, -scale, scale-epsilon},{-offset2/2.0f, -scale+offset2, scale-epsilon}};
//			triangles.push_back({t1, color});
//			triangles.push_back({t2, color});

		} break;
		case 9: // cube 9
		{
			// Face 1 - cube 9
			color = GetCubeColor(2, false, s);
			t1 = {{-scale, -offset/2.0f, -scale+offset},{-scale, offset/2.0f, -scale+offset},{-scale, offset/2.0f, -scale}};
			t2 = {{-scale, offset/2.0f, -scale},{-scale, -offset/2.0f, -scale},{-scale, -offset/2.0f, -scale+offset}};
			triangles.push_back({t1, color});
			triangles.push_back({t2, color});
			
			color = GetCubeColor(-1, false, s);
			t1 = {{-scale+epsilon, -offset2/2.0f, -scale+offset2},{-scale+epsilon, offset2/2.0f, -scale+offset2},{-scale+epsilon, offset2/2.0f, -scale}};
			t2 = {{-scale+epsilon, offset2/2.0f, -scale},{-scale+epsilon, -offset2/2.0f, -scale},{-scale+epsilon, -offset2/2.0f, -scale+offset2}};
//			triangles.push_back({t1, color});
//			triangles.push_back({t2, color});
			
			
			// Face 2 - cube 9
			color = GetCubeColor(2, true, s);
			t1 = {{-scale+offset, -offset/2.0f, -scale},{-scale+offset, offset/2.0f, -scale},{-scale, offset/2.0f, -scale}};
			t2 = {{-scale, offset/2.0f, -scale},{-scale, -offset/2.0f, -scale},{-scale+offset, -offset/2.0f, -scale}};
			triangles.push_back({t1, color});
			triangles.push_back({t2, color});
			
			color = GetCubeColor(-1, true, s);
			t1 = {{-scale+offset2, -offset2/2.0f, -scale+epsilon},{-scale+offset2, offset2/2.0f, -scale+epsilon},{-scale, offset2/2.0f, -scale+epsilon}};
			t2 = {{-scale, offset2/2.0f, -scale+epsilon},{-scale, -offset2/2.0f, -scale+epsilon},{-scale+offset2, -offset2/2.0f, -scale+epsilon}};
//			triangles.push_back({t1, color});
//			triangles.push_back({t2, color});
		} break;
		case 11: // cube 11
		{
			// Face 2 - cube 11
			color = GetCubeColor(4, false, s);
			t1 = {{scale-offset, -offset/2.0f, -scale},{scale-offset, offset/2.0f, -scale},{scale, offset/2.0f, -scale}};
			t2 = {{scale, offset/2.0f, -scale},{scale, -offset/2.0f, -scale},{scale-offset, -offset/2.0f, -scale}};
			triangles.push_back({t1, color});
			triangles.push_back({t2, color});

			color = GetCubeColor(-1, false, s);
			t1 = {{scale-offset2, -offset2/2.0f, -scale+epsilon},{scale-offset2, offset2/2.0f, -scale+epsilon},{scale, offset2/2.0f, -scale+epsilon}};
			t2 = {{scale, offset2/2.0f, -scale+epsilon},{scale, -offset2/2.0f, -scale+epsilon},{scale-offset2, -offset2/2.0f, -scale+epsilon}};
//			triangles.push_back({t1, color});
//			triangles.push_back({t2, color});
			
			// Face 3 - cube 11
			color = GetCubeColor(4, true, s);
			t1 = {{scale, -offset/2.0f, -scale+offset},{scale, offset/2.0f, -scale+offset},{scale, offset/2.0f, -scale}};
			t2 = {{scale, offset/2.0f, -scale},{scale, -offset/2.0f, -scale},{scale, -offset/2.0f, -scale+offset}};
			triangles.push_back({t1, color});
			triangles.push_back({t2, color});

			color = GetCubeColor(-1, true, s);
			t1 = {{scale-epsilon, -offset2/2.0f, -scale+offset2},{scale-epsilon, offset2/2.0f, -scale+offset2},{scale-epsilon, offset2/2.0f, -scale}};
			t2 = {{scale-epsilon, offset2/2.0f, -scale},{scale-epsilon, -offset2/2.0f, -scale},{scale-epsilon, -offset2/2.0f, -scale+offset2}};
//			triangles.push_back({t1, color});
//			triangles.push_back({t2, color});

		} break;
		case 15: // cube 15
		{
			// Face 1 - cube 15
			color = GetCubeColor(8, false, s);
			t1 = {{-scale, -offset/2.0f, scale-offset},{-scale, offset/2.0f, scale-offset},{-scale, offset/2.0f, scale}};
			t2 = {{-scale, offset/2.0f, scale},{-scale, -offset/2.0f, scale},{-scale, -offset/2.0f, scale-offset}};
			triangles.push_back({t1, color});
			triangles.push_back({t2, color});

			color = GetCubeColor(-1, false, s);
			t1 = {{-scale+epsilon, -offset2/2.0f, scale-offset2},{-scale+epsilon, offset2/2.0f, scale-offset2},{-scale+epsilon, offset2/2.0f, scale}};
			t2 = {{-scale+epsilon, offset2/2.0f, scale},{-scale+epsilon, -offset2/2.0f, scale},{-scale+epsilon, -offset2/2.0f, scale-offset2}};
//			triangles.push_back({t1, color});
//			triangles.push_back({t2, color});

			// Face 4 - cube 17
			color = GetCubeColor(8, true, s);
			t1 = {{-scale+offset, -offset/2.0f, scale},{-scale+offset, offset/2.0f, scale},{-scale, offset/2.0f, scale}};
			t2 = {{-scale, offset/2.0f, scale},{-scale, -offset/2.0f, scale},{-scale+offset, -offset/2.0f, scale}};
			triangles.push_back({t1, color});
			triangles.push_back({t2, color});

			color = GetCubeColor(-1, true, s);
			t1 = {{-scale+offset2, -offset2/2.0f, scale-epsilon},{-scale+offset2, offset2/2.0f, scale-epsilon},{-scale, offset2/2.0f, scale-epsilon}};
			t2 = {{-scale, offset2/2.0f, scale-epsilon},{-scale, -offset2/2.0f, scale-epsilon},{-scale+offset2, -offset2/2.0f, scale-epsilon}};
//			triangles.push_back({t1, color});
//			triangles.push_back({t2, color});

		} break;
		case 17: // cube 17
		{
			// Face 3 - cube 17
			color = GetCubeColor(6, false, s);
			t1 = {{scale, -offset/2.0f, scale-offset},{scale, offset/2.0f, scale-offset},{scale, offset/2.0f, scale}};
			t2 = {{scale, offset/2.0f, scale},{scale, -offset/2.0f, scale},{scale, -offset/2.0f, scale-offset}};
			triangles.push_back({t1, color});
			triangles.push_back({t2, color});

			color = GetCubeColor(-1, false, s);
			t1 = {{scale-epsilon, -offset2/2.0f, scale-offset2},{scale-epsilon, offset2/2.0f, scale-offset2},{scale-epsilon, offset2/2.0f, scale}};
			t2 = {{scale-epsilon, offset2/2.0f, scale},{scale-epsilon, -offset2/2.0f, scale},{scale-epsilon, -offset2/2.0f, scale-offset2}};
//			triangles.push_back({t1, color});
//			triangles.push_back({t2, color});

			// Face 4 - cube 15
			color = GetCubeColor(6, true, s);
			t1 = {{scale-offset, -offset/2.0f, scale},{scale-offset, offset/2.0f, scale},{scale, offset/2.0f, scale}};
			t2 = {{scale, offset/2.0f, scale},{scale, -offset/2.0f, scale},{scale-offset, -offset/2.0f, scale}};
			triangles.push_back({t1, color});
			triangles.push_back({t2, color});

			color = GetCubeColor(-1, true, s);
			t1 = {{scale-offset2, -offset2/2.0f, scale-epsilon},{scale-offset2, offset2/2.0f, scale-epsilon},{scale, offset2/2.0f, scale-epsilon}};
			t2 = {{scale, offset2/2.0f, scale-epsilon},{scale, -offset2/2.0f, scale-epsilon},{scale-offset2, -offset2/2.0f, scale-epsilon}};
//			triangles.push_back({t1, color});
//			triangles.push_back({t2, color});
		} break;
		case 19: // cube 19
		{
			// Face 2 - cube 19
			color = GetCubeColor(10, false, s);
			t1 = {{-offset/2.0f, scale-offset, -scale},{offset/2.0f, scale-offset, -scale},{offset/2.0f, scale, -scale}};
			t2 = {{offset/2.0f, scale, -scale},{-offset/2.0f, scale, -scale},{-offset/2.0f, scale-offset, -scale}};
			triangles.push_back({t1, color});
			triangles.push_back({t2, color});

			color = GetCubeColor(-1, false, s);
			t1 = {{-offset2/2.0f, scale-offset2, -scale+epsilon},{offset2/2.0f, scale-offset2, -scale+epsilon},{offset2/2.0f, scale, -scale+epsilon}};
			t2 = {{offset2/2.0f, scale, -scale+epsilon},{-offset2/2.0f, scale, -scale+epsilon},{-offset2/2.0f, scale-offset2, -scale+epsilon}};
//			triangles.push_back({t1, color});
//			triangles.push_back({t2, color});

			// Face 5 - cube 19
			color = GetCubeColor(10, true, s);
			t1 = {{-offset/2.0f, scale, -scale},{-offset/2.0f, scale, -scale+offset},{offset/2.0f, scale, -scale+offset}};
			t2 = {{offset/2.0f, scale, -scale+offset},{offset/2.0f, scale, -scale},{-offset/2.0f, scale, -scale}};
			triangles.push_back({t1, color});
			triangles.push_back({t2, color});

			color = GetCubeColor(-1, true, s);
			t1 = {{-offset2/2.0f, scale-epsilon, -scale},{-offset2/2.0f, scale-epsilon, -scale+offset2},{offset2/2.0f, scale-epsilon, -scale+offset2}};
			t2 = {{offset2/2.0f, scale-epsilon, -scale+offset2},{offset2/2.0f, scale-epsilon, -scale},{-offset2/2.0f, scale-epsilon, -scale}};
//			triangles.push_back({t1, color});
//			triangles.push_back({t2, color});

		} break;
		case 21: // cube 21
		{
			// Face 1 - cube 21
			color = GetCubeColor(9, false, s);
			t1 = {{-scale, scale-offset, -offset/2.0f},{-scale, scale-offset, offset/2.0f},{-scale, scale, offset/2.0f}};
			t2 = {{-scale, scale, offset/2.0f},{-scale, scale, -offset/2.0f},{-scale, scale-offset, -offset/2.0f}};
			triangles.push_back({t1, color});
			triangles.push_back({t2, color});

			color = GetCubeColor(-1, false, s);
			t1 = {{-scale+epsilon, scale-offset2, -offset2/2.0f},{-scale+epsilon, scale-offset2, offset2/2.0f},{-scale+epsilon, scale, offset2/2.0f}};
			t2 = {{-scale+epsilon, scale, offset2/2.0f},{-scale+epsilon, scale, -offset2/2.0f},{-scale+epsilon, scale-offset2, -offset2/2.0f}};
//			triangles.push_back({t1, color});
//			triangles.push_back({t2, color});

			// Face 5 - cube 21
			color = GetCubeColor(9, true, s);
			t1 = {{-scale, scale, offset/2.0f},{-scale, scale, -offset/2.0f},{-scale+offset, scale, -offset/2.0f}};
			t2 = {{-scale+offset, scale, -offset/2.0f},{-scale+offset, scale, offset/2.0f},{-scale, scale, offset/2.0f}};
			triangles.push_back({t1, color});
			triangles.push_back({t2, color});

			color = GetCubeColor(-1, true, s);
			t1 = {{-scale, scale-epsilon, offset2/2.0f},{-scale, scale-epsilon, -offset2/2.0f},{-scale+offset2, scale-epsilon, -offset2/2.0f}};
			t2 = {{-scale+offset2, scale-epsilon, -offset2/2.0f},{-scale+offset2, scale-epsilon, offset2/2.0f},{-scale, scale-epsilon, offset2/2.0f}};
//			triangles.push_back({t1, color});
//			triangles.push_back({t2, color});

		} break;
		case 23: // cube 23
		{
			// Face 3 - cube 23
			color = GetCubeColor(11, false, s);
			t1 = {{scale, scale-offset, -offset/2.0f},{scale, scale-offset, offset/2.0f},{scale, scale, offset/2.0f}};
			t2 = {{scale, scale, offset/2.0f},{scale, scale, -offset/2.0f},{scale, scale-offset, -offset/2.0f}};
			triangles.push_back({t1, color});
			triangles.push_back({t2, color});

			color = GetCubeColor(-1, false, s);
			t1 = {{scale-epsilon, scale-offset2, -offset2/2.0f},{scale-epsilon, scale-offset2, offset2/2.0f},{scale-epsilon, scale, offset2/2.0f}};
			t2 = {{scale-epsilon, scale, offset2/2.0f},{scale-epsilon, scale, -offset2/2.0f},{scale-epsilon, scale-offset2, -offset2/2.0f}};
//			triangles.push_back({t1, color});
//			triangles.push_back({t2, color});

			// Face 5 - cube 23
			color = GetCubeColor(11, true, s);
			t1 = {{scale, scale, offset/2.0f},{scale, scale, -offset/2.0f},{scale-offset, scale, -offset/2.0f}};
			t2 = {{scale-offset, scale, -offset/2.0f},{scale-offset, scale, offset/2.0f},{scale, scale, offset/2.0f}};
			triangles.push_back({t1, color});
			triangles.push_back({t2, color});

			color = GetCubeColor(-1, true, s);
			t1 = {{scale, scale-epsilon, offset2/2.0f},{scale, scale-epsilon, -offset2/2.0f},{scale-offset2, scale-epsilon, -offset2/2.0f}};
			t2 = {{scale-offset2, scale-epsilon, -offset2/2.0f},{scale-offset2, scale-epsilon, offset2/2.0f},{scale, scale-epsilon, offset2/2.0f}};
//			triangles.push_back({t1, color});
//			triangles.push_back({t2, color});

		} break;
		case 25: // cube 25
		{
			// Face 4 - cube 25
			color = GetCubeColor(12, false, s);
			t1 = {{-offset/2.0f, scale-offset, scale},{offset/2.0f, scale-offset, scale},{offset/2.0f, scale, scale}};
			t2 = {{offset/2.0f, scale, scale},{-offset/2.0f, scale, scale},{-offset/2.0f, scale-offset, scale}};
			triangles.push_back({t1, color});
			triangles.push_back({t2, color});

			color = GetCubeColor(-1, false, s);
			t1 = {{-offset2/2.0f, scale-offset2, scale-epsilon},{offset2/2.0f, scale-offset2, scale-epsilon},{offset2/2.0f, scale, scale-epsilon}};
			t2 = {{offset2/2.0f, scale, scale-epsilon},{-offset2/2.0f, scale, scale-epsilon},{-offset2/2.0f, scale-offset2, scale-epsilon}};
//			triangles.push_back({t1, color});
//			triangles.push_back({t2, color});

			// Face 5 - cube 25
			color = GetCubeColor(12, true, s);
			t1 = {{-offset/2.0f, scale, scale},{-offset/2.0f, scale, scale-offset},{offset/2.0f, scale, scale-offset}};
			t2 = {{offset/2.0f, scale, scale-offset},{offset/2.0f, scale, scale},{-offset/2.0f, scale, scale}};
			triangles.push_back({t1, color});
			triangles.push_back({t2, color});

			color = GetCubeColor(-1, true, s);
			t1 = {{-offset2/2.0f, scale-epsilon, scale},{-offset2/2.0f, scale-epsilon, scale-offset2},{offset2/2.0f, scale-epsilon, scale-offset2}};
			t2 = {{offset2/2.0f, scale-epsilon, scale-offset2},{offset2/2.0f, scale-epsilon, scale},{-offset2/2.0f, scale-epsilon, scale}};
//			triangles.push_back({t1, color});
//			triangles.push_back({t2, color});

		} break;
	}
}

//void RubikEdge::OpenGLDrawCube(const RubikEdgeState &s, int cube) const
//{
//	const float scale = 0.3;
//	const float offset = 0.95*2.0*scale/3.0;
//	const float offset2 = 2.0*scale/3.0;
//	const float epsilon = 0.002;
//	switch(cube)
//	{
//		case 1: // cube 1
//		{
//			// Face 0 - cube 1
//			SetCubeColor(3, false, s);
//			glVertex3f(-offset/2.0, -scale, -scale);
//			glVertex3f(-offset/2.0, -scale, -scale+offset);
//			glVertex3f(offset/2.0, -scale, -scale+offset);
//			glVertex3f(offset/2.0, -scale, -scale);
//
//			SetCubeColor(-1, false, s);
//			glVertex3f(-offset2/2.0, -scale+epsilon, -scale);
//			glVertex3f(-offset2/2.0, -scale+epsilon, -scale+offset2);
//			glVertex3f(offset2/2.0, -scale+epsilon, -scale+offset2);
//			glVertex3f(offset2/2.0, -scale+epsilon, -scale);
//
//			
//			// Face 2 - cube 1
//			SetCubeColor(3, true, s);
//			glVertex3f(-offset/2.0, -scale+offset, -scale);
//			glVertex3f(offset/2.0, -scale+offset, -scale);
//			glVertex3f(offset/2.0, -scale, -scale);
//			glVertex3f(-offset/2.0, -scale, -scale);
//
//			SetCubeColor(-1, true, s);
//			glVertex3f(-offset2/2.0, -scale+offset2, -scale+epsilon);
//			glVertex3f(offset2/2.0, -scale+offset2, -scale+epsilon);
//			glVertex3f(offset2/2.0, -scale, -scale+epsilon);
//			glVertex3f(-offset2/2.0, -scale, -scale+epsilon);
//
//		} break;
//		case 3: // cube 3
//		{
//			// Face 0 - cube 3
//			SetCubeColor(1, false, s);
//			glVertex3f(-scale, -scale, offset/2.0);
//			glVertex3f(-scale, -scale, -offset/2.0);
//			glVertex3f(-scale+offset, -scale, -offset/2.0);
//			glVertex3f(-scale+offset, -scale, offset/2.0);
//
//			SetCubeColor(-1, false, s);
//			glVertex3f(-scale, -scale+epsilon, offset2/2.0);
//			glVertex3f(-scale, -scale+epsilon, -offset2/2.0);
//			glVertex3f(-scale+offset2, -scale+epsilon, -offset2/2.0);
//			glVertex3f(-scale+offset2, -scale+epsilon, offset2/2.0);
//
//			// Face 1 - cube 3
//			SetCubeColor(1, true, s);
//			glVertex3f(-scale, -scale+offset, -offset/2.0);
//			glVertex3f(-scale, -scale+offset, offset/2.0);
//			glVertex3f(-scale, -scale, offset/2.0);
//			glVertex3f(-scale, -scale, -offset/2.0);
//
//			SetCubeColor(-1, true, s);
//			glVertex3f(-scale+epsilon, -scale+offset2, -offset2/2.0);
//			glVertex3f(-scale+epsilon, -scale+offset2, offset2/2.0);
//			glVertex3f(-scale+epsilon, -scale, offset2/2.0);
//			glVertex3f(-scale+epsilon, -scale, -offset2/2.0);
//
//		} break;
//		case 5: // cube 5
//		{
//			// Face 0 - cube 5
//			SetCubeColor(5, false, s);
//			glVertex3f(scale, -scale, offset/2.0);
//			glVertex3f(scale, -scale, -offset/2.0);
//			glVertex3f(scale-offset, -scale, -offset/2.0);
//			glVertex3f(scale-offset, -scale, offset/2.0);
//
//			SetCubeColor(-1, false, s);
//			glVertex3f(scale, -scale+epsilon, offset2/2.0);
//			glVertex3f(scale, -scale+epsilon, -offset2/2.0);
//			glVertex3f(scale-offset2, -scale+epsilon, -offset2/2.0);
//			glVertex3f(scale-offset2, -scale+epsilon, offset2/2.0);
//
//			// Face 3 - cube 5
//			SetCubeColor(5, true, s);
//			glVertex3f(scale, -scale+offset, -offset/2.0);
//			glVertex3f(scale, -scale+offset, offset/2.0);
//			glVertex3f(scale, -scale, offset/2.0);
//			glVertex3f(scale, -scale, -offset/2.0);
//			
//			SetCubeColor(-1, true, s);
//			glVertex3f(scale-epsilon, -scale+offset2, -offset2/2.0);
//			glVertex3f(scale-epsilon, -scale+offset2, offset2/2.0);
//			glVertex3f(scale-epsilon, -scale, offset2/2.0);
//			glVertex3f(scale-epsilon, -scale, -offset2/2.0);
//		} break;
//		case 7: // cube 7
//		{
//			// Face 0 - cube 7
//			SetCubeColor(7, false, s);
//			glVertex3f(-offset/2.0, -scale, scale);
//			glVertex3f(-offset/2.0, -scale, scale-offset);
//			glVertex3f(offset/2.0, -scale, scale-offset);
//			glVertex3f(offset/2.0, -scale, scale);
//
//			SetCubeColor(-1, false, s);
//			glVertex3f(-offset2/2.0, -scale+epsilon, scale);
//			glVertex3f(-offset2/2.0, -scale+epsilon, scale-offset2);
//			glVertex3f(offset2/2.0, -scale+epsilon, scale-offset2);
//			glVertex3f(offset2/2.0, -scale+epsilon, scale);
//
//			// Face 4 - cube 7
//			SetCubeColor(7, true, s);
//			glVertex3f(-offset/2.0, -scale+offset, scale);
//			glVertex3f(offset/2.0, -scale+offset, scale);
//			glVertex3f(offset/2.0, -scale, scale);
//			glVertex3f(-offset/2.0, -scale, scale);
//
//			SetCubeColor(-1, true, s);
//			glVertex3f(-offset2/2.0, -scale+offset2, scale-epsilon);
//			glVertex3f(offset2/2.0, -scale+offset2, scale-epsilon);
//			glVertex3f(offset2/2.0, -scale, scale-epsilon);
//			glVertex3f(-offset2/2.0, -scale, scale-epsilon);
//
//		} break;
//		case 9: // cube 9
//		{
//			// Face 1 - cube 9
//			SetCubeColor(2, false, s);
//			glVertex3f(-scale, -offset/2.0, -scale+offset);
//			glVertex3f(-scale, offset/2.0, -scale+offset);
//			glVertex3f(-scale, offset/2.0, -scale);
//			glVertex3f(-scale, -offset/2.0, -scale);
//
//			SetCubeColor(-1, false, s);
//			glVertex3f(-scale+epsilon, -offset2/2.0, -scale+offset2);
//			glVertex3f(-scale+epsilon, offset2/2.0, -scale+offset2);
//			glVertex3f(-scale+epsilon, offset2/2.0, -scale);
//			glVertex3f(-scale+epsilon, -offset2/2.0, -scale);
//
//			
//			// Face 2 - cube 9
//			SetCubeColor(2, true, s);
//			glVertex3f(-scale+offset, -offset/2.0, -scale);
//			glVertex3f(-scale+offset, offset/2.0, -scale);
//			glVertex3f(-scale, offset/2.0, -scale);
//			glVertex3f(-scale, -offset/2.0, -scale);
//
//			SetCubeColor(-1, true, s);
//			glVertex3f(-scale+offset2, -offset2/2.0, -scale+epsilon);
//			glVertex3f(-scale+offset2, offset2/2.0, -scale+epsilon);
//			glVertex3f(-scale, offset2/2.0, -scale+epsilon);
//			glVertex3f(-scale, -offset2/2.0, -scale+epsilon);
//		} break;
//		case 11: // cube 11
//		{
//			// Face 2 - cube 11
//			SetCubeColor(4, false, s);
//			glVertex3f(scale-offset, -offset/2.0, -scale);
//			glVertex3f(scale-offset, offset/2.0, -scale);
//			glVertex3f(scale, offset/2.0, -scale);
//			glVertex3f(scale, -offset/2.0, -scale);
//
//			SetCubeColor(-1, false, s);
//			glVertex3f(scale-offset2, -offset2/2.0, -scale+epsilon);
//			glVertex3f(scale-offset2, offset2/2.0, -scale+epsilon);
//			glVertex3f(scale, offset2/2.0, -scale+epsilon);
//			glVertex3f(scale, -offset2/2.0, -scale+epsilon);
//
//			
//			// Face 3 - cube 11
//			SetCubeColor(4, true, s);
//			glVertex3f(scale, -offset/2.0, -scale+offset);
//			glVertex3f(scale, offset/2.0, -scale+offset);
//			glVertex3f(scale, offset/2.0, -scale);
//			glVertex3f(scale, -offset/2.0, -scale);
//
//			SetCubeColor(-1, true, s);
//			glVertex3f(scale-epsilon, -offset2/2.0, -scale+offset2);
//			glVertex3f(scale-epsilon, offset2/2.0, -scale+offset2);
//			glVertex3f(scale-epsilon, offset2/2.0, -scale);
//			glVertex3f(scale-epsilon, -offset2/2.0, -scale);
//
//		} break;
//		case 15: // cube 15
//		{
//			// Face 1 - cube 15
//			SetCubeColor(8, false, s);
//			glVertex3f(-scale, -offset/2.0, scale-offset);
//			glVertex3f(-scale, offset/2.0, scale-offset);
//			glVertex3f(-scale, offset/2.0, scale);
//			glVertex3f(-scale, -offset/2.0, scale);
//
//			SetCubeColor(-1, false, s);
//			glVertex3f(-scale+epsilon, -offset2/2.0, scale-offset2);
//			glVertex3f(-scale+epsilon, offset2/2.0, scale-offset2);
//			glVertex3f(-scale+epsilon, offset2/2.0, scale);
//			glVertex3f(-scale+epsilon, -offset2/2.0, scale);
//			
//			// Face 4 - cube 17
//			SetCubeColor(8, true, s);
//			glVertex3f(-scale+offset, -offset/2.0, scale);
//			glVertex3f(-scale+offset, offset/2.0, scale);
//			glVertex3f(-scale, offset/2.0, scale);
//			glVertex3f(-scale, -offset/2.0, scale);
//			
//			SetCubeColor(-1, true, s);
//			glVertex3f(-scale+offset2, -offset2/2.0, scale-epsilon);
//			glVertex3f(-scale+offset2, offset2/2.0, scale-epsilon);
//			glVertex3f(-scale, offset2/2.0, scale-epsilon);
//			glVertex3f(-scale, -offset2/2.0, scale-epsilon);
//
//		} break;
//		case 17: // cube 17
//		{
//			// Face 3 - cube 17
//			SetCubeColor(6, false, s);
//			glVertex3f(scale, -offset/2.0, scale-offset);
//			glVertex3f(scale, offset/2.0, scale-offset);
//			glVertex3f(scale, offset/2.0, scale);
//			glVertex3f(scale, -offset/2.0, scale);
//
//			SetCubeColor(-1, false, s);
//			glVertex3f(scale-epsilon, -offset2/2.0, scale-offset2);
//			glVertex3f(scale-epsilon, offset2/2.0, scale-offset2);
//			glVertex3f(scale-epsilon, offset2/2.0, scale);
//			glVertex3f(scale-epsilon, -offset2/2.0, scale);
//
//			// Face 4 - cube 15
//			SetCubeColor(6, true, s);
//			glVertex3f(scale-offset, -offset/2.0, scale);
//			glVertex3f(scale-offset, offset/2.0, scale);
//			glVertex3f(scale, offset/2.0, scale);
//			glVertex3f(scale, -offset/2.0, scale);
//
//			SetCubeColor(-1, true, s);
//			glVertex3f(scale-offset2, -offset2/2.0, scale-epsilon);
//			glVertex3f(scale-offset2, offset2/2.0, scale-epsilon);
//			glVertex3f(scale, offset2/2.0, scale-epsilon);
//			glVertex3f(scale, -offset2/2.0, scale-epsilon);
//		} break;
//		case 19: // cube 19
//		{
//			// Face 2 - cube 19
//			SetCubeColor(10, false, s);
//			glVertex3f(-offset/2.0, scale-offset, -scale);
//			glVertex3f(offset/2.0, scale-offset, -scale);
//			glVertex3f(offset/2.0, scale, -scale);
//			glVertex3f(-offset/2.0, scale, -scale);
//
//			SetCubeColor(-1, false, s);
//			glVertex3f(-offset2/2.0, scale-offset2, -scale+epsilon);
//			glVertex3f(offset2/2.0, scale-offset2, -scale+epsilon);
//			glVertex3f(offset2/2.0, scale, -scale+epsilon);
//			glVertex3f(-offset2/2.0, scale, -scale+epsilon);
//			
//			// Face 5 - cube 19
//			SetCubeColor(10, true, s);
//			glVertex3f(-offset/2.0, scale, -scale);
//			glVertex3f(-offset/2.0, scale, -scale+offset);
//			glVertex3f(offset/2.0, scale, -scale+offset);
//			glVertex3f(offset/2.0, scale, -scale);
//
//			SetCubeColor(-1, true, s);
//			glVertex3f(-offset2/2.0, scale-epsilon, -scale);
//			glVertex3f(-offset2/2.0, scale-epsilon, -scale+offset2);
//			glVertex3f(offset2/2.0, scale-epsilon, -scale+offset2);
//			glVertex3f(offset2/2.0, scale-epsilon, -scale);
//			
//		} break;
//		case 21: // cube 21
//		{
//			// Face 1 - cube 21
//			SetCubeColor(9, false, s);
//			glVertex3f(-scale, scale-offset, -offset/2.0);
//			glVertex3f(-scale, scale-offset, offset/2.0);
//			glVertex3f(-scale, scale, offset/2.0);
//			glVertex3f(-scale, scale, -offset/2.0);
//
//			SetCubeColor(-1, false, s);
//			glVertex3f(-scale+epsilon, scale-offset2, -offset2/2.0);
//			glVertex3f(-scale+epsilon, scale-offset2, offset2/2.0);
//			glVertex3f(-scale+epsilon, scale, offset2/2.0);
//			glVertex3f(-scale+epsilon, scale, -offset2/2.0);
//
//			// Face 5 - cube 21
//			SetCubeColor(9, true, s);
//			glVertex3f(-scale, scale, offset/2.0);
//			glVertex3f(-scale, scale, -offset/2.0);
//			glVertex3f(-scale+offset, scale, -offset/2.0);
//			glVertex3f(-scale+offset, scale, offset/2.0);
//
//			SetCubeColor(-1, true, s);
//			glVertex3f(-scale, scale-epsilon, offset2/2.0);
//			glVertex3f(-scale, scale-epsilon, -offset2/2.0);
//			glVertex3f(-scale+offset2, scale-epsilon, -offset2/2.0);
//			glVertex3f(-scale+offset2, scale-epsilon, offset2/2.0);
//			
//		} break;
//		case 23: // cube 23
//		{
//			// Face 3 - cube 23
//			SetCubeColor(11, false, s);
//			glVertex3f(scale, scale-offset, -offset/2.0);
//			glVertex3f(scale, scale-offset, offset/2.0);
//			glVertex3f(scale, scale, offset/2.0);
//			glVertex3f(scale, scale, -offset/2.0);
//
//			SetCubeColor(-1, false, s);
//			glVertex3f(scale-epsilon, scale-offset2, -offset2/2.0);
//			glVertex3f(scale-epsilon, scale-offset2, offset2/2.0);
//			glVertex3f(scale-epsilon, scale, offset2/2.0);
//			glVertex3f(scale-epsilon, scale, -offset2/2.0);
//
//			// Face 5 - cube 23
//			SetCubeColor(11, true, s);
//			glVertex3f(scale, scale, offset/2.0);
//			glVertex3f(scale, scale, -offset/2.0);
//			glVertex3f(scale-offset, scale, -offset/2.0);
//			glVertex3f(scale-offset, scale, offset/2.0);
//
//			SetCubeColor(-1, true, s);
//			glVertex3f(scale, scale-epsilon, offset2/2.0);
//			glVertex3f(scale, scale-epsilon, -offset2/2.0);
//			glVertex3f(scale-offset2, scale-epsilon, -offset2/2.0);
//			glVertex3f(scale-offset2, scale-epsilon, offset2/2.0);
//			
//		} break;
//		case 25: // cube 25
//		{
//			// Face 4 - cube 25
//			SetCubeColor(12, false, s);
//			glVertex3f(-offset/2.0, scale-offset, scale);
//			glVertex3f(offset/2.0, scale-offset, scale);
//			glVertex3f(offset/2.0, scale, scale);
//			glVertex3f(-offset/2.0, scale, scale);
//
//			SetCubeColor(-1, false, s);
//			glVertex3f(-offset2/2.0, scale-offset2, scale-epsilon);
//			glVertex3f(offset2/2.0, scale-offset2, scale-epsilon);
//			glVertex3f(offset2/2.0, scale, scale-epsilon);
//			glVertex3f(-offset2/2.0, scale, scale-epsilon);
//
//			// Face 5 - cube 25
//			SetCubeColor(12, true, s);
//			glVertex3f(-offset/2.0, scale, scale);
//			glVertex3f(-offset/2.0, scale, scale-offset);
//			glVertex3f(offset/2.0, scale, scale-offset);
//			glVertex3f(offset/2.0, scale, scale);
//
//			SetCubeColor(-1, true, s);
//			glVertex3f(-offset2/2.0, scale-epsilon, scale);
//			glVertex3f(-offset2/2.0, scale-epsilon, scale-offset2);
//			glVertex3f(offset2/2.0, scale-epsilon, scale-offset2);
//			glVertex3f(offset2/2.0, scale-epsilon, scale);
//
//		} break;
//	}
//}

int RubikEdge::GetCubeColor(int which, bool face, const RubikEdgeState &s) const
{
	int cubes_first[12] = { 0, 1, 0, 2, 0, 3, 0, 1, 1, 2, 3, 4};
	int cubes_second[12] = { 1, 2, 2, 3, 3, 4, 4, 4, 5, 5, 5, 5};
	
	if (which == -1)
	{
//		glColor3f(0.0, 0.0, 0.0);
		return -1;
	}
	
	int theColor = -1;
	int cube = s.GetCubeInLoc(which-1);
	if (cube == 0xF)
	{
//		glColor3f(0.0, 0.0, 0.0);
		return -1;
	}
	bool flipped = s.GetCubeOrientation(cube);
	if (flipped == face)
		theColor = cubes_first[cube];
	else
		theColor = cubes_second[cube];
	
	return theColor;
}
//
//void RubikEdge::SetCubeColor(int which, bool face, const RubikEdgeState &s) const
//{
//	int cubes_first[12] = { 0, 1, 0, 2, 0, 3, 0, 1, 1, 2, 3, 4};
//	int cubes_second[12] = { 1, 2, 2, 3, 3, 4, 4, 4, 5, 5, 5, 5};
//
//	if (which == -1)
//	{
//		glColor3f(0.0, 0.0, 0.0);
//		return;
//	}
//	
//	int theColor = -1;
//	int cube = s.GetCubeInLoc(which-1);
//	if (cube == 0xF)
//	{
//		glColor3f(0.0, 0.0, 0.0);
//		return;
//	}
//	bool flipped = s.GetCubeOrientation(cube);
//	if (flipped == face)
//		theColor = cubes_first[cube];
//	else
//		theColor = cubes_second[cube];
//
//	switch (theColor)
//	{
//		case 0: glColor3f(1.0, 0.0, 0.0); break;
//		case 1: glColor3f(0.0, 1.0, 0.0); break;
//		case 2: glColor3f(0.0, 0.0, 1.0); break;
//		case 3: glColor3f(1.0, 1.0, 0.0); break;
//		case 4: glColor3f(1.0, 0.75, 0.0); break;
//		case 5: glColor3f(1.0, 1.0, 1.0); break;
//		default: assert(false);
//	}
//}


RubikEdgePDB::RubikEdgePDB(RubikEdge *e, const RubikEdgeState &s, std::vector<int> &distinctEdges)
:PDBHeuristic(e), edges(distinctEdges), puzzles(std::thread::hardware_concurrency())
{
	for (size_t x = 0; x < puzzles.size(); x++)
		puzzles[x].resize(12);
	SetGoal(s);
}

uint64_t RubikEdgePDB::GetStateSpaceSize()
{
#pragma message("This code belongs in the RubikEdge, not in the PDB.")
	return 479001600ll*2048ll;
}

uint64_t RubikEdgePDB::GetStateHash(const RubikEdgeState &s)
{
	uint64_t perm = 0, dual = 0;
	for (int x = 0; x < 12; x++)
	{
		int val = s.GetCubeInLoc(x);
		set(perm, x, val);
		set(dual, val, x);
	}
	
	uint64_t hashVal = 0;
	for (int x = 0; x < 11; x++)
	{
		hashVal = (hashVal<<1)+s.GetCubeOrientation(11-x);
	}
	hashVal = hashVal*Factorial(12)+RubikEdge::MRRank(12, perm, dual);
	return hashVal;
}

void RubikEdgePDB::GetStateFromHash(RubikEdgeState &s, uint64_t hash)
{
	int cnt = 0;
	uint64_t bits = hash/Factorial(12);
	uint64_t hVal = hash%Factorial(12);
	for (int x = 10; x >= 0; x--)
	{
		s.SetCubeOrientation(11-x, bits&0x1);
		cnt += bits & 0x1;
		bits >>= 1;
	}
	if (1 == cnt%2)
	{
		s.SetCubeOrientation(11-11, true);
	}
	else {
		s.SetCubeOrientation(11-11, false);
	}
	
	uint64_t val = 0;
	for (int x = 0; x < 12; x++)
		set(val, x, x);
	RubikEdge::MRUnrank2(12, hVal, val);
	for (int x = 0; x < 12; x++)
	{
		s.SetCubeInLoc(x, get(val, x));
	}
}

//

uint64_t RubikEdgePDB::GetPDBSize() const
{
	// last tile is symmetric
	uint64_t power2[] = {1, 2, 4, 8, 16, 32, 64, 128, 256, 512, 1024, 2048, 2048};
	int elts = (int)edges.size();
	return FactorialUpperK(12, 12-elts)*power2[elts];
//	return mr1.GetMaxRank()*power2[elts];
}

#define MR

uint64_t RubikEdgePDB::GetPDBHash(const RubikEdgeState &s, int threadID) const
{
#ifdef MR
	int puzzle[12] = {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1};
	int dual[16]; // seamlessly handle 0xF entries (no cube)
	int newdual[16]; // seamlessly handle 0xF entries (no cube)
	int edgeSize = edges.size();
	int lastPiece = 12-(int)edgeSize;
	for (int x = 0; x < 12; x++)
		dual[s.GetCubeInLoc(x)] = x;
	for (int x = 0; x < edgeSize; x++)
	{
		newdual[x] = dual[edges[x]];
		puzzle[dual[edges[x]]] = x;
	}
	uint64_t hashVal = 0;
	uint64_t part2 = 0;
	hashVal = mr1.Rank(puzzle, newdual, edgeSize, 12);//mr1.GetRank(puzzle, threadID);

	
	int limit = std::min((int)edges.size(), 11);
	for (int x = 0; x < limit; x++)
	{
		part2 = part2*2+(s.GetCubeOrientation(edges[x]));
	}
	return part2*FactorialUpperK(12, lastPiece)+hashVal;
#else
	int puzzle[12];
	int dual[16]; // seamlessly handle 0xF entries (no cube)
	int lastPiece = 12-(int)edges.size();
	//std::cout << "!" << s << "\n";
	for (int x = 0; x < 12; x++)
		dual[s.GetCubeInLoc(x)] = x;
	for (int x = 0; x < edges.size(); x++)
		puzzle[x] = dual[edges[x]];
	
	uint64_t hashVal = 0;
	uint64_t part2 = 0;
	int numEntriesLeft = 12;
	for (unsigned int x = 0; x < edges.size(); x++)
	{
		hashVal += puzzle[x]*FactorialUpperK(numEntriesLeft-1, lastPiece);
		
		numEntriesLeft--;
		for (unsigned y = x; y < edges.size(); y++)
		{
			if (puzzle[y] > puzzle[x])
				puzzle[y]--;
		}
	}
	int limit = std::min((int)edges.size(), 11);
	for (int x = 0; x < limit; x++)
	{
		part2 = part2*2+(s.GetCubeOrientation(edges[x])?1:0);
		//part2 = part2*3+s.GetCubeOrientation(dual[corners[x]]);
	}
	return part2*FactorialUpperK(12, lastPiece)+hashVal;
#endif
	
}

void RubikEdgePDB::GetStateFromPDBHash(uint64_t hash, RubikEdgeState &s, int threadID) const
{
#ifdef MR
	
	int lastPiece = 12-(int)edges.size();
	int puzzle[12] = {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1};
	int dual[16] = {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1};
	uint64_t hashVal = hash;
	int edgeSize = edges.size();
	hash /= FactorialUpperK(12, lastPiece); // for rotations
	hashVal = hashVal%FactorialUpperK(12, lastPiece); // for pieces
	
	mr1.Unrank(hashVal, puzzle, dual, edgeSize, 12);
	for (int x = 0; x < 12; x++)
	{
		s.SetCubeInLoc(x, 0xF);
		s.SetCubeOrientation(x, 0);
	}
	
	for (int x = 0; x < edgeSize; x++)
	{
		s.SetCubeInLoc(dual[x], edges[x]);
	}
	
	int cnt = 0;
	int limit = std::min((int)edgeSize, 11);
	for (int x = limit-1; x >= 0; x--)
	{
		s.SetCubeOrientation(edges[x], hash%2);
		cnt += hash%2;
		hash/=2;
	}
	if (edges.size() == 12)
	{
		assert(!"Be sure to test this code");
		s.SetCubeOrientation(edges[11], cnt%2);
	}
	
#else
	
	int lastPiece = 12-(int)edges.size();
	int puzzle[12];
	int dual[16];
	uint64_t hashVal = hash;
	hash /= FactorialUpperK(12, lastPiece); // for rotations
	hashVal = hashVal%FactorialUpperK(12, lastPiece); // for pieces
	
	int numEntriesLeft = lastPiece+1;
	for (int x = edges.size()-1; x >= 0; x--)
	{
		puzzle[x] = hashVal%numEntriesLeft;
		hashVal /= numEntriesLeft;
		numEntriesLeft++;
		for (int y = x+1; y < edges.size(); y++)
		{
			if (puzzle[y] >= puzzle[x])
				puzzle[y]++;
		}
	}
	for (int x = 0; x < 12; x++)
	{
		s.SetCubeInLoc(x, 0xF);
		s.SetCubeOrientation(x, 0);
	}
	
	for (int x = 0; x < edges.size(); x++)
	{
		s.SetCubeInLoc(puzzle[x], edges[x]);
		dual[edges[x]] = puzzle[x];
	}
	
	int cnt = 0;
	int limit = std::min((int)edges.size(), 11);
	for (int x = limit-1; x >= 0; x--)
	{
		s.SetCubeOrientation(edges[x], hash%2);
		//s.SetCubeOrientation(dual[corners[x]], hash%3);
		cnt += hash%2;
		hash/=2;
	}
	if (edges.size() == 12)
		s.SetCubeOrientation(edges[11], cnt%2);
#endif
}

bool RubikEdgePDB::Load(const char *prefix)
{
	FILE *f = fopen(GetFileName(prefix).c_str(), "rb");
	if (f == 0)
	{
		perror("Opening RubiksEdgePDB file");
		return false;
	}
	Save(f);
	fclose(f);
	return true;
}

void RubikEdgePDB::Save(const char *prefix)
{
	FILE *f = fopen(GetFileName(prefix).c_str(), "w+");
	if (f == 0)
	{
		perror("Opening RubiksEdgePDB file");
		return;
	}
	Save(f);
	fclose(f);
}

bool RubikEdgePDB::Load(FILE *f)
{
	if (PDBHeuristic<RubikEdgeState, RubikEdgeAction, RubikEdge, RubikEdgeState, 4>::Load(f) != true)
	{
		return false;
	}
	if (fread(&puzzleSize, sizeof(puzzleSize), 1, f) != 1)
		return false;
	if (fread(&pdbSize, sizeof(pdbSize), 1, f) != 1)
		return false;
	size_t edgeSize = edges.size();
	if (fread(&edgeSize, sizeof(edgeSize), 1, f) != 1)
		return false;
	edges.resize(edgeSize);
	if (fread(&edges[0], sizeof(edges[0]), edges.size(), f) != edgeSize)
		return false;
	return true;
}

void RubikEdgePDB::Save(FILE *f)
{
	PDBHeuristic<RubikEdgeState, RubikEdgeAction, RubikEdge, RubikEdgeState, 4>::Save(f);
	fwrite(&puzzleSize, sizeof(puzzleSize), 1, f);
	fwrite(&pdbSize, sizeof(pdbSize), 1, f);
	size_t edgeSize = edges.size();
	fwrite(&edgeSize, sizeof(edgeSize), 1, f);
	fwrite(&edges[0], sizeof(edges[0]), edges.size(), f);
}

std::string RubikEdgePDB::GetFileName(const char *prefix)
{
	std::string fileName;
	fileName += "RC-E-";
	// denote the origin state from which the PDB is computed
	for (int x = 0; x < 12; x++)
	{
		fileName += std::to_string(goalState[0].GetCubeInLoc(x));
		fileName += ".";
		fileName += std::to_string(goalState[0].GetCubeOrientation(goalState[0].GetCubeInLoc(x)));
		fileName += ";";
	}
	fileName.pop_back();
	fileName += "-";
	// denote the pattern used for the PDB
	for (size_t x = 0; x < edges.size(); x++)
	{
		fileName += std::to_string(edges[x]);
		fileName += ";";
	}
	fileName.pop_back(); // remove colon
#ifdef MR
	fileName += "-MR";
#endif
	if (std::is_same<RubikEdgeStateArray,RubikEdgeState>::value)
	{
		fileName += "-AR";
	}
	fileName += ".pdb";
	
	return fileName;
}

uint64_t RubikEdgePDB::Factorial(int val)
{
	static uint64_t table[21] =
	{ 1ll, 1ll, 2ll, 6ll, 24ll, 120ll, 720ll, 5040ll, 40320ll, 362880ll, 3628800ll, 39916800ll, 479001600ll,
		6227020800ll, 87178291200ll, 1307674368000ll, 20922789888000ll, 355687428096000ll,
		6402373705728000ll, 121645100408832000ll, 2432902008176640000ll };
	if (val > 20)
		return (uint64_t)-1;
	return table[val];
}

uint64_t RubikEdgePDB::FactorialUpperK(int n, int k)
{
	const uint64_t result[13][13] = {
		{1}, // n = 0
		{1, 1}, // n = 1
		{2, 2, 1}, // n = 2
		{6, 6, 3, 1}, // n = 3
		{24, 24, 12, 4, 1}, // n = 4
		{120, 120, 60, 20, 5, 1}, // n = 5
		{720, 720, 360, 120, 30, 6, 1}, // n = 6
		{5040, 5040, 2520, 840, 210, 42, 7, 1}, // n = 7
		{40320, 40320, 20160, 6720, 1680, 336, 56, 8, 1}, // n = 8
		{362880, 362880, 181440, 60480, 15120, 3024, 504, 72, 9, 1}, // n = 9
		{3628800, 3628800, 1814400, 604800, 151200, 30240, 5040, 720, 90, 10, 1}, // n = 10
		{39916800, 39916800, 19958400, 6652800, 1663200, 332640, 55440, 7920, 990, 110, 11, 1}, // n = 11
		{479001600, 479001600, 239500800, 79833600, 19958400, 3991680, 665280, 95040, 11880, 1320, 132, 12, 1} // n = 12
	};
	return result[n][k];
//	uint64_t value = 1;
//	assert(n >= 0 && k >= 0);
//	
//	for (int i = n; i > k; i--)
//	{
//		value *= i;
//	}
//	
//	return value;
}


#pragma mark orientation pdb

RubikEdgeOrientationPDB::RubikEdgeOrientationPDB(RubikEdge *e, const RubikEdgeState &s)
:PDBHeuristic(e)
{
	SetGoal(s);
}

uint64_t RubikEdgeOrientationPDB::GetStateSpaceSize()
{
#pragma message("This code belongs in the RubikEdge, not in the PDB.")
	return 2048ll;
}

uint64_t RubikEdgeOrientationPDB::GetStateHash(const RubikEdgeState &s)
{
	uint64_t hashVal = 0;
	for (int x = 0; x < 11; x++)
	{
		hashVal = (hashVal<<1)+s.GetCubeOrientation(11-x);
	}
	return hashVal;
}

void RubikEdgeOrientationPDB::GetStateFromHash(RubikEdgeState &s, uint64_t hash)
{
	int cnt = 0;
	for (int x = 10; x >= 0; x--)
	{
		s.SetCubeOrientation(11-x, hash&0x1);
		cnt += hash & 0x1;
		hash >>= 1;
	}
	if (1 == cnt%2)
	{
		s.SetCubeOrientation(0, true);
	}
	else {
		s.SetCubeOrientation(0, false);
	}
}

//

uint64_t RubikEdgeOrientationPDB::GetPDBSize() const
{
	return 1<<11;
}

uint64_t RubikEdgeOrientationPDB::GetPDBHash(const RubikEdgeState &s, int threadID) const
{
	return GetStateHash(s);
}

void RubikEdgeOrientationPDB::GetStateFromPDBHash(uint64_t hash, RubikEdgeState &s, int threadID) const
{
	return GetStateFromHash(s, hash);
}

bool RubikEdgeOrientationPDB::Load(const char *prefix)
{
	FILE *f = fopen(GetFileName(prefix).c_str(), "rb");
	if (f == 0)
	{
		perror("Opening RubiksEdgePDB file");
		return false;
	}
	Save(f);
	fclose(f);
	return true;
}

void RubikEdgeOrientationPDB::Save(const char *prefix)
{
	FILE *f = fopen(GetFileName(prefix).c_str(), "w+");
	if (f == 0)
	{
		perror("Opening RubiksEdgePDB file");
		return;
	}
	Save(f);
	fclose(f);
}

bool RubikEdgeOrientationPDB::Load(FILE *f)
{
	if (PDBHeuristic<RubikEdgeState, RubikEdgeAction, RubikEdge, RubikEdgeState, 4>::Load(f) != true)
	{
		return false;
	}
	return true;
}

void RubikEdgeOrientationPDB::Save(FILE *f)
{
	PDBHeuristic<RubikEdgeState, RubikEdgeAction, RubikEdge, RubikEdgeState, 4>::Save(f);
}

std::string RubikEdgeOrientationPDB::GetFileName(const char *prefix)
{
	std::string fileName;
	fileName += "RC-E12-OR.pdb";
	return fileName;
}
