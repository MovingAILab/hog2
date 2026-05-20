//
//  RubiksCorner.cpp
//  hog2 glut
//
//  Created by Nathan Sturtevant on 1/11/13.
//  Copyright (c) 2013 University of Denver. All rights reserved.
//

#include "RubiksCubeCorners.h"
#include "Constants.h"
#include <assert.h>

#define LINEAR_RANK 1
#define POLYNOMIAL_RANK 0

/**
 *
 * Implementation details:
 *
 * This just implements the 8 Corners.
 * The RubiksCornerState has 24*4 = 48 bits for where each face is located
 *
 * Faces are numbered 0...5 as follows.
 *
 *  +-------+
 *  |\       \
 *  | \   0   \
 *  |  \_______\
 *  | 1|       |
 *  \  |   2   |
 *   \ |       |
 *    \|_______|
 *
 *
 *  +-------+
 *  |       |\
 *  |   4   | \
 *  |       |  \
 *  |_______|3 |
 *  \       \  |
 *   \   5   \ |
 *    \_______\|
 *
 *
 * Corners are numbered as follows:
 *
 *
 *  4-------3
 *  |\       \
 *  | \       \
 *  |  \1_____2\
 *  8  |       |
 *  \  |       |
 *   \ |       |
 *    \5_______6
 *
 *
 *  4-------3
 *  |       |\
 *  |       | \
 *  |       |  2
 *  8_______7  |
 *  \       \  |
 *   \       \ |
 *    \5______6|
 *
 *
 * Cubies (drawing purposes only) are labeled as follows
 *
 *  +-------+
 *  |\6  7  8\
 *  | \3  4  5\
 *  |  \_______\
 *  |  |0  1  2|
 *  \  |       |
 *   \ |       |
 *    \|_______|
 *
 * Note that 9, 11, 13, 15
 *
 *  +-------+
 *  |\1516 17\
 *  | \1213 14\
 *  |  \_______\
 *  \  |9 10 11|
 *   \ |       |
 *    \|_______|
 *
 *  +-------+
 *  |\2425 26\
 *  | \2122 23\
 *  |  \_______\
 *   \ |18 1920|
 *    \|_______|
 *
 *
 * The moves for each face are labeled with comments. There are three legal
 * moves which rotate +90, -90 and +/-180.
 *
 * The rotation starts with the top/bottm face and goes counter-clockwise
 *
 */


inline int get(uint64_t state, int whichLoc)
{
	//return (state>>(4*whichLoc))&0xF;
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

	// more efficient here
	uint64_t xord = (val1 ^ val2)&0xF;
	xord = (xord << loc1) | (xord << loc2);
	state = state^xord;
	
//	const uint64_t blank = 0xF;
//	uint64_t mask = (blank<<(loc1))|(blank<<(loc2));
//	state = state&(~mask);
//	state = state|(val1<<(loc2))|(val2<<(loc1));
}

RubiksCornerStateBits::RubiksCornerStateBits()
{
	Reset();
}
void RubiksCornerStateBits::Reset()
{
	state = 0;
	for (int x = 0; x < 8; x++)
	{
		SetCubeInLoc(x, x);
	}
}

int RubiksCornerStateBits::GetCubeInLoc(unsigned int whichLoc) const
{
	assert(whichLoc < 8);
	return (state>>(16+4*whichLoc))&0xF;
}
void RubiksCornerStateBits::SetCubeInLoc(unsigned int whichLoc, int cube)
{
	assert(whichLoc < 8);
	uint64_t blank = 0xF;
	uint64_t value = cube&0xF;
	state = state&(~(blank<<(16+4*whichLoc)));
	state |= (value<<(16+4*whichLoc));
}
uint64_t RubiksCornerStateBits::GetCubeOrientation(unsigned int whichLoc) const
{
//	assert(whichLoc < 8);
	if (whichLoc > 8)
	{ return 0x3;}
	return (state>>(2*whichLoc))&0x3;
}
void RubiksCornerStateBits::SetCubeOrientation(unsigned int whichLoc, int orient) // orientation is the offset of the 0(low) side
{
	if (whichLoc > 8)
		return;
	//assert(whichLoc < 8);
	uint64_t blank = 0x3;
	uint64_t value = orient&0x3;
	state = state&(~(blank<<(2*whichLoc)));
	state |= (value<<(2*whichLoc));
}
int RubiksCornerStateBits::GetFaceInLoc(unsigned int whichLoc) const // loc 0...24
{
	assert(whichLoc < 8);
	int cube = GetCubeInLoc(whichLoc/3);
	int rot = (int)GetCubeOrientation(cube);
	return cube*3+(3+(whichLoc%3)-rot)%3;
}


void RubiksCornerStateBits::Rotate(uint64_t a, uint64_t b, uint64_t c, uint64_t d)
{
	a = (a<<2) + 16; // multiply by 4 for shifting purposes; add 16 for rotations
	b = (b<<2) + 16;
	c = (c<<2) + 16;
	d = (d<<2) + 16;
	
	const uint64_t blank = 0xF;
	
	uint64_t mask;
	uint64_t bits;
	bits =
	(((state>>a)&blank)<<b)|(((state>>b)&blank)<<c)|
	(((state>>c)&blank)<<d)|(((state>>d)&blank)<<a);
	mask = (blank<<(a))|(blank<<(b))|(blank<<(c))|(blank<<(d));
	state = (state&(~mask))|bits;
}

void RubiksCornerStateBits::Swap(uint64_t a, uint64_t b, uint64_t c, uint64_t d)
{
	a = (a<<2) + 16; // multiply by 4 for shifting purposes; add 16 for rotations
	b = (b<<2) + 16;
	c = (c<<2) + 16;
	d = (d<<2) + 16;
	
	const uint64_t blank = 0xF;
	
	uint64_t mask;
	uint64_t bits;
	bits =
	(((state>>a)&blank)<<b)|(((state>>b)&blank)<<a)|
	(((state>>c)&blank)<<d)|(((state>>d)&blank)<<c);
	mask = (blank<<(a))|(blank<<(b))|(blank<<(c))|(blank<<(d));
	state = (state&(~mask))|bits;
}

RubiksCornerStateArray::RubiksCornerStateArray()
{
	Reset();
}
void RubiksCornerStateArray::Reset()
{
	for (int x = 8; x < 16; x++)
		state[x] = 0;
	for (int x = 0; x < 8; x++)
	{
		SetCubeInLoc(x, x);
	}
}
int RubiksCornerStateArray::GetCubeInLoc(unsigned int whichLoc) const
{
	assert(whichLoc < 8);
	return state[whichLoc];
}
void RubiksCornerStateArray::SetCubeInLoc(unsigned int whichLoc, int cube)
{
	assert(whichLoc < 8);
	state[whichLoc] = cube;
}
uint64_t RubiksCornerStateArray::GetCubeOrientation(unsigned int whichLoc) const
{
	if (whichLoc > 8)
	{ return 0x3;}
	return state[whichLoc+8];
}
void RubiksCornerStateArray::SetCubeOrientation(unsigned int whichLoc, int orient) // orientation is the offset of the 0(low) side
{
	if (whichLoc > 8)
		return;
	state[whichLoc+8] = orient;
}
int RubiksCornerStateArray::GetFaceInLoc(unsigned int whichLoc) const // loc 0...24
{
	int cube = GetCubeInLoc(whichLoc/3);
	int rot = (int)GetCubeOrientation(cube);
	return cube*3+(3+(whichLoc%3)-rot)%3;
}
void RubiksCornerStateArray::Rotate(uint64_t a, uint64_t b, uint64_t c, uint64_t d)
{
	uint8_t tmp = state[d];
	state[d] = state[c];
	state[c] = state[b];
	state[b] = state[a];
	state[a] = tmp;
}

void RubiksCornerStateArray::Swap(uint64_t a, uint64_t b, uint64_t c, uint64_t d)
{
	uint8_t tmp = state[a];
	state[a] = state[b];
	state[b] = tmp;
	
	tmp = state[c];
	state[c] = state[d];
	state[d] = tmp;
}




void RubiksCorner::GetSuccessors(const RubiksCornerState &nodeID, std::vector<RubiksCornerState> &neighbors) const
{
	RubiksCornerState s;
	for (int x = 0; x < 18; x++)
	{
		GetNextState(nodeID, x, s);
		neighbors.push_back(s);
	}
}

void RubiksCorner::GetActions(const RubiksCornerState &nodeID, std::vector<RubiksCornersAction> &actions) const
{
	actions.resize(0);
	for (int x = 0; x < 18; x++)
	{
		actions.push_back(x);
	}
}

RubiksCornersAction RubiksCorner::GetAction(const RubiksCornerState &s1, const RubiksCornerState &s2) const
{
	assert(false);
	RubiksCornersAction a;
	return a;
}

void RubiksCorner::ApplyAction(RubiksCornerState &s, RubiksCornersAction a) const
{
	switch (a)
	{
		case 0: // face 0
		{
			s.Rotate(0, 1, 2, 3);
		}
			break;
		case 1:
		{
			s.Rotate(3, 2, 1, 0);
		}
			break;
		case 2:
		{
			s.Swap(0, 2, 1, 3);
		}
			break;
		case 3: // face 5
		{
			s.Rotate(4, 5, 6, 7);
		}
			break;
		case 4:
		{
			s.Rotate(7, 6, 5, 4);
		}
			break;
		case 5:
		{
			s.Swap(4, 6, 5, 7);
		}
			break;
			
		case 6: // face 2
		{
			s.Rotate(0, 1, 5, 4);
			s.SetCubeOrientation(s.GetCubeInLoc(1), (s.GetCubeOrientation(s.GetCubeInLoc(1))+2)%3);
			s.SetCubeOrientation(s.GetCubeInLoc(5), (s.GetCubeOrientation(s.GetCubeInLoc(5))+1)%3);
			s.SetCubeOrientation(s.GetCubeInLoc(4), (s.GetCubeOrientation(s.GetCubeInLoc(4))+2)%3);
			s.SetCubeOrientation(s.GetCubeInLoc(0), (s.GetCubeOrientation(s.GetCubeInLoc(0))+1)%3);
		}
			break;
		case 7:
		{
			s.Rotate(4, 5, 1, 0);
			s.SetCubeOrientation(s.GetCubeInLoc(1), (s.GetCubeOrientation(s.GetCubeInLoc(1))+2)%3);
			s.SetCubeOrientation(s.GetCubeInLoc(5), (s.GetCubeOrientation(s.GetCubeInLoc(5))+1)%3);
			s.SetCubeOrientation(s.GetCubeInLoc(4), (s.GetCubeOrientation(s.GetCubeInLoc(4))+2)%3);
			s.SetCubeOrientation(s.GetCubeInLoc(0), (s.GetCubeOrientation(s.GetCubeInLoc(0))+1)%3);
		}
			break;
		case 8:
		{
			s.Swap(0, 5, 1, 4);
		}
			break;
			
		case 9: // face 4
		{
			s.Rotate(2, 3, 7, 6);
			//std::cout << "^" << s << "\n";
			s.SetCubeOrientation(s.GetCubeInLoc(3), (s.GetCubeOrientation(s.GetCubeInLoc(3))+2)%3);
			//std::cout << "^" << s << "\n";
			s.SetCubeOrientation(s.GetCubeInLoc(2), (s.GetCubeOrientation(s.GetCubeInLoc(2))+1)%3);
			//std::cout << "^" << s << "\n";
			s.SetCubeOrientation(s.GetCubeInLoc(6), (s.GetCubeOrientation(s.GetCubeInLoc(6))+2)%3);
			//std::cout << "^" << s << "\n";
			s.SetCubeOrientation(s.GetCubeInLoc(7), (s.GetCubeOrientation(s.GetCubeInLoc(7))+1)%3);
			//std::cout << "^" << s << "\n";
		}
			break;
		case 10:
		{
			s.Rotate(6, 7, 3, 2);
			//std::cout << "*" << s << "\n";
			s.SetCubeOrientation(s.GetCubeInLoc(3), (s.GetCubeOrientation(s.GetCubeInLoc(3))+2)%3);
			//std::cout << "*" << s << "\n";
			s.SetCubeOrientation(s.GetCubeInLoc(2), (s.GetCubeOrientation(s.GetCubeInLoc(2))+1)%3);
			//std::cout << "*" << s << "\n";
			s.SetCubeOrientation(s.GetCubeInLoc(6), (s.GetCubeOrientation(s.GetCubeInLoc(6))+2)%3);
			//std::cout << "*" << s << "\n";
			s.SetCubeOrientation(s.GetCubeInLoc(7), (s.GetCubeOrientation(s.GetCubeInLoc(7))+1)%3);
			//std::cout << "*" << s << "\n";
		}
			break;
		case 11:
		{
			s.Swap(6, 3, 2, 7);
		}
			break;
			
		case 12: // face 1
		{
			s.Rotate(0, 4, 7, 3);
			s.SetCubeOrientation(s.GetCubeInLoc(0), (s.GetCubeOrientation(s.GetCubeInLoc(0))+2)%3);
			s.SetCubeOrientation(s.GetCubeInLoc(4), (s.GetCubeOrientation(s.GetCubeInLoc(4))+1)%3);
			s.SetCubeOrientation(s.GetCubeInLoc(7), (s.GetCubeOrientation(s.GetCubeInLoc(7))+2)%3);
			s.SetCubeOrientation(s.GetCubeInLoc(3), (s.GetCubeOrientation(s.GetCubeInLoc(3))+1)%3);
		}
			break;
		case 13:
		{
			s.Rotate(3, 7, 4, 0);
			s.SetCubeOrientation(s.GetCubeInLoc(0), (s.GetCubeOrientation(s.GetCubeInLoc(0))+2)%3);
			s.SetCubeOrientation(s.GetCubeInLoc(4), (s.GetCubeOrientation(s.GetCubeInLoc(4))+1)%3);
			s.SetCubeOrientation(s.GetCubeInLoc(7), (s.GetCubeOrientation(s.GetCubeInLoc(7))+2)%3);
			s.SetCubeOrientation(s.GetCubeInLoc(3), (s.GetCubeOrientation(s.GetCubeInLoc(3))+1)%3);
		}
			break;
		case 14:
		{
			s.Swap(0, 7, 4, 3);
		}
			break;
		case 15: // face 3
		{
			s.Rotate(1, 2, 6, 5);
			s.SetCubeOrientation(s.GetCubeInLoc(2), (s.GetCubeOrientation(s.GetCubeInLoc(2))+2)%3);
			s.SetCubeOrientation(s.GetCubeInLoc(6), (s.GetCubeOrientation(s.GetCubeInLoc(6))+1)%3);
			s.SetCubeOrientation(s.GetCubeInLoc(5), (s.GetCubeOrientation(s.GetCubeInLoc(5))+2)%3);
			s.SetCubeOrientation(s.GetCubeInLoc(1), (s.GetCubeOrientation(s.GetCubeInLoc(1))+1)%3);
		}
			break;
		case 16:
		{
			s.Rotate(5, 6, 2, 1);
			s.SetCubeOrientation(s.GetCubeInLoc(2), (s.GetCubeOrientation(s.GetCubeInLoc(2))+2)%3);
			s.SetCubeOrientation(s.GetCubeInLoc(6), (s.GetCubeOrientation(s.GetCubeInLoc(6))+1)%3);
			s.SetCubeOrientation(s.GetCubeInLoc(5), (s.GetCubeOrientation(s.GetCubeInLoc(5))+2)%3);
			s.SetCubeOrientation(s.GetCubeInLoc(1), (s.GetCubeOrientation(s.GetCubeInLoc(1))+1)%3);
		}
			break;
		case 17:
		{
			s.Swap(1, 6, 2, 5);
		}
			break;
		default:
			break;
	}
	
}

void RubiksCorner::GetNextState(const RubiksCornerState &s0, RubiksCornersAction a, RubiksCornerState &s1) const
{
	s1 = s0;
	ApplyAction(s1, a);
}


bool RubiksCorner::InvertAction(RubiksCornersAction &a) const
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

/** Goal Test if the goal is stored **/
bool RubiksCorner::GoalTest(const RubiksCornerStateBits &node) const
{
	return node.state == 0;
}

bool RubiksCorner::GoalTest(const RubiksCornerStateArray &node) const
{
	for (int x = 0; x < 16; x++)
		if (node.state[x] != 0)
			return false;
	return true;
}

void RubiksCorner::ApplyMove(RubiksCornerState &s, RubikCornerMove *a)
{
	ApplyAction(s, a->act);
}
void RubiksCorner::UndoMove(RubiksCornerState &s, RubikCornerMove *a)
{
	RubiksCornersAction todo = a->act;
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

int64_t RubiksCorner::getMaxSinglePlayerRank()
{
	//	static uint64_t Factorial[21] =
	//	{ 1ll, 1ll, 2ll, 6ll, 24ll, 120ll, 720ll, 5040ll, 40320ll, 362880ll, 3628800ll, 39916800ll, 479001600ll,
	//		6227020800ll, 87178291200ll, 1307674368000ll, 20922789888000ll, 355687428096000ll,
	//		6402373705728000ll, 121645100408832000ll, 2432902008176640000ll };
	//
	//	return Factorial[8]*(2187); // 3^7
	return 88179840;
}
int64_t RubiksCorner::getMaxSinglePlayerRank2()
{
	return 9;
}
int64_t RubiksCorner::getMaxSinglePlayerRank2(int64_t firstIndex)
{
	return 9797760;
}

void RubiksCorner::rankPlayerFirstTwo(const RubiksCornerState &s, int who, int64_t &rank)
{
	rank = s.GetCubeOrientation(0)*3+s.GetCubeOrientation(1);
}

void RubiksCorner::rankPlayerRemaining(const RubiksCornerState &node, int who, int64_t &rank)
{
	static uint64_t Factorial[21] =
	{ 1ll, 1ll, 2ll, 6ll, 24ll, 120ll, 720ll, 5040ll, 40320ll, 362880ll, 3628800ll, 39916800ll, 479001600ll,
		6227020800ll, 87178291200ll, 1307674368000ll, 20922789888000ll, 355687428096000ll,
		6402373705728000ll, 121645100408832000ll, 2432902008176640000ll };
#if POLYNOMIAL_RANK == 1
	
	int puzzle[12];
	for (int x = 0; x < 8; x++)
		puzzle[x] = node.GetCubeInLoc(x);
	
	uint64_t hashVal = 0;
	uint64_t part2 = 0;
	int numEntriesLeft = 8;
	for (unsigned int x = 0; x < 8; x++)
	{
		hashVal += puzzle[x]*Factorial[numEntriesLeft-1];
		numEntriesLeft--;
		for (unsigned y = x; y < 8; y++)
		{
			if (puzzle[y] > puzzle[x])
				puzzle[y]--;
		}
	}
	for (int x = 2; x < 7; x++)
	{
		part2 = part2*3+node.GetCubeOrientation(x);
	}
	rank = part2*Factorial[8]+hashVal;
#endif
#if LINEAR_RANK == 1
	uint64_t perm = 0, dual = 0;
	for (int x = 0; x < 8; x++)
	{
		set(perm, x, node.GetCubeInLoc(x));
		set(dual, node.GetCubeInLoc(x), x);
	}
	
	uint64_t hashVal = 0;
	for (int x = 2; x < 7; x++)
	{
		hashVal = hashVal*3+node.GetCubeOrientation(x);
	}
	hashVal = hashVal*Factorial[8]+MRRank(8, perm, dual);
	rank = hashVal;
#endif
}


uint64_t RubiksCorner::GetStateHash(const RubiksCornerState &node) const
{
	static uint64_t Factorial[21] =
	{ 1ll, 1ll, 2ll, 6ll, 24ll, 120ll, 720ll, 5040ll, 40320ll, 362880ll, 3628800ll, 39916800ll, 479001600ll,
		6227020800ll, 87178291200ll, 1307674368000ll, 20922789888000ll, 355687428096000ll,
		6402373705728000ll, 121645100408832000ll, 2432902008176640000ll };
	
#if POLYNOMIAL_RANK == 1
	int puzzle[12];
	for (int x = 0; x < 8; x++)
		puzzle[x] = node.GetCubeInLoc(x);
	
	uint64_t hashVal = 0;
	uint64_t part2 = 0;
	int numEntriesLeft = 8;
	for (unsigned int x = 0; x < 8; x++)
	{
		hashVal += puzzle[x]*Factorial[numEntriesLeft-1];
		numEntriesLeft--;
		for (unsigned y = x; y < 8; y++)
		{
			if (puzzle[y] > puzzle[x])
				puzzle[y]--;
		}
	}
	for (int x = 0; x < 7; x++)
	{
		part2 = part2*3+node.GetCubeOrientation(x);
	}
	return part2*Factorial[8]+hashVal;
	//	return hashVal;
#endif
#if LINEAR_RANK == 1
	uint64_t perm = 0, dual = 0;
	for (int x = 0; x < 8; x++)
	{
		int val = node.GetCubeInLoc(x);
		set(perm, x, val);
		set(dual, val, x);
	}
	
	uint64_t hashVal = 0;
	for (int x = 0; x < 7; x++)
	{
		hashVal = hashVal*3+node.GetCubeOrientation(x);
	}
	//	return (MRRank(12, perm, dual)<<11)|hashVal;
	hashVal = hashVal*Factorial[8]+MRRank(8, perm, dual);
	return hashVal;
#endif
}

/////

uint64_t RubiksCorner::MRRank(int n, uint64_t perm, uint64_t dual)
{
	int ss[12];
	int ssLoc = 0;
	
	int s;
	for (int i = n; i > 1; i--)
	{
		s = get(perm, i-1);
		ss[ssLoc] = s;
		ssLoc++;
		
		swap(perm, i-1, get(dual, i-1));
		swap(dual, s, i-1);
	}
	uint64_t result = 0;
	int cnt = 2;
	for (int i = (int)ssLoc-1; i >= 0; i--)
	{
		result *= cnt;
		result += ss[i];
		cnt++;
	}
	return result;
}


/////


void RubiksCorner::GetStateFromHash(uint64_t hash, RubiksCornerState &node) const
{
	static uint64_t Factorial[21] =
	{ 1ll, 1ll, 2ll, 6ll, 24ll, 120ll, 720ll, 5040ll, 40320ll, 362880ll, 3628800ll, 39916800ll, 479001600ll,
		6227020800ll, 87178291200ll, 1307674368000ll, 20922789888000ll, 355687428096000ll,
		6402373705728000ll, 121645100408832000ll, 2432902008176640000ll };
#if POLYNOMIAL_RANK == 1
	
	int puzzle[12];
	uint64_t hashVal = hash;
	hash /= Factorial[8]; // for rotations
	hashVal = hashVal%Factorial[8]; // for pieces
	
	int cnt = 0;
	for (int x = 6; x >= 0; x--)
	{
		node.SetCubeOrientation(x, hash%3);
		cnt += hash%3;
		hash/=3;
	}
	node.SetCubeOrientation(7, (3-(cnt%3))%3);
	
	int numEntriesLeft = 1;
	for (int x = 8-1; x >= 0; x--)
	{
		puzzle[x] = hashVal%numEntriesLeft;
		hashVal /= numEntriesLeft;
		numEntriesLeft++;
		for (int y = x+1; y < 8; y++)
		{
			if (puzzle[y] >= puzzle[x])
				puzzle[y]++;
		}
	}
	for (int x = 0; x < 8; x++)
		node.SetCubeInLoc(x, puzzle[x]);
#endif
#if LINEAR_RANK == 1
	uint64_t bits = hash/Factorial[8];
	uint64_t hVal = hash%Factorial[8];
	
	int cnt = 0;
	for (int x = 6; x >= 0; x--)
	{
		node.SetCubeOrientation(x, bits%3);
		cnt += bits%3;
		bits/=3;
	}
	node.SetCubeOrientation(7, (3-(cnt%3))%3);
	
	uint64_t val = 0;
	for (int x = 0; x < 8; x++)
		set(val, x, x);
	MRUnrank2(8, hVal, val);
	for (int x = 0; x < 8; x++)
	{
		node.SetCubeInLoc(x, get(val, x));
	}
#endif
}

void RubiksCorner::MRUnrank2(int n, uint64_t r, uint64_t &perm)
{
	for (int i = n; i > 0; i--)
	{
		swap(perm, i-1, r%i);
		r = r/i;
	}
}

Graphics::point transform(const Graphics::point &p)
{
	const float cosr = cos(-1.11f);
	const float sinr = sin(-1.11f);
	return Graphics::point(p.x+0.5*p.z*cosr, p.y+0.5*p.z*sinr);
}

bool operator<(const RubiksCorner::triangleColor &s1, const RubiksCorner::triangleColor &s2)
{
	return s1.t.p1.z+s1.t.p2.z+s1.t.p3.z > s2.t.p1.z+s2.t.p2.z+s2.t.p3.z;
}
//bool operator<(const RubiksCorner::triangleColor &s1, const RubiksCorner::triangleColor &s2)
//{
//	return std::min({s1.t.p1.z, s1.t.p2.z, s1.t.p3.z}) > std::min({s2.t.p1.z, s2.t.p2.z, s2.t.p3.z});
//}

void RubiksCorner::Draw(Graphics::Display &display, const RubiksCornerState &s) const
{
	rgbColor colors[7] = {{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}, {1.0, 1.0, 0.0}, {1.0, 0.75, 0.0},{1.0, 1.0, 1.0},};
	triangles.resize(0);
	// Get triangles
	
	GetTriangles(s, 0);
	GetTriangles(s, 2);
	GetTriangles(s, 6);
	GetTriangles(s, 8);
	GetTriangles(s, 18);
	GetTriangles(s, 20);
	GetTriangles(s, 24);
	GetTriangles(s, 26);
	
	// Do transform
	// Sort by z
	std::sort(triangles.begin(),triangles.end());

	// Transform and draw into display
	for (auto &t : triangles)
	{
		display.FillTriangle(transform(t.t.p1),
							 transform(t.t.p2),
							 transform(t.t.p3),
							 colors[1+t.color]);
	}
}

void RubiksCorner::GetTriangles(const RubiksCornerState &s, int cube) const
{
	const float scale = 0.3;
	const float offset = 0.95*2.0*scale/3.0;
	const float offset2 = 2.0*scale/3.0;
	const float epsilon = 0.002;
	switch(cube)
	{
		case 0:
		{
			// Face 0 - cube 0
			int c = GetFaceColor(0, s);
			triangles.push_back({{{-scale, -scale, -scale+offset},
				{-scale, -scale, -scale},
				{-scale+offset, -scale, -scale}}, c});
			triangles.push_back({{{-scale+offset, -scale, -scale}, {-scale+offset, -scale, -scale+offset}, {-scale, -scale, -scale+offset}}, c});
//			glVertex3f(-scale, -scale, -scale+offset);
//			glVertex3f(-scale, -scale, -scale);
//			glVertex3f(-scale+offset, -scale, -scale);
//			glVertex3f(-scale+offset, -scale, -scale+offset);

//			SetFaceColor(-1, s);
//			glVertex3f(-scale, -scale+epsilon, -scale+offset2);
//			glVertex3f(-scale, -scale+epsilon, -scale);
//			glVertex3f(-scale+offset2, -scale+epsilon, -scale);
//			glVertex3f(-scale+offset2, -scale+epsilon, -scale+offset2);

			// Face 1 - cube 0
			c = GetFaceColor(1, s);
			triangles.push_back({{{-scale, -scale+offset, -scale}, {-scale, -scale+offset, -scale+offset}, {-scale, -scale, -scale+offset}}, c});
			triangles.push_back({{{-scale, -scale, -scale+offset}, {-scale, -scale, -scale}, {-scale, -scale+offset, -scale}}, c});
//			glVertex3f(-scale, -scale+offset, -scale);
//			glVertex3f(-scale, -scale+offset, -scale+offset);
//			glVertex3f(-scale, -scale, -scale+offset);
//			glVertex3f(-scale, -scale, -scale);

//			SetFaceColor(-1, s);
//			glVertex3f(-scale+epsilon, -scale+offset2, -scale);
//			glVertex3f(-scale+epsilon, -scale+offset2, -scale+offset2);
//			glVertex3f(-scale+epsilon, -scale, -scale+offset2);
//			glVertex3f(-scale+epsilon, -scale, -scale);

			// Face 2 - cube 0
			c = GetFaceColor(2, s);
			triangles.push_back({{{-scale, -scale+offset, -scale}, {-scale+offset, -scale+offset, -scale}, {-scale+offset, -scale, -scale}}, c});
			triangles.push_back({{{-scale+offset, -scale, -scale}, {-scale, -scale, -scale}, {-scale, -scale+offset, -scale}}, c});
//			glVertex3f(-scale, -scale+offset, -scale);
//			glVertex3f(-scale+offset, -scale+offset, -scale);
//			glVertex3f(-scale+offset, -scale, -scale);
//			glVertex3f(-scale, -scale, -scale);

//			SetFaceColor(-1, s);
//			glVertex3f(-scale, -scale+offset2, -scale+epsilon);
//			glVertex3f(-scale+offset2, -scale+offset2, -scale+epsilon);
//			glVertex3f(-scale+offset2, -scale, -scale+epsilon);
//			glVertex3f(-scale, -scale, -scale+epsilon);

		} break;
		case 2:
		{
			// Face 0 - cube 2
			int c = GetFaceColor(3, s);
			triangles.push_back({{{scale-offset, -scale, -scale+offset}, {scale-offset, -scale, -scale}, {scale, -scale, -scale}}, c});
			triangles.push_back({{{scale, -scale, -scale}, {scale, -scale, -scale+offset}, {scale-offset, -scale, -scale+offset}}, c});
//			glVertex3f(scale-offset, -scale, -scale+offset);
//			glVertex3f(scale-offset, -scale, -scale);
//			glVertex3f(scale, -scale, -scale);
//			glVertex3f(scale, -scale, -scale+offset);
//
//			SetFaceColor(-1, s);
//			glVertex3f(scale-offset2, -scale+epsilon, -scale+offset2);
//			glVertex3f(scale-offset2, -scale+epsilon, -scale);
//			glVertex3f(scale, -scale+epsilon, -scale);
//			glVertex3f(scale, -scale+epsilon, -scale+offset2);

			// Face 2 - cube 2
			c = GetFaceColor(4, s);
			triangles.push_back({{{scale, -scale+offset, -scale}, {scale-offset, -scale+offset, -scale}, {scale-offset, -scale, -scale}}, c});
			triangles.push_back({{{scale-offset, -scale, -scale}, {scale, -scale, -scale}, {scale, -scale+offset, -scale}}, c});
//			glVertex3f(scale, -scale+offset, -scale);
//			glVertex3f(scale-offset, -scale+offset, -scale);
//			glVertex3f(scale-offset, -scale, -scale);
//			glVertex3f(scale, -scale, -scale);
//
//			SetFaceColor(-1, s);
//			glVertex3f(scale, -scale+offset2, -scale+epsilon);
//			glVertex3f(scale-offset2, -scale+offset2, -scale+epsilon);
//			glVertex3f(scale-offset2, -scale, -scale+epsilon);
//			glVertex3f(scale, -scale, -scale+epsilon);

			// Face 3 - cube 2
			c = GetFaceColor(5, s);
			triangles.push_back({{{scale, -scale+offset, -scale}, {scale, -scale+offset, -scale+offset}, {scale, -scale, -scale+offset}}, c});
			triangles.push_back({{{scale, -scale, -scale+offset}, {scale, -scale, -scale}, {scale, -scale+offset, -scale}}, c});
//			glVertex3f(scale, -scale+offset, -scale);
//			glVertex3f(scale, -scale+offset, -scale+offset);
//			glVertex3f(scale, -scale, -scale+offset);
//			glVertex3f(scale, -scale, -scale);
//
//			SetFaceColor(-1, s);
//			glVertex3f(scale-epsilon, -scale+offset2, -scale);
//			glVertex3f(scale-epsilon, -scale+offset2, -scale+offset2);
//			glVertex3f(scale-epsilon, -scale, -scale+offset2);
//			glVertex3f(scale-epsilon, -scale, -scale);
		} break;
		case 6:
		{
			// Face 0 - cube 6
			int c = GetFaceColor(9, s);
			triangles.push_back({{{-scale, -scale, scale}, {-scale, -scale, scale-offset}, {-scale+offset, -scale, scale-offset}}, c});
			triangles.push_back({{{-scale+offset, -scale, scale-offset}, {-scale+offset, -scale, scale}, {-scale, -scale, scale}}, c});
//			glVertex3f(-scale, -scale, scale);
//			glVertex3f(-scale, -scale, scale-offset);
//			glVertex3f(-scale+offset, -scale, scale-offset);
//			glVertex3f(-scale+offset, -scale, scale);
//
//			SetFaceColor(-1, s);
//			glVertex3f(-scale, -scale+epsilon, scale);
//			glVertex3f(-scale, -scale+epsilon, scale-offset2);
//			glVertex3f(-scale+offset2, -scale+epsilon, scale-offset2);
//			glVertex3f(-scale+offset2, -scale+epsilon, scale);
//
			
			// Face 1 - cube 6
			c = GetFaceColor(11, s);
			triangles.push_back({{{-scale, -scale+offset, scale}, {-scale, -scale+offset, scale-offset}, {-scale, -scale, scale-offset}}, c});
			triangles.push_back({{{-scale, -scale, scale-offset}, {-scale, -scale, scale}, {-scale, -scale+offset, scale}}, c});
//			glVertex3f(-scale, -scale+offset, scale);
//			glVertex3f(-scale, -scale+offset, scale-offset);
//			glVertex3f(-scale, -scale, scale-offset);
//			glVertex3f(-scale, -scale, scale);
//
//			SetFaceColor(-1, s);
//			glVertex3f(-scale+epsilon, -scale+offset2, scale);
//			glVertex3f(-scale+epsilon, -scale+offset2, scale-offset2);
//			glVertex3f(-scale+epsilon, -scale, scale-offset2);
//			glVertex3f(-scale+epsilon, -scale, scale);

			// Face 4 - cube 6
			c = GetFaceColor(10, s);
			triangles.push_back({{{-scale, -scale+offset, scale}, {-scale+offset, -scale+offset, scale}, {-scale+offset, -scale, scale}}, c});
			triangles.push_back({{{-scale+offset, -scale, scale}, {-scale, -scale, scale}, {-scale, -scale+offset, scale}}, c});
//			glVertex3f(-scale, -scale+offset, scale);
//			glVertex3f(-scale+offset, -scale+offset, scale);
//			glVertex3f(-scale+offset, -scale, scale);
//			glVertex3f(-scale, -scale, scale);
//
//			SetFaceColor(-1, s);
//			glVertex3f(-scale, -scale+offset2, scale-epsilon);
//			glVertex3f(-scale+offset2, -scale+offset2, scale-epsilon);
//			glVertex3f(-scale+offset2, -scale, scale-epsilon);
//			glVertex3f(-scale, -scale, scale-epsilon);

		} break;
		case 8:
		{
			// Face 0 - cube 8
			int c = GetFaceColor(6, s);
			triangles.push_back({{{scale-offset, -scale, scale}, {scale-offset, -scale, scale-offset}, {scale, -scale, scale-offset}}, c});
			triangles.push_back({{{scale, -scale, scale-offset}, {scale, -scale, scale}, {scale-offset, -scale, scale}}, c});
//			glVertex3f(scale-offset, -scale, scale);
//			glVertex3f(scale-offset, -scale, scale-offset);
//			glVertex3f(scale, -scale, scale-offset);
//			glVertex3f(scale, -scale, scale);
//
//			SetFaceColor(-1, s);
//			glVertex3f(scale-offset2, -scale+epsilon, scale);
//			glVertex3f(scale-offset2, -scale+epsilon, scale-offset2);
//			glVertex3f(scale, -scale+epsilon, scale-offset2);
//			glVertex3f(scale, -scale+epsilon, scale);

			// Face 3 - cube 8
			c = GetFaceColor(7, s);
			triangles.push_back({{{scale, -scale+offset, scale}, {scale, -scale+offset, scale-offset}, {scale, -scale, scale-offset}}, c});
			triangles.push_back({{{scale, -scale, scale-offset}, {scale, -scale, scale}, {scale, -scale+offset, scale}}, c});
//			glVertex3f(scale, -scale+offset, scale);
//			glVertex3f(scale, -scale+offset, scale-offset);
//			glVertex3f(scale, -scale, scale-offset);
//			glVertex3f(scale, -scale, scale);
//
//			SetFaceColor(-1, s);
//			glVertex3f(scale-epsilon, -scale+offset2, scale);
//			glVertex3f(scale-epsilon, -scale+offset2, scale-offset2);
//			glVertex3f(scale-epsilon, -scale, scale-offset2);
//			glVertex3f(scale-epsilon, -scale, scale);

			
			// Face 4 - cube 8
			c = GetFaceColor(8, s);
			triangles.push_back({{{scale, -scale+offset, scale}, {scale-offset, -scale+offset, scale}, {scale-offset, -scale, scale}}, c});
			triangles.push_back({{{scale-offset, -scale, scale}, {scale, -scale, scale}, {scale, -scale+offset, scale}}, c});
//			glVertex3f(scale, -scale+offset, scale);
//			glVertex3f(scale-offset, -scale+offset, scale);
//			glVertex3f(scale-offset, -scale, scale);
//			glVertex3f(scale, -scale, scale);
//
//			SetFaceColor(-1, s);
//			glVertex3f(scale, -scale+offset2, scale-epsilon);
//			glVertex3f(scale-offset2, -scale+offset2, scale-epsilon);
//			glVertex3f(scale-offset2, -scale, scale-epsilon);
//			glVertex3f(scale, -scale, scale-epsilon);

		} break;
		case 18:
		{
			// Face 1 - cube 18
			int c = GetFaceColor(14, s);
			triangles.push_back({{{-scale+epsilon, scale-offset2, -scale}, {-scale, scale-offset, -scale+offset}, {-scale, scale, -scale+offset}}, c});
			triangles.push_back({{{-scale, scale, -scale+offset}, {-scale, scale, -scale}, {-scale+epsilon, scale-offset2, -scale}}, c});
//			glVertex3f(-scale, scale-offset, -scale);
//			glVertex3f(-scale, scale-offset, -scale+offset);
//			glVertex3f(-scale, scale, -scale+offset);
//			glVertex3f(-scale, scale, -scale);
//
//			SetFaceColor(-1, s);
//			glVertex3f(-scale+epsilon, scale-offset2, -scale);
//			glVertex3f(-scale+epsilon, scale-offset2, -scale+offset2);
//			glVertex3f(-scale+epsilon, scale, -scale+offset2);
//			glVertex3f(-scale+epsilon, scale, -scale);

			
			// Face 2 - cube 18
			c = GetFaceColor(13, s);
			triangles.push_back({{{-scale, scale-offset, -scale}, {-scale+offset, scale-offset, -scale}, {-scale+offset, scale, -scale}}, c});
			triangles.push_back({{{-scale+offset, scale, -scale}, {-scale, scale, -scale}, {-scale, scale-offset, -scale}}, c});
//			glVertex3f(-scale, scale-offset, -scale);
//			glVertex3f(-scale+offset, scale-offset, -scale);
//			glVertex3f(-scale+offset, scale, -scale);
//			glVertex3f(-scale, scale, -scale);
//
//			SetFaceColor(-1, s);
//			glVertex3f(-scale, scale-offset2, -scale+epsilon);
//			glVertex3f(-scale+offset2, scale-offset2, -scale+epsilon);
//			glVertex3f(-scale+offset2, scale, -scale+epsilon);
//			glVertex3f(-scale, scale, -scale+epsilon);

			
			// Face 5 - cube 18
			c = GetFaceColor(12, s);
			triangles.push_back({{{-scale, scale, -scale+offset}, {-scale, scale, -scale}, {-scale+offset, scale, -scale}}, c});
			triangles.push_back({{{-scale+offset, scale, -scale}, {-scale+offset, scale, -scale+offset}, {-scale, scale, -scale+offset}}, c});
//			glVertex3f(-scale, scale, -scale+offset);
//			glVertex3f(-scale, scale, -scale);
//			glVertex3f(-scale+offset, scale, -scale);
//			glVertex3f(-scale+offset, scale, -scale+offset);
//
//			SetFaceColor(-1, s);
//			glVertex3f(-scale, scale-epsilon, -scale+offset2);
//			glVertex3f(-scale, scale-epsilon, -scale);
//			glVertex3f(-scale+offset2, scale-epsilon, -scale);
//			glVertex3f(-scale+offset2, scale-epsilon, -scale+offset2);

		} break;
		case 20:
		{
			// Face 2 - cube 20
			int c = GetFaceColor(17, s);
			triangles.push_back({{{scale, scale-offset, -scale}, {scale-offset, scale-offset, -scale}, {scale-offset, scale, -scale}}, c});
			triangles.push_back({{{scale-offset, scale, -scale}, {scale, scale, -scale}, {scale, scale-offset, -scale}}, c});
//			glVertex3f(scale, scale-offset, -scale);
//			glVertex3f(scale-offset, scale-offset, -scale);
//			glVertex3f(scale-offset, scale, -scale);
//			glVertex3f(scale, scale, -scale);
//
//			SetFaceColor(-1, s);
//			glVertex3f(scale, scale-offset2, -scale+epsilon);
//			glVertex3f(scale-offset2, scale-offset2, -scale+epsilon);
//			glVertex3f(scale-offset2, scale, -scale+epsilon);
//			glVertex3f(scale, scale, -scale+epsilon);
			
			// Face 3 - cube 20
			c = GetFaceColor(16, s);
			triangles.push_back({{{scale, scale-offset, -scale}, {scale, scale-offset, -scale+offset}, {scale, scale, -scale+offset}}, c});
			triangles.push_back({{{scale, scale, -scale+offset}, {scale, scale, -scale}, {scale, scale-offset, -scale}}, c});
//			glVertex3f(scale, scale-offset, -scale);
//			glVertex3f(scale, scale-offset, -scale+offset);
//			glVertex3f(scale, scale, -scale+offset);
//			glVertex3f(scale, scale, -scale);
//
//			SetFaceColor(-1, s);
//			glVertex3f(scale-epsilon, scale-offset2, -scale);
//			glVertex3f(scale-epsilon, scale-offset2, -scale+offset2);
//			glVertex3f(scale-epsilon, scale, -scale+offset2);
//			glVertex3f(scale-epsilon, scale, -scale);

			// Face 5 - cube 20
			c = GetFaceColor(15, s);
			triangles.push_back({{{scale-offset, scale, -scale+offset}, {scale-offset, scale, -scale}, {scale, scale, -scale}}, c});
			triangles.push_back({{{scale, scale, -scale}, {scale, scale, -scale+offset}, {scale-offset, scale, -scale+offset}}, c});
//			glVertex3f(scale-offset, scale, -scale+offset);
//			glVertex3f(scale-offset, scale, -scale);
//			glVertex3f(scale, scale, -scale);
//			glVertex3f(scale, scale, -scale+offset);
//	
//			SetFaceColor(-1, s);
//			glVertex3f(scale-offset2, scale-epsilon, -scale+offset2);
//			glVertex3f(scale-offset2, scale-epsilon, -scale);
//			glVertex3f(scale, scale-epsilon, -scale);
//			glVertex3f(scale, scale-epsilon, -scale+offset2);
		} break;
		case 24:
		{
			// Face 1 - cube 24
			int c = GetFaceColor(22, s);
			triangles.push_back({{{-scale, scale-offset, scale}, {-scale, scale-offset, scale-offset}, {-scale, scale, scale-offset}}, c});
			triangles.push_back({{{-scale, scale, scale-offset}, {-scale, scale, scale}, {-scale, scale-offset, scale}}, c});
//			glVertex3f(-scale, scale-offset, scale);
//			glVertex3f(-scale, scale-offset, scale-offset);
//			glVertex3f(-scale, scale, scale-offset);
//			glVertex3f(-scale, scale, scale);
//
//			SetFaceColor(-1, s);
//			glVertex3f(-scale+epsilon, scale-offset2, scale);
//			glVertex3f(-scale+epsilon, scale-offset2, scale-offset2);
//			glVertex3f(-scale+epsilon, scale, scale-offset2);
//			glVertex3f(-scale+epsilon, scale, scale);

			
			// Face 4 - cube 24
			c = GetFaceColor(23, s);
			triangles.push_back({{{-scale, scale-offset, scale}, {-scale+offset, scale-offset, scale}, {-scale+offset, scale, scale}}, c});
			triangles.push_back({{{-scale+offset, scale, scale}, {-scale, scale, scale}, {-scale, scale-offset, scale}}, c});
//			glVertex3f(-scale, scale-offset, scale);
//			glVertex3f(-scale+offset, scale-offset, scale);
//			glVertex3f(-scale+offset, scale, scale);
//			glVertex3f(-scale, scale, scale);
//
//			SetFaceColor(-1, s);
//			glVertex3f(-scale, scale-offset2, scale-epsilon);
//			glVertex3f(-scale+offset2, scale-offset2, scale-epsilon);
//			glVertex3f(-scale+offset2, scale, scale-epsilon);
//			glVertex3f(-scale, scale, scale-epsilon);

			
			// Face 5 - cube 24
			c = GetFaceColor(21, s);
			triangles.push_back({{{-scale, scale, scale}, {-scale, scale, scale-offset}, {-scale+offset, scale, scale-offset}}, c});
			triangles.push_back({{{-scale+offset, scale, scale-offset}, {-scale+offset, scale, scale}, {-scale, scale, scale}}, c});
//			glVertex3f(-scale, scale, scale);
//			glVertex3f(-scale, scale, scale-offset);
//			glVertex3f(-scale+offset, scale, scale-offset);
//			glVertex3f(-scale+offset, scale, scale);
//
//			SetFaceColor(-1, s);
//			glVertex3f(-scale, scale-epsilon, scale);
//			glVertex3f(-scale, scale-epsilon, scale-offset2);
//			glVertex3f(-scale+offset2, scale-epsilon, scale-offset2);
//			glVertex3f(-scale+offset2, scale-epsilon, scale);
		} break;
		case 26:
		{
			// Face 3 - cube 26
			int c = GetFaceColor(20, s);
			triangles.push_back({{{scale, scale-offset, scale}, {scale, scale-offset, scale-offset}, {scale, scale, scale-offset}}, c});
			triangles.push_back({{{scale, scale, scale-offset}, {scale, scale, scale}, {scale, scale-offset, scale}}, c});
//			glVertex3f(scale, scale-offset, scale);
//			glVertex3f(scale, scale-offset, scale-offset);
//			glVertex3f(scale, scale, scale-offset);
//			glVertex3f(scale, scale, scale);
//
//			SetFaceColor(-1, s);
//			glVertex3f(scale-epsilon, scale-offset2, scale);
//			glVertex3f(scale-epsilon, scale-offset2, scale-offset2);
//			glVertex3f(scale-epsilon, scale, scale-offset2);
//			glVertex3f(scale-epsilon, scale, scale);

			
			// Face 4 - cube 26
			c = GetFaceColor(19, s);
			triangles.push_back({{{scale, scale-offset, scale}, {scale-offset, scale-offset, scale}, {scale-offset, scale, scale}}, c});
			triangles.push_back({{{scale-offset, scale, scale}, {scale, scale, scale}, {scale, scale-offset, scale}}, c});
//			glVertex3f(scale, scale-offset, scale);
//			glVertex3f(scale-offset, scale-offset, scale);
//			glVertex3f(scale-offset, scale, scale);
//			glVertex3f(scale, scale, scale);
//
//			SetFaceColor(-1, s);
//			glVertex3f(scale, scale-offset2, scale-epsilon);
//			glVertex3f(scale-offset2, scale-offset2, scale-epsilon);
//			glVertex3f(scale-offset2, scale, scale-epsilon);
//			glVertex3f(scale, scale, scale-epsilon);

			
			// Face 5 - cube 26
			c = GetFaceColor(18, s);
			triangles.push_back({{{scale-offset, scale, scale}, {scale-offset, scale, scale-offset}, {scale, scale, scale-offset}}, c});
			triangles.push_back({{{scale, scale, scale-offset}, {scale, scale, scale}, {scale-offset, scale, scale}}, c});
//			glVertex3f(scale-offset, scale, scale);
//			glVertex3f(scale-offset, scale, scale-offset);
//			glVertex3f(scale, scale, scale-offset);
//			glVertex3f(scale, scale, scale);
//
//			SetFaceColor(-1, s);
//			glVertex3f(scale-offset2, scale-epsilon, scale);
//			glVertex3f(scale-offset2, scale-epsilon, scale-offset2);
//			glVertex3f(scale, scale-epsilon, scale-offset2);
//			glVertex3f(scale, scale-epsilon, scale);

		} break;
	}
}
//void RubiksCorner::OpenGLDraw() const
//{
//	
//}
//
//void RubiksCorner::OpenGLDraw(const RubiksCornerState&s) const
//{
////	glPushMatrix();
////	glRotatef(45, 0.0, 1.0, 0.0);
//	glBegin(GL_QUADS);
//	OpenGLDrawCube(s, 0);
//	OpenGLDrawCube(s, 2);
//	OpenGLDrawCube(s, 6);
//	OpenGLDrawCube(s, 8);
////	glEnd();
////	glPopMatrix();
////
////	glBegin(GL_QUADS);
//	OpenGLDrawCube(s, 18);
//	OpenGLDrawCube(s, 20);
//	OpenGLDrawCube(s, 24);
//	OpenGLDrawCube(s, 26);
//	glEnd();
//}
//
//void RubiksCorner::OpenGLDrawCube(const RubiksCornerState &s, int cube) const
//{
//	const float scale = 0.3;
//	const float offset = 0.95*2.0*scale/3.0;
//	const float offset2 = 2.0*scale/3.0;
//	const float epsilon = 0.002;
//	switch(cube)
//	{
//		case 0:
//		{
//			// Face 0 - cube 0
//			SetFaceColor(0, s);
//			glVertex3f(-scale, -scale, -scale+offset);
//			glVertex3f(-scale, -scale, -scale);
//			glVertex3f(-scale+offset, -scale, -scale);
//			glVertex3f(-scale+offset, -scale, -scale+offset);
//
//			SetFaceColor(-1, s);
//			glVertex3f(-scale, -scale+epsilon, -scale+offset2);
//			glVertex3f(-scale, -scale+epsilon, -scale);
//			glVertex3f(-scale+offset2, -scale+epsilon, -scale);
//			glVertex3f(-scale+offset2, -scale+epsilon, -scale+offset2);
//
//			// Face 1 - cube 0
//			SetFaceColor(1, s);
//			glVertex3f(-scale, -scale+offset, -scale);
//			glVertex3f(-scale, -scale+offset, -scale+offset);
//			glVertex3f(-scale, -scale, -scale+offset);
//			glVertex3f(-scale, -scale, -scale);
//
//			SetFaceColor(-1, s);
//			glVertex3f(-scale+epsilon, -scale+offset2, -scale);
//			glVertex3f(-scale+epsilon, -scale+offset2, -scale+offset2);
//			glVertex3f(-scale+epsilon, -scale, -scale+offset2);
//			glVertex3f(-scale+epsilon, -scale, -scale);
//
//			// Face 2 - cube 0
//			SetFaceColor(2, s);
//			glVertex3f(-scale, -scale+offset, -scale);
//			glVertex3f(-scale+offset, -scale+offset, -scale);
//			glVertex3f(-scale+offset, -scale, -scale);
//			glVertex3f(-scale, -scale, -scale);
//
//			SetFaceColor(-1, s);
//			glVertex3f(-scale, -scale+offset2, -scale+epsilon);
//			glVertex3f(-scale+offset2, -scale+offset2, -scale+epsilon);
//			glVertex3f(-scale+offset2, -scale, -scale+epsilon);
//			glVertex3f(-scale, -scale, -scale+epsilon);
//
//		} break;
//		case 2:
//		{
//			// Face 0 - cube 2
//			SetFaceColor(3, s);
//			glVertex3f(scale-offset, -scale, -scale+offset);
//			glVertex3f(scale-offset, -scale, -scale);
//			glVertex3f(scale, -scale, -scale);
//			glVertex3f(scale, -scale, -scale+offset);
//
//			SetFaceColor(-1, s);
//			glVertex3f(scale-offset2, -scale+epsilon, -scale+offset2);
//			glVertex3f(scale-offset2, -scale+epsilon, -scale);
//			glVertex3f(scale, -scale+epsilon, -scale);
//			glVertex3f(scale, -scale+epsilon, -scale+offset2);
//
//			// Face 2 - cube 2
//			SetFaceColor(4, s);
//			glVertex3f(scale, -scale+offset, -scale);
//			glVertex3f(scale-offset, -scale+offset, -scale);
//			glVertex3f(scale-offset, -scale, -scale);
//			glVertex3f(scale, -scale, -scale);
//
//			SetFaceColor(-1, s);
//			glVertex3f(scale, -scale+offset2, -scale+epsilon);
//			glVertex3f(scale-offset2, -scale+offset2, -scale+epsilon);
//			glVertex3f(scale-offset2, -scale, -scale+epsilon);
//			glVertex3f(scale, -scale, -scale+epsilon);
//
//			// Face 3 - cube 2
//			SetFaceColor(5, s);
//			glVertex3f(scale, -scale+offset, -scale);
//			glVertex3f(scale, -scale+offset, -scale+offset);
//			glVertex3f(scale, -scale, -scale+offset);
//			glVertex3f(scale, -scale, -scale);
//
//			SetFaceColor(-1, s);
//			glVertex3f(scale-epsilon, -scale+offset2, -scale);
//			glVertex3f(scale-epsilon, -scale+offset2, -scale+offset2);
//			glVertex3f(scale-epsilon, -scale, -scale+offset2);
//			glVertex3f(scale-epsilon, -scale, -scale);
//		} break;
//		case 6:
//		{
//			// Face 0 - cube 6
//			SetFaceColor(9, s);
//			glVertex3f(-scale, -scale, scale);
//			glVertex3f(-scale, -scale, scale-offset);
//			glVertex3f(-scale+offset, -scale, scale-offset);
//			glVertex3f(-scale+offset, -scale, scale);
//
//			SetFaceColor(-1, s);
//			glVertex3f(-scale, -scale+epsilon, scale);
//			glVertex3f(-scale, -scale+epsilon, scale-offset2);
//			glVertex3f(-scale+offset2, -scale+epsilon, scale-offset2);
//			glVertex3f(-scale+offset2, -scale+epsilon, scale);
//
//			
//			// Face 1 - cube 6
//			SetFaceColor(11, s);
//			glVertex3f(-scale, -scale+offset, scale);
//			glVertex3f(-scale, -scale+offset, scale-offset);
//			glVertex3f(-scale, -scale, scale-offset);
//			glVertex3f(-scale, -scale, scale);
//
//			SetFaceColor(-1, s);
//			glVertex3f(-scale+epsilon, -scale+offset2, scale);
//			glVertex3f(-scale+epsilon, -scale+offset2, scale-offset2);
//			glVertex3f(-scale+epsilon, -scale, scale-offset2);
//			glVertex3f(-scale+epsilon, -scale, scale);
//
//			// Face 4 - cube 6
//			SetFaceColor(10, s);
//			glVertex3f(-scale, -scale+offset, scale);
//			glVertex3f(-scale+offset, -scale+offset, scale);
//			glVertex3f(-scale+offset, -scale, scale);
//			glVertex3f(-scale, -scale, scale);
//
//			SetFaceColor(-1, s);
//			glVertex3f(-scale, -scale+offset2, scale-epsilon);
//			glVertex3f(-scale+offset2, -scale+offset2, scale-epsilon);
//			glVertex3f(-scale+offset2, -scale, scale-epsilon);
//			glVertex3f(-scale, -scale, scale-epsilon);
//
//		} break;
//		case 8:
//		{
//			// Face 0 - cube 8
//			SetFaceColor(6, s);
//			glVertex3f(scale-offset, -scale, scale);
//			glVertex3f(scale-offset, -scale, scale-offset);
//			glVertex3f(scale, -scale, scale-offset);
//			glVertex3f(scale, -scale, scale);
//
//			SetFaceColor(-1, s);
//			glVertex3f(scale-offset2, -scale+epsilon, scale);
//			glVertex3f(scale-offset2, -scale+epsilon, scale-offset2);
//			glVertex3f(scale, -scale+epsilon, scale-offset2);
//			glVertex3f(scale, -scale+epsilon, scale);
//
//			// Face 3 - cube 8
//			SetFaceColor(7, s);
//			glVertex3f(scale, -scale+offset, scale);
//			glVertex3f(scale, -scale+offset, scale-offset);
//			glVertex3f(scale, -scale, scale-offset);
//			glVertex3f(scale, -scale, scale);
//
//			SetFaceColor(-1, s);
//			glVertex3f(scale-epsilon, -scale+offset2, scale);
//			glVertex3f(scale-epsilon, -scale+offset2, scale-offset2);
//			glVertex3f(scale-epsilon, -scale, scale-offset2);
//			glVertex3f(scale-epsilon, -scale, scale);
//
//			
//			// Face 4 - cube 8
//			SetFaceColor(8, s);
//			glVertex3f(scale, -scale+offset, scale);
//			glVertex3f(scale-offset, -scale+offset, scale);
//			glVertex3f(scale-offset, -scale, scale);
//			glVertex3f(scale, -scale, scale);
//
//			SetFaceColor(-1, s);
//			glVertex3f(scale, -scale+offset2, scale-epsilon);
//			glVertex3f(scale-offset2, -scale+offset2, scale-epsilon);
//			glVertex3f(scale-offset2, -scale, scale-epsilon);
//			glVertex3f(scale, -scale, scale-epsilon);
//
//		} break;
//		case 18:
//		{
//			// Face 1 - cube 18
//			SetFaceColor(14, s);
//			glVertex3f(-scale, scale-offset, -scale);
//			glVertex3f(-scale, scale-offset, -scale+offset);
//			glVertex3f(-scale, scale, -scale+offset);
//			glVertex3f(-scale, scale, -scale);
//
//			SetFaceColor(-1, s);
//			glVertex3f(-scale+epsilon, scale-offset2, -scale);
//			glVertex3f(-scale+epsilon, scale-offset2, -scale+offset2);
//			glVertex3f(-scale+epsilon, scale, -scale+offset2);
//			glVertex3f(-scale+epsilon, scale, -scale);
//
//			
//			// Face 2 - cube 18
//			SetFaceColor(13, s);
//			glVertex3f(-scale, scale-offset, -scale);
//			glVertex3f(-scale+offset, scale-offset, -scale);
//			glVertex3f(-scale+offset, scale, -scale);
//			glVertex3f(-scale, scale, -scale);
//
//			SetFaceColor(-1, s);
//			glVertex3f(-scale, scale-offset2, -scale+epsilon);
//			glVertex3f(-scale+offset2, scale-offset2, -scale+epsilon);
//			glVertex3f(-scale+offset2, scale, -scale+epsilon);
//			glVertex3f(-scale, scale, -scale+epsilon);
//
//			
//			// Face 5 - cube 18
//			SetFaceColor(12, s);
//			glVertex3f(-scale, scale, -scale+offset);
//			glVertex3f(-scale, scale, -scale);
//			glVertex3f(-scale+offset, scale, -scale);
//			glVertex3f(-scale+offset, scale, -scale+offset);
//
//			SetFaceColor(-1, s);
//			glVertex3f(-scale, scale-epsilon, -scale+offset2);
//			glVertex3f(-scale, scale-epsilon, -scale);
//			glVertex3f(-scale+offset2, scale-epsilon, -scale);
//			glVertex3f(-scale+offset2, scale-epsilon, -scale+offset2);
//
//		} break;
//		case 20:
//		{
//			// Face 2 - cube 20
//			SetFaceColor(17, s);
//			glVertex3f(scale, scale-offset, -scale);
//			glVertex3f(scale-offset, scale-offset, -scale);
//			glVertex3f(scale-offset, scale, -scale);
//			glVertex3f(scale, scale, -scale);
//
//			SetFaceColor(-1, s);
//			glVertex3f(scale, scale-offset2, -scale+epsilon);
//			glVertex3f(scale-offset2, scale-offset2, -scale+epsilon);
//			glVertex3f(scale-offset2, scale, -scale+epsilon);
//			glVertex3f(scale, scale, -scale+epsilon);
//			
//			// Face 3 - cube 20
//			SetFaceColor(16, s);
//			glVertex3f(scale, scale-offset, -scale);
//			glVertex3f(scale, scale-offset, -scale+offset);
//			glVertex3f(scale, scale, -scale+offset);
//			glVertex3f(scale, scale, -scale);
//
//			SetFaceColor(-1, s);
//			glVertex3f(scale-epsilon, scale-offset2, -scale);
//			glVertex3f(scale-epsilon, scale-offset2, -scale+offset2);
//			glVertex3f(scale-epsilon, scale, -scale+offset2);
//			glVertex3f(scale-epsilon, scale, -scale);
//
//			// Face 5 - cube 20
//			SetFaceColor(15, s);
//			glVertex3f(scale-offset, scale, -scale+offset);
//			glVertex3f(scale-offset, scale, -scale);
//			glVertex3f(scale, scale, -scale);
//			glVertex3f(scale, scale, -scale+offset);
//	
//			SetFaceColor(-1, s);
//			glVertex3f(scale-offset2, scale-epsilon, -scale+offset2);
//			glVertex3f(scale-offset2, scale-epsilon, -scale);
//			glVertex3f(scale, scale-epsilon, -scale);
//			glVertex3f(scale, scale-epsilon, -scale+offset2);
//		} break;
//		case 24:
//		{
//			// Face 1 - cube 24
//			SetFaceColor(22, s);
//			glVertex3f(-scale, scale-offset, scale);
//			glVertex3f(-scale, scale-offset, scale-offset);
//			glVertex3f(-scale, scale, scale-offset);
//			glVertex3f(-scale, scale, scale);
//
//			SetFaceColor(-1, s);
//			glVertex3f(-scale+epsilon, scale-offset2, scale);
//			glVertex3f(-scale+epsilon, scale-offset2, scale-offset2);
//			glVertex3f(-scale+epsilon, scale, scale-offset2);
//			glVertex3f(-scale+epsilon, scale, scale);
//
//			
//			// Face 4 - cube 24
//			SetFaceColor(23, s);
//			glVertex3f(-scale, scale-offset, scale);
//			glVertex3f(-scale+offset, scale-offset, scale);
//			glVertex3f(-scale+offset, scale, scale);
//			glVertex3f(-scale, scale, scale);
//
//			SetFaceColor(-1, s);
//			glVertex3f(-scale, scale-offset2, scale-epsilon);
//			glVertex3f(-scale+offset2, scale-offset2, scale-epsilon);
//			glVertex3f(-scale+offset2, scale, scale-epsilon);
//			glVertex3f(-scale, scale, scale-epsilon);
//
//			
//			// Face 5 - cube 24
//			SetFaceColor(21, s);
//			glVertex3f(-scale, scale, scale);
//			glVertex3f(-scale, scale, scale-offset);
//			glVertex3f(-scale+offset, scale, scale-offset);
//			glVertex3f(-scale+offset, scale, scale);
//
//			SetFaceColor(-1, s);
//			glVertex3f(-scale, scale-epsilon, scale);
//			glVertex3f(-scale, scale-epsilon, scale-offset2);
//			glVertex3f(-scale+offset2, scale-epsilon, scale-offset2);
//			glVertex3f(-scale+offset2, scale-epsilon, scale);
//		} break;
//		case 26:
//		{
//			// Face 3 - cube 26
//			SetFaceColor(20, s);
//			glVertex3f(scale, scale-offset, scale);
//			glVertex3f(scale, scale-offset, scale-offset);
//			glVertex3f(scale, scale, scale-offset);
//			glVertex3f(scale, scale, scale);
//
//			SetFaceColor(-1, s);
//			glVertex3f(scale-epsilon, scale-offset2, scale);
//			glVertex3f(scale-epsilon, scale-offset2, scale-offset2);
//			glVertex3f(scale-epsilon, scale, scale-offset2);
//			glVertex3f(scale-epsilon, scale, scale);
//
//			
//			// Face 4 - cube 26
//			SetFaceColor(19, s);
//			glVertex3f(scale, scale-offset, scale);
//			glVertex3f(scale-offset, scale-offset, scale);
//			glVertex3f(scale-offset, scale, scale);
//			glVertex3f(scale, scale, scale);
//
//			SetFaceColor(-1, s);
//			glVertex3f(scale, scale-offset2, scale-epsilon);
//			glVertex3f(scale-offset2, scale-offset2, scale-epsilon);
//			glVertex3f(scale-offset2, scale, scale-epsilon);
//			glVertex3f(scale, scale, scale-epsilon);
//
//			
//			// Face 5 - cube 26
//			SetFaceColor(18, s);
//			glVertex3f(scale-offset, scale, scale);
//			glVertex3f(scale-offset, scale, scale-offset);
//			glVertex3f(scale, scale, scale-offset);
//			glVertex3f(scale, scale, scale);
//
//			SetFaceColor(-1, s);
//			glVertex3f(scale-offset2, scale-epsilon, scale);
//			glVertex3f(scale-offset2, scale-epsilon, scale-offset2);
//			glVertex3f(scale, scale-epsilon, scale-offset2);
//			glVertex3f(scale, scale-epsilon, scale);
//
//		} break;
//	}
//}
//
///** Draw the transition at some percentage 0...1 between two states */
//
//void RubiksCorner::OpenGLDraw(const RubiksCornerState&, const RubiksCornerState&, float) const
//{
//	
//}
//
//void RubiksCorner::OpenGLDraw(const RubiksCornerState&, const RubiksCornersAction&) const
//{
//	
//}

int RubiksCorner::GetFaceColor(int face, const RubiksCornerState &s) const
{
	int theColor = -1;
	if (face == -1)
	{
		return theColor;
	}

	int cube = s.GetFaceInLoc(face);
	switch (cube)
	{
		case 0: theColor = 0; break;
		case 1: theColor = 1; break;
		case 2: theColor = 2; break;
		case 3: theColor = 0; break;
		case 4: theColor = 2; break;
		case 5: theColor = 3; break;
		case 6: theColor = 0; break;
		case 7: theColor = 3; break;
		case 8: theColor = 4; break;
		case 9: theColor = 0; break;
		case 10: theColor = 4; break;
		case 11: theColor = 1; break;
		case 12: theColor = 5; break;
		case 13: theColor = 2; break;
		case 14: theColor = 1; break;
		case 15: theColor = 5; break;
		case 16: theColor = 3; break;
		case 17: theColor = 2; break;
		case 18: theColor = 5; break;
		case 19: theColor = 4; break;
		case 20: theColor = 3; break;
		case 21: theColor = 5; break;
		case 22: theColor = 1; break;
		case 23: theColor = 4; break;
		default: theColor = -1; break;
	}
	return theColor;
}

//void RubiksCorner::SetFaceColor(int face, const RubiksCornerState &s) const
//{
//	int theColor = -1;
//	if (face == -1)
//	{
//		glColor3f(0.0, 0.0, 0.0);
//		return;
//	}
//
//	int cube = s.GetFaceInLoc(face);
//	switch (cube)
//	{
//		case 0: theColor = 0; break;
//		case 1: theColor = 1; break;
//		case 2: theColor = 2; break;
//		case 3: theColor = 0; break;
//		case 4: theColor = 2; break;
//		case 5: theColor = 3; break;
//		case 6: theColor = 0; break;
//		case 7: theColor = 3; break;
//		case 8: theColor = 4; break;
//		case 9: theColor = 0; break;
//		case 10: theColor = 4; break;
//		case 11: theColor = 1; break;
//		case 12: theColor = 5; break;
//		case 13: theColor = 2; break;
//		case 14: theColor = 1; break;
//		case 15: theColor = 5; break;
//		case 16: theColor = 3; break;
//		case 17: theColor = 2; break;
//		case 18: theColor = 5; break;
//		case 19: theColor = 4; break;
//		case 20: theColor = 3; break;
//		case 21: theColor = 5; break;
//		case 22: theColor = 1; break;
//		case 23: theColor = 4; break;
//		default: theColor = -1; break;
//	}
//	
//	switch (theColor)
//	{
//		case 0: glColor3f(1.0, 0.0, 0.0); break;
//		case 1: glColor3f(0.0, 1.0, 0.0); break;
//		case 2: glColor3f(0.0, 0.0, 1.0); break;
//		case 3: glColor3f(1.0, 1.0, 0.0); break;
//		case 4: glColor3f(1.0, 0.75, 0.0); break;
//		case 5: glColor3f(1.0, 1.0, 1.0); break;
//		default: glColor3f(0.0, 0.0, 0.0); break;
//	}
//}



RubikCornerPDB::RubikCornerPDB(RubiksCorner *e, const RubiksCornerState &s, std::vector<int> &distinctCorners)
:PDBHeuristic(e), corners(distinctCorners)
{
	SetGoal(s);
}

uint64_t RubikCornerPDB::GetStateSpaceSize()
{
#pragma message("This code belongs in the RubikCorner, not in the PDB.")
	return 40320ll*2187ll;
}

uint64_t RubikCornerPDB::GetStateHash(const RubiksCornerState &s)
{
	RubiksCorner rc;
	return rc.GetStateHash(s);
}

void RubikCornerPDB::GetStateFromHash(RubiksCornerState &s, uint64_t hash)
{
	RubiksCorner rc;
	rc.GetStateFromHash(hash, s);
}

uint64_t RubikCornerPDB::GetPDBSize() const
{
//	uint64_t power3[] = {1, 3, 9, 27, 81, 243, 729, 2187, 2187}; // last tile is symmetric
//	int elts = (int)corners.size();
//	return FactorialUpperK(8, 8-elts)*power3[elts];
	uint64_t answer[] = {1, 24, 504, 9072, 136080, 1632960, 14696640, 88179840, 88179840};
	return answer[corners.size()];
}

#define MR

uint64_t RubikCornerPDB::GetPDBHash(const RubiksCornerState &s, int threadID) const
{
#ifdef MR
	// TODO: handle fewer according to pattern
	int puzzle[8] = {-1, -1, -1, -1, -1, -1, -1, -1};
	int dual[16]; // seamlessly handle 0xF entries (no cube)
	int newdual[16]; // seamlessly handle 0xF entries (no cube)
	int cornerSize = corners.size();
	int lastPiece = 8-(int)cornerSize;
	for (int x = 0; x < 8; x++)
		dual[s.GetCubeInLoc(x)] = x;
	for (int x = 0; x < cornerSize; x++)
	{
		newdual[x] = dual[corners[x]];
		puzzle[dual[corners[x]]] = x;
	}
	uint64_t hashVal = 0;
	uint64_t part2 = 0;
	hashVal = mr1.Rank(puzzle, newdual, cornerSize, 8);//mr1.GetRank(puzzle, threadID);

	int limit = std::min((int)cornerSize, 7);
	for (int x = 0; x < limit; x++)
	{
		part2 = part2*3+s.GetCubeOrientation(corners[x]);
		//part2 = part2*3+s.GetCubeOrientation(dual[corners[x]]);
	}
	return part2*FactorialUpperK(8, lastPiece)+hashVal;

#else

	// TODO: handle fewer according to pattern
	int puzzle[8];
	int dual[16]; // seamlessly handle 0xF entries (no cube)
	int cornerSize = corners.size();
	int lastPiece = 8-(int)corners.size();
	for (int x = 0; x < 8; x++)
		dual[s.GetCubeInLoc(x)] = x;
	for (int x = 0; x < cornerSize; x++)
		puzzle[x] = dual[corners[x]];
	
	uint64_t hashVal = 0;
	uint64_t part2 = 0;
	int numEntriesLeft = 8;
	for (unsigned int x = 0; x < cornerSize; x++)
	{
		hashVal += puzzle[x]*FactorialUpperK(numEntriesLeft-1, lastPiece);
		
		numEntriesLeft--;
		for (unsigned y = x; y < cornerSize; y++)
		{
			if (puzzle[y] > puzzle[x])
				puzzle[y]--;
		}
	}
	int limit = std::min((int)cornerSize, 7);
	for (int x = 0; x < limit; x++)
	{
		part2 = part2*3+s.GetCubeOrientation(corners[x]);
		//part2 = part2*3+s.GetCubeOrientation(dual[corners[x]]);
	}
	return part2*FactorialUpperK(8, lastPiece)+hashVal;
#endif
	
}

void RubikCornerPDB::GetStateFromPDBHash(uint64_t hash, RubiksCornerState &s, int threadID) const
{
#ifdef MR
	int lastPiece = 8-(int)corners.size();
	int puzzle[8] = {-1, -1, -1, -1, -1, -1, -1};
	int dual[16] = {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1};
	uint64_t hashVal = hash;
	int cornerSize = corners.size();
	hash /= FactorialUpperK(8, lastPiece); // for rotations
	hashVal = hashVal%FactorialUpperK(8, lastPiece); // for pieces)
	mr1.Unrank(hashVal, puzzle, dual, cornerSize, 8);
	
	for (int x = 0; x < 8; x++)
	{
		s.SetCubeInLoc(x, 0xF);
		s.SetCubeOrientation(x, 0);
	}
	
	for (int x = 0; x < cornerSize; x++)
	{
		s.SetCubeInLoc(dual[x], corners[x]);
	}
	
	int cnt = 0;
	int limit = std::min((int)cornerSize, 7);
	for (int x = limit-1; x >= 0; x--)
	{
		s.SetCubeOrientation(corners[x], hash%3);
		cnt += hash%3;
		hash/=3;
	}
	if (cornerSize == 8)
		s.SetCubeOrientation(corners[7], (3-(cnt%3))%3); // 0->0 2->1 1->2
#else
	
	int lastPiece = 8-(int)corners.size();
	int cornerSize = corners.size();
	int puzzle[12];
	int dual[16];
	uint64_t hashVal = hash;
	hash /= FactorialUpperK(8, lastPiece); // for rotations
	hashVal = hashVal%FactorialUpperK(8, lastPiece); // for pieces
	
	int numEntriesLeft = lastPiece+1;
	for (int x = corners.size()-1; x >= 0; x--)
	{
		puzzle[x] = hashVal%numEntriesLeft;
		hashVal /= numEntriesLeft;
		numEntriesLeft++;
		for (int y = x+1; y < cornerSize; y++)
		{
			if (puzzle[y] >= puzzle[x])
				puzzle[y]++;
		}
	}
	for (int x = 0; x < 8; x++)
	{
		s.SetCubeInLoc(x, 0xF);
		s.SetCubeOrientation(x, 0);
	}
	
	for (int x = 0; x < cornerSize; x++)
	{
		s.SetCubeInLoc(puzzle[x], corners[x]);
		dual[corners[x]] = puzzle[x];
	}
	
	int cnt = 0;
	int limit = std::min((int)corners.size(), 7);
	for (int x = limit-1; x >= 0; x--)
	{
		s.SetCubeOrientation(corners[x], hash%3);
		cnt += hash%3;
		hash/=3;
	}
	if (corners.size() == 8)
		s.SetCubeOrientation(corners[7], (3-(cnt%3))%3); // 0->0 2->1 1->2

#endif
}


bool RubikCornerPDB::Load(const char *prefix)
{
	FILE *f = fopen(GetFileName(prefix).c_str(), "rb");
	if (f == 0)
	{
		perror("Opening RubiksCornerPDB file");
		return false;
	}
	Save(f);
	fclose(f);
	return true;
}

void RubikCornerPDB::Save(const char *prefix)
{
	FILE *f = fopen(GetFileName(prefix).c_str(), "w+");
	if (f == 0)
	{
		perror("Opening RubiksCornerPDB file");
		return;
	}
	Save(f);
	fclose(f);
}

bool RubikCornerPDB::Load(FILE *f)
{
	if (PDBHeuristic<RubiksCornerState, RubiksCornersAction, RubiksCorner, RubiksCornerState, 4>::Load(f) == false)
		return false;
	if (fread(&puzzleSize, sizeof(puzzleSize), 1, f) != 1)
		return false;
	if (fread(&pdbSize, sizeof(pdbSize), 1, f) != 1)
		return false;
	size_t edgeSize = corners.size();
	if (fread(&edgeSize, sizeof(edgeSize), 1, f) != 1)
		return false;
	corners.resize(edgeSize);
	if (fread(&corners[0], sizeof(corners[0]), corners.size(), f) != edgeSize)
		return false;
	return true;
}

void RubikCornerPDB::Save(FILE *f)
{
	PDBHeuristic<RubiksCornerState, RubiksCornersAction, RubiksCorner, RubiksCornerState, 4>::Save(f);
	fwrite(&puzzleSize, sizeof(puzzleSize), 1, f);
	fwrite(&pdbSize, sizeof(pdbSize), 1, f);
	size_t edgeSize = corners.size();
	fwrite(&edgeSize, sizeof(edgeSize), 1, f);
	fwrite(&corners[0], sizeof(corners[0]), corners.size(), f);
}

std::string RubikCornerPDB::GetFileName(const char *prefix)
{
	std::string fileName;
	fileName += "RC-C-";
	// origin state
	for (int x = 0; x < 8; x++)
	{
		fileName += std::to_string(goalState[0].GetCubeInLoc(x));
		fileName += ".";
		fileName += std::to_string(goalState[0].GetCubeOrientation(goalState[0].GetCubeInLoc(x)));
		fileName += ";";
	}
	fileName.pop_back();
	fileName += "-";
	// pattern
	for (size_t x = 0; x < corners.size(); x++)
	{
		fileName += std::to_string(corners[x]);
		fileName += ";";
	}
	fileName.pop_back(); // remove colon
#ifdef MR
	fileName += "-MR";
#endif
	if (std::is_same<RubiksCornerStateArray,RubiksCornerState>::value)
	{
		fileName += "-AR";
	}
	fileName += ".pdb";
	
	return fileName;
}

uint64_t RubikCornerPDB::Factorial(int val) const
{
	static uint64_t table[21] =
	{ 1ll, 1ll, 2ll, 6ll, 24ll, 120ll, 720ll, 5040ll, 40320ll, 362880ll, 3628800ll, 39916800ll, 479001600ll,
		6227020800ll, 87178291200ll, 1307674368000ll, 20922789888000ll, 355687428096000ll,
		6402373705728000ll, 121645100408832000ll, 2432902008176640000ll };
	if (val > 20)
		return (uint64_t)-1;
	return table[val];
}

uint64_t RubikCornerPDB::FactorialUpperK(int n, int k) const
{
	const uint64_t result[9][9] = {
		{1}, // n = 0
		{1, 1}, // n = 1
		{2, 2, 1}, // n = 2
		{6, 6, 3, 1}, // n = 3
		{24, 24, 12, 4, 1}, // n = 4
		{120, 120, 60, 20, 5, 1}, // n = 5
		{720, 720, 360, 120, 30, 6, 1}, // n = 6
		{5040, 5040, 2520, 840, 210, 42, 7, 1}, // n = 7
		{40320, 40320, 20160, 6720, 1680, 336, 56, 8, 1}, // n = 8
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
