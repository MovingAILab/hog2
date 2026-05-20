//
//  Rubik7Edge.cpp
//  hog2 glut
//
//  Created by Nathan Sturtevant on 1/4/13.
//  Copyright (c) 2013 University of Denver. All rights reserved.
//

#include "RubiksCube7Edges.h"
#include "Constants.h"
#include <cassert>

void Rubik7EdgeState::GetDual(Rubik7EdgeState &s) const
{
//	for (int x = 0; x < 12; x++)
//	{
//		s.SetCubeInLoc(0, 0xF);
//		s.SetCubeOrientation(x, 0);
//	}
	for (int x = 0; x < 12; x++)
	{
		s.SetCubeInLoc(GetCubeInLoc(x), x);
		s.SetCubeOrientation(x, GetCubeOrientation(GetCubeInLoc(x)));
	}
}

void Rubik7Edge::GetSuccessors(const Rubik7EdgeState &nodeID, std::vector<Rubik7EdgeState> &neighbors) const
{
	Rubik7EdgeState s;
	for (int x = 0; x < 18; x++)
	{
		GetNextState(nodeID, x, s);
		neighbors.push_back(s);
	}
}

void Rubik7Edge::GetActions(const Rubik7EdgeState &nodeID, std::vector<Rubik7EdgeAction> &actions) const
{
	actions.resize(0);
	for (int x = 0; x < 18; x++)
	{
		actions.push_back(x);
	}
}

Rubik7EdgeAction Rubik7Edge::GetAction(const Rubik7EdgeState &s1, const Rubik7EdgeState &s2) const
{
	assert(false);
	Rubik7EdgeAction a;
	return a;
}

void Rubik7Edge::ApplyMove(Rubik7EdgeState &s, Rubik7EdgeMove *a)
{
	ApplyAction(s, a->act);
}

void Rubik7Edge::UndoMove(Rubik7EdgeState &s, Rubik7EdgeMove *a)
{
	Rubik7EdgeAction todo = a->act;
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

void Rubik7Edge::ApplyAction(Rubik7EdgeState &s, Rubik7EdgeAction a) const
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

void Rubik7Edge::GetNextState(const Rubik7EdgeState &s0, Rubik7EdgeAction a, Rubik7EdgeState &s1) const
{
	s1 = s0;
	ApplyAction(s1, a);
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
	uint64_t val1 = (state>>(loc1))&0xF;
	uint64_t val2 = (state>>(loc2))&0xF;
	const uint64_t blank = 0xF;
	uint64_t mask = (blank<<(loc1))|(blank<<(loc2));
	state = state&(~mask);
	state = state|(val1<<(loc2))|(val2<<(loc1));
}

int64_t Rubik7Edge::getMaxSinglePlayerRank() const
{
	static uint64_t Factorial[21] =
	{ 1ll, 1ll, 2ll, 6ll, 24ll, 120ll, 720ll, 5040ll, 40320ll, 362880ll, 3628800ll, 39916800ll, 479001600ll,
		6227020800ll, 87178291200ll, 1307674368000ll, 20922789888000ll, 355687428096000ll,
		6402373705728000ll, 121645100408832000ll, 2432902008176640000ll };
	
	return (Factorial[12]/Factorial[12-pieces])*(0x1<<pieces);
	//	return 980995276800ll;
}

int64_t Rubik7Edge::getMaxSinglePlayerRank2()
{
	//	return 12;
	//	return 16;
	//	return 64;
	return (1<<pieces);
	//return 4;
}

int64_t Rubik7Edge::getMaxSinglePlayerRank2(int64_t firstIndex)
{
	return Factorial(12)/Factorial(12-pieces);
	//	return (Factorial(12)*(0x1<<7));
	//	return (Factorial(12)*(0x1<<9));
	//	return 81749606400ll;
}

void Rubik7Edge::rankPlayerFirstTwo(const Rubik7EdgeState &node, int, int64_t &rank)
{
	uint64_t hash1 = 0;
	for (unsigned int x = 0; x < pieces; x++)
	{
		//hash1 = (hash1<<1)+node.GetCubeOrientation(locs[x]);
		hash1 = (hash1<<1)+node.GetCubeOrientation(x);
	}
	
	rank = hash1;
}

void Rubik7Edge::rankPlayerRemaining(const Rubik7EdgeState &node, int, int64_t &rank)
{
	int locs[pieces];
	for (int x = 0; x < 12; x++)
	{
		if (node.GetCubeInLoc(x) < pieces)
		{
			locs[node.GetCubeInLoc(x)] = x;
		}
	}
	
	uint64_t hash2 = 0;
	int numEntriesLeft = 12;
	for (unsigned int x = 0; x < pieces; x++)
	{
		hash2 += locs[x]*Factorial(numEntriesLeft-1)/Factorial(12-pieces);
		numEntriesLeft--;
		for (unsigned y = x+1; y < pieces; y++)
		{
			if (locs[y] > locs[x])
				locs[y]--;
		}
	}
	rank = hash2;
}


uint64_t Rubik7Edge::GetStateHash(const Rubik7EdgeState &node) const
{
	uint64_t hash1 = 0;
	int locs[pieces];
	for (int x = 0; x < 12; x++)
	{
		if (node.GetCubeInLoc(x) < pieces)
		{
			locs[node.GetCubeInLoc(x)] = x;
		}
	}
	for (unsigned int x = 0; x < pieces; x++)
	{
		hash1 = (hash1<<1)+node.GetCubeOrientation(x);
	}

	uint64_t hash2 = 0;
	int numEntriesLeft = 12;
	for (unsigned int x = 0; x < pieces; x++)
	{
		hash2 += locs[x]*Factorial(numEntriesLeft-1)/Factorial(12-pieces);
		numEntriesLeft--;
		for (unsigned y = x+1; y < pieces; y++)
		{
			if (locs[y] > locs[x])
				locs[y]--;
		}
	}
	return hash1*Factorial(12)/Factorial(12-pieces)+hash2;
}

void Rubik7Edge::GetStateFromHash(uint64_t hash, Rubik7EdgeState &node) const
{
	//int cnt = 0;
	uint64_t bits = hash*Factorial(12-pieces)/Factorial(12);
	uint64_t hash2 = hash%(Factorial(12)/Factorial(12-pieces));
	//printf("-bits: %" PRId64 " -perm: %" PRId64 "\n", bits, hash2);

	int locs[pieces];

	int numEntriesLeft = 12-pieces+2;
	for (int x = pieces-1; x >= 0; x--)
	{
//		if (numEntriesLeft == 1)
//		{
//			locs[x] = (int)hash2;
//			hash2 = 0;
//		}
//		else {
		locs[x] = (int)(hash2%(numEntriesLeft-1));
		hash2 = hash2/(numEntriesLeft-1);
//		}
		//hash2 += locs[x]*Factorial(numEntriesLeft-1)/Factorial(12-pieces);
		numEntriesLeft++;
		//printf("Converting: ");
		for (unsigned y = x+1; y < pieces; y++)
		{
			if (locs[y] >= locs[x])
				locs[y]++;
			//printf("%d ", locs[y]);
		}
		//printf("\n");
	}
//	printf("Locs: ");
//	for (int x = 0; x < pieces; x++)
//	{
//		printf("%d ", locs[x]);
//	}
//	printf("\n");

	for (int x = 0; x < 12; x++)
	{
		node.SetCubeInLoc(x, 0xF);
		//node.SetCubeOrientation(x, 0);
	}
	for (int x = pieces-1; x >= 0; x--)
	{
		node.SetCubeInLoc(locs[x], x);
		//node.SetCubeOrientation(locs[x], bits&1);
		node.SetCubeOrientation(x, bits&1);
		bits = bits>>1;
	}
	int next = pieces;
	for (int x = 0; x < 12; x++)
	{
		if (node.GetCubeInLoc(x) == 0xF)
			node.SetCubeInLoc(x, next++);
		//node.SetCubeOrientation(x, 0);
	}

}

//void Rubik7Edge::OpenGLDraw() const
//{
//	
//}
//
//void Rubik7Edge::OpenGLDraw(const Rubik7EdgeState&s) const
//{
//	float scale = 0.3;
//	float offset = 2.0*scale/3.0;
//	glBegin(GL_QUADS);
//	// top
//	//// Face 0
//	//SetCubeColor(0);
//	// 1 - top
//	SetCubeColor(1, false, s);
//	glVertex3f(-scale, -scale, offset/2.0);
//	glVertex3f(-scale, -scale, -offset/2.0);
//	glVertex3f(-scale+offset, -scale, -offset/2.0);
//	glVertex3f(-scale+offset, -scale, offset/2.0);
//	
//	SetCubeColor(5, false, s);
//	// 5 - top
//	glVertex3f(scale, -scale, offset/2.0);
//	glVertex3f(scale, -scale, -offset/2.0);
//	glVertex3f(scale-offset, -scale, -offset/2.0);
//	glVertex3f(scale-offset, -scale, offset/2.0);
//	
//	SetCubeColor(3, false, s);
//	// 3 - top
//	glVertex3f(-offset/2.0, -scale, -scale);
//	glVertex3f(-offset/2.0, -scale, -scale+offset);
//	glVertex3f(offset/2.0, -scale, -scale+offset);
//	glVertex3f(offset/2.0, -scale, -scale);
//	
//	SetCubeColor(7, false, s);
//	// 7 - top
//	glVertex3f(-offset/2.0, -scale, scale);
//	glVertex3f(-offset/2.0, -scale, scale-offset);
//	glVertex3f(offset/2.0, -scale, scale-offset);
//	glVertex3f(offset/2.0, -scale, scale);
//	
//	//// Face 5
//	SetCubeColor(9, true, s);
//	// 9 - bottom
//	glVertex3f(-scale, scale, offset/2.0);
//	glVertex3f(-scale, scale, -offset/2.0);
//	glVertex3f(-scale+offset, scale, -offset/2.0);
//	glVertex3f(-scale+offset, scale, offset/2.0);
//	
//	SetCubeColor(11, true, s);
//	// 11 - bottom
//	glVertex3f(scale, scale, offset/2.0);
//	glVertex3f(scale, scale, -offset/2.0);
//	glVertex3f(scale-offset, scale, -offset/2.0);
//	glVertex3f(scale-offset, scale, offset/2.0);
//	
//	SetCubeColor(10, true, s);
//	// 10 - bottom
//	glVertex3f(-offset/2.0, scale, -scale);
//	glVertex3f(-offset/2.0, scale, -scale+offset);
//	glVertex3f(offset/2.0, scale, -scale+offset);
//	glVertex3f(offset/2.0, scale, -scale);
//	
//	SetCubeColor(12, true, s);
//	// 12 - bottom
//	glVertex3f(-offset/2.0, scale, scale);
//	glVertex3f(-offset/2.0, scale, scale-offset);
//	glVertex3f(offset/2.0, scale, scale-offset);
//	glVertex3f(offset/2.0, scale, scale);
//	
//	//// Face 1
//	SetCubeColor(1, true, s);
//	// 1 - side
//	glVertex3f(-scale, -scale+offset, -offset/2.0);
//	glVertex3f(-scale, -scale+offset, offset/2.0);
//	glVertex3f(-scale, -scale, offset/2.0);
//	glVertex3f(-scale, -scale, -offset/2.0);
//	
//	SetCubeColor(9, false, s);
//	// 9 - side
//	glVertex3f(-scale, scale-offset, -offset/2.0);
//	glVertex3f(-scale, scale-offset, offset/2.0);
//	glVertex3f(-scale, scale, offset/2.0);
//	glVertex3f(-scale, scale, -offset/2.0);
//	
//	SetCubeColor(2, false, s);
//	// 2 - side(1)
//	glVertex3f(-scale, -offset/2.0, -scale+offset);
//	glVertex3f(-scale, offset/2.0, -scale+offset);
//	glVertex3f(-scale, offset/2.0, -scale);
//	glVertex3f(-scale, -offset/2.0, -scale);
//	
//	SetCubeColor(8, false, s);
//	// 8 - side(1)
//	glVertex3f(-scale, -offset/2.0, scale-offset);
//	glVertex3f(-scale, offset/2.0, scale-offset);
//	glVertex3f(-scale, offset/2.0, scale);
//	glVertex3f(-scale, -offset/2.0, scale);
//	
//	//// Face 3
//	SetCubeColor(5, true, s);
//	// 5 - side
//	glVertex3f(scale, -scale+offset, -offset/2.0);
//	glVertex3f(scale, -scale+offset, offset/2.0);
//	glVertex3f(scale, -scale, offset/2.0);
//	glVertex3f(scale, -scale, -offset/2.0);
//	
//	SetCubeColor(11, false, s);
//	// 11 - side
//	glVertex3f(scale, scale-offset, -offset/2.0);
//	glVertex3f(scale, scale-offset, offset/2.0);
//	glVertex3f(scale, scale, offset/2.0);
//	glVertex3f(scale, scale, -offset/2.0);
//	
//	SetCubeColor(4, true, s);
//	// 4 - side(1)
//	glVertex3f(scale, -offset/2.0, -scale+offset);
//	glVertex3f(scale, offset/2.0, -scale+offset);
//	glVertex3f(scale, offset/2.0, -scale);
//	glVertex3f(scale, -offset/2.0, -scale);
//	
//	SetCubeColor(6, false, s);
//	// 6 - side(1)
//	glVertex3f(scale, -offset/2.0, scale-offset);
//	glVertex3f(scale, offset/2.0, scale-offset);
//	glVertex3f(scale, offset/2.0, scale);
//	glVertex3f(scale, -offset/2.0, scale);
//	
//	//// Face 2
//	SetCubeColor(3, true, s);
//	// 3 - side
//	glVertex3f(-offset/2.0, -scale+offset, -scale);
//	glVertex3f(offset/2.0, -scale+offset, -scale);
//	glVertex3f(offset/2.0, -scale, -scale);
//	glVertex3f(-offset/2.0, -scale, -scale);
//	
//	SetCubeColor(10, false, s);
//	// 10 - side
//	glVertex3f(-offset/2.0, scale-offset, -scale);
//	glVertex3f(offset/2.0, scale-offset, -scale);
//	glVertex3f(offset/2.0, scale, -scale);
//	glVertex3f(-offset/2.0, scale, -scale);
//	
//	SetCubeColor(2, true, s);
//	// 2 - side
//	glVertex3f(-scale+offset, -offset/2.0, -scale);
//	glVertex3f(-scale+offset, offset/2.0, -scale);
//	glVertex3f(-scale, offset/2.0, -scale);
//	glVertex3f(-scale, -offset/2.0, -scale);
//	
//	SetCubeColor(4, false, s);
//	// 4 - side
//	glVertex3f(scale-offset, -offset/2.0, -scale);
//	glVertex3f(scale-offset, offset/2.0, -scale);
//	glVertex3f(scale, offset/2.0, -scale);
//	glVertex3f(scale, -offset/2.0, -scale);
//	
//	
//	//// Face 4
//	SetCubeColor(7, true, s);
//	// 7 - side
//	glVertex3f(-offset/2.0, -scale+offset, scale);
//	glVertex3f(offset/2.0, -scale+offset, scale);
//	glVertex3f(offset/2.0, -scale, scale);
//	glVertex3f(-offset/2.0, -scale, scale);
//	
//	SetCubeColor(12, false, s);
//	// 12 - side
//	glVertex3f(-offset/2.0, scale-offset, scale);
//	glVertex3f(offset/2.0, scale-offset, scale);
//	glVertex3f(offset/2.0, scale, scale);
//	glVertex3f(-offset/2.0, scale, scale);
//	
//	SetCubeColor(8, true, s);
//	// 8 - side
//	glVertex3f(-scale+offset, -offset/2.0, scale);
//	glVertex3f(-scale+offset, offset/2.0, scale);
//	glVertex3f(-scale, offset/2.0, scale);
//	glVertex3f(-scale, -offset/2.0, scale);
//	
//	SetCubeColor(6, true, s);
//	// 6 - side
//	glVertex3f(scale-offset, -offset/2.0, scale);
//	glVertex3f(scale-offset, offset/2.0, scale);
//	glVertex3f(scale, offset/2.0, scale);
//	glVertex3f(scale, -offset/2.0, scale);
//	
//	glEnd();
//	
//}
///** Draw the transition at some percentage 0...1 between two states */
//
//void Rubik7Edge::OpenGLDraw(const Rubik7EdgeState&, const Rubik7EdgeState&, float) const
//{
//	
//}
//
//void Rubik7Edge::OpenGLDraw(const Rubik7EdgeState&, const Rubik7EdgeAction&) const
//{
//	
//}
//
//void Rubik7Edge::SetCubeColor(int which, bool face, const Rubik7EdgeState &s) const
//{
//	int cubes_first[12] = { 0, 1, 0, 2, 0, 3, 0, 1, 1, 2, 3, 4};
//	int cubes_second[12] = { 1, 2, 2, 3, 3, 4, 4, 4, 5, 5, 5, 5};
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
