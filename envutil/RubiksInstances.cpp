//
//  RubiksInstances.cpp
//  hog2 glut
//
//  Created by Nathan Sturtevant on 8/26/19.
//

#include "RubiksInstances.h"

namespace RubiksCubeInstances {

const int maxStrLength = 1024;
char KorfInstances[10][maxStrLength] =
{
	"L2 B  D- L- F- B  R2 F  B  R- F- R2 B- L2 D2 L2 D2 L2 U2 L2 F- D  L- D2 L- F2 B2 L  U- D- L  B  D2 F  D- F- U- B  L2 D2 L2 R2 B- U  R- D2 F  R- B  L R  U- B- R2 F- L2 R  F  R2 B L- F- D- F2 U2 R  U- L  D  F2 B- R- D- L2 B- L- B2 L- D2 B2 D- B  D  R- B  D  L- B- R  F- L- F- R2 D2 L2 B- L2 B2 U  L2",
	"B- R2 B  D  B- L  B  L2 F2 R F2 D- L2 U2 L- U  L- U2 B- L- R- U  D  L- B2 D  R- U  F  D2 F  B  U  B2 L2 D2 R- B2 L- R2 U2 D2 F2 D  R2 D2 B- U- D  F- R  B2 D  R2 F  L- B  L2 R- U2 L  F2 B- D- F- B- L2 D  B2 U- D  F2 U  L2 D  L- D- R2 D- B2 U- L2 U  B- L- U- F- L- R- B- U- R  B2 U2 B  R- B- R2 F  R-",
	"L- R  F- L  R2 F2 D- L2 D  B2 R2 D- F- L- F  R  F2 U  L- B2 D- R- U- R  D  F  R  D  B2 U- F- L2 F- B  U- R  F- U  F  D- L2 R- F- B  L2 B2 D- R- B  L B  D- R  U- R2 D2 F  R  U2 B2 D2 B  R- F- L2 D2 L2 R  D  L- B2 U  F2 R  F  L  U  D  L- B2 L2 B2 D- L  D2 B- U- B- U2 B L  D  B- L- U2 L- R  D- R  B2",
	"L- B2 U2 R- D  F  U  F2 D- F U  D- R- B  R  U- R2 B  R  F D  R2 F- R- B2 R- D- R2 U- F- R  D  F- R  U- F  B  U- D2 B- L  D2 L2 B- U2 L2 F2 L  D2 B D- L  R2 B2 U2 F2 B- U2 F  D2 L2 U2 F2 L  R- U- R- D  F  L2 F2 L- R  U- L2 U  R  D  F  R- F- D- L- R2 U2 F  R- B2 D  B2 D2 B2 R  F- L- D  B- U  L2 B-",
	"B2 L- F- U2 R2 D  R2 B  U2 R B- L  R  F  R- D- R- D2 F2 R2 B  L2 D- B2 D2 L  F- R  B- R2 B2 D  B- U  R- D- L2 B2 L2 R2 B- U2 D- R2 B  U- B- R- D  L B- L  R2 D- B  L2 D2 F2 B- U2 B  D- F- B  L2 U  F- U  F- L U  R  U2 D- B- U2 D  F- L  F2 B2 L  U  B- D2 B2 L- D- L2 B- D2 F  U2 R  D2 L2 D  B- L2 R-",
	"B- R2 B  R  U- D- R2 D  B  L2 B2 U- F- B  D2 R- U  F  L  F U2 B  L  D- L- R2 B  D  R- F- B- D  L- B- L2 R- U2 D  B  R- D2 B- U2 B- L- R2 F  L  U  L- R  B- R2 D2 R2 B- U2 F- U  L D- F  B- R2 D2 L  B  L  U2 B2 R  D- B- R2 B2 U  F2 R2 U- B- R2 F2 U2 F2 B- D  F  U- F2 B R  D  R- U- L- R  U2 D- R2 F",
	"L- R- B  L- D- L  U2 B2 D- F2 D  B2 R- F- R  U2 B2 D2 L- B- D  R2 D2 R  D- R2 F- R  U  B- U2 B2 D- R- D2 F  U  D- F- U2 L2 U  F2 R- B- U- L  B- R- U- L- U2 B2 D  B- R  B  D  B2 R- U2 D- R- F  L- F- D- B  L2 R- B  R- B- D  R  U- R2 B- D2 F- R  F  L- U  L2 B2 R2 F- U- F D  B- L- F2 B  L2 U2 D- L2 B2",
	"U  F  B2 L  F2 L- D2 B2 L2 U- R  D2 L2 U- D  F  U2 L2 B- U- B- R2 B2 R  U  R2 D- B2 R  B2 L2 U- R  D  L- R2 U2 R- D  B L2 R- B2 U2 L  U2 R  F2 U  B2 L2 R2 D- F  R2 D- L2 U- R  B2 D2 R- U2 L- B- L- F- L  R  F L- B- D  L- R2 F2 U- F  L- U2 B  U  R2 F  U  R- B  D  B  R- B- L- B2 R- F  L- B2 L- F2 D2",
	"L  F2 L2 B2 D  B  R  D- R- F2 U2 D2 F2 B- U- L- R2 B  U2 R F- D2 B2 U- R- F  L- U2 R- B L2 R2 B2 L2 R- B- D  F2 L  U2 D  L- B2 L  D- R2 D  B2 U  R- F2 U- R  F2 R2 B  U2 D2 R- U2 L2 R- D2 F2 R2 B- U  B  U  F D- F  D2 R- B- U2 B2 L  D  B- L- B2 D  B2 D- B  D- F- B- L- U  B2 D2 B- D- F  D- L  F  R",
	"B2 D- B- U- R- D- B- U2 L- R- B2 U2 B2 L- U  B- D  F  L2 F2 D  F- L- D- B2 L- U2 F  B2 R2 D  L- U  D2 F  B- L2 B  R- U B- L  B2 D  F  R  U- D- F  R2 U2 L- B2 L- R- D- L2 R- F2 D L- D  B2 D  L  B- R- D  B- L2 B2 D2 F  B2 U2 R- D- L- B2 R- D- L2 F- D- R  U  F  L2 D- R- U- L2 B  U- F2 U  B- D  F2 D2"
};

int GetNextRCMove(const char *input, int &base)
{
	int used = 0;
	if (isdigit(input[0])) // this is our move notation - numeric
	{
		int curr = 0;
		base = input[curr]-'0';
		curr++;
		if (isdigit(input[curr]))
		{
			base = base*10+input[curr]-'0';
			curr++;
		}
		while (!isdigit(input[curr]) && input[curr] != '\n' && input[curr] != 0)
		{
			curr++;
		}
		return curr;
	}
	switch (input[0])
	{
		case 'F': used = 1; base = 1*3; break;
		case 'B': used = 1; base = 3*3; break;
		case 'L': used = 1; base = 2*3; break;
		case 'R': used = 1; base = 4*3; break;
		case 'U': used = 1; base = 0*3; break;
		case 'D': used = 1; base = 5*3; break;
		default: break;
	}
	if (used == 0)
		return 0;
//	if (input[0] != 'U')
//	{
//		bool stillGoing = true;
//		int offset = 1;
//		while (stillGoing)
//		{
//			switch (input[offset++])
//			{
//				case ' ': used++; break;
//				case '\n': stillGoing = false; break;
//				case '2': base += 2; used++; break;
//				case '-': base += 1; used++; break;
//				default: stillGoing = false; break;
//			}
//		}
//	}
	{
//		base++;
		bool stillGoing = true;
		int offset = 1;
		while (stillGoing)
		{
			switch (input[offset++])
			{
				case ' ': used++; break;
				case '\n': stillGoing = false; break;
				case '2': base += 2; used++; break;
				case '-': base += 1; used++; break;
				default: stillGoing = false; break;
			}
		}
	}
	return used;
}

int GetNextRubicCubeMove(const char *input, int &base)
{
	int used = 0;
	if (isdigit(input[0])) // this is our move notation - numeric
	{
		int curr = 0;
		base = input[curr]-'0';
		curr++;
		if (isdigit(input[curr]))
		{
			base = base*10+input[curr]-'0';
			curr++;
		}
		while (!isdigit(input[curr]) && input[curr] != '\n' && input[curr] != 0)
		{
			curr++;
		}
		return curr;
	}
	switch (input[0])
	{
		case 'F': used = 1; base = 2*3; break;
		case 'B': used = 1; base = 3*3; break;
		case 'L': used = 1; base = 4*3; break;
		case 'R': used = 1; base = 5*3; break;
		case 'U': used = 1; base = 0*3; break;
		case 'D': used = 1; base = 1*3; break;
		default: break;
	}
	if (used == 0)
		return 0;
	if (input[0] != 'U')
	{
		bool stillGoing = true;
		int offset = 1;
		while (stillGoing)
		{
			switch (input[offset++])
			{
				case ' ': used++; break;
				case '\n': stillGoing = false; break;
				case '2': base += 2; used++; break;
				case '-': base += 1; used++; break;
				default: stillGoing = false; break;
			}
		}
	}
	else {
		base++;
		bool stillGoing = true;
		int offset = 1;
		while (stillGoing)
		{
			switch (input[offset++])
			{
				case ' ': used++; break;
				case '\n': stillGoing = false; break;
				case '2': base += 1; used++; break;
				case '-': base -= 1; used++; break;
				default: stillGoing = false; break;
			}
		}
	}
	return used;
}

void GetKorfRubikInstance(RCState &start, int which)
{
	assert(which >= 0 && which < 10);
	RC c;
	
	char *string = KorfInstances[which];
	start.Reset();
	
	//	if (result == 0)
	//	{
	//		printf("No more entries found; exiting.\n");
	//		exit(0);
	//	}
	int index = 0;
	string[maxStrLength-1] = 0;
	if (strlen(string) == maxStrLength-1)
	{
		printf("Warning: input hit maximum string length!\n");
		exit(0);
	}
	while (true)
	{
		int act;
		int cnt = GetNextRCMove(&string[index], act);
		if (cnt == 0)
		{
			break;
		}
		else {
			index += cnt;
		}
		c.ApplyAction(start, act);
	}
}

void GetKorfRubikInstance(RubiksState &start, int which)
{
	const int maxStrLength = 1024;
	assert(which >= 0 && which < 10);
	RubiksCube c;
	
	char *string = KorfInstances[which];
	start.Reset();
	
	//	if (result == 0)
	//	{
	//		printf("No more entries found; exiting.\n");
	//		exit(0);
	//	}
	int index = 0;
	string[maxStrLength-1] = 0;
	if (strlen(string) == maxStrLength-1)
	{
		printf("Warning: input hit maximum string length!\n");
		exit(0);
	}
	while (true)
	{
		int act;
		int cnt = GetNextRubicCubeMove(&string[index], act);
		if (cnt == 0)
		{
			break;
		}
		else {
			index += cnt;
		}
		c.ApplyAction(start, act);
	}
}

const char *GetStringFromMove(int move)
{
	const char *str[] = {"U", "D", "L", "R", "B", "F", "U-", "D-", "L-", "R-", "B-", "F-", "U2", "D2", "L2", "R2", "B2", "F2"};
	for (int x = 0; x < 18; x++)
	{
		int act;
		GetNextRubicCubeMove(str[x], act);
		if (act == move)
			return str[x];
	}
	return "?";
}

void GetRandomN(RubiksState &start, int N, int which)
{
	start.Reset();
	RubiksCube c;
	srandom(which);
	std::vector<RubiksAction> acts;
	c.SetPruneSuccessors(true);
	for (int x = 0; x < N; x++)
	{
		c.GetActions(start, acts);
		c.ApplyAction(start, acts[random()%acts.size()]);
	}
}

void GetRandomN(RCState &start, int N, int which)
{
	start.Reset();
	RC c;
	srandom(which);
	std::vector<RubiksAction> acts;
	c.SetPruneSuccessors(true);
	for (int x = 0; x < N; x++)
	{
		c.GetActions(start, acts);
		c.ApplyAction(start, acts[random()%acts.size()]);
	}
}


void GetDepth20(RubiksState &start, int which)
{
	const int maxStrLength = 1024;
	char string[10][maxStrLength] = //"U R2 F B R B2 R U2 L B2 R U- D- R2 F R- L B2 U2 F2";
	{
		"B2 L B2 R- F- U- B- L D- F- L U L2 B2 L- D2 B2 D2 R2 B2",
		"R U2 R D2 R2 B2 L- D- B- F U B- R- U2 L- D R2 F- U2 L2",
		"D2 R2 F2 D2 F2 D2 R- F2 D- L2 R B L- F U R- B F2 R2 F-",
		"D- F- U B2 R2 F R- U2 B- L D F R D2 R2 L2 D- R2 F2 D-",
		"U2 R2 F2 D- U F2 U2 B U B- R U- F L B R- F L2 D- B",
		"D B2 D- B2 R2 D- R2 U L R- D B- D R F- D2 R2 U- F- R",
		"B D- L- F- L F B U- D2 F- R2 B- U F2 R- L U2 R2 F2 B2",
		"U2 L- U2 F2 L- R D2 L2 B- D2 L F- R- U- L U2 F- D- R B",
		"F- L B2 R U- B- L U2 D3 F L- R2 U2 D2 B2 R2 D R2 L2 F2",
		"U2 R2 D2 B U2 B- F D- B- R- D U2 B2 F2 R- D- B U- F- R2"
	};
	
	RubiksCube c;
	
	start.Reset();
	
	int index = 0;
	while (true)
	{
		int act;
		int cnt = GetNextRubicCubeMove(&string[which][index], act);
		if (cnt == 0)
		{
			break;
		}
		else {
			index += cnt;
		}
		c.ApplyAction(start, act);
	}
}

void GetSuperFlip(RubiksState &start)
{
	RubiksCube c;
	const int maxStrLength = 64;
	char string[maxStrLength] = "U R2 F B R B2 R U2 L B2 R U- D- R2 F R- L B2 U2 F2";
	
	start.Reset();
	
	int index = 0;
	string[maxStrLength-1] = 0;
	if (strlen(string) == maxStrLength-1)
	{
		printf("Warning: input hit maximum string length!\n");
		exit(0);
	}
	while (true)
	{
		int act;
		int cnt = GetNextRubicCubeMove(&string[index], act);
		if (cnt == 0)
		{
			break;
		}
		else {
			index += cnt;
		}
		c.ApplyAction(start, act);
	}
}

void GetSuperFlip(RCState &start)
{
	RC c;
	const int maxStrLength = 64;
	char string[maxStrLength] = "U R2 F B R B2 R U2 L B2 R U- D- R2 F R- L B2 U2 F2";
	
	start.Reset();
	
	int index = 0;
	string[maxStrLength-1] = 0;
	if (strlen(string) == maxStrLength-1)
	{
		printf("Warning: input hit maximum string length!\n");
		exit(0);
	}
	while (true)
	{
		int act;
		int cnt = GetNextRCMove(&string[index], act);
		if (cnt == 0)
		{
			break;
		}
		else {
			index += cnt;
		}
		c.ApplyAction(start, act);
	}
}
}

