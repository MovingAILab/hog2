//
//  PDBRankingTest.cpp
//  hog2 glut
//
//  Created by Nathan Sturtevant on 10/14/15.
//  Copyright © 2015 University of Denver. All rights reserved.
//

#include "PDBRankingTest.h"
#include "MR1PermutationPDB.h"
#include "LexPermutationPDB.h"
#include "PancakePuzzle.h"
#include "TreePermutationPDB.h"
#include "PermutationPDB.h"
#include "MR1Permutation.h"

enum ranks {
	kLex = 0,
	kMR = 1,
	kTree = 2
};

void FullRankingTest()
{
	PancakePuzzle p(4);
	PancakePuzzleState s(4);
	std::vector<int> pattern = {0, 1, 2, 3};
	MR1PermutationPDB<PancakePuzzleState, PancakePuzzleAction, PancakePuzzle> pdb(&p, s, pattern);
	
	for (int x = 0; x < pdb.GetPDBSize(); x++)
	{
		pdb.GetStateFromPDBHash(x, s);
		uint64_t hash = pdb.GetPDBHash(s);
		std::cout << x << ": " << s << "(" << hash << ")\n";
		if (hash != x)
		{
			std::cout << "Error: " << s << " came from rank " << x << " but ranked back to " << pdb.GetPDBHash(s) << "\n";
			exit(0);
		}
	}
}

void PDBTest()
{
	PancakePuzzle p(10);
	PancakePuzzleState s(10);
	std::vector<int> pattern = {0, 9, 3, 2};
	MR1PermutationPDB<PancakePuzzleState, PancakePuzzleAction, PancakePuzzle> pdb(&p, s, pattern);
	
	for (int x = 0; x < pdb.GetPDBSize(); x++)
	{
		pdb.GetStateFromPDBHash(x, s);
		uint64_t hash = pdb.GetPDBHash(s);
		std::cout << x << ": " << s << "(" << hash << ")\n";
		if (hash != x)
		{
			std::cout << "Error: " << s << " came from rank " << x << " but ranked back to " << pdb.GetPDBHash(s) << "\n";
			exit(0);
		}
	}

}

void TreePDBTest()
{
	PancakePuzzle p(10);
	PancakePuzzleState s(10);
	std::vector<int> pattern = {0, 9, 3, 2};
	TreePermutationPDB<PancakePuzzleState, PancakePuzzleAction, PancakePuzzle> pdb(&p, s, pattern);
	
	for (int x = 0; x < pdb.GetPDBSize(); x++)
	{
		pdb.GetStateFromPDBHash(x, s);
		uint64_t hash = pdb.GetPDBHash(s);
		std::cout << x << ": " << s << "(" << hash << ")\n";
		if (hash != x)
		{
			std::cout << "Error: " << s << " came from rank " << x << " but ranked back to " << pdb.GetPDBHash(s) << "\n";
			exit(0);
		}
	}
	
}


void JointTest()
{
	PancakePuzzle p(6);
	PancakePuzzleState s(6);
	PancakePuzzleState s2(6);
	PancakePuzzleState s3(6);
	std::vector<int> pattern = {0, 1, 2, 3, 4, 5};
	std::vector<int> pattern2 = {3, 1, 4};
	MR1PermutationPDB<PancakePuzzleState, PancakePuzzleAction, PancakePuzzle> pdb(&p, s, pattern);
	MR1PermutationPDB<PancakePuzzleState, PancakePuzzleAction, PancakePuzzle> pdb2(&p, s, pattern2);
	
	for (int x = 0; x < pdb.GetPDBSize(); x++)
	{
		pdb.GetStateFromPDBHash(x, s);
		uint64_t hash = pdb2.GetPDBHash(s);
		pdb2.GetStateFromPDBHash(hash, s2);
		std::cout << x << ": " << s << "\n";
		std::cout << "   " << hash << ": " << s2 << "\n";
		for (int x = 0; x < s.puzzle.size(); x++)
		{
			if (s2.puzzle[x] != -1 && s2.puzzle[x] != s.puzzle[x])
			{
				std::cout << "[MR] Full and pdb state don't match " << s << " " << s2 << "\n";
				exit(0);
			}
		}
	}
}

void JointLexTest()
{
	PancakePuzzle p(6);
	PancakePuzzleState s(6);
	PancakePuzzleState s2(6);
	std::vector<int> pattern = {0, 1, 2, 3, 4, 5};
	std::vector<int> pattern2 = {3, 1, 4};
	LexPermutationPDB<PancakePuzzleState, PancakePuzzleAction, PancakePuzzle> pdb(&p, s, pattern);
	TreePermutationPDB<PancakePuzzleState, PancakePuzzleAction, PancakePuzzle> pdb2(&p, s, pattern2);
	
	for (int x = 0; x < pdb.GetPDBSize(); x++)
	{
		pdb.GetStateFromPDBHash(x, s);
		uint64_t hash = pdb2.GetPDBHash(s);
		pdb2.GetStateFromPDBHash(hash, s2);
		std::cout << x << ": " << s << "\n";
		std::cout << "   " << hash << ": " << s2 << "\n";
		for (int x = 0; x < s.puzzle.size(); x++)
		{
			if (s2.puzzle[x] != -1 && s2.puzzle[x] != s.puzzle[x])
			{
				std::cout << "[MR] Full and pdb state don't match " << s << " " << s2 << "\n";
				exit(0);
			}
		}
	}
}

void FullTimingTest(ranks which, int numElts, int pdbElts)
{
	if (which == kLex)
	{
		Timer t;
		t.StartTimer();
		PancakePuzzle p(numElts);
		PancakePuzzleState s(numElts);
		std::vector<int> pattern;
		for (int x = 0; x < pdbElts; x++)
			pattern.push_back(x);
		
		LexPermutationPDB<PancakePuzzleState, PancakePuzzleAction, PancakePuzzle> pdb(&p, s, pattern);
		uint64_t hash = 0;
		uint64_t count = pdb.GetPDBSize();
		uint64_t step = std::max(1ull, count>>24);
		uint64_t total = 0;
		for (uint64_t x = 0; x < count; x += step)
		{
			total++;
			pdb.GetStateFromPDBHash(x, s);
			hash += pdb.GetPDBHash(s);
		}
		t.EndTimer();
		printf("[Lex %d %d] %1.2f elapsed (%llu - %llu [%llu])\n", pdbElts, numElts, t.GetElapsedTime(), step, count, total);
	}
	else if (which == kMR)
	{
		Timer t;
		t.StartTimer();
		PancakePuzzle p(numElts);
		PancakePuzzleState s(numElts);
		std::vector<int> pattern;
		for (int x = 0; x < pdbElts; x++)
			pattern.push_back(x);
		
		MR1PermutationPDB<PancakePuzzleState, PancakePuzzleAction, PancakePuzzle> pdb(&p, s, pattern);
		uint64_t hash = 0;
		uint64_t count = pdb.GetPDBSize();
		uint64_t step = std::max(1ull, count>>24);
		uint64_t total = 0;
		for (uint64_t x = 0; x < count; x += step)
		{
			total++;
			pdb.GetStateFromPDBHash(x, s);
			hash += pdb.GetPDBHash(s);
		}
		t.EndTimer();
		printf("[MR %d %d] %1.2f elapsed (%llu - %llu [%llu])\n", pdbElts, numElts, t.GetElapsedTime(), step, count, total);
	}
	else if (which == kTree)
	{
		Timer t;
		t.StartTimer();
		PancakePuzzle p(numElts);
		PancakePuzzleState s(numElts);
		std::vector<int> pattern;
		for (int x = 0; x < pdbElts; x++)
			pattern.push_back(x);
		
		TreePermutationPDB<PancakePuzzleState, PancakePuzzleAction, PancakePuzzle> pdb(&p, s, pattern);
		uint64_t hash = 0;
		uint64_t count = pdb.GetPDBSize();
		uint64_t step = std::max(1ull, count>>24);
		uint64_t total = 0;
		for (uint64_t x = 0; x < count; x += step)
		{
			total++;
			pdb.GetStateFromPDBHash(x, s);
			hash += pdb.GetPDBHash(s);
		}
		t.EndTimer();
		printf("[Tree %d %d] %1.2f elapsed (%llu - %llu [%llu])\n", pdbElts, numElts, t.GetElapsedTime(), step, count, total);
	}
}

void RankTimingTest(ranks which, int numElts, int pdbElts)
{
	srandom(181);
	if (which == kLex)
	{
		Timer t;
		t.StartTimer();
		PancakePuzzle p(numElts);
		PancakePuzzleState s(numElts);
		std::vector<int> pattern;
		for (int x = 0; x < pdbElts; x++)
			pattern.push_back(x);
		
		LexPermutationPDB<PancakePuzzleState, PancakePuzzleAction, PancakePuzzle> pdb(&p, s, pattern);
		uint64_t hash = 0;
		uint64_t count = pdb.GetPDBSize();
		uint64_t step = std::max(1ull, count>>24);
		uint64_t total = 0;
		for (uint64_t x = 0; x < count; x += step)
		{
			total++;
			swap(s.puzzle[random()%numElts], s.puzzle[random()%numElts]);
			hash += pdb.GetPDBHash(s);
		}
		t.EndTimer();
		printf("[Lex %d %d] %1.2f elapsed (%llu - %llu [%llu])\n", pdbElts, numElts, t.GetElapsedTime(), step, count, total);
	}
	else if (which == kMR)
	{
		Timer t;
		t.StartTimer();
		PancakePuzzle p(numElts);
		PancakePuzzleState s(numElts);
		std::vector<int> pattern;
		for (int x = 0; x < pdbElts; x++)
			pattern.push_back(x);
		
		MR1PermutationPDB<PancakePuzzleState, PancakePuzzleAction, PancakePuzzle> pdb(&p, s, pattern);
		uint64_t hash = 0;
		uint64_t count = pdb.GetPDBSize();
		uint64_t step = std::max(1ull, count>>24);
		uint64_t total = 0;
		for (uint64_t x = 0; x < count; x += step)
		{
			total++;
			swap(s.puzzle[random()%numElts], s.puzzle[random()%numElts]);
			hash += pdb.GetPDBHash(s);
		}
		t.EndTimer();
		printf("[MR %d %d] %1.2f elapsed (%llu - %llu [%llu])\n", pdbElts, numElts, t.GetElapsedTime(), step, count, total);
	}
	else if (which == kTree)
	{
		Timer t;
		t.StartTimer();
		PancakePuzzle p(numElts);
		PancakePuzzleState s(numElts);
		std::vector<int> pattern;
		for (int x = 0; x < pdbElts; x++)
			pattern.push_back(x);
		
		TreePermutationPDB<PancakePuzzleState, PancakePuzzleAction, PancakePuzzle> pdb(&p, s, pattern);
		uint64_t hash = 0;
		uint64_t count = pdb.GetPDBSize();
		uint64_t step = std::max(1ull, count>>24);
		uint64_t total = 0;
		for (uint64_t x = 0; x < count; x += step)
		{
			total++;
			swap(s.puzzle[random()%numElts], s.puzzle[random()%numElts]);
			hash += pdb.GetPDBHash(s);
		}
		t.EndTimer();
		printf("[Tree %d %d] %1.2f elapsed (%llu - %llu [%llu])\n", pdbElts, numElts, t.GetElapsedTime(), step, count, total);
	}
}

void UnrankTimingTest(ranks which, int numElts, int pdbElts)
{
	if (which == kLex)
	{
		Timer t;
		t.StartTimer();
		PancakePuzzle p(numElts);
		PancakePuzzleState s(numElts);
		std::vector<int> pattern;
		for (int x = 0; x < pdbElts; x++)
			pattern.push_back(x);
		
		LexPermutationPDB<PancakePuzzleState, PancakePuzzleAction, PancakePuzzle> pdb(&p, s, pattern);
		uint64_t hash = 0;
		uint64_t count = pdb.GetPDBSize();
		uint64_t step = std::max(1ull, count>>24);
		uint64_t total = 0;
		for (uint64_t x = 0; x < count; x += step)
		{
			total++;
			pdb.GetStateFromPDBHash(x, s);
		}
		t.EndTimer();
		printf("[Lex %d %d] %1.2f elapsed (%llu - %llu [%llu])\n", pdbElts, numElts, t.GetElapsedTime(), step, count, total);
	}
	else if (which == kMR)
	{
		Timer t;
		t.StartTimer();
		PancakePuzzle p(numElts);
		PancakePuzzleState s(numElts);
		std::vector<int> pattern;
		for (int x = 0; x < pdbElts; x++)
			pattern.push_back(x);
		
		MR1PermutationPDB<PancakePuzzleState, PancakePuzzleAction, PancakePuzzle> pdb(&p, s, pattern);
		uint64_t hash = 0;
		uint64_t count = pdb.GetPDBSize();
		uint64_t step = std::max(1ull, count>>24);
		uint64_t total = 0;
		for (uint64_t x = 0; x < count; x += step)
		{
			total++;
			pdb.GetStateFromPDBHash(x, s);
		}
		t.EndTimer();
		printf("[MR %d %d] %1.2f elapsed (%llu - %llu [%llu])\n", pdbElts, numElts, t.GetElapsedTime(), step, count, total);
	}
	else if (which == kTree)
	{
		Timer t;
		t.StartTimer();
		PancakePuzzle p(numElts);
		PancakePuzzleState s(numElts);
		std::vector<int> pattern;
		for (int x = 0; x < pdbElts; x++)
			pattern.push_back(x);
		
		TreePermutationPDB<PancakePuzzleState, PancakePuzzleAction, PancakePuzzle> pdb(&p, s, pattern);
		uint64_t hash = 0;
		uint64_t count = pdb.GetPDBSize();
		uint64_t step = std::max(1ull, count>>24);
		uint64_t total = 0;
		for (uint64_t x = 0; x < count; x += step)
		{
			total++;
			pdb.GetStateFromPDBHash(x, s);
		}
		t.EndTimer();
		printf("[Tree %d %d] %1.2f elapsed (%llu - %llu [%llu])\n", pdbElts, numElts, t.GetElapsedTime(), step, count, total);
	}
}

void PDBRankingTest()
{
	printf("[MR] Full Rank\n");
	FullRankingTest();
	printf("\n\n[MR] PDB Rank\n");
	PDBTest();
	printf("\n[MR] Mix PDB/Full Rank\n");
	JointTest();
	TreePDBTest();
	printf("\n[Lex] Mix PDB/Full Rank\n");
	JointLexTest();
	printf("Ranking Timing tests\n");
	Timer t;
	t.StartTimer();
	for (int x = 8; x <= 20; x+= 4)
	{
		for (int z = 2; z <= x; z += 2)
		{
			for (int y = 2; y < 3; y++)
			{
				RankTimingTest(ranks(y), x, z);
			}
		}
	}
	printf("Unranking Timing tests\n");
	for (int x = 8; x <= 20; x+= 4)
	{
		for (int z = 2; z <= x; z += 2)
		{
			for (int y = 2; y < 3; y++)
			{
				UnrankTimingTest(ranks(y), x, z);
			}
		}
	}
	t.EndTimer();
	printf("Total time: %1.2f\n", t.GetElapsedTime());
}
