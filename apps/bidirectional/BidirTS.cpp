//
//  BidirPancake.cpp
//  hog2 glut
//
//  Created by Nathan Sturtevant on 2/7/17.
//  Copyright © 2017 University of Denver. All rights reserved.
//

#include "BidirPancake.h"
#include "TopSpin.h"
#include "TemplateAStar.h"
#include "NBS.h"
#include "IDAStar.h"
#include "MM.h"
#include "BSStar.h"
#include "WeightedVertexGraph.h"
#include "HeuristicError.h"
#include "MR1PermutationPDB.h"

const int N = 12;
const int k = 4;

void TestTSRandom();

typedef MR1PermutationPDB<TopSpinState<N>, TopSpinAction, TopSpin<N, k>> TSPDB;
void MakePDBs(const TopSpinState<N> g, Heuristic<TopSpinState<N>> &h, TopSpin<N, k> &ts, bool small = false)
{
//	std::vector<int> p1 = {0,1,2,3,4};
//	std::vector<int> p2 = {5,6,7,8,9};
//	std::vector<int> p3 = {10,11,12,13,14};
	std::vector<int> p1 = {0,1,2,3};
	std::vector<int> p2 = {4,5,6,7};
	std::vector<int> p3 = {8,9,10,11};
	//	std::vector<int> p4 = {0,12,13,14,15};
	//	mnp.StoreGoal(g);
	if (small)
	{
		p1.pop_back();
		p2.pop_back();
		p3.pop_back();
	}
	TSPDB *pdb1 = new TSPDB(&ts, g, p1);
	TSPDB *pdb2 = new TSPDB(&ts, g, p2);
	TSPDB *pdb3 = new TSPDB(&ts, g, p3);
	//	STPPDB *pdb4 = new STPPDB(&mnp, g, p4);
	pdb1->BuildPDB(g, std::thread::hardware_concurrency());
	pdb2->BuildPDB(g, std::thread::hardware_concurrency());
	pdb3->BuildPDB(g, std::thread::hardware_concurrency());
	//	pdb4->BuildPDB(g, std::thread::hardware_concurrency());
	h.lookups.resize(0);
	h.lookups.push_back({kMaxNode, 1, 3});
	h.lookups.push_back({kLeafNode, 0, 0});
	h.lookups.push_back({kLeafNode, 1, 1});
	h.lookups.push_back({kLeafNode, 2, 2});
	h.heuristics.resize(0);
	h.heuristics.push_back(pdb1);
	h.heuristics.push_back(pdb2);
	h.heuristics.push_back(pdb3);
}


void TestTopspin()
{
	TestTSRandom();
	exit(0);
}

void TestTSRandom()
{
	srandom(2017218);
	TopSpinState<N> start;
	TopSpinState<N> original;
	TopSpinState<N> goal;
	TopSpin<N, k> ts;
	ts.SetPruneSuccessors(false);
	std::vector<TopSpinState<N>> nbsPath;
	std::vector<TopSpinState<N>> bsPath;
	std::vector<TopSpinState<N>> astarPath;
	std::vector<TopSpinState<N>> mmPath;
	std::vector<TopSpinAction> idaPath;
	Timer t1, t2, t3, t4, t5;
	
	Heuristic<TopSpinState<N>> h_f;
	MakePDBs(goal, h_f, ts);

	for (int count = 0; count < 50; count++)
	{
		srandom(random());
		
		goal.Reset();
		original.Reset();
		for (int x = 0; x < N; x++)
			std::swap(original.puzzle[x], original.puzzle[x+random()%(N-x)]);
		
		printf("Problem %d of %d\n", count+1, 50);
		std::cout << original << "\n";
		start = original;

		Heuristic<TopSpinState<N>> h_b;
		MakePDBs(original, h_b, ts);

		if (1)
		{
			goal.Reset();
			BidirectionalProblemAnalyzer<TopSpinState<N>, TopSpinAction, TopSpin<N, k>> p(start, goal, &ts, &h_f, &h_b);
		}
		// A*
		if (0)
		{
			TemplateAStar<TopSpinState<N>, TopSpinAction, TopSpin<N, k>> astar;
			start = original;
			t1.StartTimer();
			astar.GetPath(&ts, start, goal, astarPath);
			t1.EndTimer();
			printf("A* found path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n", ts.GetPathLength(astarPath),
				   astar.GetNodesExpanded(), astar.GetNecessaryExpansions(), t1.GetElapsedTime());
		}
		
		// NBS-e
		if (1)
		{
			NBS<TopSpinState<N>, TopSpinAction, TopSpin<N, k>, NBSQueue<TopSpinState<N>, 1>> nbs;
			goal.Reset();
			start = original;
			t2.StartTimer();
			nbs.GetPath(&ts, start, goal, &h_f, &h_b, nbsPath);
			t2.EndTimer();
			printf("NBS-e found path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed; %f meeting\n", ts.GetPathLength(nbsPath),
				   nbs.GetNodesExpanded(), nbs.GetNecessaryExpansions(), t2.GetElapsedTime(), nbs.GetMeetingPoint());
		}

		if (1)
		{
			NBS<TopSpinState<N>, TopSpinAction, TopSpin<N, k>, NBSQueue<TopSpinState<N>, 0>> nbs;
			goal.Reset();
			start = original;
			t2.StartTimer();
			nbs.GetPath(&ts, start, goal, &h_f, &h_b, nbsPath);
			t2.EndTimer();
			printf("NBS-noe found path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed; %f meeting\n", ts.GetPathLength(nbsPath),
				   nbs.GetNodesExpanded(), nbs.GetNecessaryExpansions(), t2.GetElapsedTime(), nbs.GetMeetingPoint());
		}

		// BS*
		if (0)
		{
			BSStar<TopSpinState<N>, TopSpinAction, TopSpin<N, k>> bs;
			goal.Reset();
			start = original;
			t2.StartTimer();
			bs.GetPath(&ts, start, goal, &ts, &ts, bsPath);
			t2.EndTimer();
			printf("BS* found path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n", ts.GetPathLength(bsPath),
				   bs.GetNodesExpanded(), bs.GetNecessaryExpansions(), t2.GetElapsedTime());
		}
		
		// IDA*
		if (0)
		{
			IDAStar<TopSpinState<N>, TopSpinAction, false> idastar;
			goal.Reset();
			start = original;
			t3.StartTimer();
			idastar.GetPath(&ts, start, goal, idaPath);
			t3.EndTimer();
			printf("IDA* found path length %ld; %llu expanded; %llu generated; %1.2fs elapsed\n", idaPath.size(),
				   idastar.GetNodesExpanded(), idastar.GetNodesTouched(), t3.GetElapsedTime());
		}
		
		// MM
		if (0)
		{
			MM<TopSpinState<N>, TopSpinAction, TopSpin<N, k>> mm;
			goal.Reset();
			start = original;
			t4.StartTimer();
			mm.GetPath(&ts, start, goal, &ts, &ts, mmPath);
			t4.EndTimer();
			printf("MM found path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n", ts.GetPathLength(mmPath),
				   mm.GetNodesExpanded(), mm.GetNecessaryExpansions(), t1.GetElapsedTime());
		}
		
		// MM0
		if (0)
		{
			MM<TopSpinState<N>, TopSpinAction, TopSpin<N, k>> mm;
			ZeroHeuristic<TopSpinState<N>> z;
			goal.Reset();
			start = original;
			t4.StartTimer();
			mm.GetPath(&ts, start, goal, &z, &z, mmPath);
			t4.EndTimer();
			printf("MM0 found path length %1.0f; %llu expanded; %llu necessary; %1.2fs elapsed\n", ts.GetPathLength(mmPath),
				   mm.GetNodesExpanded(), mm.GetNecessaryExpansions(), t1.GetElapsedTime());
		}
		
		while (h_b.heuristics.size() > 0)
		{
			delete h_b.heuristics.back();
			h_b.heuristics.pop_back();
		}
	}
}


