//
//  BidirSTP.cpp
//  hog2 glut
//
//  Created by Nathan Sturtevant on 1/30/17.
//  Copyright © 2017 University of Denver. All rights reserved.
//

#include "BidirSTP.h"
#include "MNPuzzle.h"
#include "NBS.h"
#include "IDAStar.h"
#include "MM.h"
#include "BSStar.h"
#include "TemplateAStar.h"
#include "WeightedVertexGraph.h"
#include "STPInstances.h"
#include "LexPermutationPDB.h"
#include "MR1PermutationPDB.h"

typedef MR1PermutationPDB<MNPuzzleState<4, 4>, slideDir, MNPuzzle<4, 4>> STPPDB;
void MakePDBs(MNPuzzleState<4, 4> g, Heuristic<MNPuzzleState<4, 4>> &h, MNPuzzle<4,4> &mnp)
{
	std::vector<int> p1 = {0,1,2,3,4,5};
	std::vector<int> p2 = {0,10,11,12,13,14,15};
	std::vector<int> p3 = {0,6,7,8,9};
//	std::vector<int> p4 = {0,12,13,14,15};
//	mnp.StoreGoal(g);
	STPPDB *pdb1 = new STPPDB(&mnp, g, p1);
	STPPDB *pdb2 = new STPPDB(&mnp, g, p2);
	STPPDB *pdb3 = new STPPDB(&mnp, g, p3);
//	STPPDB *pdb4 = new STPPDB(&mnp, g, p4);
	pdb1->BuildPDB(g, std::thread::hardware_concurrency());
	pdb2->BuildPDB(g, std::thread::hardware_concurrency());
	pdb3->BuildPDB(g, std::thread::hardware_concurrency());
//	pdb4->BuildPDB(g, std::thread::hardware_concurrency());
	h.lookups.resize(0);
	h.lookups.push_back({kMaxNode, 1, 4});
	h.lookups.push_back({kLeafNode, 0, 0});
	h.lookups.push_back({kLeafNode, 1, 1});
	h.lookups.push_back({kLeafNode, 2, 2});
	h.lookups.push_back({kLeafNode, 3, 3});
//	h.lookups.push_back({kLeafNode, 4, 4});
	h.heuristics.resize(0);
	h.heuristics.push_back(&mnp);
	h.heuristics.push_back(pdb1);
	h.heuristics.push_back(pdb2);
	h.heuristics.push_back(pdb3);
//	h.heuristics.push_back(pdb4);
}

void TestSTP(int algorithm)
{
	NBS<MNPuzzleState<4, 4>, slideDir, MNPuzzle<4,4>> nbs;
	MM<MNPuzzleState<4, 4>, slideDir, MNPuzzle<4,4>> mm;
	BSStar<MNPuzzleState<4, 4>, slideDir, MNPuzzle<4,4>> bs;
	TemplateAStar<MNPuzzleState<4, 4>, slideDir, MNPuzzle<4,4>> astar;
	MNPuzzle<4,4> mnp;
	
	Heuristic<MNPuzzleState<4, 4>> h_f;
	MNPuzzleState<4, 4> start, goal;
	MakePDBs(goal, h_f, mnp);

	
	for (int x = 0; x < 100; x++) // 547 to 540
	{
		Heuristic<MNPuzzleState<4, 4>> h_b;
		printf("Problem %d of %d\n", x+1, 100);
		
		std::vector<MNPuzzleState<4,4>> nbsPath;
		std::vector<MNPuzzleState<4,4>> astarPath;
		Timer t1, t2;

		start = STP::GetKorfInstance(x);
		goal.Reset();
		MakePDBs(start, h_b, mnp);

		if (algorithm == -1 || algorithm == 10) // Optimal Analysis
		{
			start = STP::GetKorfInstance(x);
			goal.Reset();

			std::string t = "/Users/nathanst/bidir/stp/stp_";
			t += std::to_string(x+1);
			
			BidirectionalProblemAnalyzer<MNPuzzleState<4, 4>, slideDir, MNPuzzle<4, 4>> p(start, goal, &mnp, &h_f, &h_b);
//			p.drawFullGraph = true;
//			p.drawProblemInstance = false;
//			p.drawMinimumVC = true;
//			p.drawAllG = false;
//			p.drawStatistics = false;
//			p.SaveSVG((t+"-full.svg").c_str());
//			p.drawFullGraph = false;
//			p.drawProblemInstance = false;
//			p.drawAllG = true;
//			p.drawStatistics = false;
//			p.SaveSVG((t+"-min.svg").c_str());
//			printf("Forward: %d\n", p.GetForwardWork());
//			printf("Backward: %d\n", p.GetBackwardWork());
//			printf("Minimum: %d\n", p.GetMinWork());
//			int maxg = p.GetNumGCosts();
//			p.SaveSVG((t+"-shrunk.svg").c_str(), (maxg+11)/12);

		}

		if (algorithm == 0 || algorithm == 10) // A*
		{
			goal.Reset();
			start = STP::GetKorfInstance(x);
			t1.StartTimer();
			astar.SetHeuristic(&h_f);
			astar.GetPath(&mnp, start, goal, astarPath);
			t1.EndTimer();
			printf("A* found path length %1.0f; %llu expanded; %llu necessary; %llu generated; %1.2fs elapsed\n", mnp.GetPathLength(astarPath),
				   astar.GetNodesExpanded(), astar.GetNecessaryExpansions(), astar.GetNodesTouched(), t1.GetElapsedTime());
		}
		if (algorithm == 1) // BS*
		{
			goal.Reset();
			start = STP::GetKorfInstance(x);
			t2.StartTimer();
			bs.GetPath(&mnp, start, goal, &mnp, &mnp, nbsPath);
			t2.EndTimer();
			printf("BS* found path length %1.0f; %llu expanded; %llu necessary; %llu generated; %1.2fs elapsed\n", mnp.GetPathLength(nbsPath),
				   bs.GetNodesExpanded(), bs.GetNecessaryExpansions(), bs.GetNodesTouched(), t2.GetElapsedTime());
		}
		if (algorithm == 2) // MM
		{
			goal.Reset();
			start = STP::GetKorfInstance(x);
			t2.StartTimer();
			mm.GetPath(&mnp, start, goal, &mnp, &mnp, nbsPath);
			t2.EndTimer();
			printf("MM found path length %1.0f; %llu expanded; %llu necessary; %llu generated; %1.2fs elapsed\n", mnp.GetPathLength(nbsPath),
				   mm.GetNodesExpanded(), mm.GetNecessaryExpansions(), mm.GetNodesTouched(), t2.GetElapsedTime());
		}
		if (algorithm == 3||algorithm == 10) // NBS
		{
			goal.Reset();
			start = STP::GetKorfInstance(x);
			t2.StartTimer();
			nbs.GetPath(&mnp, start, goal, &h_f, &h_b, nbsPath);
			t2.EndTimer();
			printf("NBS found path length %1.0f; %llu expanded; %llu necessary; %llu generated; %1.2fs elapsed\n", mnp.GetPathLength(nbsPath),
				   nbs.GetNodesExpanded(), nbs.GetNecessaryExpansions(), nbs.GetNodesTouched(), t2.GetElapsedTime());
		}
		if (algorithm == 4) // MM0
		{
			ZeroHeuristic<MNPuzzleState<4,4>> z;
			goal.Reset();
			start = STP::GetKorfInstance(x);
			t2.StartTimer();
			mm.GetPath(&mnp, start, goal, &z, &z, nbsPath);
			t2.EndTimer();
			printf("MM found path length %1.0f; %llu expanded; %llu necessary; %llu generated; %1.2fs elapsed\n", mnp.GetPathLength(nbsPath),
				   mm.GetNodesExpanded(), mm.GetNecessaryExpansions(), mm.GetNodesTouched(), t2.GetElapsedTime());
		}
		

		delete h_b.heuristics[1];
		delete h_b.heuristics[2];
		delete h_b.heuristics[3];
//		delete h_b.heuristics[4];
//
//		std::cout << astar.GetNodesExpanded() << "\t" << nbs.GetNodesExpanded() << "\t";
//		std::cout << t1.GetElapsedTime() << "\t" <<  t2.GetElapsedTime() << "\n";
	}
	exit(0);
}

void TestSTPFull()
{
	NBS<MNPuzzleState<4, 4>, slideDir, MNPuzzle<4,4>> nbs;
	MM<MNPuzzleState<4, 4>, slideDir, MNPuzzle<4,4>> mm;
	MNPuzzle<4,4> mnp;
	IDAStar<MNPuzzleState<4,4>, slideDir> ida;
	TemplateAStar<MNPuzzleState<4, 4>, slideDir, MNPuzzle<4,4>> astar;

	for (int x = 0; x < 100; x++) // 547 to 540
	{
		MNPuzzleState<4, 4> start, goal;
		printf("Problem %d of %d\n", x+1, 100);
		
		std::vector<slideDir> idaPath;
		std::vector<MNPuzzleState<4,4>> nbsPath;
		std::vector<MNPuzzleState<4,4>> astarPath;
		std::vector<MNPuzzleState<4,4>> mmPath;
		Timer t1, t2, t3, t4;
		
		goal.Reset();
		start = STP::GetKorfInstance(x);
		t1.StartTimer();
		ida.GetPath(&mnp, start, goal, idaPath);
		t1.EndTimer();
		printf("IDA* found path length %ld; %llu expanded; %1.2fs elapsed\n", idaPath.size(),  ida.GetNodesExpanded(), t1.GetElapsedTime());

		goal.Reset();
		start = STP::GetKorfInstance(x);
		t2.StartTimer();
		astar.GetPath(&mnp, start, goal, astarPath);
		t2.EndTimer();
		printf("A* found path length %ld; %llu expanded; %1.2fs elapsed\n", astarPath.size()-1,  astar.GetNodesExpanded(), t2.GetElapsedTime());

		goal.Reset();
		start = STP::GetKorfInstance(x);
		t3.StartTimer();
		nbs.GetPath(&mnp, start, goal, &mnp, &mnp, nbsPath);
		t3.EndTimer();
		printf("NBS found path length %ld; %llu expanded; %1.2fs elapsed\n", nbsPath.size()-1,  nbs.GetNodesExpanded(), t3.GetElapsedTime());

		goal.Reset();
		start = STP::GetKorfInstance(x);
		t4.StartTimer();
		mm.GetPath(&mnp, start, goal, &mnp, &mnp, mmPath);
		t4.EndTimer();
		printf("MM found path length %ld; %llu expanded; %1.2fs elapsed\n", mmPath.size()-1,  mm.GetNodesExpanded(), t3.GetElapsedTime());


		std::cout << ida.GetNodesExpanded() << "\t" <<  astar.GetNodesExpanded() << "\t" << nbs.GetNodesExpanded() << "\t";
		std::cout << t1.GetElapsedTime() << "\t" <<  t2.GetElapsedTime() << "\t" << t3.GetElapsedTime() << "\n";
		
		//if (!fequal)
		if (nbsPath.size() != idaPath.size()+1)
		{
			std::cout << "error solution cost:\t expected cost\n";
			std::cout << nbsPath.size() << "\t" << idaPath.size() << "\n";
//			double d;
//			for (auto x : correctPath)
//			{
//				astar.GetClosedListGCost(x, d);
//				auto t = nbs.GetNodeForwardLocation(x);
//				auto u = nbs.GetNodeBackwardLocation(x);
//				std::cout << x << " is on " << t << " and " << u << "\n";
//				std::cout << "True g: " << d;
//				if (t != kUnseen)
//					std::cout << " forward g: " << nbs.GetNodeForwardG(x);
//				if (u != kUnseen)
//					std::cout << " backward g: " << nbs.GetNodeBackwardG(x);
//				std::cout << "\n";
//			}
			exit(0);
		}
		
	}
	exit(0);
}
