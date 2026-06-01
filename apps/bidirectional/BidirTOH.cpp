//
//  BidirTOH.cpp
//  hog2 glut
//
//  Created by Nathan Sturtevant on 2/14/17.
//  Copyright © 2017 University of Denver. All rights reserved.
//

#include "BidirTOH.h"
#include "TOH.h"
#include "TemplateAStar.h"
#include "NBS.h"
#include "MM.h"
#include "BSStar.h"
#include "WeightedVertexGraph.h"

//// Builds two PDBs according to the sizes, plus 2 smaller pdbs that also cover the state space
//template <int numDisks, int pdb1Disks, int pdb2Disks = numDisks-pdb1Disks>
//Heuristic<TOHState<numDisks>> *BuildPDB(const TOHState<numDisks> &goal)
//{
//	TOH<numDisks> toh;
//	TOH<pdb1Disks> absToh1;
//	TOH<pdb2Disks> absToh2;
//	TOH<pdb2Disks-2> absToh3;
//	TOH<pdb1Disks+2> absToh4;
//	TOHState<pdb1Disks> absTohState1;
//	TOHState<pdb2Disks> absTohState2;
//
//
//	TOHPDB<pdb1Disks, numDisks, pdb2Disks> *pdb1 = new TOHPDB<pdb1Disks, numDisks, pdb2Disks>(&absToh1, goal); // top disks
//	TOHPDB<pdb2Disks, numDisks> *pdb2 = new TOHPDB<pdb2Disks, numDisks>(&absToh2, goal); // bottom disks
//	TOHPDB<pdb2Disks-2, numDisks, pdb1Disks+2> *pdb3 = new TOHPDB<pdb2Disks-2, numDisks, pdb1Disks+2>(&absToh3, goal); // top disks
//	TOHPDB<pdb1Disks+2, numDisks> *pdb4 = new TOHPDB<pdb1Disks+2, numDisks>(&absToh4, goal); // top disks
//	pdb1->BuildPDB(goal, std::thread::hardware_concurrency());
//	pdb2->BuildPDB(goal, std::thread::hardware_concurrency());
//	pdb3->BuildPDB(goal, std::thread::hardware_concurrency());
//	pdb4->BuildPDB(goal, std::thread::hardware_concurrency());
//
//	Heuristic<TOHState<numDisks>> *h = new Heuristic<TOHState<numDisks>>;
//
//	h->lookups.resize(0);
//
//	h->lookups.push_back({kMaxNode, 1, 2});
//	h->lookups.push_back({kAddNode, 3, 2});
//	h->lookups.push_back({kAddNode, 5, 2});
//	h->lookups.push_back({kLeafNode, 0, 0});
//	h->lookups.push_back({kLeafNode, 1, 1});
//	h->lookups.push_back({kLeafNode, 2, 2});
//	h->lookups.push_back({kLeafNode, 3, 3});
//	h->heuristics.resize(0);
//	h->heuristics.push_back(pdb1);
//	h->heuristics.push_back(pdb2);
//	h->heuristics.push_back(pdb3);
//	h->heuristics.push_back(pdb4);
//
//	return h;
//}

//// Builds two PDBs that are symmetric for the standard goal - but not necessarily for random goals
//template <int numDisks, int pdb1Disks, int pdb2Disks = numDisks-pdb1Disks>
//Heuristic<TOHState<numDisks>> *BuildPDB(const TOHState<numDisks> &goal)
//{
//	TOH<numDisks> toh;
//	TOH<pdb1Disks> absToh1;
//	TOH<pdb2Disks> absToh2;
//	TOHState<pdb1Disks> absTohState1;
//	TOHState<pdb2Disks> absTohState2;
//
//
//	TOHPDB<pdb1Disks, numDisks, pdb2Disks> *pdb1 = new TOHPDB<pdb1Disks, numDisks, pdb2Disks>(&absToh1, goal); // top disks
//	TOHPDB<pdb2Disks, numDisks> *pdb2 = new TOHPDB<pdb2Disks, numDisks>(&absToh2, goal); // bottom disks
//	TOHPDB<pdb2Disks, numDisks, pdb1Disks> *pdb3 = new TOHPDB<pdb2Disks, numDisks, pdb1Disks>(&absToh2, goal); // top disks
//	TOHPDB<pdb1Disks, numDisks> *pdb4 = new TOHPDB<pdb1Disks, numDisks>(&absToh1, goal); // bottom disks
//	pdb1->BuildPDB(goal, std::thread::hardware_concurrency());
//	pdb2->BuildPDB(goal, std::thread::hardware_concurrency());
//	pdb3->BuildPDB(goal, std::thread::hardware_concurrency());
//	pdb4->BuildPDB(goal, std::thread::hardware_concurrency());
//
//	Heuristic<TOHState<numDisks>> *h = new Heuristic<TOHState<numDisks>>;
//
//	h->lookups.resize(0);
//
//	h->lookups.push_back({kMaxNode, 1, 2});
//	h->lookups.push_back({kAddNode, 3, 2});
//	h->lookups.push_back({kAddNode, 5, 2});
//	h->lookups.push_back({kLeafNode, 0, 0});
//	h->lookups.push_back({kLeafNode, 1, 1});
//	h->lookups.push_back({kLeafNode, 2, 2});
//	h->lookups.push_back({kLeafNode, 3, 3});
//	h->heuristics.resize(0);
//	h->heuristics.push_back(pdb1);
//	h->heuristics.push_back(pdb2);
//	h->heuristics.push_back(pdb3);
//	h->heuristics.push_back(pdb4);
//
//	return h;
//}

// Builds two PDBs that are symmetric for the standard goal - but not necessarily for random goals
template <int numDisks, int pdb1Disks, int pdb2Disks = numDisks-pdb1Disks>
Heuristic<TOHState<numDisks>> *BuildPDB(const TOHState<numDisks> &goal)
{
	TOH<numDisks> toh;
	TOH<pdb1Disks> absToh1;
	TOH<pdb2Disks> absToh2;
	TOHState<pdb1Disks> absTohState1;
	TOHState<pdb2Disks> absTohState2;
	
	
	TOHPDB<pdb1Disks, numDisks, pdb2Disks> *pdb1 = new TOHPDB<pdb1Disks, numDisks, pdb2Disks>(&absToh1, goal); // top disks
	TOHPDB<pdb2Disks, numDisks> *pdb2 = new TOHPDB<pdb2Disks, numDisks>(&absToh2, goal); // bottom disks
	pdb1->BuildPDB(goal, std::thread::hardware_concurrency());
	pdb2->BuildPDB(goal, std::thread::hardware_concurrency());
	
	Heuristic<TOHState<numDisks>> *h = new Heuristic<TOHState<numDisks>>;
	
	h->lookups.resize(0);
	
	h->lookups.push_back({kAddNode, 1, 2});
	h->lookups.push_back({kLeafNode, 0, 0});
	h->lookups.push_back({kLeafNode, 1, 1});
	h->heuristics.resize(0);
	h->heuristics.push_back(pdb1);
	h->heuristics.push_back(pdb2);
	
	return h;
}


template <int N, int pdb1Disks>
void TestTOH(int first, int last)
{
	TemplateAStar<TOHState<N>, TOHMove, TOH<N>> astar;
	NBS<TOHState<N>, TOHMove, TOH<N>, NBSQueue<TOHState<N>, 0>> nbse0;
	NBS<TOHState<N>, TOHMove, TOH<N>, NBSQueue<TOHState<N>, 1>> nbse1;
	MM<TOHState<N>, TOHMove, TOH<N>> mm;
	BSStar<TOHState<N>, TOHMove, TOH<N>> bs;

	TOH<N> toh;
	TOHState<N> s;
	TOHState<N> g;
	std::vector<TOHState<N>> thePath;
	std::vector<TOHMove> actionPath;
	Heuristic<TOHState<N>> *f;
	Heuristic<TOHState<N>> *b;
	ZeroHeuristic<TOHState<N>> z;
//	g.Reset();
//	f = BuildPDB<N, pdb1Disks>(g);
	
	int table[] = {52058078,116173544,208694125,131936966,141559500,133800745,194246206,50028346,167007978,207116816,163867037,119897198,201847476,210859515,117688410,121633885};
	int table2[] = {145008714,165971878,154717942,218927374,182772845,5808407,19155194,137438954,13143598,124513215,132635260,39667704,2462244,41006424,214146208,54305743};
	for (int count = first; count < last; count++)
	{
		printf("Seed: %d\n", table[count&0xF]^table2[(count>>4)&0xF]);
		srandom(table[count&0xF]^table2[(count>>4)&0xF]);

		s.counts[0] = s.counts[1] = s.counts[2] = s.counts[3] = 0;
		for (int x = N; x > 0; x--)
		{
			int whichPeg = random()%4;
			s.disks[whichPeg][s.counts[whichPeg]] = x;
			s.counts[whichPeg]++;
		}
		b = BuildPDB<N, pdb1Disks>(s);

		g.counts[0] = g.counts[1] = g.counts[2] = g.counts[3] = 0;
		for (int x = N; x > 0; x--)
		{
			int whichPeg = random()%4;
			g.disks[whichPeg][g.counts[whichPeg]] = x;
			g.counts[whichPeg]++;
		}
		// Using canonical goal currently - comment to use random goal
		//g.Reset();
		f = BuildPDB<N, pdb1Disks>(g);

		Timer timer;
	
//		printf("Starting heuristics: %f %f\n", f->HCost(s, g), b->HCost(g, s));
		
		std::cout << s << "\n";
		std::cout << g << "\n";

		if (0)
		{
			printf("-=-=-NBSe0-=-=-\n");
			timer.StartTimer();
			nbse0.GetPath(&toh, s, g, f, b, thePath);
			timer.EndTimer();
			printf("I%d-%d-%d\t%d\t", N, pdb1Disks, count, (int)toh.GetPathLength(thePath));
			printf("%llu nodes\t%llu necessary\t", nbse0.GetNodesExpanded(), nbse0.GetNecessaryExpansions());
			printf("%1.2fs elapsed\n", timer.GetElapsedTime());
		}
		if (0)
		{
			printf("-=-=-NBSe1-=-=-\n");
			timer.StartTimer();
			nbse1.GetPath(&toh, s, g, f, b, thePath);
			timer.EndTimer();
			printf("I%d-%d-%d\t%d\t", N, pdb1Disks, count, (int)toh.GetPathLength(thePath));
			printf("%llu nodes\t%llu necessary\t", nbse1.GetNodesExpanded(), nbse1.GetNecessaryExpansions());
			printf("%1.2fs elapsed\n", timer.GetElapsedTime());
		}
		if (0)
		{
			printf("-=-=-NBS0e1-=-=-\n");
			timer.StartTimer();
			nbse1.GetPath(&toh, s, g, &z, &z, thePath);
			timer.EndTimer();
			printf("I%d-%d-%d\t%d\t", N, pdb1Disks, count, (int)toh.GetPathLength(thePath));
			printf("%llu nodes\t%llu necessary\t", nbse1.GetNodesExpanded(), nbse1.GetNecessaryExpansions());
			printf("%1.2fs elapsed\n", timer.GetElapsedTime());
		}
		if (1)
		{
			BidirectionalProblemAnalyzer<TOHState<N>, TOHMove, TOH<N>>::GetWeightedVertexGraph(s, g, &toh, f, b);
		}
//		if (1)
//		{
//			printf("-=-=-BS-=-=-\n");
//			timer.StartTimer();
//			bs.GetPath(&toh, s, g, f, b, thePath);
//			timer.EndTimer();
//			printf("I%d-%d-%d\t%d\t", N, pdb1Disks, count, (int)toh.GetPathLength(thePath));
//			printf("%llu nodes\t%llu necessary\t", bs.GetNodesExpanded(), bs.GetNecessaryExpansions());
//			printf("%1.2fs elapsed\n", timer.GetElapsedTime());
//		}
//		if (1)
//		{
//			printf("-=-=-MM-=-=-\n");
//			timer.StartTimer();
//			mm.GetPath(&toh, s, g, f, b, thePath);
//			timer.EndTimer();
//			printf("I%d-%d-%d\t%d\t", N, pdb1Disks, count, (int)toh.GetPathLength(thePath));
//			printf("%llu nodes\t%llu necessary\t", mm.GetNodesExpanded(), mm.GetNecessaryExpansions());
//			printf("%1.2fs elapsed\n", timer.GetElapsedTime());
//		}
//		if (1)
//		{
//			printf("-=-=-A*-=-=-\n");
//			astar.SetHeuristic(f);
//			timer.StartTimer();
//			astar.GetPath(&toh, s, g, thePath);
//			timer.EndTimer();
//			printf("I%d-%d-%d\t%d\t", N, pdb1Disks, count, (int)toh.GetPathLength(thePath));
//			printf("%llu nodes\t%llu necessary\t", astar.GetNodesExpanded(), astar.GetNecessaryExpansions());
//			printf("%1.2fs elapsed\n", timer.GetElapsedTime());
//		}
//		while (b->heuristics.size() > 0)
//		{
//			delete b->heuristics.back();
//			b->heuristics.pop_back();
//		}
//		delete b;
//		while (f->heuristics.size() > 0)
//		{
//			delete f->heuristics.back();
//			f->heuristics.pop_back();
//		}
//		delete f;
	}

}

void TOHTest()
{
//	TestTOH<14, 2>(0, 50);
	TestTOH<14, 4>(0, 50);
//	TestTOH<14, 5>(0, 50);
//	TestTOH<14, 6>(0, 50);
//	TestTOH<14, 7>(0, 50);
//	TestTOH<14, 8>(0, 50);
//	TestTOH<14, 9>(0, 50);
//	TestTOH<14, 10>(0, 50);
//	const int numDisks = 16; // [disks - 2] (4^14 - 256 million)
//	const int pdb1Disks = 10;
//	const int pdb2Disks = 6;
//	TOH<numDisks> toh;
//	TOHState<numDisks> s, g;
//	
//	TOHState<numDisks> goal;
//	TOH<pdb1Disks> absToh1;
//	TOH<pdb2Disks> absToh2;
//	TOHState<pdb1Disks> absTohState1;
//	TOHState<pdb2Disks> absTohState2;
//
//	TOHPDB<pdb1Disks, numDisks, pdb2Disks> pdb1(&absToh1); // top disks
//	TOHPDB<pdb2Disks, numDisks> pdb2(&absToh2); // bottom disks
//
//	ZeroHeuristic<TOHState<numDisks>> z;
//	
//	goal.Reset();
//	pdb1.BuildPDB(goal, std::thread::hardware_concurrency());
//	pdb2.BuildPDB(goal, std::thread::hardware_concurrency());
//	
////	s.Reset();
////	goal.Reset();
////	for (int x = 0; x < 100; x++)
////	{
////		std::vector<TOHMove> actionPath;
////		for (int x = 0; x < 20000; x++)
////		{
////			toh.GetActions(s, actionPath);
////			toh.ApplyAction(s, actionPath[random()%actionPath.size()]);
////		}
////		std::cout << s << "\n";
////		std::cout << "H1: " << pdb1.HCost(s, goal) << "\n";
////		std::cout << "H2: " << pdb2.HCost(s, goal) << "\n";
////	}
////	exit(0);
//	goal.Reset();
//	Heuristic<TOHState<numDisks>> h;
//	
//	h.lookups.resize(0);
//	h.lookups.push_back({kAddNode, 1, 2});
//	h.lookups.push_back({kLeafNode, 0, 0});
//	h.lookups.push_back({kLeafNode, 1, 1});
//	h.heuristics.resize(0);
//	h.heuristics.push_back(&pdb1);
//	h.heuristics.push_back(&pdb2);
//	printf("-=-=-=-==-=-=-=-=-\n");
//	printf("With %d and %d\n", pdb1Disks, pdb2Disks);
//	TestTOH<numDisks>(&h, 0, 10);
//
//	printf("-=-=-=-==-=-=-=-=-\n");
//	h.heuristics[1] = &z;
//	printf("With just %d\n", pdb1Disks);
//	TestTOH<numDisks>(&h, 0, 10);
	exit(0);
}
