#include <cstdlib>
#include "MNPuzzleRunner.h"
#include "Common.h"
#include "TemplateAStar.h"


void InstallHandlers();
int MyCLHandler(char *argument[], int maxNumArgs);
void MyFrameHandler(unsigned long windowID, unsigned int viewport, void*);
void TestParallelIDA_MD();
void TestParallelIDA_PDB();

MNPuzzleRunner runner;
void TestWAPDB(bool PDB);

int main(int argc, char* argv[])
{
//	TestWAPDB(false);
	TestWAPDB(true);
	return 0;
	TestParallelIDA_PDB();
	TestParallelIDA_MD();

	InstallHandlers();
	RunHOGGUI(argc, argv, 640, 640);
	
	return 0;
}

void InstallHandlers() {
	InstallCommandLineHandler(MyCLHandler, "-all", "-all", "Run all tests");
	InstallCommandLineHandler(MyCLHandler, "-bfs", "-bfs", "Runs BFS");
	InstallCommandLineHandler(MyCLHandler, "-dfs", "-dfs", "Runs DFS");
	InstallCommandLineHandler(MyCLHandler, "-p", "-p <start> <end>", "Hash two integers to get a start and end state to path find in");
	InstallCommandLineHandler(MyCLHandler, "-o", "-o <fileanem>", "Name of log file");
	InstallFrameHandler(MyFrameHandler, 0, NULL);
}

void MyFrameHandler(unsigned long windowID, unsigned int viewport, void*) {
	runner.runTests();
	exit(0);
}

int MyCLHandler(char *argument[], int maxNumArgs) 
{
	if (strcmp(argument[0], "-all") == 0) 
	{
		runner.addAllTests();
	}
	else if (strcmp(argument[0], "-bfs") == 0)
	{
		runner.addTest("bfs");
	}
	else if (strcmp(argument[0], "-dfs") == 0)
	{
		runner.addTest("dfs");
	}
	else if (strcmp(argument[0], "-o") == 0)
	{
		if (maxNumArgs > 1) 
		{
			runner.setLogFile(argument[1]);
			return 2;
		} 
		else {
			std::cout << "failed -o <file>: missing file name\n";
			exit(0);
		}
	} 
	else if (strcmp(argument[0], "-p") == 0)
	{
		if (maxNumArgs > 2) 
		{
			char* end1 = argument[1];
			char* end2 = argument[2];
			while (*end1 != '\0') {
				++end1;
			}
			while (*end2 != '\0') {
				++end2;
			}
			runner.addStates(std::strtoull(argument[1], &end1, 10), std::strtoull(argument[2], &end2, 10));
			
			return 3;
		} 
		else {
			std::cout << "failed -p <start> <end>; missing hash values\n";
			exit(0);
		}
	}
	
	return 1;
}

#include "ParallelIDAStar.h"
#include "STPInstances.h"

void TestParallelIDA_MD()
{
	MNPuzzle<4,4> mnp44;
	MNPuzzleState<4,4> start, goal;
	std::vector<slideDir> actions;
	ParallelIDAStar<MNPuzzle<4,4>, MNPuzzleState<4,4>, slideDir> pida;

	Timer t;
	for (int x = 0; x < 100; x++)
	{
		start = STP::GetKorfInstance(x);
		goal.Reset();
		t.StartTimer();
		pida.GetPath(&mnp44, start, goal, actions);
		t.EndTimer();
		printf("Problem %d solved in %f seconds. %llu expanded. Solution length %zu actions\n", x+1,
			   t.GetElapsedTime(), pida.GetNodesExpanded(), actions.size());
	}
}

#include "LexPermutationPDB.h"
void TestParallelIDA_PDB()
{
	MNPuzzle<4,4> mnp44;
	MNPuzzleState<4,4> start, goal;
	std::vector<slideDir> actions;
	ParallelIDAStar<MNPuzzle<4,4>, MNPuzzleState<4,4>, slideDir> pida;
	std::vector<int> p1 = {0, 1, 2, 3, 4, 5, 6, 7};
	std::vector<int> p2 = {0, 8, 9, 10, 11, 12, 13, 14, 15};
	goal.Reset();
	
	STPLexPDB<4, 4> pdb1(&mnp44, goal, p1);
	STPLexPDB<4, 4> pdb2(&mnp44, goal, p2);

	// Need to set pattern to property get additive g-costs
	mnp44.SetPattern(p1);
	pdb1.BuildAdditivePDB(goal, 16);
	mnp44.SetPattern(p2);
	pdb2.BuildAdditivePDB(goal, 16);
	
	Heuristic<MNPuzzleState<4,4>> h;
	h.lookups.push_back({kAddNode, 1, 3});
	h.lookups.push_back({kLeafNode, 0, 0});
	h.lookups.push_back({kLeafNode, 1, 1});
	
	h.heuristics.push_back(&pdb1);
	h.heuristics.push_back(&pdb2);
	
	pida.SetHeuristic(&h);
	
	Timer t;
	for (int x = 0; x < 100; x++)
	{
		start = STP::GetKorfInstance(x);
		goal.Reset();
		t.StartTimer();
		pida.GetPath(&mnp44, start, goal, actions);
		t.EndTimer();
		printf("Problem %d solved in %f seconds. %llu expanded. Solution length %zu actions\n", x+1,
			   t.GetElapsedTime(), pida.GetNodesExpanded(), actions.size());
	}
}



void TestWAPDB(bool PDB)
{
	MNPuzzle<4,4> mnp44;
	MNPuzzleState<4,4> start, goal;
	std::vector<slideDir> actions;
	std::vector<MNPuzzleState<4,4>> path;
	TemplateAStar<MNPuzzleState<4,4>, slideDir, MNPuzzle<4,4>> astar;
	std::vector<int> p1 = {0, 1, 2, 3, 4, 5, 6, 7};
	std::vector<int> p2 = {0, 8, 9, 10, 11, 12, 13, 14, 15};
	goal.Reset();
	
	STPLexPDB<4, 4> pdb1(&mnp44, goal, p1);
	STPLexPDB<4, 4> pdb2(&mnp44, goal, p2);
	Heuristic<MNPuzzleState<4,4>> h;

	if (PDB)
	{
		// Need to set pattern to property get additive g-costs
		mnp44.SetPattern(p1);
		pdb1.BuildAdditivePDB(goal, 16);
		mnp44.SetPattern(p2);
		pdb2.BuildAdditivePDB(goal, 16);
		
		h.lookups.push_back({kAddNode, 1, 3});
		h.lookups.push_back({kLeafNode, 0, 0});
		h.lookups.push_back({kLeafNode, 1, 1});
		
		h.heuristics.push_back(&pdb1);
		h.heuristics.push_back(&pdb2);
		
		astar.SetHeuristic(&h);
	}
	
	Timer t;
	for (float w = 9; w >= 1.05; w=(w-1)/2.0f+1) // weight
//		for (float w = 9; w >= 1.2; w=(w-1)/2.0f+1) // weight
	//for (float w = 1.25; w <= 10; w=(w-1)*2+1) // weight
	{
		for (float alpha = 0; flesseq(alpha, 1); alpha += 0.25f*0.5f) // alpha
		{
			for (int z = 0; z < 4; z++) // priority functions
			{
				switch (z)
				{
					case 0:
					{
						// 0: XDP/XUP            (no reopenings)
						float w1 = 1.0f+alpha*(2.0f*w-2.0f);
						float w2 = (2*w-1)-alpha*(2.0f*w-2.0f);
						astar.SetReopenNodes(false);
						astar.SetWeight(w);
						astar.SetPhi([=](double h,double g){
							if (g <= w1*h) return (g+w1*h)/w1;
							else return (g+w2*h)/w;
						});
					}
						break;
					case 1:
					{
						// 1: XDP/XUP-0.5 offset (no reopenings)
						float w1 = 1.0f+alpha*(2.0f*w-2.0f);
						float w2 = (2*w-1)-alpha*(2.0f*w-2.0f);
						astar.SetReopenNodes(false);
						astar.SetWeight(w);
						astar.SetPhi([=](double h,double g){
							if (3*g <= w1*h) return (g+w1*h)/w1;
							else if (g <= h*(4*w-w1)) return 4*(g+w2*h)/(w1+3*w2);
							else return (g+w1*h)/w;
						});
					}
						break;
					case 2:
					{
						// 0: XDP/XUP            (no reopenings)
						float w1 = 1.0f+alpha*(2.0f*w-2.0f);
						float w2 = (2*w-1)-alpha*(2.0f*w-2.0f);
						astar.SetReopenNodes(true);
						astar.SetWeight(w);
						astar.SetPhi([=](double h,double g){
							if (g <= w1*h) return (g+w1*h)/w1;
							else return (g+w2*h)/w;
						});
					}
						break;
					case 3:
					{
						// 3: XDP/XUP-0.5 offset (   reopenings)
						float w1 = 1.0f+alpha*(2.0f*w-2.0f);
						float w2 = (2*w-1)-alpha*(2.0f*w-2.0f);
						astar.SetReopenNodes(true);
						astar.SetWeight(w);
						astar.SetPhi([=](double h,double g){
							if (3*g <= w1*h) return (g+w1*h)/w1;
							else if (g <= h*(4*w-w1)) return 4*(g+w2*h)/(w1+3*w2);
							else return (g+w1*h)/w;
						});
					}
						break;
				}
				printf("Starting w=%1.2f, alpha=%1.2f, alg = %d\n", w, alpha, z);
				for (int x = 0; x < 100; x++)
				{
					start = STP::GetKorfInstance(x);
					goal.Reset();
					t.StartTimer();
					astar.GetPath(&mnp44, start, goal, path);
					t.EndTimer();
					printf("[Re-open: %s. w = %1.2f. alpha=%1.2f. Alg: %d.] Problem %d solved in %f seconds. %llu expanded. Solution length %zu states\n",
						   astar.GetReopenNodes() ? "yes" : "no", w, alpha, z,
						   x+1,
						   t.GetElapsedTime(), astar.GetNodesExpanded(), path.size());
				}
			}
		}
	}
}
