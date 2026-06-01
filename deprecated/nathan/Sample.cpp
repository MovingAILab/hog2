/*
 *  $Id: sample.cpp
 *  hog2
 *
 *  Created by Nathan Sturtevant on 5/31/05.
 *  Modified by Nathan Sturtevant on 02/29/20.
 *
 * This file is part of HOG2. See https://github.com/nathansttt/hog2 for licensing information.
 *
 */

#include "Common.h"
#include "Sample.h"
#include "UnitSimulation.h"
#include "EpisodicSimulation.h"
#include "MNPuzzle.h"
#include "TopSpin.h"
#include <deque>
#include "IDAStar.h"
#include "TemplateAStar.h"
#include "BurnedPancakePuzzle.h"
#include "PancakePuzzle.h"
#include "SFIDAStar.h"
#include "Fling.h"

void SolveNextProblem(MNPuzzle *mnp1, MNPuzzle *mnp2);
void SolveRandomProblem(MNPuzzle *mnp1, MNPuzzle *mnp2);
void TestPDB();
void BuildPDB();
void TestTopSpinPDB();
void BuildTopSpinPDB();


bool mouseTracking;
int px1, py1, px2, py2;
int absType = 0;

std::vector<PuzzleSimulation *> unitSims;

int main(int argc, char* argv[])
{
	InstallHandlers();
	RunHOGGUI(argc, argv);
}


/**
 * This function is used to allocate the unit simulated that you want to run.
 * Any parameters or other experimental setup can be done at this time.
 */
void CreateSimulation(int id)
{
	Map *map;
	if (gDefaultMap[0] == 0)
	{
		map = new Map(40, 40);
		MakeMaze(map, 1);
	}
	else
		map = new Map(gDefaultMap);

	unitSims.resize(id+1);
	unitSims[id] = new PuzzleSimulation(new MNPuzzle(4, 4));
	unitSims[id]->SetStepType(kMinTime);
}

/**
 * Allows you to install any keyboard handlers needed for program interaction.
 */
void InstallHandlers()
{
	InstallKeyboardHandler(MyDisplayHandler, "Toggle Abstraction", "Toggle display of the ith level of the abstraction", kAnyModifier, '0', '9');
	InstallKeyboardHandler(MyDisplayHandler, "Cycle Abs. Display", "Cycle which group abstraction is drawn", kAnyModifier, '\t');
	InstallKeyboardHandler(MyDisplayHandler, "Pause Simulation", "Pause simulation execution.", kNoModifier, 'p');
	InstallKeyboardHandler(MyDisplayHandler, "Step Simulation", "If the simulation is paused, step forward .1 sec.", kNoModifier, 'o');
	InstallKeyboardHandler(MyDisplayHandler, "Step History", "If the simulation is paused, step forward .1 sec in history", kAnyModifier, '}');
	InstallKeyboardHandler(MyDisplayHandler, "Step History", "If the simulation is paused, step back .1 sec in history", kAnyModifier, '{');
	InstallKeyboardHandler(MyDisplayHandler, "Step Abs Type", "Increase abstraction type", kAnyModifier, ']');
	InstallKeyboardHandler(MyDisplayHandler, "Step Abs Type", "Decrease abstraction type", kAnyModifier, '[');

	InstallKeyboardHandler(MyPDBKeyHandler, "Build PDB", "Build a sliding-tile PDB", kNoModifier, 'b');
	InstallKeyboardHandler(MyPDBKeyHandler, "Test PDB", "Test a sliding-tile PDB", kNoModifier, 't');

	InstallKeyboardHandler(MyPancakeHandler, "Run Burned Pancake", "Run Burned Pancake", kNoModifier, 'r');
	InstallKeyboardHandler(MyFlingHandler, "Run Fling", "Run Fling", kNoModifier, 'f');

	InstallCommandLineHandler(MyCLHandler, "-map", "-map filename", "Selects the default map to be loaded.");
	
	InstallWindowHandler(MyWindowHandler);

	InstallMouseClickHandler(MyClickHandler);
}

void MyWindowHandler(unsigned long windowID, tWindowEventType eType)
{
	if (eType == kWindowDestroyed)
	{
		printf("Window %ld destroyed\n", windowID);
		RemoveFrameHandler(MyFrameHandler, windowID, 0);
	}
	else if (eType == kWindowCreated)
	{
		printf("Window %ld created\n", windowID);
		InstallFrameHandler(MyFrameHandler, windowID, 0);
		CreateSimulation(windowID);
	}
}

void MyFrameHandler(unsigned long windowID, unsigned int viewport, void *)
{
	if ((windowID < unitSims.size()) && (unitSims[windowID] == 0))
		return;

	if (viewport == 0)
	{
		unitSims[windowID]->StepTime(1.0/30.0);
	}
	unitSims[windowID]->OpenGLDraw();
	
}

int MyCLHandler(char *argument[], int maxNumArgs)
{
	if (maxNumArgs <= 1)
		return 0;
	strncpy(gDefaultMap, argument[1], 1024);
	return 2;
}

void MyDisplayHandler(unsigned long windowID, tKeyboardModifier mod, char key)
{
	switch (key)
	{
		case '\t':
			if (mod != kShiftDown)
				SetActivePort(windowID, (GetActivePort(windowID)+1)%GetNumPorts(windowID));
			else
			{
				SetNumPorts(windowID, 1+(GetNumPorts(windowID)%MAXPORTS));
			}
			break;
		case 'p': unitSims[windowID]->SetPaused(!unitSims[windowID]->GetPaused()); break;
		case 'o':
			if (unitSims[windowID]->GetPaused())
			{
				unitSims[windowID]->SetPaused(false);
				unitSims[windowID]->StepTime(1.0/30.0);
				unitSims[windowID]->SetPaused(true);
			}
			break;
		case ']': absType = (absType+1)%3; break;
		case '[': absType = (absType+4)%3; break;
//		case '{': unitSim->setPaused(true); unitSim->offsetDisplayTime(-0.5); break;
//		case '}': unitSim->offsetDisplayTime(0.5); break;
		default:
			break;
	}
}


void MyPDBKeyHandler(unsigned long, tKeyboardModifier, char key)
{
	if (key == 'b')
	{
		BuildTopSpinPDB();
	}
	if (key == 't')
	{
		for (int x = 0; x < 200; x++)
		{
			printf("Testing top spin! (%d of %d)\n", x+1, 200);
			TestTopSpinPDB();
		}
	}
}

void BuildPDB()
{
	MNPuzzle *mnp = new MNPuzzle(4, 4);
	MNPuzzleState s(4, 4);
	std::vector<uint8_t> DB(57657600);
	for (unsigned int x = 0; x < DB.size(); x++)
		DB[x] = 255;
	int entries = 0;
	std::vector<int> PDB(7);
	PDB[0] = 0;
	PDB[1] = 6;//4;//6;
	PDB[2] = 7;//5;//7;
	PDB[3] = 10;//8;//10;
	PDB[4] = 11;//9;//11;
	PDB[5] = 14;//12;//14;
	PDB[6] = 15;//13;//15;
	
	std::cout << s << std::endl;
	std::cout << mnp->GetStateHash(s) << std::endl;
	std::cout << mnp->GetPDBHash(s, PDB) << std::endl;
	
	std::deque<MNPuzzleState> q;
	std::vector<MNPuzzleState> moves;
	q.push_back(s);
	DB[mnp->GetPDBHash(s, PDB)] = 0;
	entries++;
	while (entries < 57657600)
	{
		if ((entries%100) == 0)
			std::cout << entries << std::endl;
		MNPuzzleState next = q.front();
		q.pop_front();
		mnp->GetSuccessors(next, moves);
		for (unsigned int x = 0; x < moves.size(); x++)
		{
			if (DB[mnp->GetPDBHash(moves[x], PDB)] == 255)
			{
				if (std::find(PDB.begin(), PDB.end(), next.puzzle[moves[x].blank]) != PDB.end())
				{
					DB[mnp->GetPDBHash(moves[x], PDB)] = DB[mnp->GetPDBHash(next, PDB)]+1;
					q.push_back(moves[x]);
					entries++;
				}
				else {
					DB[mnp->GetPDBHash(moves[x], PDB)] = DB[mnp->GetPDBHash(next, PDB)]+0;
					q.push_front(moves[x]);
					entries++;
				}
			}
		}
	}
	FILE *f = fopen("/Users/nathanst/PDB1a.db", "w+");
	if (f)
    {
		fwrite(&(DB[0]), sizeof(uint8_t), 57657600, f);
		fclose(f);
    }
}

void TestPDB()
{
	MNPuzzle *mnp = new MNPuzzle(4, 4);
	MNPuzzle *mnp2 = new MNPuzzle(4, 4);
	MNPuzzleState s(4, 4);

	std::vector<int> P1(7);
	P1[0] = 0; P1[1] = 6; P1[2] = 7; P1[3] = 10; P1[4] = 11; P1[5] = 14; P1[6] = 15;
	std::vector<int> P2(7);
	P2[0] = 0; P2[1] = 4; P2[2] = 5; P2[3] = 8; P2[4] = 9; P2[5] = 12; P2[6] = 13;

	//mnp->LoadPDB("/Users/nathanst/PDB1.db", P1, true);
	//mnp->LoadPDB("/Users/nathanst/PDB2.db", P2, true);

//	for (int x = 0; x < 10; x++)
//		SolveRandomProblem(mnp, mnp2);
	while (!feof(stdin))
		SolveNextProblem(mnp, mnp2);
}

void SolveRandomProblem(MNPuzzle *mnp1, MNPuzzle *mnp2)
{
	IDAStar<MNPuzzleState, slideDir> ida;
	std::vector<MNPuzzleState> path1;
	MNPuzzleState s(4, 4);
	MNPuzzleState g(4, 4);
	for (unsigned int x = 0; x < 150; x++)
	{
		std::vector<slideDir> acts;
		mnp1->GetActions(s, acts);
		mnp1->ApplyAction(s, acts[random()%acts.size()]);
//		std::cout << "mnp1 heuristic:" << mnp1->HCost(s, g) << std::endl;
//		std::cout << "mnp2 heuristic:" << mnp2->HCost(s, g) << std::endl;
	}
	std::cout << "Searching from: " << std::endl << s << std::endl << g << std::endl;
	std::cout << "mnp1 heuristic:" << mnp1->HCost(s, g) << std::endl;
	std::cout << "mnp2 heuristic:" << mnp2->HCost(s, g) << std::endl;

	Timer t;
	t.StartTimer();
	ida.GetPath(mnp1, s, g, path1);
	t.EndTimer();
	std::cout << "Path found (mnp1), length " << path1.size() << " time:" << t.GetElapsedTime() << std::endl;
	
	t.StartTimer();
	ida.GetPath(mnp2, s, g, path1);
	t.EndTimer();
	std::cout << "Path found (mnp2), length " << path1.size() << " time:" << t.GetElapsedTime() << std::endl;
}

void SolveNextProblem(MNPuzzle *mnp1, MNPuzzle *mnp2)
{
	IDAStar<MNPuzzleState, slideDir> ida;
	std::vector<slideDir> path1;
	MNPuzzleState s(4, 4);
	MNPuzzleState g(4, 4);
	for (unsigned int x = 0; x < 16; x++)
	{
		int val;
		scanf("%d", &val);
		s.puzzle[x] = val;
		if (val == 0)
			s.blank = val;
	}
	std::cout << "Searching from: " << std::endl << s << std::endl << g << std::endl;
	std::cout << "mnp1 heuristic:" << mnp1->HCost(s, g) << std::endl;
	std::cout << "mnp2 heuristic:" << mnp2->HCost(s, g) << std::endl;
	
	Timer t;
	t.StartTimer();
	ida.GetPath(mnp1, s, g, path1);
	t.EndTimer();
	std::cout << "Path found (mnp1), length " << path1.size() << " time:" << t.GetElapsedTime() << std::endl;
	
	t.StartTimer();
	ida.GetPath(mnp2, s, g, path1);
	t.EndTimer();
	std::cout << "Path found (mnp2), length " << path1.size() << " time:" << t.GetElapsedTime() << std::endl;
}


bool MyClickHandler(unsigned long , int, int, point3d , tButtonType , tMouseEventType )
{
	return false;
}

void BuildTopSpinPDB()
{
	int psize = 12;
	static int PDB = 1;
	PDB++;
	TopSpin *ts = new TopSpin(psize, 4);

	std::vector<int> P1(psize);
	for (int x = 0; x < psize; x++)
		P1[x] = x;
	graphState s = ts->GetState(P1);
	
	std::vector<uint8_t> DB(ts->GetPDBSize(psize, PDB));
	for (unsigned int x = 0; x < DB.size(); x++)
		DB[x] = 255;
	int entries = 0;
	
	std::cout << s << std::endl;
	std::cout << ts->GetStateHash(s) << std::endl;
	std::cout << ts->GetPDBHash(s, PDB) << std::endl;
	std::cout << "PDB size: " << ts->GetPDBSize(psize, PDB) << std::endl;
	
	std::deque<graphState> q;
	std::vector<graphState> moves;
	q.push_back(s);
	DB[ts->GetPDBHash(s, PDB)] = 0;
	entries++;
	while (entries < (int)DB.size())
	{
		graphState next = q.front();
//		printf("Expanding %lu\n", next);
		q.pop_front();
		ts->GetSuccessors(next, moves);
		for (unsigned int x = 0; x < moves.size(); x++)
		{
//			printf("   Child %d: val %lu -- #%d - (db: --) hash: %d\n",
//				   x, moves[x], entries,
//				   ts->GetPDBHash(moves[x], PDB));
			
			if (DB[ts->GetPDBHash(moves[x], PDB)] == 255)
			{
				DB[ts->GetPDBHash(moves[x], PDB)] = DB[ts->GetPDBHash(next, PDB)]+1;
				q.push_back(moves[x]);
				entries++;

				if ((entries%100) == 0)
					std::cout << entries << std::endl;
			}
		}
		if (q.size() == 0)
		{
			printf("Error! q-size is 0!\n");
			break;
		}
	}
	printf("%d of %d entries found\n", entries, (int)DB.size());
	char filename[256];
	sprintf(filename, "/Users/nathanst/TS_%d_%d_%d.db", psize, 4, PDB);
	FILE *f = fopen(filename, "w+");
	if (f)
    {
		fwrite(&(DB[0]), sizeof(uint8_t), DB.size(), f);
		fclose(f);
    }
}

void TestTopSpinPDB()
{
	const int psize = 12;
	TopSpinGraphHeuristic *tsh;
	TopSpin *ts = new TopSpin(psize, 4, tsh = new TopSpinGraphHeuristic(12, 4, 5));
	//TopSpin *ts = new TopSpin(psize, 4, tsh = new TopSpinGraphHeuristic());
	tsh->SetState(ts);

	std::vector<int> P1(psize);
	for (int x = 0; x < psize; x++)
		P1[x] = x;
	graphState g1 = ts->GetState(P1);
	assert(g1 == 0);
	
	std::vector<int> P2;
	P2 = P1;
	graphState s1 = ts->GetState(P2);
	
	std::vector<graphState> path1;
	for (int x = 0; x < 200; x++)
	{
		ts->GetSuccessors(s1, path1);
		s1 = path1[random()%path1.size()];
	}

//	std::vector<graphState> path1;
//	IDAStar<graphState, graphMove> ida;
//	ida.GetPath(ts, s1, g1, path1);	
//	printf("Solution:\n");
//	for (unsigned int x = 0; x < path1.size(); x++)
//		printf("%lu   ", path1[x]);
//	printf("\n");
//	printf("%lu nodes expanded\n", ida.GetNodesExpanded());

	path1.resize(0);
	TemplateAStar<graphState, graphMove, TopSpin> a;
	a.GetPath(ts, s1, g1, path1);
	printf("Solution:\n");
	for (unsigned int x = 0; x < path1.size(); x++)
		printf("%lu   ", path1[x]);
	printf("\n");
	printf("%lld nodes expanded\n", a.GetNodesExpanded());
}

#include "SFBDPancakePuzzle.h"

void makeHard(PancakePuzzleState &s)
{
	int vals[5] = {0, 2, 4, 1, 3};
	for (unsigned int x = 0; x < s.puzzle.size(); x++)
	{
		s.puzzle[x] = vals[x%5]+floor(x/5.0)*5;
	}
}

//void MyPancakeHandler3(unsigned long windowID, tKeyboardModifier, char key)
//{
//	for (int count = 10; count < 100; count+=5)
//	{
//		int puzzleSize = count;
//		PancakePuzzleState a(puzzleSize);
//		PancakePuzzleState b(puzzleSize);
//		
////		PancakePuzzle pp(puzzleSize);
////		for (unsigned int x = 0; x < 150; x++)
////		{
////			std::vector<unsigned> acts;
////			pp.GetActions(a, acts);
////			pp.ApplyAction(a, acts[random()%acts.size()]);
////		}
//		makeHard(a);
//		
//		SFBDPancakeEnvironment p(puzzleSize);
//		pancakeStatePair s(a, b);
//		pancakeStatePair g(a, b);
//		
//		IDAStar<pancakeStatePair, pancakeMovePair> ida;
//		std::vector<pancakeMovePair> path1;
//
////		for (unsigned int x = 0; x < 0; x++)
////		{
////			std::vector<pancakeMovePair> acts;
////			p.GetActions(g, acts);
////			p.ApplyAction(g, acts[random()%acts.size()]);
////		}
////		for (unsigned int x = 0; x < 150; x++)
////		{
////			std::vector<pancakeMovePair> acts;
////			p.GetActions(s, acts);
////			p.ApplyAction(s, acts[random()%acts.size()]);
////		}
////		std::cout << "Searching from: " << std::endl << s << std::endl << g << std::endl;
////		std::cout << "mnp1 heuristic:" << p.HCost(s, g) << std::endl;
//		
//		Timer t;
//		t.StartTimer();
//		ida.GetPath(&p, s, g, path1);
//		t.EndTimer();
//		std::cout << count << "\t" << path1.size() << "\t" << t.GetElapsedTime() << "\t" << ida.GetNodesExpanded() << "\t\t" << s.start << std::endl;
////		std::cout << "Path found (mnp1), length " << path1.size() << " time:" << t.GetElapsedTime() << std::endl;
////		std::cout << ida.GetNodesExpanded() << " nodes expanded" << std::endl;
//	}
////	for (unsigned int x = 0; x < path1.size(); x++)
////	{
////		std::cout << s << std::endl;
////		p.ApplyAction(s, path1[x]);
////	}
////	std::cout << s << std::endl;
//}


void MyPancakeHandler7(unsigned long , tKeyboardModifier, char )
{
	for (int count = 0; count < 100; count+=1)
	{
		int puzzleSize = 50;
		PancakePuzzle p(puzzleSize);
		PancakePuzzleState s(puzzleSize);
		PancakePuzzleState g(puzzleSize);
		
		SFIDAStar<PancakePuzzleState, unsigned> ida;
		std::vector<unsigned> path1;
		
		for (unsigned int x = 0; x < 0; x++)
		{
			std::vector<unsigned> acts;
			p.GetActions(g, acts);
			p.ApplyAction(g, acts[random()%acts.size()]);
		}
		for (unsigned int x = 0; x < 150; x++)
		{
			std::vector<unsigned> acts;
			p.GetActions(s, acts);
			p.ApplyAction(s, acts[random()%acts.size()]);
		}
//		makeHard(s);
		
		Timer t;
		t.StartTimer();
		ida.GetPath(&p, s, g, path1);
		t.EndTimer();
		std::cout << count << "\t" << path1.size() << "\t" << t.GetElapsedTime() << "\t" << ida.GetNodesExpanded() << std::endl;
		//		std::cout << "Path found (mnp1), length " << path1.size() << " time:" << t.GetElapsedTime() << std::endl;
		//		std::cout << ida.GetNodesExpanded() << " nodes expanded" << std::endl;
	}
	//	for (unsigned int x = 0; x < path1.size(); x++)
	//	{
	//		std::cout << s << std::endl;
	//		p.ApplyAction(s, path1[x]);
	//	}
	//	std::cout << s << std::endl;
}

void MyPancakeHandler1(unsigned long , tKeyboardModifier, char )
{
	for (int count = 0; count <= 50; count+=1)
	{
		int puzzleSize = 10;
		BurnedPancakePuzzle pp(puzzleSize);
		BurnedPancakePuzzleState a(puzzleSize);
		BurnedPancakePuzzleState b(puzzleSize);
		
//		IDAStar<PancakePuzzleState, unsigned> ida;
//		std::vector<unsigned> path1;
		
		for (unsigned int x = 0; x < 150; x++)
		{
			std::vector<unsigned> acts;
			pp.GetActions(a, acts);
			pp.ApplyAction(a, acts[random()%acts.size()]);
		}
//		for (unsigned int x = 0; x < 150; x++)
//		{
//			std::vector<unsigned> acts;
//			pp.GetActions(b, acts);
//			pp.ApplyAction(b, acts[random()%acts.size()]);
//		}
		SFBDPancakeEnvironment p(puzzleSize);
		pancakeStatePair s(a, b);
		pancakeStatePair g(a, b);
		
		IDAStar<pancakeStatePair, pancakeMovePair> ida;
		std::vector<pancakeMovePair> path1;
		
		Timer t;
		t.StartTimer();
		ida.GetPath(&p, s, g, path1);
		t.EndTimer();
		std::cout << count << "\t" << path1.size() << "\t" << t.GetElapsedTime() << "\t" << ida.GetNodesExpanded();
		std::cout << "\t\t" << s.start << std::endl;
	}
}

//void MyPancakeHandler(unsigned long , tKeyboardModifier, char )
//{
//	for (int count = 0; count <= 100; count+=1)
//	{
//		int puzzleSize = 10;
//		BurnedPancakePuzzle p(puzzleSize);
//		BurnedPancakePuzzleState s(puzzleSize);
//		BurnedPancakePuzzleState g(puzzleSize);
//		
//		IDAStar<BurnedPancakePuzzleState, unsigned> ida;
//		ida.SetUseBDPathMax(true);
//		std::vector<unsigned> path1;
//		
//		for (unsigned int x = 0; x < 0; x++)
//		{
//			std::vector<unsigned> acts;
//			p.GetActions(g, acts);
//			p.ApplyAction(g, acts[random()%acts.size()]);
//		}
//		for (unsigned int x = 0; x < 150; x++)
//		{
//			std::vector<unsigned> acts;
//			p.GetActions(s, acts);
//			p.ApplyAction(s, acts[random()%acts.size()]);
//		}
//		Timer t;
//		t.StartTimer();
//		ida.GetPath(&p, s, g, path1);
//		t.EndTimer();
//		std::cout << count << "\t" << path1.size() << "\t" << t.GetElapsedTime() << "\t" << ida.GetNodesExpanded();
//		std::cout << "\t\t" << s<< std::endl;
//	}
//}

void MyPancakeHandler(unsigned long , tKeyboardModifier, char )
{
	for (int count = 0; count <= 100; count+=1)
	{
		MNPuzzle p(4, 4);
		MNPuzzleState s(4, 4);
		MNPuzzleState g(4, 4);
		
		IDAStar<MNPuzzleState, slideDir> ida;
		SFIDAStar<MNPuzzleState, slideDir> sfida;
		ida.SetUseBDPathMax(true);
		std::vector<slideDir> path1;
		std::vector<slideDir> path2;
		
//		std::cout << s << std::endl;
		for (unsigned int x = 0; x < 175; x++)
		{
			std::vector<slideDir> acts;
			p.GetActions(s, acts);
			assert(acts.size() > 0);
			p.ApplyAction(s, acts[random()%acts.size()]);
//			std::cout << s << std::endl;
		}
		Timer t1, t2;
		t1.StartTimer();
		ida.GetPath(&p, s, g, path1);
		t1.EndTimer();
		t2.StartTimer();
		sfida.GetPath(&p, s, g, path2);
		t2.EndTimer();
		std::cout << count << "\t" << path1.size() << "\t" << t1.GetElapsedTime() << "\t" << ida.GetNodesExpanded();
		std::cout << "\t" << path2.size() << "\t" << t2.GetElapsedTime() << "\t" << sfida.GetNodesExpanded();
		std::cout << "\t\t" << s << std::endl;
	}
}

bool DFS(FlingBoard &f, Fling *env);

void MyFlingHandler(unsigned long , tKeyboardModifier, char )
{
	FlingBoard b;
	b.AddFling(0, 5);
	b.AddFling(0, 7);
	b.AddFling(1, 1);
	b.AddFling(1, 5);
	b.AddFling(1, 7);
	b.AddFling(4, 0);
	b.AddFling(4, 1);
	b.AddFling(6, 2);
	b.AddFling(6, 3);
	b.AddFling(6, 7);
	std::cout << b << std::endl;

	Fling f;
	DFS(b, &f);
//	std::vector<FlingBoard> neighbors;
//	f.GetSuccessors(b, neighbors);
//	std::cout << neighbors.size() << " successors" << std::endl;
//	for (unsigned int x = 0; x < neighbors.size(); x++)
//		std::cout << neighbors[x] << std::endl;
	exit(0);
}

bool DFS(FlingBoard &f, Fling *env)
{
	std::vector<FlingBoard> neighbors;
	if (env->GoalTest(f, f))
	{ std::cout << f << std::endl; return true; }
	
	env->GetSuccessors(f, neighbors);
	if (neighbors.size() == 0)
		return false;
	
	for (unsigned int x = 0; x < neighbors.size(); x++)
	{
		if (DFS(neighbors[x], env))
		{ std::cout << f << std::endl; return true; }	
	}
	return false;
}
