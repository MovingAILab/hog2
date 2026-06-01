/*
 *  $Id: Driver.cpp
 *  hog2
 *
 *  Created by Nathan Sturtevant on 7/16/21.
 *
 * This file is part of HOG2. See https://github.com/nathansttt/hog2 for licensing information.
 *
 */

#include "Common.h"
#include "Driver.h"
#include <string>
#include "RC.h"
#include "RubiksInstances.h"
#include "ParallelIDAStar.h"
#include "IDAStar.h"
#include "Timer.h"

RCState cube;
RC env;

void RunNewPDBTest();
void RunOldPDBTest();
void ComputeHeuristicStrength(Heuristic<RCState> h);

//static CubeHolder *cubeHolder;
int moveType = 0;

int main(int argc, char* argv[])
{
	//MAKE THE CUBES (Drawing)
	//cubeHolder = new CubeHolder();

	InstallHandlers();
	RunHOGGUI(argc, argv, 1600, 800);

	return 0;
}

/**
 * Allows you to install any keyboard handlers needed for program interaction.
 */
void InstallHandlers()
{
	// HASH FUNCTION
	InstallKeyboardHandler(MyDisplayHandler, "Test9", "HASH TEST", kAnyModifier, '9');
	

	InstallKeyboardHandler(MyDisplayHandler, "Optimal", "Show optimal solution", kAnyModifier, 'o');
	InstallKeyboardHandler(MyDisplayHandler, "Randomize", "Get Random State", kAnyModifier, 's');
	InstallKeyboardHandler(MyDisplayHandler, "Turn0", "Turn Face 0", kAnyModifier, '0');
	InstallKeyboardHandler(MyDisplayHandler, "Turn1", "Turn Face 1", kAnyModifier, '1');
	InstallKeyboardHandler(MyDisplayHandler, "Turn2", "Turn Face 2", kAnyModifier, '2');
	InstallKeyboardHandler(MyDisplayHandler, "Turn3", "Turn Face 3", kAnyModifier, '3');
	InstallKeyboardHandler(MyDisplayHandler, "Turn4", "Turn Face 4", kAnyModifier, '4');
	InstallKeyboardHandler(MyDisplayHandler, "Turn5", "Turn Face 5", kAnyModifier, '5');
	InstallKeyboardHandler(MyDisplayHandler, "MoveType", "Choose Move Type", kAnyModifier, 'm');
	InstallKeyboardHandler(MyDisplayHandler, "TurnStop", "Stop Cube Passive Rotation", kAnyModifier, 'n');
	InstallKeyboardHandler(MyDisplayHandler, "Reset", "Reset cube to solved", kAnyModifier, 'r');
	InstallKeyboardHandler(MyDisplayHandler, "TestPDB", "Test New PDB", kAnyModifier, 't');
	InstallKeyboardHandler(MyDisplayHandler, "TestPDB", "Test Old PDB", kAnyModifier, 'u');
	InstallWindowHandler(MyWindowHandler);
	InstallCommandLineHandler(MyCLHandler, "-test", "-test <new/old>", "Run standard test");
	
	InstallMouseClickHandler(MyClickHandler, static_cast<tMouseEventType>(kMouseMove|kMouseDown));
	srandom(time(0));
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

		//glClearColor(0.99, 0.99, 0.99, 1.0);
		InstallFrameHandler(MyFrameHandler, windowID, 0);

		ReinitViewports(windowID, {-1, -1, 1, 1}, kScaleToSquare);
		env.passiveRot = false;
	}
}


void MyFrameHandler(unsigned long windowID, unsigned int viewport, void *)
{
	Graphics::Display &display = getCurrentContext()->display;
	display.FillRect({-1, -1, 1, 1}, Colors::white);
	
	//Draw here
	env.TestUpdate();
	env.Draw(display, cube);
	//env.TestDraw(display, cube);

	return;
}

int MyCLHandler(char *argument[], int maxNumArgs)
{
	if (maxNumArgs > 1)
	{
		if (strcmp(argument[1], "new") == 0)
			RunNewPDBTest();
		else
			RunOldPDBTest();
	}
	exit(0);
}

void MyDisplayHandler(unsigned long windowID, tKeyboardModifier mod, char key)
{
	// TODO: Check if already rotating before allowing an input from 0-5
	switch (key)
	{
		// HASH FUNCTION
		case '9':
		{
			static int a = 0;
			env.GetStateFromPDBHashCorner(a++, cube, 0);
			printf("rank is %llu\n", env.GetPDBHashCorner(cube, 0));
		}
			break;
		case 't':
			RunNewPDBTest();
			break;
		case 'u':
			RunOldPDBTest();
			break;
		case 'm':
			moveType++;
			if (moveType > 2) moveType = 0;
			std::cout << "Set Movetype to: " + std::to_string(moveType) << '\n';
			break;	
		case 'n':
			env.passiveRot = !env.passiveRot;		
			break;	
		case '0':	
		case '1':	
		case '2':
		case '3':
		case '4':
		case '5':
		{
			int ikey = key - '0';
			// TEMP: DO NOT ROTATE RC
			// env.RotateFace(ikey, moveType);
			env.ApplyAction(cube, ikey*3+moveType);
			//cube.RotateFace(ikey*3+moveType);
			
			// Test
			std::cout << "State: " << '\n'; cube.PrintState();
			std::cout << '\n';
			break;
		}
		case 'o':
		{
			// TEMP: Print data from cubies from each RC
//			std::cout << "Env 1:" << std::endl;
//			env.cubies[0].PrintData();
//			std::cout << std::endl;
//			std::cout << "Env 2:" << std::endl;
//			env2.cubies[0].PrintData();
			// TEMP: Rotate on x axis
			float rot[3] = {0.3f, 0, 0};
			env.RotateCubies(rot);
			break;
		}
		case 's':
		{
			RubiksCubeInstances::GetSuperFlip(cube);
			// TODO: When hash function is finished you can use it to get random states 
			// TEMP: Sets the state to a custom one
//			for (int i = 0; i < 12; i++)
//			{
//				cube.rotation[i] = 1;
//			}
			break;
		}
		case 'r':
		{
			cube.Reset();
			break;
		}
		case '?':
		{
		}
			break;
		default:
			break;
	}
}


bool MyClickHandler(unsigned long , int windowX, int windowY, point3d loc, tButtonType button, tMouseEventType mType)
{
	// TODO: add interactions later
	switch (mType)
	{
	case kMouseMove:
		{
		}
		break;
	case kMouseDown:
		{
		}
	default: return true;
	}
	return true;
}

void RunNewPDBTest()
{
	ParallelIDAStar<RC, RCState, RCAction> ida;
	IDAStar<RCState, RCAction> i;
	RCState goal;
	RC rc;
	goal.Reset();
	std::vector<RCAction> path;

//	for (int x = 0; x < 100; x++)
//	{
//		RCState start;
//		RubiksCubeInstances::GetRandomN(start, 6, x*10);
//		printf("NP: ");
//		rc.SetPruneSuccessors(false);
//		ida.GetPath(&rc, start, goal, path);
//		printf("Path length %lu\n", path.size());
//
//		printf(" P: ");
//		rc.SetPruneSuccessors(true);
//		ida.GetPath(&rc, start, goal, path);
//		printf("Path length %lu\n", path.size());
//	}
//	exit(0);
	
	RCPDB pdb0(&rc,
			   {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
			   {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1}, // 12 edges in place ignoring rotations
			   {0, 0, 0, 0, 0, 0, 0, 0},
			   {0, 0, 0, 0, 0, 0, 0, 0}
			   );
	RCPDB pdb1(&rc,
			   {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
			   {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
			   {0, 0, 0, 0, 0, 0, 0, 0},
			   {1, 1, 1, 1, 1, 1, 1, 1} // 8 edges in place + 12-edge rotations
			   );
//	RCPDB pdb2(&rc,
//			   {1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0},
//			   {0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1},
//			   {0, 0, 0, 0, 0, 0, 0, 0},
//			   {0, 0, 0, 0, 0, 0, 0, 0});
	RCPDB pdb3(&rc,
			   {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1}, // all rotations correct
			   {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
			   {1, 1, 1, 1, 1, 1, 1, 1},
			   {0, 0, 0, 0, 0, 0, 0, 0}
			   );
	RCPDB pdb4(&rc,
			   {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
			   {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
			   {1, 1, 1, 1, 1, 1, 1, 1},
			   {1, 1, 1, 1, 1, 1, 1, 1}
			   );
	Heuristic<RCState> h;
	h.lookups.push_back({kMaxNode, 1, 4});
	h.lookups.push_back({kLeafNode, 0, 0});
	h.lookups.push_back({kLeafNode, 1, 1});
	h.lookups.push_back({kLeafNode, 2, 2});
	h.lookups.push_back({kLeafNode, 3, 3});
//	h.lookups.push_back({kLeafNode, 4, 4});

	h.heuristics.push_back(&pdb4);
	h.heuristics.push_back(&pdb3);
//	h.heuristics.push_back(&pdb2);
	h.heuristics.push_back(&pdb1);
	h.heuristics.push_back(&pdb0);

//					  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
//					  {1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0},
//					  {1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0},
//					  {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
//					  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
//					  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
//					  {1, 1, 1, 1, 1, 1, 1, 1},
//					  {1, 1, 1, 1, 1, 1, 1, 1}
//					  {0, 0, 0, 0, 0, 0, 0, 0},
//					  {0, 0, 0, 0, 0, 0, 0, 0}
//					  );

	
	for (auto &j : h.heuristics)
	{
//		RCState tmp;
		RCPDB *i = (RCPDB*)j;
		std::cout << "PDB Size " << i->GetPDBSize() << "\n";
		//	printf("Verifying hashes:\n");
		for (uint64_t x = 0; x < i->GetPDBSize(); x++)
		{
			i->GetStateFromPDBHash(x, goal);
			assert(x == i->GetPDBHash(goal));
//			rc.GetActions(goal, path);
//			for (auto act : path)
//			{
//				rc.ApplyAction(goal, act);
//				auto rank = i->GetPDBHash(goal);
//				i->GetStateFromPDBHash(rank, tmp);
//				assert(i->AbstractEqual(tmp, goal));
//				rc.UndoAction(goal, act);
//			}
		}
		printf("Hash verified\n");

		goal.Reset();
		i->SetGoal(goal);
		i->BuildPDB(goal, 16);
		std::cout << i->HCost(goal, goal);
	}
//			std::vector<RCState> succ;
//			rc.GetSuccessors(goal, succ);
//			for (auto i : succ)
//				std::cout << i << "\n";

//	goal.Reset();
//	pdb1.SetGoal(goal);
//	pdb1.BuildPDB(goal, 16);
//	std::cout << pdb1.HCost(goal, goal);
//
//	goal.Reset();
//	pdb2.SetGoal(goal);
//	pdb2.BuildPDB(goal, 16);
//	std::cout << pdb2.HCost(goal, goal);
//
//	goal.Reset();
//	pdb3.SetGoal(goal);
//	pdb3.BuildPDB(goal, 16);
//	std::cout << pdb3.HCost(goal, goal);
//
//	goal.Reset();
//	pdb4.SetGoal(goal);
//	pdb4.BuildPDB(goal, 16);
//	std::cout << pdb4.HCost(goal, goal);

	ida.SetHeuristic(&h);
	RCState start;
	rc.SetPruneSuccessors(true);
	Timer t;
	for (int x = 0; x < 10; x++)
	{
		RubiksCubeInstances::GetKorfRubikInstance(start, x);
		t.StartTimer();
		ida.GetPath(&rc, start, goal, path);
		t.EndTimer();
		printf("%1.2fs elapsed. Found path length %lu. %" PRId64 " expanded %" PRId64 " generated\n",
			   t.GetElapsedTime(), path.size(), ida.GetNodesExpanded(), ida.GetNodesTouched());
	}
	exit(0);
}

void RunOldPDBTest()
{
	ParallelIDAStar<RC, RCState, RCAction> ida;
	IDAStar<RCState, RCAction> i;
	RCState goal;
	RC rc;
	goal.Reset();
	std::vector<RCAction> path;

//	for (int x = 0; x < 100; x++)
//	{
//		RCState start;
//		RubiksCubeInstances::GetRandomN(start, 6, x*10);
//		printf("NP: ");
//		rc.SetPruneSuccessors(false);
//		ida.GetPath(&rc, start, goal, path);
//		printf("Path length %lu\n", path.size());
//
//		printf(" P: ");
//		rc.SetPruneSuccessors(true);
//		ida.GetPath(&rc, start, goal, path);
//		printf("Path length %lu\n", path.size());
//	}
//	exit(0);
	
	RCPDB pdb0(&rc,
			   {1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0},
			   {1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0},
			   {0, 0, 0, 0, 0, 0, 0, 0},
			   {0, 0, 0, 0, 0, 0, 0, 0}
			   );
	RCPDB pdb1(&rc,
			   {0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1},
			   {0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1},
			   {0, 0, 0, 0, 0, 0, 0, 0},
			   {0, 0, 0, 0, 0, 0, 0, 0}
			   );
	RCPDB pdb2(&rc,
			   {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
			   {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
			   {1, 1, 1, 1, 1, 1, 1, 1},
			   {1, 1, 1, 1, 1, 1, 1, 1}
			   );
	Heuristic<RCState> h;
	h.lookups.push_back({kMaxNode, 1, 3});
	h.lookups.push_back({kLeafNode, 0, 0});
	h.lookups.push_back({kLeafNode, 1, 1});
	h.lookups.push_back({kLeafNode, 2, 2});
	
	h.heuristics.push_back(&pdb0);
	h.heuristics.push_back(&pdb1);
	h.heuristics.push_back(&pdb2);

	printf("Verifying hashes:\n");
	for (auto &j : h.heuristics)
	{
		RCState tmp;
		RCPDB *i = (RCPDB*)j;
		std::cout << "PDB Size " << i->GetPDBSize() << "\n";
		for (uint64_t x = 0; x < i->GetPDBSize(); x++)
		{
			i->GetStateFromPDBHash(x, goal);
			assert(x == i->GetPDBHash(goal));
//			rc.GetActions(goal, path);
//			for (auto act : path)
//			{
//				rc.ApplyAction(goal, act);
//				auto rank = i->GetPDBHash(goal);
//				i->GetStateFromPDBHash(rank, tmp);
//				assert(i->AbstractEqual(tmp, goal));
//				rc.UndoAction(goal, act);
//			}
		}
		printf("Hash verified\n");

		goal.Reset();
		i->SetGoal(goal);
		i->BuildPDB(goal, 16);
		std::cout << i->HCost(goal, goal);
	}

	ida.SetHeuristic(&h);
	RCState start;
	rc.SetPruneSuccessors(true);
	Timer t;
	for (int x = 0; x < 10; x++)
	{
		RubiksCubeInstances::GetKorfRubikInstance(start, x);
		t.StartTimer();
		ida.GetPath(&rc, start, goal, path);
		t.EndTimer();
		printf("%1.2fs elapsed. Found path length %lu. %" PRId64 " expanded %" PRId64 " generated\n",
			   t.GetElapsedTime(), path.size(), ida.GetNodesExpanded(), ida.GetNodesTouched());
	}
	exit(0);
}

void RunOlderPDBTest()
{
	RubiksCube cube;
	RubiksState start, goal;
	goal.Reset();
	std::vector<RubiksAction> path;

//	std::vector<int> edges;
//	std::vector<int> corners;
	RubikPDB pdb1(&cube, goal, {0, 1, 2, 3, 4, 5}, {});
	RubikPDB pdb2(&cube, goal, {6, 7, 8, 9, 10, 11}, {});
	RubikPDB pdb3(&cube, goal, {}, {0, 1, 2, 3, 4, 5, 6, 7});
	pdb1.BuildPDB(goal, 16);
	pdb2.BuildPDB(goal, 16);
	pdb3.BuildPDB(goal, 16);

	Heuristic<RubiksState> h;
	h.lookups.push_back({kMaxNode, 1, 3});
	h.lookups.push_back({kLeafNode, 0, 0});
	h.lookups.push_back({kLeafNode, 1, 1});
	h.lookups.push_back({kLeafNode, 2, 2});

	h.heuristics.push_back(&pdb1);
	h.heuristics.push_back(&pdb2);
	h.heuristics.push_back(&pdb3);

	ParallelIDAStar<RubiksCube, RubiksState, RubiksAction> ida;
	ida.SetHeuristic(&h);
	Timer t;
	cube.SetPruneSuccessors(true);
	for (int x = 0; x < 10; x++)
	{
		RubiksCubeInstances::GetKorfRubikInstance(start, x);
		t.StartTimer();
		ida.GetPath(&cube, start, goal, path);
		t.EndTimer();
		printf("%1.2fs elapsed. Found path length %lu. %" PRId64 " expanded %" PRId64 " generated\n",
			   t.GetElapsedTime(), path.size(), ida.GetNodesExpanded(), ida.GetNodesTouched());
	}

}

void ComputeHeuristicStrength(Heuristic<RCState> h)
{
	
}
