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
#include "Driver.h"
#include "UnitSimulation.h"
#include "EpisodicSimulation.h"
#include "IDAStar.h"
#include "Timer.h"
#include "Fling.h"
#include "BFS.h"
#include <fstream>
#include <iostream>
#include <iomanip>
#include "TextOverlay.h"
#include <pthread.h>
#include <unordered_map>
#include <thread>
#include <mutex>
uint64_t DoLimitedBFS(FlingBoard b, std::vector<FlingBoard> &path);
void RemoveDups();
bool UniquelySolvable(FlingBoard &b, int &solutions, int *goalLocations);
bool UniquelySolvable(FlingBoard &b, int &solutions);
void AnalyzeEndLocs(int level);
void ThreadedEndLocAnalyze(int level, uint64_t start, uint64_t end, std::mutex *lock);
void AnalyzeRocks(int level);
void AnalyzeFinalPieces(int level);

int main(int argc, char* argv[])
{
	InstallHandlers();
	RunHOGGUI(argc, argv, 620, 700);
	return 0;
}


/**
 * This function is used to allocate the unit simulated that you want to run.
 * Any parameters or other experimental setup can be done at this time.
 */
TextOverlay text;
Fling f;
FlingBoard b, g;
FlingMove m;
std::vector<FlingBoard> path;
std::vector<FlingMove> wins;
std::vector<int> counts;
FlingBoard win;
FlingBoard valid;
int pathLoc = 0;

bool screenShot = false;
char screenShotFile[255];

//bool ReadData(std::vector<bool> &data, const char *fName);
//void WriteData(BitVector *data, const char *fName);

void CreateSimulation(int id)
{
	if (screenShot)
	{
	  //SetZoom(id, 3.0);
		return;
	}
	for (int x = 0; x < 10; x++)
	{
		int x1 = random()%b.width;
		int y1 = random()%b.height;
		if (!b.HasPiece(x1, y1) && !b.HasHole(x1, y1))
			b.AddFling(x1, y1);
	}
	std::cout << b << std::endl;
	uint64_t h1 = f.GetStateHash(b);
	f.GetStateFromHash(h1, g);
	uint64_t h2 = f.GetStateHash(g);
	if (h1 != h2)
	{
		std::cout << b << std::endl;
		exit(0);
	}
	if (!(b == g))
	{
		std::cout << b << std::endl;
		std::cout << g << std::endl;
		exit(0);
	}
	text.SetBold(true);
	b.SetObstacle(24);
	b.SetObstacle(31);
	
//	std::vector<FlingBoard> path;
//	BFS<FlingBoard, FlingMove> bfs;
//	bfs.GetPath(&f, b, g, path);
//	IDAStar<FlingBoard, FlingMove> ida;
//	ida.GetPath(&f, b, g, path);
//	std::cout << ida.GetNodesExpanded() << " nodes expanded" << std::endl;
//	for (int x = 0; x < path.size(); x++)
//	{
//		if (x > 0)
//		{
//			std::cout << "Action: " << f.GetAction(path[x], path[x-1]) << std::endl;
//		}
//		std::cout << x << std::endl << path[x] << std::endl;
//	}
}

void GetSolveActions(FlingBoard &solve, std::vector<FlingMove> &acts)
{
	acts.resize(0);
	BFS<FlingBoard, FlingMove, Fling> bfs;
	bfs.GetPath(&f, solve, g, path);
	for (int x = 0; x < path.size(); x++)
		path[x].SetObstacles(solve.GetObstacles());
	for (int x = 1; x < path.size(); x++)
	{
		acts.push_back(f.GetAction(path[x], path[x-1]));
	}
	//printf("Got path length %ld; returning %d actions\n", path.size(), acts.size());
}

/**
 * Allows you to install any keyboard handlers needed for program interaction.
 */
void InstallHandlers()
{
	InstallKeyboardHandler(MyDisplayHandler, "Reset Board", "Reset board on the screen", kAnyModifier, 'r');
	InstallKeyboardHandler(MyDisplayHandler, "History", "Move ?? in history", kAnyModifier, '[');
	InstallKeyboardHandler(MyDisplayHandler, "History", "Move ?? in history", kAnyModifier, ']');
	InstallKeyboardHandler(MyDisplayHandler, "Load Preset", "Load Preset State", kAnyModifier, '1', '9');

	InstallKeyboardHandler(SolveAndSaveInstance, "Solve & Save", "Solve and save the current instance", kNoModifier, 's');
	InstallKeyboardHandler(SolveRandomFlingInstance, "Solve Random Instance", "Generates a random instance and uses a BFS to solve it", kNoModifier, '0');
	InstallKeyboardHandler(FindLogicalMoves, "FindLogicalMoves", "In the sequence of moves, find those which are logical", kNoModifier, 'f');
	InstallKeyboardHandler(TestRanking, "Test Ranking Function", "Test ranking function", kNoModifier, 't');
	InstallKeyboardHandler(BuildTables, "Build Exhaustive Tables", "Build Exhaustive Tables", kNoModifier, 'e');
	InstallKeyboardHandler(ReadTables, "Load Tables", "Load tables sizes 2...9", kNoModifier, 'l');
	InstallKeyboardHandler(AnalyzeBoard, "Analyze Board", "Analyze board to generate statistics", kAnyModifier, 'a');
	InstallKeyboardHandler(AnalyzeBoard, "Analyze Board", "Analyze board to generate statistics", kAnyModifier, 'A');
	InstallKeyboardHandler(DeepAnalyzeBoard, "Deep Analyze Board", "Analyze board to generate statistics", kAnyModifier, 'd');
	InstallKeyboardHandler(DeepAnalyzeBoard, "Deep Analyze Board", "Analyze board to generate statistics", kAnyModifier, 'D');
	InstallKeyboardHandler(BFSearch, "Do BFS", "Analyze board with BFS", kAnyModifier, 'b');
	InstallKeyboardHandler(MassAnalysis, "Mass Analysis", "Analyze 100 boards", kNoModifier, 'm');
	InstallKeyboardHandler(CaptureScreen, "Capture Screen", "Capture Screen Shot", kNoModifier, 'c');
	InstallKeyboardHandler(RemoveDuplicates, "Remove duplicates", "Analyze states passed in and remove dups", kNoModifier, 'o');

	InstallCommandLineHandler(MyCLHandler, "-rankTest", "-rankTest", "Test incremental versus full unranking.");
	InstallCommandLineHandler(MyCLHandler, "-generate", "-generate n", "Generate a problem with n tiles and run a BFS.");
	InstallCommandLineHandler(MyCLHandler, "-extract", "-extract n", "Extract unique boards at level n.");
	InstallCommandLineHandler(MyCLHandler, "-solve", "-solve n", "Solve all boards up to size n.");
	InstallCommandLineHandler(MyCLHandler, "-bfs", "-bfs theState <goal loc>", "Perform a BFS on theState; goal loc is optional");
	InstallCommandLineHandler(MyCLHandler, "-screen", "-screen theState file", "Capture a shot of theState in <file>");
	InstallCommandLineHandler(MyCLHandler, "-getCanonical", "-getCanonical theState", "Get canonical version of theState");
	InstallCommandLineHandler(MyCLHandler, "-removeDups", "-removeDups", "Read states from stdin and remove similar states");
	InstallCommandLineHandler(MyCLHandler, "-analyze", "-analyze theState", "Perform a move analysis on theState");
	InstallCommandLineHandler(MyCLHandler, "-fix", "-fix file", "fix the file format");
	InstallCommandLineHandler(MyCLHandler, "-solveState", "-solveState <state> <type> <goal loc>", "solve state; type=rocks,panda,final goal loc is only required for panda or final");

	InstallCommandLineHandler(MyCLHandler, "-interSolutionDistance", "-interSolutionDistance <state> <goal loc>", "Measure difficulty of solving a problem");
	InstallCommandLineHandler(MyCLHandler, "-analyzeEndLocs", "-analyzeEndLocs <level>", "find states with a single way to end with a piece in a square");
	InstallCommandLineHandler(MyCLHandler, "-analyzeRocks", "-analyzeRocks <level>", "find states with rocks in the middle that have unique solutions");
	InstallCommandLineHandler(MyCLHandler, "-analyzeFinalPieces", "-analyzeFinalPieces <level>", "Find states with unique locations for final pieces");

	InstallCommandLineHandler(MyCLHandler, "-uniqueSolution", "-uniqueSolution <state>", "Determine if a state has a unique solution");
	InstallCommandLineHandler(MyCLHandler, "-showSolutionLocs", "-showSolutionLocs <state>", "Show all locations that can be reached at the end.");
	InstallCommandLineHandler(MyCLHandler, "-measureSolutionSimilarity", "-measureSolutionSimilarity <state>", "Measure how many times the solution changes pieces");
	
	
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
		SetNumPorts(windowID, 1);
	}
}

//ida.GetPath(&f, b, g, path);

void MyFrameHandler(unsigned long windowID, unsigned int viewport, void *)
{
	if (screenShot)
	{
	  //		f.OpenGLDrawPlain(b);
		//SaveScreenshot(windowID, screenShotFile);
		exit(0);
	}
	//f.OpenGLDraw(b);
	//	for (int x = 0; x < wins.size(); x++)
	//	f.OpenGLDraw(win, wins[x]);
	//if (wins.size() > 0)
	{
	  //		f.OpenGLDrawAlternate(valid);
	}
	for (int x = 0; x < counts.size(); x++)
	{
		char val[3] = {0, 0, 0};
		FlingBoard tmp;
		tmp.AddFling(x);
		val[0] = '0'+counts[x]/10;
		val[1] = '0'+counts[x]%10;
		//f.GLLabelState(tmp, val);
	}
	
	//	text.OpenGLDraw(windowID);
}

int BitCount(uint64_t value)
{
	int count = 0;
	while (value != 0)
	{
		value &= (value-1);
		count++;
	}
	return count;
}

int dist(int loc1, int loc2)
{
	int x1 = loc1%7;
	int y1 = loc1/7;
	int x2 = loc2%7;
	int y2 = loc2/7;
	return abs(x1-x2)+abs(y1-y2);
}

void rankTest()
{
	const int numPieces = 8;
	FlingBoard b1, b2;
	Fling f;
	uint64_t maxRank = f.getMaxSinglePlayerRank(56, numPieces);
	printf("Max rank: %llu\n", maxRank);
	f.unrankPlayer(0, numPieces, b1);
	f.unrankPlayer(0, numPieces, b2);
	float currPerc = 1;
	printf("Correctness Check\n");
	for (uint64_t r = 1; r < maxRank; r++)
	{
		assert(b1 == b2);
		if (100.0*r/(float)maxRank > currPerc)
			printf("%d ", (int)currPerc++);
		f.IncrementRank(b1);
		f.unrankPlayer(r, numPieces, b2);
	}
	assert(b1 == b2);
	printf("Test successful\n");
	printf("Speed Check:\n");
	f.unrankPlayer(0, numPieces, b1);
	f.unrankPlayer(0, numPieces, b2);
	Timer t;
	t.StartTimer();
	for (uint64_t r = 1; r < maxRank; r++)
	{
		f.IncrementRank(b1);
	}
	t.EndTimer();
	printf("Incremental: %fs elapsed\n", t.GetElapsedTime());
	t.StartTimer();
	for (uint64_t r = 1; r < maxRank; r++)
	{
		f.unrankPlayer(r, numPieces, b2);
	}
	t.EndTimer();
	printf("Full: %fs elapsed\n", t.GetElapsedTime());
}

int MyCLHandler(char *argument[], int maxNumArgs)
{
//	if (maxNumArgs <= 1)
//	{
//		printf("Insufficient arguments\n");
//		return 0;
//	}
	//strncpy(gDefaultMap, argument[1], 1024);
	if (strcmp(argument[0], "-generate") == 0)
	{
		SolveRandomFlingInstance(0, kNoModifier, '0');
	}
	else if (strcmp(argument[0], "-rankTest") == 0)
	{
		rankTest();
		exit(0);
	}
	else if (strcmp(argument[0], "-extract") == 0)
	{
		int cnt = atoi(argument[1]);
		ExtractUniqueStates(cnt);
	}
	else if (strcmp(argument[0], "-solve") == 0)
	{
		int cnt = atoi(argument[1]);
		for (int x = 2; x <= cnt; x++)
		{
			BuildTables(0, kNoModifier, 'e');
		}
	}
	else if (strcmp(argument[0], "-screen") == 0)
	{
		unsigned long long which = strtoull(argument[1], 0, 10);
		std::cout << "Hash: " << which << std::endl;
		//std::cout << "Pieces: " << atoi(argument[1]) << std::endl;
		f.GetStateFromHash(which, b);
		screenShot = true;
		strncpy(screenShotFile, argument[2], 255);
		return 3;
	}
	else if (strcmp(argument[0], "-bfs") == 0)
	{
		unsigned long long which = strtoull(argument[1], 0, 10);
		std::cout << "Hash: " << which << std::endl;
		//std::cout << "Pieces: " << atoi(argument[1]) << std::endl;
		f.GetStateFromHash(which, b);
		//f.unrankPlayer(which, atoi(argument[1]), b);

		if (maxNumArgs > 2) // also read goal location
		{
			if (strcmp(argument[2], "rocks") == 0)
			{
				b.SetObstacle(24);
				b.SetObstacle(31);
			}
			else {
				printf("Using goal %d\n", atoi(argument[2]));
				f.SetGoalLoc(atoi(argument[2]));
			}
		}
		std::cout << b << std::endl;
		
		BFS<FlingBoard, FlingMove, Fling> bfs;
		bfs.GetPath(&f, b, g, path);
		printf("%llu total nodes expanded in pure bfs\n", bfs.GetNodesExpanded());
		uint64_t nodesExpanded = DoLimitedBFS(b, path);
		printf("%llu total nodes expanded in logically limited bfs\n", nodesExpanded);
		printf("Ratio: %llu %llu %1.4f\n", bfs.GetNodesExpanded(), nodesExpanded, double(bfs.GetNodesExpanded())/double(nodesExpanded));
		f.ClearGoalLoc();
	}
	else if (strcmp(argument[0], "-getCanonical") == 0)
	{
		unsigned long long which = strtoull(argument[1], 0, 10);
		std::cout << "Hash: " << which << std::endl;
		f.GetStateFromHash(which, b);
		which = GetCanonicalHash(which);
		std::cout << "Original:\n" << b << std::endl;
		f.GetStateFromHash(which, b);
		std::cout << "Canonical:\n" << b << std::endl;
		printf("%llu\n", which);
	}
	else if (strcmp(argument[0], "-removeDups") == 0)
	{
		RemoveDups();
	}
	else if (strcmp(argument[0], "-analyze") == 0)
	{
		ReadTables(0, kNoModifier, 'l');
		while (!feof(stdin))
		{
			uint64_t which;
			scanf("%llu", &which);
			//unsigned long long which = strtoull(argument[1], 0, 10);
			std::cout << "Hash: " << which << std::endl;
			//std::cout << "Pieces: " << atoi(argument[1]) << std::endl;
			f.GetStateFromHash(which, b);
			//f.unrankPlayer(which, atoi(argument[1]), b);
			std::cout << b << std::endl;
			
			AnalyzeBoard(0, kNoModifier, 'a');
			AnalyzeBoard(0, kNoModifier, 'A');
		}
	}
	else if (strcmp(argument[0], "-fix") == 0)
	{
		FILE *file = fopen(argument[1], "r");
		int next;
		int cnt = 0;
		do {
			next = fgetc(file);
			if (next == '.' || next == 'o')
			{
				if (next == 'o')
					b.SetPiece(cnt);
				else
					b.ClearPiece(cnt);
				cnt++;
			}
			if (cnt == 56)
			{
				std::cout << b << std::endl << std::endl;
				std::cout << "Hash: " << f.rankPlayer(b) << " " << b.locs.size() << std::endl;
				cnt = 0;
			}
		} while (next != EOF);
	}
	else if (strcmp(argument[0], "-solveState") == 0)
	{
		std::vector<FlingMove> stateActs;
		unsigned long long which = strtoull(argument[1], 0, 10);
		f.GetStateFromHash(which, b);

		std::cout << b << "\n";
		if (maxNumArgs > 2) // also read goal location
		{
			if (strcmp(argument[2], "rocks") == 0)
			{
				b.SetObstacle(24);
				b.SetObstacle(31);
			}
			else if (strcmp(argument[2], "final") == 0)
			{
				printf("Using goal %d\n", atoi(argument[3]));
				f.SetGoalLoc(atoi(argument[3]));
			}
			else if (strcmp(argument[2], "panda") == 0)
			{
				int which = b.GetIndexInLocs(atoi(argument[3]));
				printf("Using panda %d - id %d\n", b.locs[which].first, b.locs[which].second);
				f.SetGoalPanda(b.locs[which].second);
			}
		}

		GetSolveActions(b, stateActs);
		for (size_t x = 0; x < stateActs.size(); x++)
		{
			std::cout << stateActs[stateActs.size()-1-x] << " ";
		}
		std::cout << "\nS: ";
		for (int t = stateActs.size()-1; t > 1; t--)
		{
			if (b.LocationAfterAction(stateActs[t]) == stateActs[t-1].startLoc)
			{
				printf("1");
			}
			else {
				printf("0");
			}
			f.ApplyAction(b, stateActs[t]);
		}
		std::cout << "\n";
		f.GetStateFromHash(which, b);
		for (size_t x = 0; x < stateActs.size(); x++)
		{
			std::cout << stateActs[stateActs.size()-1-x] << "\n";
			f.ApplyAction(b, stateActs[stateActs.size()-1-x]);
			std::cout << b << "\n";
		}
	}
	else if (strcmp(argument[0], "-interSolutionDistance") == 0)
	{
		std::vector<FlingMove> stateActs;
		unsigned long long which = strtoull(argument[1], 0, 10);
		f.GetStateFromHash(which, b);
		
		if (maxNumArgs > 2) // also read goal location
		{
			if (strcmp(argument[2], "rocks") == 0)
			{
				b.SetObstacle(24);
				b.SetObstacle(31);
			}
			else {
				printf("Using goal %d\n", atoi(argument[2]));
				f.SetGoalLoc(atoi(argument[2]));
			}
		}
		
		GetSolveActions(b, stateActs);
		std::cout << b << "\n";
		for (size_t x = 0; x < stateActs.size(); x++)
		{
			std::cout << stateActs[stateActs.size()-1-x] << " ";
		}
		std::cout << "\nDIST: ";
		double sum = 0;
		double w1 = 5, w2 = 3, w3 = 1, w4 = 1;
		FlingBoard bNext;
		for (int t = stateActs.size()-1; t > 1; t--)
		{
			f.GetNextState(b, stateActs[t], bNext);
			// Take the min of:
			// w1. Distance from end of one move to the start of the next[!]
			// w2. Distance from end of one move to the end of the next
			// w3. Distance from the start of one move to the start of the next
			// w4. Distance from the start of one move to the end of the next

			// opposite direction moves
			if ((stateActs[t].dir%2) != (stateActs[t-1].dir%2))
			{
				int x1, x2, y1, y2, t1, t2;
				// second move is in the y-axis
				if (GetX(stateActs[t-1].startLoc) == GetX(bNext.LocationAfterAction(stateActs[t-1])))
				{
					x1 = GetX(stateActs[t].startLoc);
					x2 = GetX(b.LocationAfterAction(stateActs[t]));
					y1 = GetY(stateActs[t-1].startLoc);
					y2 = GetY(bNext.LocationAfterAction(stateActs[t-1]));
					
					
					// bounds of first move (x-axis) are outside the second move
					if (GetX(stateActs[t-1].startLoc) < max(x1, x2) &&
						GetX(stateActs[t-1].startLoc) > min(x1, x2))
					{
						// Need to compare the tree of wrong moves to the
						// tree of right moves.
						sum += 8;
					}
				}
				else {
					x1 = GetX(stateActs[t-1].startLoc);
					x2 = GetX(bNext.LocationAfterAction(stateActs[t-1]));
					y1 = GetY(stateActs[t].startLoc);
					y2 = GetY(b.LocationAfterAction(stateActs[t]));
					
					// bounds of first move (y-axis) are outside the second move
					if (GetY(stateActs[t].startLoc) < max(y1, y2) &&
						GetY(stateActs[t].startLoc) > min(y1, y2))
					{
						// Need to compare the tree of wrong moves to the
						// tree of right moves.
						sum += 8;
					}
				}

	
//				if (((min(x1, x2) < GetX(stateActs[t1].startLoc) && max(x1, x2) == GetX(stateActs[t1].startLoc)) ||
//					 (min(x1, x2) == GetX(stateActs[t1].startLoc) && max(x1, x2) > GetX(stateActs[t1].startLoc)))
//					&&
//					((min(y1, y2) < GetY(stateActs[t1].startLoc) && max(y1, y2) == GetY(stateActs[t1].startLoc)) ||
//					 (min(y1, y2) == GetY(stateActs[t1].startLoc) && max(y1, y2) > GetY(stateActs[t1].startLoc))))
//				{
//					sum += 8;
//				}
//					min(y1, y2) < GetY(stateActs[t2].startLoc) &&
//					max(y1, y2) > GetY(stateActs[t2].startLoc))

			}
//			// I really want to know if the next move interacts piece-wise with no other piece.
//			// That's a huge penalty.
//			if ((dist(b.LocationAfterAction(stateActs[t]), stateActs[t-1].startLoc) > 3) &&
//				(dist(b.LocationAfterAction(stateActs[t]), bNext.LocationAfterAction(stateActs[t-1])) > 3) &&
//				(dist(stateActs[t].startLoc, stateActs[t-1].startLoc) > 3) &&
//				(dist(stateActs[t].startLoc, bNext.LocationAfterAction(stateActs[t-1])) > 3))
//			{
//				sum += 8;
//			}
			
//			// same piece
//			if (b.LocationAfterAction(stateActs[t]) == stateActs[t-1].startLoc)
//			{
//				sum += 0;
//			}
//			// nearby piece
//			else if (dist(b.LocationAfterAction(stateActs[t]), stateActs[t-1].startLoc) <= 2)
//			{
//				sum += 0;
//			}
//			else if (dist(b.LocationAfterAction(stateActs[t]), bNext.LocationAfterAction(stateActs[t-1])) <= 2)
//			{
//				sum += 0;
//			}
//			else {
//				sum += 4;
//			}
			
			
//			sum += w1*dist(b.LocationAfterAction(stateActs[t]), stateActs[t-1].startLoc);
//			sum += w2*dist(b.LocationAfterAction(stateActs[t]), bNext.LocationAfterAction(stateActs[t-1]));
//			sum += w3*dist(stateActs[t].startLoc, stateActs[t-1].startLoc);
//			sum += w4*dist(stateActs[t].startLoc, bNext.LocationAfterAction(stateActs[t-1]));
			f.ApplyAction(b, stateActs[t]);
		}
		std::cout << std::setw(2) << sum;
		std::cout << "\n";
	}
	else if (strcmp(argument[0], "-uniqueSolution") == 0)
	{
		std::vector<FlingMove> stateActs;
		unsigned long long which = strtoull(argument[1], 0, 10);
		f.GetStateFromHash(which, b);
		int sol;
		
		UniquelySolvable(b, sol);
		std::cout << b << std::endl;
		std::cout << sol << " solutions found.\n";
	}
	else if (strcmp(argument[0], "-showSolutionLocs") == 0)
	{
		unsigned long long which = strtoull(argument[1], 0, 10);
		f.GetStateFromHash(which, b);
		uint64_t r = f.rankPlayer(b);
		std::mutex l;
		ThreadedEndLocAnalyze(b.NumPieces(), r, r+1, &l);
	}
	else if (strcmp(argument[0], "-analyzeEndLocs") == 0)
	{
		AnalyzeEndLocs(atoi(argument[1]));
	}
	else if (strcmp(argument[0], "-analyzeRocks") == 0)
	{
		AnalyzeRocks(atoi(argument[1]));
	}
	else if (strcmp(argument[0], "-analyzeFinalPieces") == 0)
	{
		AnalyzeFinalPieces(atoi(argument[1]));
	}
	else if (strcmp(argument[0], "-measureSolutionSimilarity") == 0)
	{
		std::vector<FlingMove> stateActs;
		unsigned long long which = strtoull(argument[1], 0, 10);
		f.GetStateFromHash(which, b);
		GetSolveActions(b, stateActs);
		//printf("%d actions to solve\n", stateActs.size());

		for (int t = stateActs.size()-1; t > 1; t--)
		{
			if (b.LocationAfterAction(stateActs[t]) == stateActs[t-1].startLoc)
			{
				printf("1");
			}
			else {
				printf("0");
			}
			f.ApplyAction(b, stateActs[t]);
		}

	}
	printf("\n");
	exit(0);
	return 2;
}

void MyDisplayHandler(unsigned long windowID, tKeyboardModifier mod, char key)
{
	switch (key)
	{
		case '[':
			pathLoc--;
			if (pathLoc < 0)
				pathLoc = 0;
			if (pathLoc < path.size())
				b = path[pathLoc];
			break;
		case ']':
			if (path.size() == 0)
				break;
			pathLoc++;
			if (pathLoc >= path.size())
				pathLoc = path.size()-1;
			if (pathLoc < path.size())
				b = path[pathLoc];
			break;
		case 'r':
			if (b.locs.size() == 0)
			{
				counts.resize(0);
				for (int x = 0; x < 10; x++)
				{
					int x1 = random()%b.width;
					int y1 = random()%b.height;
					if (!b.HasPiece(x1, y1))
						b.AddFling(x1, y1);
				}
			}
			else {
				b.Reset();
				g.Reset();
				valid.Reset();
				win.Reset();
				counts.resize(0);
				wins.resize(0);
			}
			break;
		case '1':
			f.GetStateFromHash(45179843855354896ull, b);
			break;
		case '2':
			f.GetStateFromHash(6786211670857232ull, b);
			break;
		case '3':
			f.GetStateFromHash(11288156058953760ull, b);
			break;
		case '4':
			f.GetStateFromHash(27303083721900308ull, b);
			break;
		case '5':
			f.GetStateFromHash(18157695840586785ull, b);
			break;
		case '6':
			f.GetStateFromHash(37154696925872128ull, b);
			break;
		default:
			break;
	}
}

void GetReducedMoves(FlingBoard s1, FlingBoard s2, std::vector<FlingMove> &moves)
{
	std::vector<FlingMove> m1, m2;
	FlingMove between;
	
	/**
	 * Note:
	 * Sometimes there are 2 different moves that can be the between action. In this
	 * case, one might lead to pruning and the other not. Thus, there can be different
	 * #'s of moves in the BFS depending on how moves are generated.
	 */
	between = f.GetAction(s1, s2);
	
	f.GetActions(s1, m1);
	f.GetActions(s2, m2);
	
	std::vector<FlingMove> commonMoves;
	
	moves.resize(0);
	// find common moves
	for (int x = 0; x < m1.size(); x++)
	{
		for (int y = 0; y < m2.size(); y++)
		{
			if (m1[x] == m2[y])
			{
				commonMoves.push_back(m1[x]);
				//std::cout << "Common moves include " << m1[x] << std::endl;
			}
		}
	}
	
	FlingBoard t1, t2;
	t1.SetObstacles(s1.GetObstacles());
	t2.SetObstacles(s2.GetObstacles());
	moves.resize(0);
	// apply common moves (besides between) to s1
	for (int x = 0; x < commonMoves.size(); x++)
	{
		f.GetNextState(s1, commonMoves[x], t1);
		if (f.LegalMove(t1, between))
		{
			f.ApplyAction(t1, between);
			if (f.LegalMove(s2, commonMoves[x]))
			{
				f.GetNextState(s2, commonMoves[x], t2);
				if (t1.GetRawBoard() != t2.GetRawBoard())
				{
//					std::cout << commonMoves[x] << " and " << between <<
//					" in different orders result in different states" << std::endl;
					moves.push_back(commonMoves[x]);
					//					std::cout << t1 << "\n-and-\n"<< t2 << std::endl;
				}
			}
			else {
				moves.push_back(commonMoves[x]);
			}
		}
		else {
			moves.push_back(commonMoves[x]);
//			std::cout << commonMoves[x] << " and " << between <<
//			" can't be interchanged" << std::endl;
		}
	}
	// add non-common moves
	for (int x = 0; x < m2.size(); x++)
	{
		bool found = false;
		for (int y = 0; y < commonMoves.size(); y++)
		{
			if (m2[x] == commonMoves[y])
			{
				found = true;
				break;
			}
		}
		if (!found)
		{
			moves.push_back(m2[x]);
		}
	}
}

void FindLogicalMoves(unsigned long , tKeyboardModifier , char)
{
	printf("%d\n", pathLoc);
	if (pathLoc+1 >= path.size())
		return;
	FlingBoard s1, s2;
	s1 = path[pathLoc+1];
	s2 = path[pathLoc];
	// find moves taken between s1 and s2
	win = s2;
	GetReducedMoves(s1, s2, wins);
}

void SolveRandomFlingInstance(unsigned long windowID, tKeyboardModifier , char)
{
	b.Reset();
	for (int x = 0; x < 10; x++)
	{
		int x1 = random()%b.width;
		int y1 = random()%b.height;
		if (!b.HasPiece(x1, y1))
			b.AddFling(x1, y1);
	}
	std::cout << b << std::endl;
	
	std::vector<FlingBoard> path;
	BFS<FlingBoard, FlingMove, Fling> bfs;
	bfs.GetPath(&f, b, g, path);
}

void SolveAndSaveInstance(unsigned long , tKeyboardModifier , char)
{
	std::ofstream myfile;
	myfile.open("/Users/nathanst/boards.txt", std::ios::app | std::ios::out );
	
	BFS<FlingBoard, FlingMove, Fling> bfs;
	bfs.GetPath(&f, b, g, path);

	myfile << "Hash: " << f.GetStateHash(b) << std::endl;
	myfile << b << std::endl;
	myfile << "\n----\n\n";
	myfile.close();
	
	for (unsigned int x = 0; x < path.size(); x++)
	{
		std::cout << "(" << x << ")" << std::endl;
		std::cout << path[x] << std::endl << std::endl;
	}
	
	//b.Reset();
	pathLoc = path.size()-1;
}

const int THREADS = 16;
#include "BitVector.h"
std::vector<BitVector*> table;
std::vector<BitVector*> unique;
//std::vector<std::vector<BitVector*> > table;
int currSize = 2;
pthread_mutex_t writeLock = PTHREAD_MUTEX_INITIALIZER;
int64_t solvable;
int64_t uniqueSolvable;

void *ThreadedWorker(void *arg)
{
	int id = (long)arg;
	int64_t solved = 0;
	int64_t uniqueSolved = 0;
	FlingBoard currState, tmp;
	std::vector<FlingBoard> succ;
	std::vector<FlingMove> acts;

	std::vector<int64_t> buffer;
	std::vector<int64_t> uniqueBuffer;
	buffer.reserve(4*1024+100);
	uniqueBuffer.reserve(4*1024+100);
	for (int64_t val = id; val < f.getMaxSinglePlayerRank(56, currSize); val+=THREADS)
	{
		f.unrankPlayer(val, currSize, currState);
		//f.GetSuccessors(currState, succ);
		f.GetActions(currState, acts);
		if (currSize == 2)
		{
			if (acts.size() > 0)
			{
				buffer.push_back(val);
				uniqueBuffer.push_back(val);
				//table[currSize][val] = true;
				solved++;
				uniqueSolved++;
			}
		}
		else {
			int cnt = 0;
			int uniqueCnt = 0;
			for (int x = 0; x < acts.size(); x++)
			{
				f.GetNextState(currState, acts[x], tmp);
				uint64_t rank = f.rankPlayer(tmp);
				if (table[tmp.locs.size()]->Get(rank))
				{
					cnt++;
				}
				if (unique[tmp.locs.size()]->Get(rank))
				{
					uniqueCnt++;
				}
			}
			if (cnt > 0) {
				buffer.push_back(val);
				solved++;
			}
			if (cnt == 1 && uniqueCnt == 1)
			{
				uniqueBuffer.push_back(val);
				uniqueSolved++;
//				pthread_mutex_lock (&writeLock);
//				std::cout << currState << std::endl;
//				std::cout << acts.size() << " moves; ";
//				std::cout << cnt << " lead to solvable states; ";
//				std::cout << uniqueCnt << " of these are unique\n";
//				pthread_mutex_unlock(&writeLock);
			}
		}
		// flush buffer
		if (buffer.size() > 4*1024)
		{
			pthread_mutex_lock (&writeLock);
			while (buffer.size() > 0)
			{
				table[currSize]->Set(buffer.back(), true);// = true;
				buffer.pop_back();
			}
			while (uniqueBuffer.size() > 0)
			{
				unique[currSize]->Set(uniqueBuffer.back(), true);// = true;
				uniqueBuffer.pop_back();
			}
			pthread_mutex_unlock(&writeLock);
		}
	}

	pthread_mutex_lock (&writeLock);
	while (buffer.size() > 0)
	{
		table[currSize]->Set(buffer.back(), true);
		buffer.pop_back();
	}
	while (uniqueBuffer.size() > 0)
	{
		unique[currSize]->Set(uniqueBuffer.back(), true);// = true;
		if (uniqueBuffer.size() == 1)
		{
			f.unrankPlayer(uniqueBuffer.back(), currSize, b);
		}
		uniqueBuffer.pop_back();
	}
	solvable += solved;
	uniqueSolvable += uniqueSolved;
	pthread_mutex_unlock(&writeLock);

	pthread_exit(NULL);
}

void ExtractUniqueStates(int depth)
{
	char fname[255];
	sprintf(fname, "fling-unique-%d.dat", depth);
	printf("Reading from '%s'\n", fname);
	BitVector *b = new BitVector(f.getMaxSinglePlayerRank(56, depth), fname);
	uint64_t maxVal = f.getMaxSinglePlayerRank(56, depth);
	FlingBoard board;
	for (uint64_t t = 0; t < maxVal; t++)
	{
		if (b->Get(t))
		{
			f.unrankPlayer(t, depth, board);
			bool ok = false;
			for (unsigned int x = 0; x < board.width; x++)
			{
				if (board.HasPiece(x, 0))
				{
					ok = true;
					break;
				}
			}
			if (ok == false)
				continue;
			ok = false;
			for (unsigned int y = 0; y < board.height; y++)
			{
				if (board.HasPiece(0, y))
				{
					ok = true;
					break;
				}
			}
			if (ok == false)
				continue;
			printf("Rank: %llu\n", t);
			printf("Board: %llu\n", board.GetRawBoard());

			//if (depth < 5)
			{
				BFS<FlingBoard, FlingMove, Fling> bfs;
				bfs.GetPath(&f, board, g, path);
				//printf("%llu total nodes expanded in pure bfs\n", bfs.GetNodesExpanded());
				uint64_t nodesExpanded = DoLimitedBFS(board, path);
				//printf("%llu total nodes expanded in logically limited bfs\n", nodesExpanded);
				printf("Board: %llu Full: %llu Reduced: %llu Ratio: %1.4f\n", board.GetRawBoard(), bfs.GetNodesExpanded(), nodesExpanded, float(bfs.GetNodesExpanded())/float(nodesExpanded));
//				std::cout << board << std::endl;
			}
		}
	}
	b->Save(fname);
}

void BuildTables(unsigned long , tKeyboardModifier, char)
{
	table.resize(currSize+1);
	unique.resize(currSize+1);
	std::cout << "Starting work on board with " << currSize << " pieces. ";
	std::cout << f.getMaxSinglePlayerRank(56, currSize) << " entries." << std::endl;

	char fname[255];
	sprintf(fname, "fling-%d.dat", currSize);
	table[currSize] = new BitVector(f.getMaxSinglePlayerRank(56, currSize), fname);
	sprintf(fname, "fling-unique-%d.dat", currSize);
	unique[currSize] = new BitVector(f.getMaxSinglePlayerRank(56, currSize), fname);
	//table[currSize].resize(f.getMaxSinglePlayerRank(56, currSize));

	solvable = 0;
	uniqueSolvable = 0;
	Timer t;
	t.StartTimer();
	std::vector<pthread_t> threads(THREADS);
	for (int x = 0; x < THREADS; x++)
	{
		pthread_create(&threads[x], NULL, ThreadedWorker, (void *)x);
	}
	for (int x = 0; x < THREADS; x++)
	{
		int result = pthread_join(threads[x], NULL);
		if (result != 0)
		{
			printf("Unknown error joining with thread %d\n", x);
		}
	}
	double perc = solvable;
	perc /= (double)f.getMaxSinglePlayerRank(56, currSize);
	printf("%lld are solvable (%3.1f%%)\n", solvable, 100*perc);

	perc = uniqueSolvable;
	perc /= (double)f.getMaxSinglePlayerRank(56, currSize);
	printf("%lld are uniquely solvable (%3.1f%%)\n%3.2f sec elapsed\n", uniqueSolvable, 100*perc, t.EndTimer());
	fflush(stdout);
	// TODO: save bits -- old code did so when using mmap
//	char fname[255];
//	sprintf(fname, "/Users/nathanst/fling-%d.dat", currSize);
	//t.StartTimer();
	//WriteData(table[currSize], fname);
	//printf("%3.2f sec writing data to disk\n", t.EndTimer());
	//std::cout << solvable << " are solvable " << std::setprecision(3) << perc << std::endl;
	//std::cout << t.EndTimer() << " elapsed" << std::endl;
	currSize++;
}

void ReadTables(unsigned long , tKeyboardModifier, char)
{
	table.resize(0); // clear old data
//	table.resize(9);

	Timer t;
	for (int x = 2; x <= 10; x++)
	{
		table.resize(x+1);
		std::cout << "Loading board with " << x << " pieces. ";
		std::cout << f.getMaxSinglePlayerRank(56, x) << " entries." << std::endl;
		//table[x] = new BitVector(f.getMaxSinglePlayerRank(56, x));
		//table[x].resize(f.getMaxSinglePlayerRank(56, x));

		char fname[255];
		sprintf(fname, "/Users/nathanst/hog2/apps/fling/fling-%d.dat", x);
		t.StartTimer();
		table[x] = new BitVector(f.getMaxSinglePlayerRank(56, x), fname);
//		if (ReadData(table[x], fname) != true)
//		{
//			printf("Error reading data\n");
//		}
		printf("%3.2f sec reading data from disk\n", t.EndTimer());
	}
}

std::unordered_map<uint64_t,bool> visitedStates;
bool useTable = true;

bool IsSolvable(FlingBoard &theState)
{
	if (theState.locs.size() == 1)
		return true;
	if (useTable && theState.locs.size() < table.size())
	{
		// lookup
		//return (table[theState.locs.size()][f.rankPlayer(theState)]);
		if (table[theState.locs.size()] != 0)
			return (table[theState.locs.size()]->Get(f.rankPlayer(theState)));
		else {
			printf("Table size %d is null\n", theState.locs.size());
			return true;
		}
	}
	// in hash table
	if (visitedStates.find(f.rankPlayer(theState)) != visitedStates.end())
	{
		return visitedStates[f.rankPlayer(theState)];
	}
	std::vector<FlingMove> moves;
	f.GetActions(theState, moves);
	FlingBoard cpy;
	for (unsigned int x = 0; x < moves.size(); x++)
	{
		f.GetNextState(theState, moves[x], cpy);
		if (IsSolvable(cpy))
		{
			visitedStates[f.rankPlayer(theState)] = true;
			return true;
		}
	}
	visitedStates[f.rankPlayer(theState)] = false;
	return false;
}

uint64_t RecursiveBFS(FlingBoard from, std::vector<FlingBoard> &thePath, Fling *env)
{
	typedef std::unordered_map<uint64_t, uint64_t, Hash64> BFSClosedList;
	std::deque<FlingBoard> mOpen;
	std::deque<int> depth;
	BFSClosedList mClosed; // store parent id!
	thePath.resize(0);
	FlingBoard parentState;
	std::vector<FlingMove> moves;
	
	uint64_t nodesExpanded = 0;
	uint64_t weightedNodes = 0;
	int weight = from.locs.size();
	
	mOpen.clear();
	mClosed.clear();
	depth.clear();
	
	depth.push_back(0);
	mOpen.push_back(from);
	mClosed[env->GetStateHash(from)] = env->GetStateHash(from);
	//	printf("Setting parent of %llu to be %llu\n", env->GetStateHash(from),
	//		   env->GetStateHash(from));
	
	int currDepth = 0;
	uint64_t lastNodes = 0, lastIter = 0;
	FlingBoard s, tmp;
	s.SetObstacles(from.GetObstacles());
	tmp.SetObstacles(from.GetObstacles());
	while (mOpen.size() > 0)
	{
		assert(mOpen.size() == depth.size());
		s = mOpen.front();
		mOpen.pop_front();
		if (depth.front() != currDepth)
		{
//			printf("%d tot %llu inc %lld b %.2f\n", currDepth, nodesExpanded, nodesExpanded-lastNodes, (double)(nodesExpanded-lastNodes)/lastIter);
			lastIter = nodesExpanded-lastNodes;
			lastNodes = nodesExpanded;
		}
		currDepth = depth.front();
		depth.pop_front();
		
		nodesExpanded++;
		//weightedNodes += (weight-currDepth);
		if (mClosed.find(env->GetStateHash(s)) != mClosed.end())
		{
			moves.resize(0);
			env->GetStateFromHash(mClosed[env->GetStateHash(s)], parentState);
			parentState.SetObstacles(from.GetObstacles());
			if (currDepth > 0)
			{
				GetReducedMoves(parentState, s, moves);

//				std::cout << "From:\n" << parentState << "To:\n" << s << "Moves:";
//				for (auto &x : moves)
//					std::cout << x << " ";
//				std::cout << "\n";
			}
			else {
				env->GetActions(s, moves);
			}
			for (int x = 0; x < moves.size(); x++)
			{
				env->GetNextState(s, moves[x], tmp);
				thePath.push_back(tmp);
			}
		}
		else {
			env->GetSuccessors(s, thePath);
		}

		// TODO: print state and generated actions
		for (unsigned int x = 0; x < thePath.size(); x++)
		{
			if (mClosed.find(env->GetStateHash(thePath[x])) == mClosed.end())
			{
				mOpen.push_back(thePath[x]);
				depth.push_back(currDepth+1);
				//				printf("Setting parent of %llu to be %llu\n", env->GetStateHash(thePath[x]),
				//					   env->GetStateHash(s));
				mClosed[env->GetStateHash(thePath[x])] = env->GetStateHash(s);
//				FlingBoard tmpb;
//				GetMirror(thePath[x], tmpb, true, false);
//				std::cout << thePath[x].GetRawBoard() << "\n";
//				std::cout << tmpb.GetRawBoard() << "\n";
			}
		}
	}
//	printf("%d tot %llu inc %lld b %.2f\n", currDepth, nodesExpanded, nodesExpanded-lastNodes, (double)(nodesExpanded-lastNodes)/lastIter);
//	std::cout << "Final state:\n" << s << std::endl;
	
	thePath.resize(0);
	uint64_t parent, lastParent;
	//	std::cout << s << std::endl;
	do {
		lastParent = env->GetStateHash(s);
		thePath.push_back(s);
		parent = mClosed[env->GetStateHash(s)];
		env->GetStateFromHash(parent, s);
		//		std::cout << s << std::endl;
	} while (parent != lastParent);
//	printf("Final depth: %d, Nodes Expanded %llu, Exponential BF: %f\n", currDepth, nodesExpanded, pow(nodesExpanded, (double)1.0/currDepth));
	return nodesExpanded;
//	return weightedNodes;
}

uint64_t DoLimitedBFS(FlingBoard b, std::vector<FlingBoard> &path)
{
	return RecursiveBFS(b, path, &f);
}

void BFSearch(unsigned long windowID, tKeyboardModifier mod, char key)
{
	text.Clear();
	Timer t;
	t.StartTimer();
	uint64_t nodesExpanded;
	// do limited search
	if (mod == kShiftDown || key == 'B')
	{
		text.AddLine("Limited search");
		nodesExpanded = DoLimitedBFS(b, path);
	}
	else {
		text.AddLine("Full search");
		BFS<FlingBoard, FlingMove, Fling> bfs;
		bfs.GetPath(&f, b, g, path);
		pathLoc = path.size()-1;
		nodesExpanded = bfs.GetNodesExpanded();
	}
	t.EndTimer();
	
	char line[255];
	sprintf(line, "%llu total nodes", nodesExpanded);
	text.AddLine(line);
	sprintf(line, "Problem %s solvable (%4.3f elapsed)", (path.size() == b.locs.size())?"is":"is not", t.GetElapsedTime());
	text.AddLine(line);
}

void MassAnalysis(unsigned long w, tKeyboardModifier mod, char c)
{
	for (int x = 0; x < 100; x++)
	{
		// generate random board with N pieces
		b.Reset();
		while (b.locs.size() < 16)
		{
			int next = random()%56;
			if (!b.HasPiece(next))
				b.AddFling(next);
		}
		AnalyzeBoard(w, kNoModifier, 'a');
		AnalyzeBoard(w, kNoModifier, 'A');
	}
}

void RemoveDuplicates(unsigned long , tKeyboardModifier , char )
{
	RemoveDups();
}

void CaptureScreen(unsigned long windowID, tKeyboardModifier mod, char c)
{
	static int cnt = 0;
	char fname[255];
	sprintf(fname, "/Users/nathanst/Movies/FLING-%d%d%d", (cnt/100)%10, (cnt/10)%10, cnt%10);
	//SaveScreenshot(windowID, fname);
	printf("Saved %s\n", fname);
	cnt++;
}

// 1. count size of tree
// (store full and logically reduced size)

void GetWinningMoves(const FlingBoard &b, std::vector<FlingMove> &w)
{
	w.resize(0);
	std::vector<FlingMove> moves;
	f.GetActions(b, moves);
	FlingBoard cpy;
	//	std::cout << " Testing:\n" << b << std::endl;
	for (unsigned int x = 0; x < moves.size(); x++)
	{
		f.GetNextState(b, moves[x], cpy);
		//std::cout << "Move " << moves[x] << " leads to \n" << cpy << std::endl;
		if (IsSolvable(cpy))
		{
			w.push_back(moves[x]);
			//			std::cout << moves[x] << " is a solving move\n";
		}
	}
}

void AnalyzeBoard(unsigned long , tKeyboardModifier mod, char c)
{
	counts.resize(0);
	if (mod == kShiftDown || c == 'A')
	{
		// don't use db
		useTable = false;
		printf("NOT using pre-computed table\n");
	}
	else {
		useTable = true;
		printf("USING pre-computed table\n");
	}
	visitedStates.clear();
	// check every square to see if placing/removing a piece from there would make
	// the game solvable or not
	Timer t;
	t.StartTimer();
	valid.Reset();
	for (int x = 0; x < b.width*b.height; x++)
	{
		if (b.HasPiece(x))
		{
			b.RemoveFling(x);
			if (IsSolvable(b))
				valid.AddFling(x);
			b.AddFling(x);
		}
		else {
			b.AddFling(x);
			if (IsSolvable(b))
				valid.AddFling(x);
			b.RemoveFling(x);
		}
	}
//	std::cout << "Locs where we can add/remove pieces and still be solvable:" << std::endl;
//	std::cout << valid << std::endl;
	printf("%3.2f sec elapsed for placement analysis\n", t.EndTimer());
	// check to see what legal moves will solve the game
	// put them in wins array to be drawn on the screen
	t.StartTimer();
	GetWinningMoves(b, wins);
	win = b;
	printf("%3.2f sec elapsed for move analysis\n", t.EndTimer());
}

void DeepAnalyzeBoard(unsigned long , tKeyboardModifier mod, char c)
{
	if (mod == kShiftDown || c == 'D')
	{
		// don't use db
		useTable = false;
		printf("NOT using pre-computed table\n");
	}
	else {
		useTable = true;
		printf("USING pre-computed table\n");
	}
	visitedStates.clear();
	// check every square to see if placing/removing a piece from there would make
	// the game solvable or not
	Timer t;
	t.StartTimer();
	valid.Reset();
	std::vector<FlingMove> winCount;
	counts.resize(0);
	counts.resize(b.width*b.height);
	for (int x = 0; x < b.width*b.height; x++)
	{
		if (b.HasPiece(x))
		{
			b.RemoveFling(x);

			GetWinningMoves(b, winCount);
			if (winCount.size() > 0)
			{
				//if (IsSolvable(b))
				valid.AddFling(x);
				counts[x] = winCount.size();
			}
			b.AddFling(x);
		}
		else {
			b.AddFling(x);
//			if (IsSolvable(b))
//				valid.AddFling(x);
			GetWinningMoves(b, winCount);
			if (winCount.size() > 0)
			{
				//if (IsSolvable(b))
				valid.AddFling(x);
				counts[x] = winCount.size();
			}

			b.RemoveFling(x);
		}
	}
	//	std::cout << "Locs where we can add/remove pieces and still be solvable:" << std::endl;
	//	std::cout << valid << std::endl;
	printf("%3.2f sec elapsed for placement analysis\n", t.EndTimer());
	// check to see what legal moves will solve the game
	// put them in wins array to be drawn on the screen
	t.StartTimer();
	win = b;
	GetWinningMoves(b, wins);
	printf("%3.2f sec elapsed for move analysis\n", t.EndTimer());
}


void TestRanking(unsigned long , tKeyboardModifier, char)
{
	int cnt1;
	int board[56];
	for (int x = 0; x < 56; x++)
		board[x] = 0;
	UniquelySolvable(b, cnt1, board);
	for (int y = 0; y < 8; y++)
	{
		for (int x = 0; x < 7; x++)
		{
			std::cout << std::setw(4);
			std::cout << board[y*7+x];
		}
		std::cout << "\n";
	}
	std::cout << b.GetRawBoard() << " " << cnt1 << "\n";
}

bool MyClickHandler(unsigned long , int, int, point3d loc, tButtonType button, tMouseEventType event)
{
//	if (event != kMouseDown)
//		return false;
	
	if (button == kRightButton)
	{
		static int lastx, lasty;
		static bool active = false;
		int currx, curry;
		switch (event)
		{
			case kMouseDown:
				if (f.GetXYFromPoint(b, loc, lastx, lasty))
				{
					if (b.HasPiece(lastx, lasty))
					{
						active = true;
						printf("Trying to move (%d, %d)\n", lastx, lasty);
					}
				}
				break;
			case kMouseUp:
				printf("Handling mouse up!\n");
				if (f.GetXYFromPoint(b, loc, currx, curry) && active)
				{
					active = false;
					std::vector<FlingMove> acts;
					f.GetActions(b, acts);
					tFlingDir d = kLeft;
					if (currx == lastx && curry < lasty)
					{
						d = kUp;
						printf("Trying to move Up\n");
					}
					if (currx == lastx && curry > lasty)
					{
						d = kDown;
						printf("Trying to move Down\n");
					}
					if (currx < lastx && curry == lasty)
					{
						d = kLeft;
						printf("Trying to move Left\n");
					}
					if (currx > lastx && curry == lasty)
					{
						d = kRight;
						printf("Trying to move Right\n");
					}
					for (unsigned int x = 0; x < acts.size(); x++)
					{
						for (int y = 0; y < b.locs.size(); y++)
						{
							if (acts[x].dir == d &&
								b.locs[y].first == acts[x].startLoc &&
								b.locs[y].first == lasty*b.width+lastx)
							{
								printf("Applying action\n");
								f.ApplyAction(b, acts[x]);
								return true;
							}
						}
					}
				}
				break;
			default: break;
				//b.Reset();
		}
		return true;
	}
	else {
		if (event != kMouseDown)
			return false;

		int x, y;
		if (f.GetXYFromPoint(b, loc, x, y))
		{
			if (b.HasPiece(x, y))
			{
				b.RemoveFling(x, y);
				printf("State %llu\n", f.GetStateHash(b));
			}
			else {
				b.AddFling(x, y);
				printf("State %llu\n", f.GetStateHash(b));
			}
		}
		return true;
	}
	return false;
}

// these two states have the same solution actions (if 0 is flipped both directions)
// but they are different states because the pieces moved are different
//0
//..o..oo
//o......
//.......
//o......
//......o
//.......
//.......
//o......
//
//
//17
//.o.....
//......o
//.....o.
//.......
//o......
//.......
//.o...o.
//o......

// Do you consider these to be identical? They are pretty close...
//2
//o......
//.....o.
//.......
//...o.o.
//o......
//.......
//...o...
//o......
//
//
//8
//o......
//......o
//.......
//o......
//...o..o
//.......
//...o...
//o......

// These have the same solution, but should be different
// The issue is that the piece moved changes in one but not the other
//0
//...o...
//o......
//.......
//.......
//.......
//..o..o.
//.o.....
//.......
//
//
//5
//o...o..
//......o
//.......
//.oo....
//.......
//.......
//.......
//.......
//
void RemoveDups(std::vector<uint64_t> &values);
void RemoveDups()
{
	std::vector<uint64_t> values;
	while (true)
	{
		uint64_t value;
		int res = scanf("%llu", &value);
		if (res == EOF || res == 0 || value == 0)
			break;
		values.push_back(GetCanonicalHash(value));
	}
	std::reverse(values.begin(), values.end());
	RemoveDups(values);
}

void RemoveDups(std::vector<uint64_t> &values)
{
	std::vector<std::vector<FlingMove> > stateActs;

	for (size_t x = 0; x < values.size(); x++)
	{
		for (size_t y = x+1; y < values.size(); y++)
		{
			while (y < values.size() && values[x] == values[y])
				//if (BitCount(values[x]^values[y]) <= 4)
			{
				values.erase(values.begin()+y);
			}
		}
	}
	stateActs.resize(values.size());
	for (size_t x = 0; x < values.size(); x++)
	{
		f.GetStateFromHash(values[x], b);
		GetSolveActions(b, stateActs[x]);
		for (int t = 0; t < stateActs[x].size(); t++)
			std::cout << stateActs[x][t].dir << " ";
		printf("(%d)\n", x);
	}
	// find exact correlation between moves
	for (size_t x = 0; x < values.size(); x++)
	{
		for (size_t y = x+1; y < values.size(); y++)
		{
			f.GetStateFromHash(values[x], b);
			std::cout << x << "\n" << b << "\n\n";
			f.GetStateFromHash(values[y], b);
			std::cout << y << "\n" << b << "\n\n";
			printf("->Comparing board %d to board %d<-\n", x, y);
			int acts[4] = {-1, -1, -1, -1};
			bool match = true;
			// skip the first move, because the last one could go right OR left / down OR up
			for (size_t t = 0; t < stateActs[x].size(); t++)
				std::cout << stateActs[x][t].dir << " ";
			printf("(%d)\n", x);
			for (size_t t = 0; t < stateActs[y].size(); t++)
				std::cout << stateActs[y][t].dir << " ";
			printf("(%d)\n", y);

			// 1. need constant mapping between actions
			for (size_t t = 1; t < stateActs[x].size(); t++)
			{
				std::cout << "Comparing [" << t << "] " << stateActs[x][t].dir << " to " << stateActs[y][t].dir << "\n";
				if (acts[stateActs[x][t].dir] == -1)
				{
					std::cout << "-->Assigning " << stateActs[x][t].dir << " to " << stateActs[y][t].dir << std::endl;
					acts[stateActs[x][t].dir] = stateActs[y][t].dir;
				}
				else if (acts[stateActs[x][t].dir] != stateActs[y][t].dir)
				{
					match = false;
					break;
				}
				else {
					std::cout << "-->Matched " << stateActs[x][t].dir << " to " << stateActs[y][t].dir << std::endl;
				}
				printf("(%d : %d); (%d : %d); (%d : %d); (%d : %d)\n", 0, acts[0], 1, acts[1], 2, acts[2], 3, acts[3]);
			}
			if (!match)
				printf("Failed consistent map test\n");
			if (match)
			{
				for (int x = 0; x < 4; x++)
				{
					if (acts[x] == -1)
						continue;
					for (int y = x+1; y < 4; y++)
					{
						if (acts[x] == acts[y])
						{
							match = false;
						}
					}
				}
				if (!match)
					printf("Failed unique map test\n");
			}
			
			if (match)
			{
				// 2. also need constant relative flipping
				int diff = (abs(stateActs[x][1].dir - stateActs[y][1].dir)+2)%2;
				printf("Looking for offset of %d\n", diff);
				for (size_t t = 1; t < stateActs[x].size(); t++)
				{
					printf("Offset between %d and %d is %d\n", stateActs[x][t].dir, stateActs[y][t].dir,
						   (2+abs(stateActs[x][t].dir - stateActs[y][t].dir))%2);
					if (((2+abs(stateActs[x][t].dir - stateActs[y][t].dir))%2) != diff)
					{
						match = false;
						break;
					}
				}
				if (!match)
					printf("Failed symmetric flip test\n");
			}

			if (match)
			{
				// 3. also needs actions to move the same number of pieces
				FlingBoard s1a, s2a;
				FlingBoard s1b, s2b;
				f.GetStateFromHash(values[x], s1a);
				f.GetStateFromHash(values[y], s2a);
				
				f.GetStateFromHash(values[x], s1b);
				f.GetStateFromHash(values[y], s2b);
				f.ApplyAction(s1b, stateActs[x].back());
				f.ApplyAction(s2b, stateActs[y].back());

				for (int t = stateActs[x].size()-1; t > 0; t--)
				{
					if (BitCount(s1a.GetRawBoard()^s1b.GetRawBoard()) != BitCount(s2a.GetRawBoard()^s2b.GetRawBoard()))
					{
						std::cout << s1a << "\n" << s1b << "\n";
						printf("%d bits\n", BitCount(s1a.GetRawBoard()^s1b.GetRawBoard()));
						std::cout << s2a << "\n" << s2b << "\n";
						printf("%d bits\n", BitCount(s2a.GetRawBoard()^s2b.GetRawBoard()));
						match = false;
						break;
					}
					f.ApplyAction(s1a, stateActs[x][t]);
					f.ApplyAction(s2a, stateActs[y][t]);

					f.ApplyAction(s1b, stateActs[x][t-1]);
					f.ApplyAction(s2b, stateActs[y][t-1]);
				}
				if (!match)
					printf("Failed identical action test\n");
			}
			if (match)
			{
				FlingBoard s1a, s2a;
				FlingBoard s1b, s2b;
				f.GetStateFromHash(values[x], s1a);
				f.GetStateFromHash(values[y], s2a);
				
				f.GetStateFromHash(values[x], s1b);
				f.GetStateFromHash(values[y], s2b);
				f.ApplyAction(s1b, stateActs[x].back());
				f.ApplyAction(s2b, stateActs[y].back());
				
				for (int t = stateActs[x].size()-1; t > 1; t--)
				{
					bool same1 = ((s1a.GetRawBoard()^s1b.GetRawBoard())&(1ull<<stateActs[x][t-1].startLoc)); // same piece moved
					bool same2 = ((s2a.GetRawBoard()^s2b.GetRawBoard())&(1ull<<stateActs[y][t-1].startLoc)); // same piece moved
					if (same1 != same2)
					{
						std::cout << s1a << "\n" << s1b << "\n";
						std::cout << (same1?"moved same\n":"same not moved\n");
						std::cout << stateActs[x][t-1] << std::endl;
						std::cout << s2a << "\n" << s2b << "\n";
						std::cout << (same2?"moved same\n":"same not moved\n");
						std::cout << stateActs[y][t-1] << std::endl;
						match = false;
						break;
					}
					
					f.ApplyAction(s1a, stateActs[x][t]);
					f.ApplyAction(s2a, stateActs[y][t]);
					
					f.ApplyAction(s1b, stateActs[x][t-1]);
					f.ApplyAction(s2b, stateActs[y][t-1]);
				}
				
				if (!match)
					printf("Failed identical piece move test\n");
			}
			if (match)
			{
				std::cout << "->Match!" << std::endl;
				stateActs.erase(stateActs.begin()+y);
				values.erase(values.begin()+y);
				y--;
			}
			else {
				std::cout << "->No Match!" << std::endl;
			}

		}
	}
	printf("Final states:\n");
	for (size_t x = 0; x < values.size(); x++)
	{
		f.GetStateFromHash(values[x], b);
		std::cout << x << "\n" << b << "\n\n";
		std::cout << "hash:" << values[x] << "\n";
	}
}

#include <unordered_map>
template <typename Func>
int DFS(Fling &f, FlingBoard &b, std::unordered_map<uint64_t, int> &dd,
		Func func, bool includedups)
{
	int count = 0;
	if (b.NumPieces() == 1)
	{
		func(b);
		//goalLocations[b.GetPieceLocation(0)]++;
		return 1;
	}
	if (!includedups)
	{
		auto val = dd.find(b.GetRawBoard());
		if (val != dd.end())
		{
			return val->second;
		}
	}
	std::vector<FlingBoard> moves;
	f.GetSuccessors(b, moves);
	for (int x = 0; x < moves.size(); x++)
	{
		count += DFS(f, moves[x], dd, func, includedups);
	}
	dd[b.GetRawBoard()] = count;
	return count;
}

bool UniquelySolvable(FlingBoard &b, int &solutions)
{
	std::unordered_map<uint64_t, int> dd;
	int goalLocations[56];
	Fling f;
	solutions = DFS(f, b, dd, [&] (const FlingBoard &b) { goalLocations[b.GetPieceLocation(0)]++; }, false);
	return solutions == 2;
}

bool UniquelySolvable(FlingBoard &b, int &solutions, int *goalLocations)
{
	std::unordered_map<uint64_t, int> dd;
	Fling f;
	solutions = DFS(f, b, dd, [&] (const FlingBoard &b) { goalLocations[b.GetPieceLocation(0)]++; }, true);
	return solutions == 2;
}

void ThreadedEndLocAnalyze(int level, uint64_t start, uint64_t end, std::mutex *lock)
{
	Fling fling;
	uint64_t maxVal = fling.getMaxSinglePlayerRank(56, level);
	FlingBoard theBoard;
	for (uint64_t t = start; t < end; t++)
	{
		if (0 == t%1000000)
		{
			lock->lock();
			std::cout << t << " of " << maxVal << "\n";
			lock->unlock();
		}
		fling.unrankPlayer(t, level, theBoard);
		int cnt;
		int locs[56];
		for (int x = 0; x < 56; x++)
			locs[x] = 0;

		UniquelySolvable(theBoard, cnt, locs);
		int oneCnt = 0;
		int last1 = 0;
		int otherCnt = 0;
		int otherSum = 0;
		for (int x = 0; x < 56; x++)
		{
			if (locs[x] == 1)
			{
				oneCnt++;
				last1 = x;
			}
			else if (locs[x] > 0)
			{
				otherCnt++;
				otherSum+= locs[x];
			}
		}
		if (oneCnt == 1 && otherCnt > 0)
		{
			lock->lock();
			std::cout << "--=-=-=--\n";
			std::cout << theBoard << "\n";
			for (int y = 0; y < 8; y++)
			{
				for (int x = 0; x < 7; x++)
				{
					std::cout << std::setw(3) << locs[y*7+x];
				}
				std::cout << "\n";
			}
			std::cout << "A: ";
			std::cout << theBoard.GetRawBoard() << " " << last1 << " ";
			std::cout << cnt << " " << otherCnt << " " << otherSum << "\n";
			lock->unlock();
		}
	}
}

// This code is inefficient. It is more efficient to build up the solution
// from smaller boards upwards, but this was written quickly as test code
// and serves it purpose. It just won't be as great for analyzing really
// large boards.
void AnalyzeEndLocs(int level)
{
	uint64_t maxVal = f.getMaxSinglePlayerRank(56, level);
	const auto numThreads = std::thread::hardware_concurrency();
	std::cout << "Running with " << numThreads << " threads\n";
	
	uint64_t perThread = maxVal/numThreads;
	std::vector<std::thread*> threads;
	std::mutex lock;
	for (size_t x = 0; x < numThreads; x++)
	{
		threads.push_back(new std::thread(ThreadedEndLocAnalyze, level, x*perThread, (x+1)*perThread, &lock));
	}
	for (size_t x = 0; x < threads.size(); x++)
	{
		threads[x]->join();
		delete threads[x];
	}
}

void ThreadedRockAnalyze(int level, uint64_t start, uint64_t end, std::mutex *lock)
{
	Fling fling;
	uint64_t maxVal = fling.getMaxSinglePlayerRank(56, level);
	FlingBoard theBoard, g;
	theBoard.SetObstacle(24);
	theBoard.SetObstacle(31);
	std::vector<FlingBoard> path;
	BFS<FlingBoard, FlingMove, Fling> bfs;

	for (uint64_t t = start; t < end; t++)
	{
		if (0 == t%1000000)
		{
			lock->lock();
			std::cout << t << " of " << maxVal << "\n";
			lock->unlock();
		}
		fling.unrankPlayer(t, level, theBoard);
		if (theBoard.HasPiece(24) || theBoard.HasPiece(31))
			continue;
		theBoard.SetObstacle(24);
		theBoard.SetObstacle(31);

		bfs.GetPath(&fling, theBoard, g, path);
		int tmp;
		if (path.size() > level && UniquelySolvable(theBoard, tmp))
		{
			lock->lock();
			std::cout << "--=-=-=--\n";
			std::cout << theBoard << "\n";
			std::cout << "A: ";
			std::cout << theBoard.GetRawBoard() << " " << path.size() << "\n";
			lock->unlock();
		}
	}
}

// This code is also inefficient for the same reasons as above
void AnalyzeRocks(int level)
{
	uint64_t maxVal = f.getMaxSinglePlayerRank(56, level);
	const auto numThreads = std::thread::hardware_concurrency();
	std::cout << "Running with " << numThreads << " threads\n";
	
	uint64_t perThread = maxVal/numThreads;
	std::vector<std::thread*> threads;
	std::mutex lock;
	for (size_t x = 0; x < numThreads; x++)
	{
		threads.push_back(new std::thread(ThreadedRockAnalyze, level, x*perThread, (x+1)*perThread, &lock));
	}
	for (size_t x = 0; x < threads.size(); x++)
	{
		threads[x]->join();
		delete threads[x];
	}
}

void ThreadedFinalPieceAnalyze(int level, uint64_t start, uint64_t end, std::mutex *lock)
{
	Fling fling;
	uint64_t maxVal = fling.getMaxSinglePlayerRank(56, level);
	FlingBoard theBoard, g;
	std::vector<FlingBoard> path;
	BFS<FlingBoard, FlingMove, Fling> bfs;
	
	for (uint64_t t = start; t < end; t++)
	{
		if (0 == t%1000000)
		{
			lock->lock();
			std::cout << t << " of " << maxVal << "\n";
			lock->unlock();
		}
		fling.unrankPlayer(t, level, theBoard);
		
		std::unordered_map<uint64_t, int> dd;
		int finalPiece[14] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0};
		Fling f;
		int solutions = DFS(f, theBoard, dd, [&] (const FlingBoard &fb) { finalPiece[fb.locs[0].second]++; }, true);

		// don't show solutions which qualify under regular 1-solution rule
		// [[But, perhaps we could give the winning panda/final space as a clue?]]
		if (solutions <= 2)
			continue;
		int valid = 0;
		for (int x = 0; x < 14; x++)
		{
			if (finalPiece[x] == 1)
			{
				valid++;
			}
		}
		if (valid)
		{
			lock->lock();
			if (valid == level)
			{ std::cout << "AMAZING\n"; }
			std::cout << "--=-=-=--\n";
			std::cout << theBoard << "\n";
			std::cout << "A: ";
			std::cout << theBoard.GetRawBoard() << " " << path.size();

			BFS<FlingBoard, FlingMove, Fling> bfs;
			bfs.GetPath(&f, theBoard, g, path);
			uint64_t nodesExpanded = DoLimitedBFS(theBoard, path);
			printf(" Ratio: %llu %llu %1.4f\n", bfs.GetNodesExpanded(), nodesExpanded, double(bfs.GetNodesExpanded())/double(nodesExpanded));

			
			for (const auto &x : theBoard.locs)
			{
				std::cout << x.first << " " << x.second << " " << finalPiece[x.second] << "\n";
			}
			lock->unlock();
		}
	}
}

void AnalyzeFinalPieces(int level)
{
	uint64_t maxVal = f.getMaxSinglePlayerRank(56, level);
	int numThreads = std::thread::hardware_concurrency();
	std::cout << "Running with " << numThreads << " threads\n";
	
	uint64_t perThread = maxVal/numThreads;
	std::vector<std::thread*> threads;
	std::mutex lock;
	for (int x = 0; x < numThreads; x++)
	{
		threads.push_back(new std::thread(ThreadedFinalPieceAnalyze, level, x*perThread, (x+1)*perThread, &lock));
	}
	for (int x = 0; x < threads.size(); x++)
	{
		threads[x]->join();
		delete threads[x];
	}
}
