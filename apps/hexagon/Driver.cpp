/*
 *  $Id: Driver.cpp
 *  hog2
 *
 *  Created by Nathan Sturtevant on 5/31/05.
 *  Modified by Nathan Sturtevant on 11/13/21.
 *
 * This file is part of HOG2. See https://github.com/nathansttt/hog2 for licensing information.
 *
 */

#include <cstring>
#include "Common.h"
#include "Driver.h"
#include "Timer.h"
#include "SVGUtil.h"
#include "Hexagon.h"
#include "Combinations.h"
#include <filesystem>
#include <sys/types.h>
#include <sys/stat.h>
#include <climits>

using namespace std;

// CONTROL PANEL
string outputPath = ".";
int numberOfSolutionCutoff = INT_MAX, maxPuzzlesPerCat = 3000;//673;//INT_MAX
bool recording = false, svg = true, dotted = true, doColorSets = true;
bool practiceMode = false; int addedPieces = 2;//true

void GenerateCurriculum();

bool saveAndExit = false;
string filename, outputFile;
Hexagon h;
HexagonState hs;
vector<HexagonSearchState> goals;

vector<vector<HexagonAction>> acts;
HexagonSearchState hss, init;
HexagonEnvironment he;
int currDepth = 0;
long expansionsGlob = 0;//, generetions = 0, leaves = 0;

int main(int argc, char* argv[])
{
	for (int x = 0; x < argc; x++)
		printf("%s ", argv[x]);
	setvbuf(stdout, NULL, _IONBF, 0);
	InstallHandlers();
	RunHOGGUI(argc, argv, 512, 1024);
	return 0;
}

/**
 * Allows you to install any keyboard handlers needed for program interaction.
 */
void InstallHandlers()
{
	InstallKeyboardHandler(MyDisplayHandler, "Save goals", "Save all the currently generated goals", kAnyModifier, 's');
	InstallKeyboardHandler(MyDisplayHandler, "Next Action", "Take Next action", kAnyModifier, ']');
	InstallKeyboardHandler(MyDisplayHandler, "Prev Action", "Take prev action", kAnyModifier, '[');
	InstallKeyboardHandler(MyDisplayHandler, "Next", "Next DFS step", kAnyModifier, 'n');
	InstallKeyboardHandler(MyDisplayHandler, "All", "Get All Goals", kAnyModifier, 'a');
	InstallKeyboardHandler(MyDisplayHandler, "Flip", "Flip Board", kAnyModifier, 'f');
	InstallKeyboardHandler(MyDisplayHandler, "Rotate", "Rotate Board", kAnyModifier, 'r');
	InstallKeyboardHandler(MyDisplayHandler, "Analyze1", "Build curriculum", kAnyModifier, 'm');
	InstallKeyboardHandler(MyDisplayHandler, "Search", "Constraint space search", kAnyModifier, 'c');
	InstallKeyboardHandler(MyDisplayHandler, "Analyze2", "Analyze which pieces to make unflippable", kAnyModifier, 'o');
	InstallKeyboardHandler(MyDisplayHandler, "Get Coordinates", "Get baseline coordinates of all pieces", kAnyModifier, '=');
	
	InstallCommandLineHandler(MyCLHandler, "-generateCurriculum", "-generateCurriculum <location>", "Generate curriculum and store in the provided location");
	InstallCommandLineHandler(MyCLHandler, "-loadPuzzle", "-loadPuzzle <file>", "Load level from file.");
	InstallCommandLineHandler(MyCLHandler, "-loadSolution", "-loadSolution <file>", "Load solution from file.");
	InstallCommandLineHandler(MyCLHandler, "-comparePuzzles", "-comparePuzzles <directory>", "Load all files from directory and compare them to generated goals.");
	InstallCommandLineHandler(MyCLHandler, "-svg", "-svg <file>", "Write SVG to file. Also requires that a file is loaded.");
}

bool MyClickHandler(unsigned long, int, int, point3d p, tButtonType , tMouseEventType e)
{
	return false;
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
		SetNumPorts(windowID, 1);
		ReinitViewports(windowID, {-1, -1, 0, 1}, kScaleToSquare);
		AddViewport(windowID, {0, -1, 1, 1}, kScaleToSquare);
		if (true)
		{
			//			h.LoadSolution(filename.c_str(), hs);
			//			h.LoadPuzzle(filename.c_str(), hs);
		}
		acts.resize(1);
		he.GetActions(hss, acts[0]);
		currDepth = 0;
		
		if (saveAndExit)
		{
			Graphics::Display d;
			h.Draw(d);
			h.Draw(d, hs);
			MakeSVG(d, outputFile.c_str(), 1024, 1024);
			exit(0);
		}
	}
}


void MyFrameHandler(unsigned long windowID, unsigned int viewport, void *)
{
	Graphics::Display &d = GetContext(windowID)->display;
	if (viewport == 0)
	{
		he.Draw(d, hss);
		//		h.Draw(d);
		//		h.Draw(d, hs);
	}
	if (viewport == 1)
	{
		//		h.DrawSetup(d);
	}
}

int forbiddenPiece;

bool AddGoal(HexagonSearchState s)
{
	for (int x = 0; x < goals.size(); x++)
	{
		HexagonSearchState tmp = goals[x];
		for (int y = 0; y < 6; y++)
		{
			if (s == tmp)
			{
				return false;
			}
			he.RotateCW(tmp);
		}
		he.Flip(tmp);
		for (int y = 0; y < 6; y++)
		{
			if (s == tmp)
			{
				return false;
			}
			he.RotateCW(tmp);
		}
	}
	
	s.index = goals.size();
	s.forbiddenPiece = forbiddenPiece;
	he.BuildAdjacencies(s);
	
	goals.push_back(s);
	
	return true;
}

void CheckDuplicateGoals()
{
	cout << "Running duplicate analysis..\n";
	
	bool dup = false;
	
	for (int x = 0; x < goals.size(); x++)
	{
		HexagonSearchState s = goals[x];
		
		for (int y = x+1; y < goals.size(); y++)
		{
			if (x == y) continue;
			
			HexagonSearchState tmp = goals[y];
			for (int k = 0; k < 6; k++)
			{
				if (s == tmp)
				{
					cout << "Duplicate caught:" << x << "," << y << "," << k << "\n";
					dup = true;
				}
				he.RotateCW(tmp);
			}
			he.Flip(tmp);
			for (int k = 0; k < 6; k++)
			{
				if (s == tmp)
				{
					cout << "Duplicate caught F:" << x << "," << y << "," << k << "\n";
					dup = true;
				}
				he.RotateCW(tmp);
			}
		}
	}
	
	if (!dup) cout << "No duplicates found\n";
}

int MyCLHandler(char *argument[], int maxNumArgs)
{
	if (strcmp(argument[0], "-generateCurriculum") == 0)
	{
		if (maxNumArgs > 1)
		{
			outputPath = argument[1];
		}
		GenerateCurriculum();
		exit(0);
	}
	if (strcmp(argument[0], "-loadSolution") == 0)
	{
		if (maxNumArgs > 1)
		{
			h.LoadSolution(argument[1], hs);
			return 2;
		}
		printf("Error: too few arguments to -loadSolution. Input file required\n");
		exit(0);
	}
	else if (strcmp(argument[0], "-loadPuzzle") == 0)
	{
		if (maxNumArgs > 1)
		{
			h.LoadPuzzle(argument[1], hs);
			return 2;
		}
		printf("Error: too few arguments to -loadPuzzle. Input file required\n");
		exit(0);
	}
	else if (strcmp(argument[0], "-comparePuzzles") == 0)
	{
		if (maxNumArgs > 1)
		{
			//            array<array<uint64_t, numPieces>, (14*6*2+1)> locationsNew = he.GetLocationTable();
			vector<HexagonState> fileGoals, notFoundFiles;
			vector<int> matchedGoals;
			
			array<tFlipType, numPieces> toFlip;
			for (int x = 0; x < numPieces; x++) toFlip[x] = kHoles;
			he.SetFlippable(toFlip);
			
			he.SetPieces({kHexagon, kElbow, kLine, kMountains, kWrench, kTriangle, kHook, kButterfly, kTrapezoid, kTrapezoid});
			
			static int totalGoals = 0;
			static int totalExpansions = 0;
			totalGoals = totalExpansions = 0;
			goals.clear();
			hss.Reset();
			acts.resize(1);
			he.GetActions(hss, acts[0]);
			currDepth = 0;
			while (true)
			{
				if (acts[currDepth].size() > 0)
				{
					he.ApplyAction(hss, acts[currDepth].back());
					currDepth++;
					acts.resize(currDepth+1);
					he.GetActions(hss, acts.back());
					totalExpansions++;
				}
				else if (currDepth > 0) {
					currDepth--;
					he.UndoAction(hss, acts[currDepth].back());
					acts[currDepth].pop_back();
				}
				else {
					printf("& %lu & %d & %1.2f \\\\\n", goals.size(), totalGoals, totalExpansions/1000000.0);
					break;
				}
				if (he.GoalTest(hss))
				{
					AddGoal(hss);
					totalGoals++;
					printf("& %lu & %d & %1.2f \\\\\n", goals.size(), totalGoals, totalExpansions/1000000.0);
					
				}
			}
			
			return 2;
		}
		printf("Error: too few arguments to -comparePuzzles. Input file required\n");
		exit(0);
	}
	else if (strcmp(argument[0], "-svg") == 0)
	{
		if (maxNumArgs > 1)
		{
			saveAndExit = true;
			outputFile = argument[1];
			return 2;
		}
		printf("Error: too few arguments to -svg. Output filename required.\n");
		exit(0);
	}
	return 0;
}

uint64_t oddDots = he.BitsFromArray({1,3,5,8,10,12,14,17,19,21,23,25,27,29,31,33,35,37,38,40,42,44,46,47,49,51,53});
//broken -> uint64_t oddDots = he.BitsFromArray({1,3,5,7,8,10,12,14,16,17,19,21,23,25,27,29,31,33,35,37,40,42,44,46,49,51,53});


void GenerateAllSolutions()
{
	array<tFlipType, numPieces> toFlip;
	for (int x = 0; x < numPieces; x++)
		toFlip[x] = kHoles;
	he.SetFlippable(toFlip);
	
	const vector<tPieceName> allPieces =
	{kLine, kMountains, kWrench, kTriangle, kHook, kSnake, kHexagon, kElbow, kButterfly, kTrapezoid, kTrapezoid};
	
	
	vector<tPieceName> pieces =
	{kLine, kMountains, kWrench, kTriangle, kHook, kSnake, kHexagon, kElbow, kButterfly, kTrapezoid, kTrapezoid};
	
	forbiddenPiece = pieces.back();
	he.SetPieces(pieces);
	forbiddenPiece = pieces.back();
	MyDisplayHandler(0, kNoModifier, 'a');
	
	for (int x = 0; x < allPieces.size() - 1; x++)//allPieces.size() - 1
	{
		pieces = allPieces;
		
		forbiddenPiece = pieces[x];
		
		pieces.erase(pieces.begin()+x);
		he.SetPieces(pieces);
		int oldCount = goals.size();
		MyDisplayHandler(0, kNoModifier, 'a');
		
		printf("%s: %d ", pieceNames[allPieces[x]].c_str(), goals.size() - oldCount);
		
		if (goals.size() >= numberOfSolutionCutoff)
			break;
	}
	
	if (goals.size() < numberOfSolutionCutoff)
	{
		pieces = allPieces;
		pieces.pop_back();
		pieces.pop_back();
		
		forbiddenPiece = kTrapezoid;
		he.SetPieces(pieces);
		MyDisplayHandler(0, kNoModifier, 'a');
	}
}

void ConstraintSpaceSearch()
{
	Timer t;
	t.StartTimer();
	GenerateAllSolutions();
	t.EndTimer();
	cout<< "elapsed: " << t.GetElapsedTime() << " expansions: " << expansionsGlob << "\n";
	t.StartTimer();
	he.ConstraintSpaceSearch(goals);
	t.EndTimer();
	cout<< "elapsed: " << t.GetElapsedTime() << "\n";
}

void GenerateCurriculum()
{
	cout << "\n------------------------------------------------------\n";
	cout << "Generating all solutions...";
	cout << "\n------------------------------------------------------\n";
	
	GenerateAllSolutions();
	
	mkdir((outputPath + "/all_puzzles/").c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
	for (int x = 0; x < goals.size(); x++)
	{
		string fileName = outputPath + "/all_puzzles/"+ to_string(x) +".svg";
		Graphics::Display d;
		
		he.ConvertToHexagonState(goals[x], hs, true);
		hs.forbiddenPiece = goals[x].forbiddenPiece;
		
		h.Draw(d);
		h.Draw(d, hs);
		MakeSVG(d, fileName.c_str(), 1024, 1024);
	}
	
	cout << "\n------------------------------------------------------\n";
	cout << "Finished generating full solutions...";
	cout << "\n------------------------------------------------------\n";
	
	vector<vector<HexagonSearchState>> selectedSolutions(6);
	
	int categories = doColorSets ? 6 : 1;
	
	he.FullAnalysis(goals, selectedSolutions, categories);
	
	cout << "\n------------------------------------------------------\n";
	cout << "Generating full solution SVGs...";
	cout << "\n------------------------------------------------------\n";
	
	mkdir((outputPath + "/gen_puzzles/").c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
	
	for (int i = 0; i < categories; i++)
	{
		mkdir((outputPath + "/gen_puzzles/"+to_string(i)).c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
		
		vector<int> solCounts(goals.size());
		
		for (int j = 0; j < min(selectedSolutions[i].size(), maxPuzzlesPerCat); j++)
		{
			Graphics::Display d;
			
			string fileName = outputPath + "/gen_puzzles/"+ to_string(i) +"/" + to_string(selectedSolutions[i][j].index) + "-" + to_string(solCounts[selectedSolutions[i][j].index])+ ".svg";
			
			//            if (i==5) cout << "idx: " <<selectedSolutions[i][j].index << " constraint: " << selectedSolutions[i][j].constraints[0] << "\n";
			
			solCounts[selectedSolutions[i][j].index]++;
			
			he.ConvertToHexagonState(selectedSolutions[i][j], hs, true);
			
			hs.forbiddenPiece = selectedSolutions[i][j].forbiddenPiece;
			
			h.Draw(d);
			h.Draw(d, hs);
			
			if (svg)
				MakeSVG(d, fileName.c_str(), 1024, 1024);
		}
	}
	
	
	cout << "\n------------------------------------------------------\n";
	cout << "Finished generating solution SVGs";
	cout << "\n------------------------------------------------------\n";
	
	if (!practiceMode)
	{
		cout << "\n------------------------------------------------------\n";
		cout << "Calculating entropies...";
		cout << "\n------------------------------------------------------\n";
		
		for (int i = 0; i < categories; i++)
		{
			cout << "\nAnalyzing group: " << i << " ("<< selectedSolutions[i].size() << " items)\n";
			for (int j = 0; j < min(selectedSolutions[i].size(), maxPuzzlesPerCat); j++)
			{
				vector<tPieceName> pcs;
				for (int r = 0; r < selectedSolutions[i][j].cnt; r++)
				{
					tPieceName x = static_cast<tPieceName>(selectedSolutions[i][j].state[r].piece);
					pcs.push_back(x);
				}
				
				he.SetPieces(pcs);
				
				init = he.GetInitState(selectedSolutions[i][j]);
				selectedSolutions[i][j].entropy = selectedSolutions[i][j].entropyWithAddedPieces = he.GetEntropy(init);
				//            cout << "Entropy [" << selectedSolutions[i][j].index << "] = " << hss.entropy << "\n";
			}
		}
		
		cout << "\n------------------------------------------------------\n";
		cout << "Finished calculating entropies";
		cout << "\n------------------------------------------------------\n";
	}
	
	cout << "\n------------------------------------------------------\n";
	cout << "Generating inital state SVGs...";
	cout << "\n------------------------------------------------------\n";
	
	mkdir((outputPath + "/gen_puzzles_init/").c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
	
	for (int i = 0; i < categories; i++) {
		mkdir((outputPath + "/gen_puzzles_init/" + to_string(i)).c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
		vector<int> solCounts(goals.size());
		for (int j = 0; j < min(selectedSolutions[i].size(), maxPuzzlesPerCat); j++) {
			Graphics::Display d;
			
			string fileName = outputPath + "/gen_puzzles_init/" + to_string(i) + "/" + to_string(selectedSolutions[i][j].entropy) + "-" + to_string(selectedSolutions[i][j].index) + "-" + to_string(solCounts[selectedSolutions[i][j].index]) + ".svg";
			
			solCounts[selectedSolutions[i][j].index]++;
			
			//            selectedSolutions[i][j].dots = i == 0 ? 0 : oddDots;
			
			he.ConvertToHexagonState(selectedSolutions[i][j], hs);
			
			hs.forbiddenPiece = selectedSolutions[i][j].forbiddenPiece;
			hs.entropy = selectedSolutions[i][j].entropy;
			
			h.Draw(d);
			h.Draw(d, hs);
			
			if (svg)
				MakeSVG(d, fileName.c_str(), 1024, 1024);
		}
	}
	
	cout << "\n------------------------------------------------------\n";
	cout << "Finished generating inital state SVGs";
	cout << "\n------------------------------------------------------\n";
	
	if (practiceMode)
	{
		cout << "\n------------------------------------------------------\n";
		cout << "Generating easier init state SVGs...";
		cout << "\n------------------------------------------------------\n";
		
		mkdir((outputPath + "/gen_puzzles_init_modified/").c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
		
		for (int i = 0; i < categories; i++) {
			mkdir((outputPath + "/gen_puzzles_init_modified/" + to_string(i)).c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
			vector<int> solCounts(goals.size());
			int easier = 0;

			for (int j = 0; j < min(selectedSolutions[i].size(), maxPuzzlesPerCat); j++) {
				Graphics::Display d;
				
				vector<tPieceName> pcs;
				for (int r = 0; r < selectedSolutions[i][j].cnt; r++)
				{
					tPieceName x = static_cast<tPieceName>(selectedSolutions[i][j].state[r].piece);
					pcs.push_back(x);
				}
				
				he.SetPieces(pcs);
				
				
				solCounts[selectedSolutions[i][j].index]++;
				
				he.AddPiecesToInitState(selectedSolutions[i][j], addedPieces, easier == 1);
				
				he.ConvertToHexagonState(selectedSolutions[i][j], hs);
				
				init = he.GetInitState(selectedSolutions[i][j], &selectedSolutions[i][j].addedInitPieces);
				selectedSolutions[i][j].entropyWithAddedPieces = he.GetEntropy(init, 0);
				
				hs.forbiddenPiece = selectedSolutions[i][j].forbiddenPiece;
				hs.entropy = selectedSolutions[i][j].entropyWithAddedPieces;
				
				h.Draw(d);
				h.Draw(d, hs);
								
				string fileName = outputPath + "/gen_puzzles_init_modified/" + to_string(i) + "/" + to_string(selectedSolutions[i][j].entropyWithAddedPieces) + "-" + to_string(selectedSolutions[i][j].index) + "-" + to_string(solCounts[selectedSolutions[i][j].index]) + (easier == 0 ? " - HARDER - " : " - EASIER - ") + ".svg";
				
				
				if (svg)
					MakeSVG(d, fileName.c_str(), 1024, 1024);
			}
		}
		
		cout << "\n------------------------------------------------------\n";
		cout << "Finished generating modified inital state SVGs";
		cout << "\n------------------------------------------------------\n";
		
		cout << "\n------------------------------------------------------\n";
		cout << "Generating entropy inital state SVGs...";
		cout << "\n------------------------------------------------------\n";
		
		for(short mode : {0b1, 0b10, 0b100})// = 1; mode < 7; mode++)
		{
			mkdir((outputPath + "/gen_puzzles_entropy_" + to_string(mode)).c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
			
			for (int i = 0; i < categories; i++)
			{
				mkdir((outputPath + "/gen_puzzles_entropy_" + to_string(mode) + "/" + to_string(i)).c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
				vector<int> solCounts(goals.size());
				for (int j = 0; j < min(selectedSolutions[i].size(), maxPuzzlesPerCat); j++) {
					Graphics::Display d;
					
					vector<tPieceName> pcs;
					for (int r = 0; r < selectedSolutions[i][j].cnt; r++)
					{
						tPieceName x = static_cast<tPieceName>(selectedSolutions[i][j].state[r].piece);
						pcs.push_back(x);
					}
					
					he.SetPieces(pcs);
					
					init = he.GetInitState(selectedSolutions[i][j], &selectedSolutions[i][j].addedInitPieces);
					
					float newEntropy = he.GetEntropy(init, mode);
					float entropyDiff = selectedSolutions[i][j].entropyWithAddedPieces - newEntropy;
										
					string fileName = outputPath + "/gen_puzzles_entropy_" + to_string(mode) + "/" + to_string(i) + "/" + to_string(entropyDiff) + "-" + to_string(selectedSolutions[i][j].index) + "-" + to_string(solCounts[selectedSolutions[i][j].index]) + ".svg";
					
					solCounts[selectedSolutions[i][j].index]++;
					
					//            selectedSolutions[i][j].dots = i == 0 ? 0 : oddDots;
					
					he.ConvertToHexagonState(selectedSolutions[i][j], hs);
					
					hs.forbiddenPiece = selectedSolutions[i][j].forbiddenPiece;
					hs.entropy = newEntropy;
					
					h.Draw(d);
					h.Draw(d, hs);
					
					if (svg)
						MakeSVG(d, fileName.c_str(), 1024, 1024);
				}
			}
		}
		cout << "\n------------------------------------------------------\n";
		cout << "Finished entropy inital state SVGs";
		cout << "\n------------------------------------------------------\n";
	}
	
	cout << "\n------------------------------------------------------\n";
	cout << "END OF PROGRAM";
	cout << "\n------------------------------------------------------\n";
	
	return;
}

void AnalyzeWhichPiecesToFlip()
{
	Combinations<6> c;
	int items[3];
	for (uint64_t i = 0; i < c.MaxRank(3)*8; i++)
	{
		uint8_t order = i%8;
		// Get which itesms to be able to flip this time
		c.Unrank(i/8, items, 3);
		
		// Clear and then set the items
		array<tFlipType, numPieces> toFlip;
		for (int x = 0; x < numPieces; x++)
			toFlip[x] = kHoles;
		for (int x = 0; x < 3; x++)
		{
			switch(order&1)
			{
				case 0:
					toFlip[items[x]] = kSide1;
					break;
				case 1:
					toFlip[items[x]] = kSide2;
					break;
			}
			order/=2;
		}
		he.SetFlippable(toFlip);
		
		const vector<tPieceName> allPieces =
		{kLine, kMountains, kWrench, kTriangle, kHook, kSnake, kHexagon, kElbow, kButterfly, kTrapezoid, kTrapezoid};
		vector<tPieceName> pieces;
		
		printf("\n---- %s%s, %s%s, %s%s ----\n",
			   pieceNames[allPieces[items[0]]].c_str(), (toFlip[items[0]]==kSide1)?"1":(toFlip[items[0]]==kSide2)?"2":"",
			   pieceNames[allPieces[items[1]]].c_str(), (toFlip[items[1]]==kSide1)?"1":(toFlip[items[1]]==kSide2)?"2":"",
			   pieceNames[allPieces[items[2]]].c_str(), (toFlip[items[2]]==kSide1)?"1":(toFlip[items[2]]==kSide2)?"2":"");
		
		for (int x = 0; x < 1; x++)
		{
			pieces = allPieces;
			
			printf("%s%s ", pieceNames[allPieces[x]].c_str(), (toFlip[x]==kSide1)?"1":(toFlip[x]==kSide2)?"2":"");
			pieces.erase(pieces.begin()+x);
			he.SetPieces(pieces);
			MyDisplayHandler(0, kNoModifier, 'a');
		}
		// now remove trapezoids
		//		pieces = allPieces;
		//		pieces.pop_back();
		//		pieces.pop_back();
		//		printf("%s ", pieceNames[allPieces[9]].c_str());
		//		he.SetPieces(pieces);
		//		MyDisplayHandler(0, kNoModifier, 'a');
	}
}

int wasEqualSize = 0;

void MyDisplayHandler(unsigned long windowID, tKeyboardModifier mod, char key)
{
	switch (key)
	{
		case 's':
		{
			for (auto g : goals)
			{
				Graphics::Display d;
				d.StartFrame();
				he.Draw(d, g);
				d.EndFrame();
				
			}
			break;
		}
		case 'f':
			he.Flip(hss);
			break;
		case 'r':
			he.RotateCW(hss);
			break;
		case 'n':
		{
			static int totalGoals = 0;
			do
			{
				if (acts[currDepth].size() > 0)
				{
					he.ApplyAction(hss, acts[currDepth].back());
					currDepth++;
					acts.resize(currDepth+1);
					he.GetActions(hss, acts.back());
				}
				else if (currDepth > 0) {
					currDepth--;
					he.UndoAction(hss, acts[currDepth].back());
					acts[currDepth].pop_back();
				}
				else {
					printf("Done!\n");
					break;
				}
				if (he.GoalTest(hss))
				{
					if (AddGoal(hss))
					{
						he.ConvertToHexagonState(hss, hs);
						totalGoals++;
						//						printf("%d total goals\n", totalGoals);
						break;
					}
				}
			} while (true);
		}
			
			//
			break;
		case 'o':
			AnalyzeWhichPiecesToFlip();
			break;
		case 'm':
			GenerateCurriculum();
			break;
		case 'c':
			ConstraintSpaceSearch();
			break;
		case '=':
		{
			cout<<"\n\nGENERATING PIECE COORDS..\n\n";
			for (int x = 0; x < numPieces; x++)
			{
				he.GeneratePieceCoordinates((tPieceName)x);
			}
			cout<<"\n\nDONE GENERATING PIECE COORDS\n\n";
			cout<<"\n\GENERATING BOARD COORDS...\n\n";
			he.GenerateBoardBorder();
			cout<<"\n\DONE GENERATING BOARD COORDS\n\n";
			
			break;
		}
			break;
		case 'a':
		{
			static int totalGoals = 0;
			static int totalExpansions = 0;
			totalGoals = totalExpansions = 0;
			hss.Reset();
			hss.dots = dotted ? oddDots : 0;
			acts.resize(1);
			he.GetActions(hss, acts[0]);
			currDepth = 0;
			wasEqualSize = 0;
			
			while (true)
			{
				if (acts[currDepth].size() > 0 && goals.size() < numberOfSolutionCutoff)
				{
					he.ApplyAction(hss, acts[currDepth].back());
					currDepth++;
					acts.resize(currDepth+1);
					he.GetActions(hss, acts.back());
					totalExpansions++;
					expansionsGlob++;
				}
				else if (currDepth > 0 && goals.size() < numberOfSolutionCutoff)
				{
					currDepth--;
					he.UndoAction(hss, acts[currDepth].back());
					acts[currDepth].pop_back();
				}
				else {
					//                    CheckDuplicateGoals();
					break;
				}
				if (he.GoalTest(hss))
				{
					int sizeBefore = goals.size();
					AddGoal(hss);
					if (goals.size() == sizeBefore)
					{
						if (wasEqualSize > 2)
							break;
						else
							wasEqualSize++;
					}
					else
						wasEqualSize = 0;
					
					//                    cout << goals.size() << " total goals\n";
					
					totalGoals++;
				}
			}
			break;
		}
		case ']':
			if (acts[currDepth].size() > 0)
			{
				he.ApplyAction(hss, acts[currDepth].back());
				currDepth++;
				acts.resize(currDepth+1);
				he.GetActions(hss, acts.back());
			}
			else {
				//printf("No more actions to apply at depth %d\n", currDepth);
			}
			break;
		case '[':
			if (acts[currDepth].size() == 0)
			{
				printf("Nothing to undo\n");
				break;
			}
			
			currDepth--;
			he.UndoAction(hss, acts[currDepth].back());
			acts[currDepth].pop_back();
			//
			//			he.ApplyAction(hss, acts[currAct]);
			//			currAct = (currAct+acts.size()-1)%acts.size();
			//			he.ApplyAction(hss, acts[currAct]);
			printf("%d\n", currDepth);
			break;
		default:
			break;
	}
	
	/*
	 main
	 chooseWhichPiecesToUse
	 generateAllPuzzlesForPieceSet
	 loopThroughAllPossibleHolePatternsForAllPiecesInTheSet
	 filterGeneratedPuzzlesBasedOnSelectedHolePatternSet
	 storeNumberOfRemainingPuzzlesAfterFiltration
	 
	 
	 questions:
	 - shape of array stored
	 - pattern representation
	 - which patterns to cover (some patterns are meaningless - dont cover them)
	 -
	 
	 */
	
}
