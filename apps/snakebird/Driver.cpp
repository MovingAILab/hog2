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

#include <cstring>
#include "Common.h"
#include "Driver.h"
#include "Timer.h"
#include <deque>
#include "Combinations.h"
#include "SVGUtil.h"
#include "TemplateAStar.h"
#include "BFS.h"
#include "ScreenTransition.h"

#include <sys/stat.h>
bool fileExists(const char *name)
{
	struct stat buffer;
	return (stat(name, &buffer) == 0);
}
#include "SnakeBird.h"

bool mapAlreadyLoaded = false;
bool recording = false;
bool quitWhenDoneRecording = false;
bool autoSolve = false;
double globalTime = 0;
double frameTime = 0; // seconds elapsed in current frame
double timePerFrame = 0.1; // seconds for drawing each step of frame
double frameRate = 1.0/30.0;
double gSpeedAdjust = 1;
SnakeBird::SnakeBirdAnimationStep actionInProgressStep;
bool actionInProgress = false;
SnakeBird::SnakeBirdAction inProgress;
bool gRefreshBackground = true;
bool gEditMap = false;
int gMouseX = -1;
int gMouseY = -1;
int gMouseEditorX = -1;
int gMouseEditorY = -1;
SnakeBird::SnakeBirdWorldObject gEditorMode;
bool assistiveEditor = true;
bool loadPrimer = false;

bool incrementalAnalysis = true;
const int kNotAnalyzed = 32768;
const int kDontAnalyze = -32768;
std::vector<int> editorOverlay;
int editorOverlayMax = 1;
int editorOverlayMin = -1;
int gSolutionLength = 0;

#ifdef __EMSCRIPTEN__
const int gNodeLimit = 1000000;
#else
const int gNodeLimit = 25000000;
#endif

enum EditorColor {
	kCanAdd,
	kCanRemove,
	kCannotAddRemove,
};
enum EditorColumn {
	kColumn1 = 1,
	kColumn2 = 9,
	kRightMargin = 20,
	kBottomMargin = 25
};
struct EditorItem {
	int x, y;
	SnakeBird::SnakeBirdWorldObject icon;
	std::string text;
	int xLineEnd;
	char keyEquivalent;
	bool selectable;
	bool highlightable;
};
std::vector<EditorItem> editorItems =
{
	{kColumn1, 1, SnakeBird::kNothing, "Edit Map", kColumn2, '\0', false, false},
	{kColumn1, 3, SnakeBird::kEmpty, "Sky", kColumn2, 'k', true, true},
	{kColumn1, 5, SnakeBird::kGround, "Ground", kColumn2, 'g', true, true},
	{kColumn1, 7, SnakeBird::kSpikes, "Spikes", kColumn2, 's', true, true},
	{kColumn1, 9, SnakeBird::kFruit, "Fruit", kColumn2, 'f', true, true},
	{kColumn1, 11, SnakeBird::kPortal1, "Portal", kColumn2, 'p', true, true},
	{kColumn1, 13, SnakeBird::kBlock1, "Block", kColumn2, 'b', true, true},
	{kColumn1, 15, SnakeBird::kExit, "Exit", kColumn2, 'x', true, true},
	{kColumn1, 17, SnakeBird::kSnake1, "Snake 1", kColumn2, 'i', true, true},
	{kColumn1, 19, SnakeBird::kSnake2, "Snake 2", kColumn2, 'j', true, true},

	{kColumn2, 1, SnakeBird::kNothing, "EPCG AI Analysis", kRightMargin, '\0', false, false},
	{kColumn2, 3, SnakeBird::kSpikes, "Increase Sol. Length", kRightMargin, 'c', false, true},
	{kColumn2, 5, SnakeBird::kSpikes, "Decrease Sol. Length", kRightMargin, 'v', false, true},

	{kColumn2, 9, SnakeBird::kFruit, "Increase Sol. Length", kRightMargin, '3', false, true},
	{kColumn2, 11, SnakeBird::kPortal2, "Increase Sol. Length", kRightMargin, '4', false, true},
	{kColumn2, 13, SnakeBird::kBlock1, "Increase Sol. Length", kRightMargin, '1', false, true},
	{kColumn2, 15, SnakeBird::kExit, "Increase Sol. Length", kRightMargin, '2', false, true},
};
int kSelectedEditorItem = 1;

std::string message = "Welcome to Anhinga! Eat the grapes (if present) and leave via the yellow exit.";
double messageBeginTime = globalTime-1;
double messageExpireTime = globalTime+10;

std::string editorMessage;

Timer worldClock;
double lastFrameStart;
LineTransition transition(20, 60, Colors::white);

SnakeBird::SnakeBird sb(20, 20);
SnakeBird::SnakeBirdState snake;
SnakeBird::SnakeBirdState lastFrameSnake;
SnakeBird::SnakeBird editor(kRightMargin, kBottomMargin);
int snakeControl = 0;
std::vector<SnakeBird::SnakeBirdState> history;
std::vector<SnakeBird::SnakeBirdState> future;

void SetupMapChanges();
void ProcessSingleMapChange();
void AnalyzeMapChanges(bool maximize, int nodeLimit=1000000);
void AnalyzeObject(SnakeBird::SnakeBirdWorldObject oldobj, SnakeBird::SnakeBirdWorldObject newobj, int nodeLimit=1000000);
void ChangeMap(int x, int y, tMouseEventType o);
EditorColor CanChangeMap(int x, int y);
void GetLevelSolution(int nodeLimit = 250000000);
void StepForwardStoredSolution();

int main(int argc, char* argv[])
{
	InstallHandlers();
	RunHOGGUI(argc, argv, 800, 400);
	return 0;
}

/**
 * Allows you to install any keyboard handlers needed for program interaction.
 */
void InstallHandlers()
{
	InstallKeyboardHandler(GamePlayKeyboardHandler, "Up", "Move up", kAnyModifier, 'w');
	InstallKeyboardHandler(GamePlayKeyboardHandler, "Up", "Move up", kAnyModifier, kUpArrow);
	InstallKeyboardHandler(GamePlayKeyboardHandler, "Down", "Move down", kAnyModifier, 's');
	InstallKeyboardHandler(GamePlayKeyboardHandler, "Down", "Move down", kAnyModifier, kDownArrow);
	InstallKeyboardHandler(GamePlayKeyboardHandler, "Left", "Move left", kAnyModifier, 'a');
	InstallKeyboardHandler(GamePlayKeyboardHandler, "Left", "Move left", kAnyModifier, kLeftArrow);
	InstallKeyboardHandler(GamePlayKeyboardHandler, "Right", "Move right", kAnyModifier, 'd');
	InstallKeyboardHandler(GamePlayKeyboardHandler, "Right", "Move right", kAnyModifier, kRightArrow);
	InstallKeyboardHandler(GamePlayKeyboardHandler, "Undo", "Undo last move", kAnyModifier, 'q');
	InstallKeyboardHandler(GamePlayKeyboardHandler, "Next", "Select Next Snake", kAnyModifier, 'e');
	InstallKeyboardHandler(GamePlayKeyboardHandler, "Run", "Run solution (build if needed)", kAnyModifier, 'n');
	InstallKeyboardHandler(GamePlayKeyboardHandler, "Reset", "Reset Level", kAnyModifier, 'r');
	InstallKeyboardHandler(GamePlayKeyboardHandler, "Framerate", "Adjust framerate", kAnyModifier, 'f');

	InstallKeyboardHandler(EditorStudyKeyboardHandler, "SimpleEditor", "SimpleEditor", kAnyModifier, '$');
	InstallKeyboardHandler(EditorStudyKeyboardHandler, "BetterEditor", "BetterEditor", kAnyModifier, '^');

#ifndef __EMSCRIPTEN__
	InstallKeyboardHandler(GamePlayKeyboardHandler, "Level", "Goto nth level", kAnyModifier, '0', '9');
	InstallKeyboardHandler(GamePlayKeyboardHandler, "Toggle", "Toggle 0..9 loading regular/primer levels", kAnyModifier, '/');
	InstallKeyboardHandler(GamePlayKeyboardHandler, "Print", "Print screen to file", kAnyModifier, '&');
	InstallKeyboardHandler(GamePlayKeyboardHandler, "Movie", "Record movie frames", kAnyModifier, 'o');
	InstallKeyboardHandler(GamePlayKeyboardHandler, "Solve", "Build solution", kAnyModifier, 'b');
	InstallKeyboardHandler(GamePlayKeyboardHandler, "Step", "Take next step in solution", kAnyModifier, ']');
#endif
//	InstallKeyboardHandler(GamePlayKeyboardHandler, "Change", "Make one change to increase solution length", kAnyModifier, 'c');
//	InstallKeyboardHandler(GamePlayKeyboardHandler, "Change", "Make one change to decrease solution length", kAnyModifier, 'x');
	InstallKeyboardHandler(GamePlayKeyboardHandler, "Edit", "Edit Level", kAnyModifier, 't');
	
	InstallKeyboardHandler(EditorKeyBoardHandler, "Fruit", "Edit Fruit", kAnyModifier, 'f');
	InstallKeyboardHandler(EditorKeyBoardHandler, "Exit", "Edit Exit", kAnyModifier, 'x');
	InstallKeyboardHandler(EditorKeyBoardHandler, "Ground", "Edit Ground", kAnyModifier, 'g');
	InstallKeyboardHandler(EditorKeyBoardHandler, "Spikes", "Edit Spikes", kAnyModifier, 's');
	InstallKeyboardHandler(EditorKeyBoardHandler, "Sky", "Edit Sky", kAnyModifier, 'k');
	InstallKeyboardHandler(EditorKeyBoardHandler, "Toggle Ground", "Toggle Ground Mode", kAnyModifier, 'h');
	InstallKeyboardHandler(EditorKeyBoardHandler, "Portal", "Edit Portals", kAnyModifier, 'p');
	InstallKeyboardHandler(EditorKeyBoardHandler, "Blocks", "Edit Blocks", kAnyModifier, 'b');
	InstallKeyboardHandler(EditorKeyBoardHandler, "SnakeBird1", "Edit SnakeBird1", kAnyModifier, 'i');
	InstallKeyboardHandler(EditorKeyBoardHandler, "SnakeBird2", "Edit SnakeBird2", kAnyModifier, 'j');
	InstallKeyboardHandler(EditorKeyBoardHandler, "Height", "Increase Height", kAnyModifier, 'u');
	InstallKeyboardHandler(EditorKeyBoardHandler, "Width", "Increase Width", kAnyModifier, 'y');
	InstallKeyboardHandler(EditorKeyBoardHandler, "Height", "Decrease Height", kAnyModifier, 'z');
	InstallKeyboardHandler(EditorKeyBoardHandler, "Width", "Decrease Width", kAnyModifier, 'm');
	InstallKeyboardHandler(EditorKeyBoardHandler, "Toggle", "Toggle Modes", kAnyModifier, 'd');
	InstallKeyboardHandler(EditorKeyBoardHandler, "Increase", "Increase Solution", kAnyModifier, 'c');
	InstallKeyboardHandler(EditorKeyBoardHandler, "Decrease", "Decrease Solution", kAnyModifier, 'v');
	InstallKeyboardHandler(EditorKeyBoardHandler, "EPCG+Block", "Increase length with block", kAnyModifier, '1');
	InstallKeyboardHandler(EditorKeyBoardHandler, "EPCG+Exit", "Increase length with exit", kAnyModifier, '2');
	InstallKeyboardHandler(EditorKeyBoardHandler, "EPCG+Fruit", "Increase length with fruit", kAnyModifier, '3');
	InstallKeyboardHandler(EditorKeyBoardHandler, "EPCG+Portal", "Increase length with portal", kAnyModifier, '4');
	InstallKeyboardHandler(EditorKeyBoardHandler, "Solve", "Show solution", kAnyModifier, 'n');

	InstallCommandLineHandler(MyCLHandler, "-change", "-change <input> <nodelimit> <iter>", "Run <iter> times to make the level harder or easier. Outputs the new depth and file coding.");
	InstallCommandLineHandler(MyCLHandler, "-load", "-load <file>", "Run snake bird with the given file");
	InstallCommandLineHandler(MyCLHandler, "-svg", "-svg <input> <output>", "Make an .svg of the given level then quit");
	InstallCommandLineHandler(MyCLHandler, "-printsvg", "-printsvg <input>", "Print an .svg of the given level then quit");
	InstallCommandLineHandler(MyCLHandler, "-bfs", "-bfs <file>", "Run BFS on the given level and return the info");
	InstallCommandLineHandler(MyCLHandler, "-analyzeSolution", "-analyzeSolution <file>", "Given a solution to the problem, find the maximum times that a single state was re-visited by the head of the first snakebird.");
	InstallCommandLineHandler(MyCLHandler, "-encode", "-encode <file>", "Encode level as string");
	InstallCommandLineHandler(MyCLHandler, "-decode", "-decode <string>", "Decode level from string");
	InstallCommandLineHandler(MyCLHandler, "-solve", "-solve <level>", "Solve level and output .svg files for the full solution");
//	InstallCommandLineHandler(MyCLHandler, "-test", "-test", "Basic test with MD heuristic");
	
	InstallWindowHandler(MyWindowHandler);
	InstallMouseClickHandler(MyClickHandler, static_cast<tMouseEventType>(kMouseMove|kMouseUp|kMouseDown|kMouseDrag));
}

void Save(const SnakeBird::SnakeBird &sb, const SnakeBird::SnakeBirdState &sbs, std::string prefix)
{
	Graphics::Display d;
	sb.Draw(d);
	sb.Draw(d, 0);
	sb.Draw(d, snake, snakeControl);
	std::string fname = "/Users/nathanst/Pictures/SVG/tmp/"+prefix;
	int count = 0;
	while (fileExists((fname+std::to_string(count)+".svg").c_str()))
	{
		count++;
	}
	printf("Save to '%s'\n", (fname+std::to_string(count)+".svg").c_str());
	MakeSVG(d, (fname+std::to_string(count)+".svg").c_str(), 400, 400, 0);
}

void UpdateActiveSnake()
{
	if (future.size() > 0)
	{
		auto act = sb.GetAction(snake, future.back());
		if (snake.IsInPlay(act.bird))
			snakeControl = act.bird;
	}

	if (snakeControl >= snake.GetNumSnakes())
		snakeControl = 0;
	
	// If snake went into goal, switch active snake
	for (int x = 0; x < snake.GetNumSnakes(); x++)
	{
		if (snake.IsInPlay(snakeControl))
			break;
		snakeControl = (snakeControl+1)%snake.GetNumSnakes();
	}
}

void UpdateLevelLink()
{
	setTextBufferVisibility(false);
	std::string str = "<input type=\"text\" value=\"https://movingai.com/snakebird/play.html#";
	str += sb.EncodeLevel();
	str += "\" id=\"playableurl\">";
	submitTextToBuffer(str.c_str());
	std::cout << str << "\n";
}

bool SnakeDead()
{
	return !sb.LivingState(snake);
}

bool NoActions()
{
	static std::vector<SnakeBird::SnakeBirdAction> acts;
	if (sb.GoalTest(snake, snake))
		return false;
	sb.GetActions(snake, acts);
	if (acts.size() == 0)
		return true;
	if (acts.size() > 1)
		return false;
	if (acts[0].direction != SnakeBird::kUp)
		return false;
	SnakeBird::SnakeBirdState tmp = snake;
	sb.ApplyAction(tmp, acts[0]);
	if (tmp == snake)
		return true;
	return false;
}

void LoadLevel19()
{
	sb.BeginEditing();
	sb.Reset();
	sb.SetGroundType(14, 5, SnakeBird::kFruit);

	sb.SetGroundType(1, 4, SnakeBird::kSpikes);
	sb.SetGroundType(1, 5, SnakeBird::kGround);
	sb.SetGroundType(4, 3, SnakeBird::kSpikes);
	sb.SetGroundType(4, 4, SnakeBird::kGround);
	sb.SetGroundType(3, 5, SnakeBird::kExit);

	sb.SetGroundType(4, 9, SnakeBird::kGround);
	sb.SetGroundType(4, 10, SnakeBird::kGround);
	sb.SetGroundType(4, 11, SnakeBird::kGround);
	sb.SetGroundType(4, 12, SnakeBird::kGround);

	sb.AddSnake(4, 6, {SnakeBird::kRight});
	sb.AddSnake(5, 7, {SnakeBird::kLeft, SnakeBird::kLeft, SnakeBird::kDown, SnakeBird::kDown, SnakeBird::kDown});
	sb.AddSnake(4, 8, {SnakeBird::kRight, SnakeBird::kDown, SnakeBird::kDown, SnakeBird::kDown});


	lastFrameSnake = snake = sb.GetStart();
	history.clear();
	history.push_back(snake);
	future.clear();
	actionInProgressStep.Reset();
	timePerFrame = 0.01;
	UpdateActiveSnake();
	sb.EndEditing();
	UpdateLevelLink();
	gRefreshBackground = true;
}

void LoadLevel22()
{
	sb.BeginEditing();
	sb.Reset();
	sb.SetGroundType(3, 7, SnakeBird::kGround);
	sb.SetGroundType(3, 8, SnakeBird::kGround);
	sb.SetGroundType(3, 9, SnakeBird::kGround);
	sb.SetGroundType(3, 10, SnakeBird::kGround);
	sb.SetGroundType(4, 7, SnakeBird::kGround);
	sb.SetGroundType(4, 8, SnakeBird::kGround);
	sb.SetGroundType(4, 9, SnakeBird::kGround);
	sb.SetGroundType(4, 10, SnakeBird::kGround);
	sb.SetGroundType(5, 10, SnakeBird::kGround);
	sb.SetGroundType(6, 8, SnakeBird::kGround);
	sb.SetGroundType(6, 10, SnakeBird::kGround);
	sb.SetGroundType(7, 10, SnakeBird::kGround);
	sb.SetGroundType(8, 9, SnakeBird::kGround);
	sb.SetGroundType(8, 10, SnakeBird::kGround);
	sb.SetGroundType(9, 9, SnakeBird::kGround);
	sb.SetGroundType(9, 10, SnakeBird::kGround);
	
	sb.SetGroundType(5, 1, SnakeBird::kExit);
	
	sb.AddSnake(4, 6, {SnakeBird::kLeft, SnakeBird::kLeft});
	//		sb.AddSnake(10, 10, {SnakeBird::kRight});
	//		sb.AddSnake(15, 10, {SnakeBird::kRight});
	//		sb.AddSnake(20, 10, {SnakeBird::kRight});
	
	sb.SetGroundType(8, 7, SnakeBird::kBlock1);
	sb.SetGroundType(8, 8, SnakeBird::kBlock1);
	sb.SetGroundType(9, 7, SnakeBird::kBlock1);
	sb.SetGroundType(9, 8, SnakeBird::kBlock1);
		
	lastFrameSnake = snake = sb.GetStart();
	history.clear();
	history.push_back(snake);
	future.clear();
	actionInProgressStep.Reset();
	timePerFrame = 0.01;
	UpdateActiveSnake();
	sb.EndEditing();
	UpdateLevelLink();
	gRefreshBackground = true;
}

void LoadLevel39()
{
	sb.BeginEditing();
	sb.Reset();
	for (int x = 3; x <= 14; x++)
		sb.SetGroundType(x, 12, SnakeBird::kGround);
	for (int x = 3; x <= 6; x++)
		sb.SetGroundType(x, 11, SnakeBird::kGround);
	for (int x = 9; x <= 14; x++)
		sb.SetGroundType(x, 11, SnakeBird::kGround);
	for (int x = 11; x <= 14; x++)
	{
		sb.SetGroundType(x, 9, SnakeBird::kGround);
		sb.SetGroundType(x, 6, SnakeBird::kGround);
	}
	for (int x = 10; x <= 14; x++)
	{
		sb.SetGroundType(x, 10, SnakeBird::kGround);
		sb.SetGroundType(x, 8, SnakeBird::kGround);
		sb.SetGroundType(x, 7, SnakeBird::kGround);
		sb.SetGroundType(x, 5, SnakeBird::kGround);
		sb.SetGroundType(x, 4, SnakeBird::kGround);
	}

	
	sb.SetGroundType(12, 1, SnakeBird::kExit);
	
	sb.AddSnake(6, 8, {SnakeBird::kLeft, SnakeBird::kLeft, SnakeBird::kDown, SnakeBird::kDown});
	//		sb.AddSnake(10, 10, {SnakeBird::kRight});
	//		sb.AddSnake(15, 10, {SnakeBird::kRight});
	//		sb.AddSnake(20, 10, {SnakeBird::kRight});
	
	sb.SetGroundType(5, 10, SnakeBird::kBlock1);
	sb.SetGroundType(6, 10, SnakeBird::kBlock1);
	sb.SetGroundType(5, 9, SnakeBird::kBlock2);
	sb.SetGroundType(6, 9, SnakeBird::kBlock2);
		
	lastFrameSnake = snake = sb.GetStart();
	history.clear();
	history.push_back(snake);
	future.clear();
	actionInProgressStep.Reset();
	timePerFrame = 0.01;
	UpdateActiveSnake();
	sb.EndEditing();
	UpdateLevelLink();
	gRefreshBackground = true;
}

void LoadLevel32()
{
	sb.BeginEditing();
	sb.Reset();
	for (int x = 5; x <= 10; x++)
		sb.SetGroundType(x, 9, SnakeBird::kGround);
	for (int x = 11; x <= 12; x++)
		sb.SetGroundType(x, 9, SnakeBird::kSpikes);
	for (int x = 12; x <= 14; x++)
		sb.SetGroundType(x, 7, SnakeBird::kGround);
	for (int y = 5; y <= 7; y++)
		sb.SetGroundType(8, y, SnakeBird::kGround);
	for (int y = 4; y <= 5; y++)
		sb.SetGroundType(13, y, SnakeBird::kGround);

	sb.SetGroundType(6, 6, SnakeBird::kExit);
	
	sb.AddSnake(9, 7, {SnakeBird::kRight});
	sb.AddSnake(10, 8, {SnakeBird::kLeft});

	sb.SetGroundType(12, 6, SnakeBird::kBlock1);

	sb.SetGroundType(8, 8, SnakeBird::kPortal1);
	sb.SetGroundType(13, 6, SnakeBird::kPortal2);

	lastFrameSnake = snake = sb.GetStart();
	history.clear();
	history.push_back(snake);
	future.clear();
	actionInProgressStep.Reset();
	timePerFrame = 0.01;
	UpdateActiveSnake();
	sb.EndEditing();
	UpdateLevelLink();
	gRefreshBackground = true;
}

void LoadLevel44()
{
	sb.BeginEditing();
	sb.Reset();
	sb.SetGroundType(4, 6, SnakeBird::kSpikes);
	sb.SetGroundType(6, 7, SnakeBird::kSpikes);
	sb.SetGroundType(4, 10, SnakeBird::kSpikes);
	sb.SetGroundType(11, 10, SnakeBird::kSpikes);
	for (int x = 5; x <= 8; x++)
		sb.SetGroundType(x, 10, SnakeBird::kGround);
	sb.SetGroundType(12, 10, SnakeBird::kGround);

	sb.SetGroundType(5, 6, SnakeBird::kFruit);
	sb.SetGroundType(8, 5, SnakeBird::kFruit);

	sb.SetGroundType(12, 5, SnakeBird::kExit);
	
	sb.AddSnake(5, 9, {SnakeBird::kRight});
	sb.AddSnake(6, 8, {SnakeBird::kLeft});

	sb.SetGroundType(6, 4, SnakeBird::kPortal1);
	sb.SetGroundType(8, 8, SnakeBird::kPortal2);

	lastFrameSnake = snake = sb.GetStart();
	history.clear();
	history.push_back(snake);
	future.clear();
	actionInProgressStep.Reset();
	timePerFrame = 0.01;
	UpdateActiveSnake();
	sb.EndEditing();
	UpdateLevelLink();
	gRefreshBackground = true;
}

void LoadLevel63()
{
	sb.BeginEditing();
	sb.Reset();
	for (int x = 5; x <= 11; x++)
		sb.SetGroundType(x, 6, SnakeBird::kGround);
	for (int x = 13; x <= 13; x++)
		sb.SetGroundType(x, 6, SnakeBird::kGround);
	for (int x = 14; x <= 15; x++)
		sb.SetGroundType(x, 5, SnakeBird::kGround);
	for (int x = 13; x <= 15; x++)
		sb.SetGroundType(x, 4, SnakeBird::kGround);

	sb.SetGroundType(14, 2, SnakeBird::kExit);
	
	sb.AddSnake(10, 5, {SnakeBird::kRight});
	sb.AddSnake(5, 5, {SnakeBird::kRight});
//	sb.AddSnakeHead(10,5);

	sb.SetGroundType(10, 4, SnakeBird::kBlock1);
	sb.SetGroundType(9, 3, SnakeBird::kBlock1);
	sb.SetGroundType(10, 3, SnakeBird::kBlock1);
	sb.SetGroundType(11, 3, SnakeBird::kBlock1);

	sb.SetGroundType(6, 3, SnakeBird::kPortal1);
	sb.SetGroundType(13, 5, SnakeBird::kPortal2);

	lastFrameSnake = snake = sb.GetStart();
	history.clear();
	history.push_back(snake);
	future.clear();
	actionInProgressStep.Reset();
	timePerFrame = 0.01;
	UpdateActiveSnake();
	sb.EndEditing();
	UpdateLevelLink();

	gRefreshBackground = true;
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
		if (!mapAlreadyLoaded)
		{
			//LoadLevel19();
			//LoadLevel22();
			//LoadLevel39();
			LoadLevel63();
			//LoadLevel32();
			//LoadLevel44();
		}
		if (sb.GetNumFruit() == 0)
			message = "Welcome to Anhinga! Leave via the yellow exit to pass the level.";
		else
			message = "Welcome to Anhinga! Eat the grapes and then leave via the yellow exit.";
		messageBeginTime = globalTime+0.5;
		messageExpireTime = globalTime+10;

		worldClock.StartTimer();
		lastFrameStart = worldClock.GetElapsedTime();
//		ReinitViewports(windowID, {-1.0f, -1.0f, 0.0f, 1.0f}, kScaleToSquare);
		ReinitViewports(windowID, {-1.0f, -1.0f, 0.0f, 1.0f}, kScaleToSquare);
		AddViewport(windowID, {1.0f, -1.0f, 2.0f, 1.0f}, kScaleToSquare); // (will be editor)
	}
}


static void DrawEditorViewport(unsigned long windowID)
{
	Graphics::Display &d = GetContext(windowID)->display;
	d.FillRect({-1, -1, 1, 1}, Colors::lightgray);
	if (gEditMap == false)
		return;
	
	if (incrementalAnalysis == true && editorOverlay.size() == sb.GetWidth()*sb.GetHeight())
		ProcessSingleMapChange();

	editor.SetColor(Colors::black); //keep the title black

//	editor.DrawLabel(d, kColumn1, 1, "Edit Map");
//	editor.DrawLabel(d, kColumn2, 1, "EPCG AI Analysis");

	for (size_t t = 0; t < editorItems.size(); t++)
	{
		if (kSelectedEditorItem == t)
		{
			editor.SetColor(rgbColor::mix(Colors::blue, Colors::cyan, 0.5));
			editor.Draw(d, editorItems[t].x, editorItems[t].y);
			editor.SetColor(rgbColor::mix(Colors::blue, Colors::cyan, 0.5));
		}
		else if (gMouseEditorY == editorItems[t].y &&
				 gMouseEditorX >= editorItems[t].x &&
				 gMouseEditorX < editorItems[t].xLineEnd &&
				 editorItems[t].highlightable) // mouse over
		{
			editor.SetColor(Colors::cyan);
		}
		else {
			editor.SetColor(Colors::black);
		}
		editor.DrawLabel(d, editorItems[t].x+1, editorItems[t].y, editorItems[t].text.c_str());
		editor.DrawObject(d, editorItems[t].x, editorItems[t].y, editorItems[t].icon, globalTime);
	}
	editor.SetColor(Colors::blue);
	editor.DrawLabel(d, kColumn1, 21, editorMessage.c_str());
//	return;
	
	if (recording)
	{
		std::string fname = "/Users/nathanst/Pictures/SVG/sb-editor_";
		static int count = 0;
		while (fileExists((fname+std::to_string(count)+".svg").c_str()))
		{
			count++;
		}
		printf("Save to '%s'\n", (fname+std::to_string(count)+".svg").c_str());
		MakeSVG(d, (fname+std::to_string(count)+".svg").c_str(), 400, 400, 1);
		
		if (!autoSolve && quitWhenDoneRecording)
		{
			exit(0);
		}
	}
	
	return;
}

static void DrawGameViewport(unsigned long windowID) {
	frameRate = worldClock.EndTimer()-lastFrameStart;
	frameRate *= gSpeedAdjust;
	lastFrameStart = worldClock.EndTimer();
	if (recording)
		frameRate = 1.0/60.0; // fixed frame rate when recording
	//	printf("%f\n", frameRate);
	//	frameRate = 1.0/120.0; // fixed frame rate when recording
	//	frameRate /= 20;
	
	globalTime += frameRate;
	frameTime += frameRate;
	if (frameTime > timePerFrame)
	{
		frameTime = 0;
		if (actionInProgress == true)
		{
			lastFrameSnake = snake;
			bool complete = sb.ApplyPartialAction(snake, inProgress, actionInProgressStep);
			actionInProgress = !complete;
			timePerFrame = actionInProgressStep.animationDuration;
			
			if (!actionInProgress)
			{
				if (SnakeDead())
					GamePlayKeyboardHandler(windowID, kNoModifier, 'q');
				UpdateActiveSnake();
				
				if (autoSolve == false && NoActions())
				{
					message = "No legal actions; press undo or reset level to continue";
					messageBeginTime = globalTime+0.5;
					messageExpireTime = globalTime+10;
				}
				
			}
			
			if (actionInProgress == false && autoSolve == true)
			{
				StepForwardStoredSolution();
			}
		}
		else {
			autoSolve = false;
			lastFrameSnake = snake;
			if (SnakeDead())
			{
				GamePlayKeyboardHandler(windowID, kNoModifier, 'q');
			}
		}
	}
	Graphics::Display &d = GetContext(windowID)->display;
	if (gRefreshBackground)
	{
		d.StartBackground();
		sb.Draw(d);
		d.EndBackground();
	}
	//sb.Draw(d, snake, snakeControl, globalTime);
	sb.Draw(d, globalTime);
	// TODO::FIX THIS!
	if (timePerFrame <= 0)
		timePerFrame = 0.01;
	sb.Draw(d, lastFrameSnake, snake, snakeControl, frameTime/timePerFrame, globalTime);
	if (gEditMap == true)
	{
		// draw hints at possible changes - drawn below indications from cursor
		if (incrementalAnalysis == true && editorOverlay.size() == sb.GetWidth()*sb.GetHeight())
		{
			for (int x = 0; x < sb.GetWidth(); x++)
			{
				for (int y = 0; y < sb.GetHeight(); y++)
				{
					if (editorOverlay[y*sb.GetWidth()+x] != kNotAnalyzed &&
						editorOverlay[y*sb.GetWidth()+x] != kDontAnalyze &&
						editorOverlay[y*sb.GetWidth()+x] != 0)
					{
						
						sb.SetColor(Colors::yellow);
						sb.DrawSmallLabel(d, x, y, std::to_string(editorOverlay[y*sb.GetWidth()+x]).c_str());
						if (editorOverlay[y*sb.GetWidth()+x] > 0)
							sb.SetColor(rgbColor(0.5, 1.0, 0.5));
						else
							sb.SetColor(rgbColor(1.0, 0.5, 0.5));
						sb.Draw(d, x, y, 0.25);
					}
				}
			}
		}

		if (CanChangeMap(gMouseX, gMouseY) == kCanRemove) //remove objects
		{
			sb.DrawObject(d, gMouseX, gMouseY, gEditorMode, globalTime);
			sb.SetColor(Colors::red);
			sb.Draw(d, gMouseX, gMouseY);
		}
		else if (CanChangeMap(gMouseX, gMouseY) == kCanAdd) //add objects
		{
			sb.DrawObject(d, gMouseX, gMouseY, gEditorMode, globalTime);
			sb.SetColor(Colors::yellow);
			sb.Draw(d, gMouseX, gMouseY);
		}
		else //if you can't do anything at all :(
		{
			sb.SetColor(Colors::gray);
			sb.Draw(d, gMouseX, gMouseY);
		}
	}
	
	if (sb.GoalTest(snake) && actionInProgress == false && quitWhenDoneRecording == false )
	{
		message = "";
		if (transition.Step(frameRate*3))
		{
			d.DrawText("Great Job", {0,0}, Colors::black, 0.25f, Graphics::textAlignCenter);
			d.DrawText("Level Passed!", {0,0.25}, Colors::black, 0.25f, Graphics::textAlignCenter);
//#ifdef __EMSCRIPTEN__
//			submitTextToBuffer("Done");
//			d.DrawText("Window will close automatically", {0,0.4}, Colors::black, 0.075f, Graphics::textAlignCenter);
//#else
			d.DrawText("Press 'r' to reset level", {0,0.4}, Colors::black, 0.075f, Graphics::textAlignCenter);
//#endif
		}
		transition.Draw(d);
	}
	else {
		transition.Reset(0);
	}
	
	
	auto c = Colors::black;
#ifdef __EMSCRIPTEN__
	c = Colors::white;
#endif
	
	// crop outer edges of display
	d.FillRect({-2, -2, 3, -1}, c);
	d.FillRect({-2, 1, 3, 2}, c);
	d.FillRect({-2, -2, -1, 2}, c);
	d.FillRect({1, -2, 3, 2}, c);
	
	if (recording)
	{
		std::string fname = "/Users/nathanst/Pictures/SVG/sb-game";
		static int count = 0;
		while (fileExists((fname+std::to_string(count)+".svg").c_str()))
		{
			count++;
		}
		printf("Save to '%s'\n", (fname+std::to_string(count)+".svg").c_str());
		MakeSVG(d, (fname+std::to_string(count)+".svg").c_str(), 400, 400, 0);
		
		if (!autoSolve && quitWhenDoneRecording)
		{
			exit(0);
		}
	}
	
	if (messageExpireTime >= globalTime && messageBeginTime <= globalTime)
	{
		rgbColor c = Colors::black;
		if (messageExpireTime-globalTime < 1.0)
		{
			rgbColor tmp = rgbColor::mix(Colors::cyan, Colors::lightblue, 0.5);
			c.mix(tmp, 1-(messageExpireTime-globalTime));
		}
		else if (globalTime - messageBeginTime < 1.0)
		{
			rgbColor tmp = rgbColor::mix(Colors::cyan, Colors::lightblue, 0.5);
			c.mix(tmp, 1-(globalTime-messageBeginTime));
		}
		d.DrawText(message.c_str(), {-1+sb.GetRadius(),-1+1.5f*sb.GetRadius()}, c, sb.GetRadius(), Graphics::textAlignLeft, "Helvetica");
	}
}

void MyFrameHandler(unsigned long windowID, unsigned int viewport, void *)
{
	if (viewport == 1)
	{
		DrawEditorViewport(windowID);
	}
	else if (viewport == 0)
	{
		DrawGameViewport(windowID);
	}
}

int MyCLHandler(char *argument[], int maxNumArgs)
{
	if (strcmp(argument[0], "-load") == 0)
	{
		if (maxNumArgs > 1)
		{
			sb.Load(argument[1]);
			UpdateLevelLink();
			lastFrameSnake = snake = sb.GetStart();
			history.push_back(snake);
			mapAlreadyLoaded = true;
			return 2;
		}
		printf("Failed -load <file>: missing file name");
		return 1;
	}
	if (strcmp(argument[0], "-svg") == 0)
	{
		if (maxNumArgs > 2)
		{
			Graphics::Display d;
			if (!sb.DecodeLevel(argument[1]))
				sb.Load(argument[1]);
			lastFrameSnake = snake = sb.GetStart();
			sb.Draw(d);
			sb.Draw(d, 0);
			sb.Draw(d, snake, snakeControl);
			std::string fname = argument[2];
			MakeSVG(d, argument[2], 400, 400, 0);
		}
		else
			printf("Failed -svg <file>: missing file name");
		exit(0);
	}
	if (strcmp(argument[0], "-printsvg") == 0)
	{
		if (maxNumArgs > 1)
		{
			Graphics::Display d;
			if (!sb.DecodeLevel(argument[1]))
				sb.Load(argument[1]);
			lastFrameSnake = snake = sb.GetStart();
			sb.Draw(d);
			sb.Draw(d, 0);
			sb.Draw(d, snake, snakeControl);
			std::cout << MakeSVG(d, 150, 150, 0, "", true) << "\n";
		}
		exit(0);
	}
	// -change <input> <nodelimit> <iter>
	if (strcmp(argument[0], "-change") == 0)
	{
		if (maxNumArgs < 4)
		{
			printf("Insufficient arguments: -change <input> <nodelimit> <iter>\n");
			return 0;
		}
		if (maxNumArgs > 4)
			printf("Ignoring extra input: '%s'\n", argument[4]);

		if (!sb.DecodeLevel(argument[1]))
			sb.Load(argument[1]);
		lastFrameSnake = snake = sb.GetStart();
		
		BFS<SnakeBird::SnakeBirdState, SnakeBird::SnakeBirdAction, SnakeBird::SnakeBird> bfs;
		bfs.SetNodeLimit(atoi(argument[2])); // max 150 million expansions
		bfs.SetVerbose(false);
		bfs.GetPath(&sb, snake, snake, future);
		if (future.size() == 0)
		{
			printf("Length 0\n%s\n", sb.EncodeLevel().c_str());			
			exit(0);
		}
		printf("Length %lu\n", future.size());
		
		int iter = atoi(argument[3]);
		bool harden = iter>0;
		iter = abs(iter);
		for (int x = 0; x < iter; x++)
			AnalyzeMapChanges(harden, atoi(argument[2]));
		std::cout << sb.EncodeLevel() << "\n";
		exit(0);
	}
	if (strcmp(argument[0], "-bfs") == 0)
	{
		if (maxNumArgs > 1)
		{
			if (!sb.DecodeLevel(argument[1]))
				sb.Load(argument[1]);
			//if (sb.Load(argument[1]) == false) // couldn't load file; try encoding
			lastFrameSnake = snake = sb.GetStart();
			GetLevelSolution();
			exit(0);	
		}	
	}
	if (strcmp(argument[0], "-analyzeSolution") == 0)
	{
		if (!sb.DecodeLevel(argument[1]))
			sb.Load(argument[1]);
		BFS<SnakeBird::SnakeBirdState, SnakeBird::SnakeBirdAction, SnakeBird::SnakeBird> bfs;
		bfs.SetNodeLimit(250000000); // max 250 million expansions
		bfs.SetVerbose(false);
		snake = sb.GetStart();
		bfs.GetPath(&sb, snake, snake, future);
		std::unordered_map<int, int> count;
		for (int x = 0; x < future.size(); x++)
		{
			int loc = future[x].GetSnakeHeadLoc(0);
			count[loc]++;
//			printf("%d : %d\n", loc, count[loc]);
		}
		int maxVal = 0;
		for (auto i = count.begin(); i != count.end(); i++)
		{
			maxVal = std::max(maxVal, i->second);
		}
		std::cout << argument[1] << " " << maxVal << "\n";
		exit(0);
	}
	if (strcmp(argument[0], "-encode") == 0)
	{
		if (maxNumArgs > 1)
		{
			sb.Load(argument[1]);
			auto str = sb.EncodeLevel();
			std::cout << str << "\n";
			sb.DecodeLevel(sb.EncodeLevel());
			auto str2 = sb.EncodeLevel();
			if (str != str2)
				std::cerr << "ENCODING FAILURE!\n";
			exit(0);
		}
	}
	if (strcmp(argument[0], "-decode") == 0)
	{
		if (maxNumArgs > 1)
		{
			sb.DecodeLevel(argument[1]);
			UpdateLevelLink();
			if (strcmp(argument[1], sb.EncodeLevel().c_str()) != 0)
			{
				printf("Decode failure:\n");
				printf("%s\n", argument[1]);
				printf("%s\n", sb.EncodeLevel().c_str());
			}
			lastFrameSnake = snake = sb.GetStart();
			history.push_back(snake);
			mapAlreadyLoaded = true;
			return 2;
		}
		return 1;
	}
	if (strcmp(argument[0], "-solve") == 0)
	{
		if (maxNumArgs > 1)
		{
			if (!sb.DecodeLevel(argument[1]))
				sb.Load(argument[1]);
			lastFrameSnake = snake = sb.GetStart();
			history.push_back(snake);
			mapAlreadyLoaded = true;
			recording = true;
			quitWhenDoneRecording = true;
			GamePlayKeyboardHandler(0, tKeyboardModifier::kNoModifier, 'n');
			return 2;
		}
		return 1;
	}

	return 0;
}

void HurryUpFinishAction()
{
	while (actionInProgress == true)
	{
		bool complete = sb.ApplyPartialAction(snake, inProgress, actionInProgressStep);
		actionInProgress = !complete;
	}
}

void GetLevelSolution(int nodeLimit)
{
	Timer t;
	BFS<SnakeBird::SnakeBirdState, SnakeBird::SnakeBirdAction, SnakeBird::SnakeBird> bfs;
	//bfs.SetNodeLimit(20000000);
	//bfs.SetNodeLimit(75000000); // max 75 million expansions
	bfs.SetNodeLimit(nodeLimit); // max 250 million expansions
	//bfs.DoBFS(&sb, snake);
	t.StartTimer();
	bfs.GetPath(&sb, snake, snake, future);
	t.EndTimer();
	printf("BFS to goal complete in %1.2fs: %llu states reached; path length %lu.\n",
		   t.GetElapsedTime(), bfs.GetNodesExpanded(), future.size());
	
	if (future.size() == 0)
	{
		message = "Level unsolvable. Reset or undo actions and try again.";
		messageExpireTime = globalTime + 5;
	}
	if (future.size() > 0)
	{
		std::reverse(future.begin(), future.end());
		// remove the current state
		future.pop_back();
		UpdateActiveSnake();
	}
}

void StepForwardStoredSolution()
{
	HurryUpFinishAction();
	if (future.size() > 0)
	{
		actionInProgress = true;
		actionInProgressStep.Reset();
		inProgress = sb.GetAction(snake, future.back());
		frameTime = timePerFrame;
		history.push_back(snake);
		//snake = future.back();
		future.pop_back();
	}
}

void EditorStudyKeyboardHandler(unsigned long windowID, tKeyboardModifier mod, char key)
{
	assistiveEditor = true;
	incrementalAnalysis = true;
	switch (key)
	{
		case '$': // simple editor
			assistiveEditor = false;
			incrementalAnalysis = false;
		case '^': // complex editor
			editorItems =
			{
				{kColumn1, 1, SnakeBird::kNothing, "Edit Map", kColumn2, '\0', false, false},
				{kColumn1, 3, SnakeBird::kEmpty, "Sky", kColumn2, 'k', true, true},
				{kColumn1, 5, SnakeBird::kGround, "Ground", kColumn2, 'g', true, false},
				{kColumn1, 7, SnakeBird::kSpikes, "Spikes", kColumn2, 's', true, true},
				{kColumn1, 9, SnakeBird::kFruit, "Fruit", kColumn2, 'f', true, true},
//				{kColumn1, 11, SnakeBird::kPortal1, "Portal", kColumn2, 'p', true, true},
//				{kColumn1, 13, SnakeBird::kBlock1, "Block", kColumn2, 'b', true, true},
				{kColumn1, 11, SnakeBird::kExit, "Exit", kColumn2, 'x', true, true},
			};
			break;
	}
}


void GamePlayKeyboardHandler(unsigned long windowID, tKeyboardModifier mod, char key)
{
	static std::vector<std::string> sblevels =
	{
		"26b17EkcGamanaobdbebfbgbubvbwbxclcmcncoczdcdddedfdtdudvdweiekelemenfbfcfdfefnfoftfufvgegfggghgigkgwgxhbhchnhqhrhshthuiiijikiljajbjcjrjsjtkfkgkikjkkkxkzlalblqlrlsmhmimjmymznanonpnqnrofogKFixnnSdsLL",
		"12b22EioGfggfggghgigjhchdhehfhvhyhziaibiuivjnKFfhirShxL",
		"19b13EffGbjbkblbtbubvbwbxbycfcgchcicjckclcqcrcsctcxcydddedfdgdkdldrdsdtdudwdxdyejekelewfjfkfwfxfyKFdhgvSfiRU",
		"20b13EfrGdwdxejekelewexfifjfvgigjgkgvgwgxgyhihjhkhlhuhvhwhxhyihiiijKeuevhhFegigSfuRR",
		"20b15EhrGeifzgdgvgwhjhkhlhuKexeyezfogshfFgeSgnLLD",
		"20b13EfrGesevfkgggiKeifwfxgjFegftSfjLD",
		"20b13EhhGdsdweeefegehejekepeqeresexfcfdfkftfvKewFetSdvLDD",
		// 10,24,44 (24 and 44 have 2 snakes but they are easier)
		"20b15EfiGclczdadodpdtedeeefegeiejexeyfcfdfmfnfofpfqfrfsghgqgrgtgugvgwhdhehfhihjhkhsKFdcgsSflRRU",
		"20b16EhuGfkflfsgagbgcgqgrKFfrBerfhfxesfifyfjSgoRUSgpRD",
		"20b13EgfGcxcydkdldxekelgkglKcgckdhfxFefctPdeeiSdiLScwR"
		// 7-9 have 2 snakes
//		"19b14EgbGczdadbeneresgjgxhlhzKFSgvRRDSgiRR",
//		"20b13EisGbwcjckcwcxdjdkdwdxejekgxihiuivjhjiKewfjfwfxgkhihjhkhviiFSduLLDLSdiRR",
//		"20b22EhsGeyfoftfugpgqhlhmhyihizjdjekaKhfibiuFSgoLLDSjcRDRD"
	};
	
	static std::vector<std::string> sbprimer =
	{
		"20b14EijGamazbabmbnbocacbcccocpcqdcdddedmdndqdrdseaeeefegeseteufdfefgfhfifsfufvfwgigjgkgwgxgyhdhehjhkhlhmhrhshthxhyhziaieifigihiliminioisitiuizjajbjcjgjhjkjljmjnjojpjqjzkakbkckdkekqkrksKFSdpLL",
		"20b13EdeGchctcucvcwdgdhdidjdkdldtdudvdwdxdyeieleyflfygkglgxgyhjhkhlhwhxhyijikiliwixiyjkjlKFhghuhiiiSefL",
		"20b13EgiGcicucvdhdidudveiekexeyfkflfxfygkglgxgyhlKFcjcwdjdwejewSdgL",
		"19b20EhiGeneofgfhfifwfzgagbgcgtgugvgwhkhnhohpihiijcjwjxjykqkrkslklllmmfmgKFgqkpShmR",
		"20b11EdqGcicjckcsctcucvcwcxdddedhdidodsdtdzedeeepeteuezfafefffgfkflfwKFemSdbD",
		// replace 5 with 63
		"20b15EijGdidsdtdxdyemeneofbfcfqfrfsgfgggugygzhahnhohphqhrhshwhyhziaicidieifigihiliminioipirisitiuiviwjajbjcjdjejgjhjiKFPduhxBeyfngcfoSfpR",
		"20b20EhuGfafufvgugvgwhmhohphqhwhxicieifigijikitiujajdjejnjojxjykrkslmKFftizSjwLL",
		"20b15EdcGcecfcgcpcqcrctcucvdedidjdkdtdveneyezfifjfofrgngogvgwhkhlhthuhvhziaikixizjrjskgkhkukvkwKFfyexhjSdhL",
		"20b15EdcGbnboccctcxcyczdedidmdndodpdtebecedeeefegeheiejeqereseteuevewfffgfhfifjfofqfufvhticidieikilirisitiuiviwjgjhjijjjkjljmjvjwjxjyjzkakbkekfkkklkmknkokpkqkrktkukzlalblcldlelflgKFSksRDD",
		"20b12EfgGcfcgcqcrcsdcdddedodpdqeaebecekemeneofafhfifjflfmftfufxfygjgkgvgwhhhihuKFgtSdnL"
	};
	//	SnakeBird::SnakeBirdAction a;
	static std::vector<SnakeBird::SnakeBirdAction> acts;
	//	a.bird = snakeControl;
	
	// always be able to print screen
	if (key == '&')
	{
		Graphics::Display &d = getCurrentContext()->display;
		
		//Graphics::Display d;
		//sb.Draw(d);
		//sb.Draw(d, globalTime);
		//sb.Draw(d, lastFrameSnake, snake, snakeControl, frameTime/timePerFrame, globalTime);
		//			std::string fname = "/Users/nathanst/Desktop/SVG/sb_";
		std::string fname = "/Users/nathanst/Pictures/SVG/sb_";
		int count = 0;
		while (fileExists((fname+std::to_string(count)+".svg").c_str()))
		{
			count++;
		}
		printf("Save to '%s'\n", (fname+std::to_string(count)+".svg").c_str());
		MakeSVG(d, (fname+std::to_string(count)+".svg").c_str(), 800, 400, -1, sb.EncodeLevel().c_str(), true);
	}

	
	// ignore key presses in edit mode
	if (gEditMap && key != 't')
		return;
	
	switch (key)
	{
		case 'f':
			if (gSpeedAdjust > 0.5)
				gSpeedAdjust = 0.1;
			else
				gSpeedAdjust = 1.0;
			break;
		case '/': loadPrimer = !loadPrimer; break;
		case '0':
		case '1':
		case '2':
		case '3':
		case '4':
		case '5':
		case '6':
		case '7':
		case '8':
		case '9':
			if (loadPrimer)
				sb.DecodeLevel(sbprimer[key-'0']);
			else
				sb.DecodeLevel(sblevels[key-'0']);
			lastFrameSnake = snake = sb.GetStart();
			history.clear();
			history.push_back(snake);
			future.clear();
			actionInProgressStep.Reset();
			timePerFrame = 0.01;
			UpdateActiveSnake();
			break;
//		case '0': LoadLevel19(); break;
//		case '1': LoadLevel22(); break;
//		case '2': LoadLevel32(); break;
//		case '3': LoadLevel39(); break;
//		case '4': LoadLevel44(); break;
//		case '5': LoadLevel63(); break;
		case 'w':
		case kUpArrow:
			future.clear();
			HurryUpFinishAction();
			sb.GetActions(snake, acts);
			for (auto &a : acts)
			{
				if (a.bird == snakeControl && a.direction == SnakeBird::kUp)
				{
					//sb.ApplyAction(snake, a);
					actionInProgress = true;
					actionInProgressStep.Reset();
					inProgress = a;
					frameTime = timePerFrame;
					history.push_back(snake);
				}
			}
			UpdateActiveSnake();
			break;
		case 's':
		case kDownArrow:
			future.clear();
			HurryUpFinishAction();
			sb.GetActions(snake, acts);
			for (auto &a : acts)
			{
				if (a.bird == snakeControl && a.direction == SnakeBird::kDown)
				{
					actionInProgress = true;
					actionInProgressStep.Reset();
					inProgress = a;
					frameTime = timePerFrame;
					history.push_back(snake);
					
					//					sb.ApplyAction(snake, a);
					//					history.push_back(snake);
				}
			}
			UpdateActiveSnake();
			break;
		case 'a':
		case kLeftArrow:
			future.clear();
			HurryUpFinishAction();
			sb.GetActions(snake, acts);
			for (auto &a : acts)
			{
				if (a.bird == snakeControl && a.direction == SnakeBird::kLeft)
				{
					actionInProgress = true;
					actionInProgressStep.Reset();
					inProgress = a;
					frameTime = timePerFrame;
					history.push_back(snake);
					//					sb.ApplyAction(snake, a);
					//					history.push_back(snake);
				}
			}
			UpdateActiveSnake();
			break;
		case 'd':
		case kRightArrow:
			future.clear();
			HurryUpFinishAction();
			sb.GetActions(snake, acts);
			for (auto &a : acts)
			{
				if (a.bird == snakeControl && a.direction == SnakeBird::kRight)
				{
					actionInProgress = true;
					actionInProgressStep.Reset();
					inProgress = a;
					frameTime = timePerFrame;
					history.push_back(snake);
					//					sb.ApplyAction(snake, a);
					//					history.push_back(snake);
				}
			}
			UpdateActiveSnake();
			break;
		case 'q':
			//if (message == "No legal actions; press undo or reset level to continue")
			message = "";
			future.clear();
			if (history.size() > 1)
			{
				while (snake == history.back() && history.size() > 2)
					history.pop_back();
				snake = history.back();
				lastFrameSnake = snake;
				timePerFrame = 0.001;
				history.pop_back();
			}
			UpdateActiveSnake();
			break;
		case 'e':
			for (int x = 0; x < snake.GetNumSnakes(); x++)
			{
				snakeControl = (snakeControl+1)%snake.GetNumSnakes();
				if (snake.IsInPlay(snakeControl))
					break;
			}
			break;
		case ']':
			StepForwardStoredSolution();
			break;
		case 'n':
			GetLevelSolution(gNodeLimit);
			autoSolve = true;
			StepForwardStoredSolution();
			break;
//		case 'c':
//			AnalyzeMapChanges(true);
//			break;
//		case 'x':
//			AnalyzeMapChanges(false);
//			break;
		case 'b':
		{
			GetLevelSolution(gNodeLimit);
			break;
		}
		case 'o':
			recording = !recording;
			break;
		case 't':
		{
			if (!gEditMap)
			{
				// Reset the level
				GamePlayKeyboardHandler(windowID, kNoModifier, 'r');
				gEditMap = true;
				gEditorMode = SnakeBird::kEmpty;
				kSelectedEditorItem = 1;
				gMouseY = gMouseX = -1;
				message = "Editing mode enabled.";
				messageBeginTime = globalTime;
				messageExpireTime = globalTime+5;
//				ReinitViewports(windowID, {-1.0f, -1.0f, 0.0f, 1.0f}, kScaleToSquare);
				MoveViewport(windowID, 1, {0.0f, -1.0f, 1.0f, 1.0f});
//				AddViewport(windowID, {1.0f, -1.0f, 2.0f, 1.0f}, {0.0f, -1.0f, 1.0f, 1.0f}, kScaleToSquare); // (will be editor)
//				SnakeBird::SnakeBird tmp(sb.GetWidth(), sb.GetHeight());
				ShowSolutionLength();
				SetupMapChanges();
			}
			else {
				message = "Editing mode disabled.";
				messageBeginTime = globalTime;
				messageExpireTime = globalTime+5;
				gEditMap = false;
				MoveViewport(windowID, 1, {1.0f, -1.0f, 2.0f, 1.0f});
				//ReinitViewports(windowID, {-1.0f, -1.0f, 0.0f, 1.0f}, kScaleToSquare);
				UpdateLevelLink();
			}
		}
			break;
		case 'y':
			break;
		case 'v':
			break;
		case 'r':
			message = "";
			snake = history[0];
//			snake.MakeSnakeLonger(0, SnakeBird::kUp);
			timePerFrame = 0.01;
			history.resize(1);
			future.clear();
			autoSolve = false;
			if (gEditMap)
			{
				message = "Editing mode disabled.";
				messageBeginTime = globalTime;
				messageExpireTime = globalTime+5;
				gEditMap = false;
				MoveViewport(windowID, 1, {1.0f, -1.0f, 2.0f, 1.0f});
				//ReinitViewports(windowID, {-1.0f, -1.0f, 0.0f, 1.0f}, kScaleToSquare);
				UpdateLevelLink();
			}
			break;
		case '\t':
//				if (mod != kShiftDown)
//					SetActivePort(windowID, (GetActivePort(windowID)+1)%GetNumPorts(windowID));
//				
//					else
//					{
//						SetNumPorts(windowID, 1+(GetNumPorts(windowID)%MAXPORTS));
//					}
//					break;
				default:
					break;
				}
}

void EditorKeyBoardHandler(unsigned long windowID, tKeyboardModifier mod, char key)
{
	//	SnakeBird::SnakeBirdAction a;
	static std::vector<SnakeBird::SnakeBirdAction> acts;
	//	a.bird = snakeControl;
	
	if (gEditMap)
	{
		switch (key)
		{
			case 'n':
				// turn off editor
				GamePlayKeyboardHandler(windowID, mod, 't');
				GetLevelSolution(gNodeLimit);
				autoSolve = true;
				StepForwardStoredSolution();
				break;
			case 'f': //fruit
				message = "Editing Mode: Adding Fruit";
				messageBeginTime = globalTime;
				messageExpireTime = globalTime+5;
				gEditorMode = SnakeBird::kFruit;
				SetupMapChanges();
				break;
			case 'x': //exit
				message = "Editing Mode: Moving the Exit";
				messageBeginTime = globalTime;
				messageExpireTime = globalTime+5;
				gEditorMode = SnakeBird::kExit;
				SetupMapChanges();
				break;
			case 'g': //ground
				message = "Editing Mode: Changing the Ground";
				messageBeginTime = globalTime;
				messageExpireTime = globalTime+5;
				gEditorMode = SnakeBird::kGround;
				SetupMapChanges();
				break;
			case 's': //spikes
				message = "Editing Mode: Changing Spikes";
				messageBeginTime = globalTime;
				messageExpireTime = globalTime+5;
				gEditorMode = SnakeBird::kSpikes;
				SetupMapChanges();
				break;
			case 'k': //sky
				message = "Editing Mode: Changing the Sky";
				messageBeginTime = globalTime;
				messageExpireTime = globalTime+5;
				gEditorMode = SnakeBird::kEmpty;
				SetupMapChanges();
				break;
			case 'p': //portal
				message = "Editing Mode: Changing Portals";
				messageBeginTime = globalTime;
				messageExpireTime = globalTime+5;
				gEditorMode = SnakeBird::kPortal;
				editorOverlay.resize(0);
				break;
			case 'b':
				message = "Editing Mode: Changing Blocks";
				messageBeginTime = globalTime;
				messageExpireTime = globalTime+5;
				gEditorMode = SnakeBird::kBlock1;
				editorOverlay.resize(0);
				break;
			case 'i':
				message = "Editing Mode: Changing SnakeBird 1";
				messageBeginTime = globalTime;
				messageExpireTime = globalTime+5;
				gEditorMode = SnakeBird::kSnake1;
				editorOverlay.resize(0);
				break;
			case 'j':
				message = "Editing Mode: Changing SnakeBird 2";
				messageBeginTime = globalTime;
				messageExpireTime = globalTime+5;
				gEditorMode = SnakeBird::kSnake2;
				editorOverlay.resize(0);
				break;
			case 'u':
				message = "Editing Mode: Increase Height";
				messageBeginTime = globalTime;
				messageExpireTime = globalTime+5;
				sb.BiggerMapHeight();
				editorOverlay.resize(0);
				lastFrameSnake = snake = sb.GetStart();
				history.clear();
				future.clear();
				history.push_back(snake);
				actionInProgressStep.Reset();
				UpdateActiveSnake();
				sb.EndEditing();
				UpdateLevelLink();
				gRefreshBackground = true;
				break;
			case 'y':
				message = "Editing Mode: Increase Width";
				messageBeginTime = globalTime;
				messageExpireTime = globalTime+5;
				sb.BiggerMapWidth();
				editorOverlay.resize(0);
				break;
			case 'z':
				message = "Editing Mode: Decrease Height";
				messageBeginTime = globalTime;
				messageExpireTime = globalTime+5;
				sb.SmallerMapHeight();
				editorOverlay.resize(0);
				lastFrameSnake = snake = sb.GetStart();
				history.clear();
				future.clear();
				history.push_back(snake);
				actionInProgressStep.Reset();
				UpdateActiveSnake();
				sb.EndEditing();
				UpdateLevelLink();
				gRefreshBackground = true;
				break;
			case 'm':
				message = "Editing Mode: Decrease Width";
				messageBeginTime = globalTime;
				messageExpireTime = globalTime+5;
				sb.SmallerMapWidth();
				editorOverlay.resize(0);
				break;
			case 'c':
				AnalyzeMapChanges(true, gNodeLimit);
				break;
			case 'v':
				AnalyzeMapChanges(false, gNodeLimit);
				break;
			case '1':
				AnalyzeObject(SnakeBird::kEmpty, SnakeBird::kBlock1);
				break;
			case '2':
				AnalyzeObject(SnakeBird::kEmpty, SnakeBird::kExit);
				break;
			case '3':
				AnalyzeObject(SnakeBird::kEmpty, SnakeBird::kFruit);
				break;
			case '4':
				if (sb.GetNumPortals() == 1)
					AnalyzeObject(SnakeBird::kEmpty, SnakeBird::kPortal);
				else {
					message = "Must have exactly one portal placed before using EPCG.";
					messageBeginTime = globalTime+0.01;
					messageExpireTime = globalTime+5;
				}
				break;
			case 'h': //toggle ground modes
			{
				switch (gEditorMode)
				{
					case SnakeBird::kGround:
					{
						EditorKeyBoardHandler(windowID, kNoModifier, 's');
						break;
					}
					case SnakeBird::kSpikes:
					{
						EditorKeyBoardHandler(windowID, kNoModifier, 'k');
						break;
					}
					case SnakeBird::kEmpty:
					{
						EditorKeyBoardHandler(windowID, kNoModifier, 'g');
						break;
					}
					default:
						break;
				}
				break;
			}
			case 'd': //toggle modes
			{
				switch (gEditorMode)
				{
					case SnakeBird::kGround:
					{
						EditorKeyBoardHandler(windowID, kNoModifier, 's');
						break;
					}
					case SnakeBird::kSpikes:
					{
						EditorKeyBoardHandler(windowID, kNoModifier, 'f');
						break;
					}
					case SnakeBird::kFruit:
					{
						EditorKeyBoardHandler(windowID, kNoModifier, 'x');
						break;
					}
					case SnakeBird::kExit:
					{
						EditorKeyBoardHandler(windowID, kNoModifier, 'k');
						break;
					}
					case SnakeBird::kPortal:
					{
						EditorKeyBoardHandler(windowID, kNoModifier, 'p');
						break;
					}
					case SnakeBird::kEmpty:
					{
						EditorKeyBoardHandler(windowID, kNoModifier, 'g');
						break;
					}
					default:
						return;
				}
				break;
			}
			default:
				return;
		}
	}
}

bool MyClickHandler(unsigned long windowID, int viewport, int, int, point3d p, tButtonType , tMouseEventType e)
{
	if (viewport == 1)
	{
		gMouseX = -1;
		gMouseY = -1;
		int x, y;
		editor.GetPointFromCoordinate(p, x, y);
		gMouseEditorX = x;
		gMouseEditorY = y;
		if (e == kMouseDown)
		{
			for (size_t t = 0; t < editorItems.size(); t++)
			{
				if (gMouseEditorY == editorItems[t].y &&
					gMouseEditorX >= editorItems[t].x &&
					gMouseEditorX < editorItems[t].xLineEnd) // mouse over
				{
					if (editorItems[t].selectable)
						kSelectedEditorItem = t;
					EditorKeyBoardHandler(windowID, kNoModifier, editorItems[t].keyEquivalent);
				}
			}
		}
		return true;
	}

	if ((e == kMouseDrag) && (gEditMap == true)) // ignore movement with mouse button down
	{
		int x, y;
		if (sb.GetPointFromCoordinate(p, x, y))
		{
			gMouseX = x;
			gMouseY = y;
			ChangeMap(x, y, e);
		}
		else
		{
			gMouseX = -1;
			gMouseY = -1;
		}
		//return true;
	}
	
	if ((e == kMouseDown) && (gEditMap == true))
	{
		int x, y;
		if (sb.GetPointFromCoordinate(p, x, y))
		{
			//printf("Hit (%f, %f) -> (%d, %d)\n", p.x, p.y, x, y);
			ChangeMap(x, y, e);
//			CanChangeMap(gMouseX, gMouseY);
			ShowSolutionLength();
			SetupMapChanges();
		}
	}
	
	if (e == kMouseUp)
	{
	}
	if (e == kMouseMove && gEditMap == true)
	{
		int x, y;
		if (sb.GetPointFromCoordinate(p, x, y))
		{
			gMouseX = x;
			gMouseY = y;
		}
		else
		{
			gMouseX = -1;
			gMouseY = -1;
		}
	}
	return true;
}

void SetupMapChanges()
{
	editorOverlayMax = 1;
	editorOverlayMin = -1;
	editorOverlay.resize(sb.GetWidth()*sb.GetHeight());
	for (int x = 0; x < sb.GetWidth(); x++)
	{
		for (int y = 0; y < sb.GetHeight()-1; y++)
		{
			EditorColor c = CanChangeMap(x, y);
			if (c == kCanAdd)
			{
				editorOverlay[y*sb.GetWidth()+x] = kNotAnalyzed;
				printf("[%d, %d]\n", x, y);
//				editorOverlay[y*sb.GetWidth()+x] = random()%11-5;
//				editorOverlayMax = std::max(editorOverlayMax, editorOverlay[y*sb.GetWidth()+x]);
//				editorOverlayMin = std::min(editorOverlayMin, editorOverlay[y*sb.GetWidth()+x]);
			}
			else {
				editorOverlay[y*sb.GetWidth()+x] = kDontAnalyze;
			}
		}
	}
}

bool AnalyzeChangeAtLoc(int x, int y)
{
	if ((gEditorMode&SnakeBird::kSnakeMask) == SnakeBird::kSnakeMask)
		return false;
	if (editorOverlay[y*sb.GetWidth()+x] == kNotAnalyzed)
	{
		BFS<SnakeBird::SnakeBirdState, SnakeBird::SnakeBirdAction, SnakeBird::SnakeBird> bfs;
		SnakeBird::SnakeBird curr = sb;
		SnakeBird::SnakeBirdState s;
		
		curr.BeginEditing();
		curr.SetGroundType(x, y, gEditorMode);
		curr.EndEditing();
		
		s = curr.GetStart();
		bfs.SetVerbose(false);
		bfs.SetNodeLimit(50000); // real-time limit
		bfs.GetPath(&curr, s, s, future);
		if (future.size() > 0)
		{
			editorOverlay[y*sb.GetWidth()+x] = future.size()-gSolutionLength;
			editorOverlayMax = std::max(editorOverlayMax, editorOverlay[y*sb.GetWidth()+x]);
			editorOverlayMin = std::min(editorOverlayMin, editorOverlay[y*sb.GetWidth()+x]);
			//printf("%d, %d -> %d\n", x, y, editorOverlay[y*sb.GetWidth()+x]);
		}
		else {
			//printf("{%d, %d}\n", x, y);
			editorOverlay[y*sb.GetWidth()+x] = kDontAnalyze;
		}
		return true;
	}
	return false;
}

void ProcessSingleMapChange()
{
	int yStart = sb.GetY(sb.GetStart().GetSnakeHeadLoc(0));
	int yUp = yStart;
	int yDown = yStart+1;
	while (yUp >= 0 || yDown < sb.GetHeight()-1)
	{
		if (yUp >= 0)
		{
			for (int x = 0; x < sb.GetWidth(); x++)
			{
				if (AnalyzeChangeAtLoc(x, yUp))
					return;
			}
			yUp--;
		}
		if (yDown < sb.GetHeight()-1)
		{
			for (int x = 0; x < sb.GetWidth(); x++)
			{
				if (AnalyzeChangeAtLoc(x, yDown))
					return;
			}
			yDown++;
		}
	}
}

void AnalyzeMapChanges(bool maximize, int nodeLimit)
{
	BFS<SnakeBird::SnakeBirdState, SnakeBird::SnakeBirdAction, SnakeBird::SnakeBird> bfs;
	bfs.SetNodeLimit(nodeLimit); // max 1 million expansions
	bfs.SetVerbose(false);
	std::vector<SnakeBird::SnakeBirdState> path;

	size_t maxLength = 0;
	size_t minLength = 100000;
	SnakeBird::SnakeBird bestMin, bestMax;
	SnakeBird::SnakeBird curr = sb;
	std::vector<SnakeBird::SnakeBirdWorldObject> order;
	for (int x = 0; x < sb.GetWidth(); x++)
	{
		for (int y = 0; y < sb.GetHeight()-1; y++)
		{
			auto renderedType = sb.GetRenderedGroundType(sb.GetStart(), x, y);
			bool valid = false;
			switch (renderedType)
			{
				case SnakeBird::kGround:
				{
					valid = true;
					order.push_back(SnakeBird::kEmpty);
					order.push_back(SnakeBird::kSpikes);
					break;
				}
				case SnakeBird::kSpikes:
				{
					valid = true;
					order.push_back(SnakeBird::kGround);
					order.push_back(SnakeBird::kEmpty);
					break;
				}
				case SnakeBird::kEmpty:
				{
					valid = true;
					order.push_back(SnakeBird::kSpikes);
					order.push_back(SnakeBird::kGround);
//					order.push_back(SnakeBird::kBlock1);
					break;
				}
				default:
					break;
			}
	
			while (order.size() > 0)
			{
				sb.BeginEditing();
				sb.SetGroundType(x, y, order.back());
				sb.EndEditing();
				snake = sb.GetStart();
				//Save(sb, snake, "EPCG-");

				gRefreshBackground = true;
				order.pop_back();
				bfs.GetPath(&sb, snake, snake, path);
				if (path.size() < minLength && path.size() != 0)
				{
					minLength = path.size();
					bestMin = sb;
				}
				if (path.size() > maxLength)
				{
					maxLength = path.size();
					bestMax = sb;
				}
				sb = curr;
			}
			sb = curr;//sb.SetStart(current);
			snake = sb.GetStart();
		}
	}
	bfs.GetPath(&sb, snake, snake, path);
//	printf("Old: %lu, ", path.size());
	if (maximize)
	{
		if (maxLength > path.size()) // found better path
		{
			sb = bestMax;
			snake = sb.GetStart();
			history[0] = snake;
//			bfs.GetPath(&sb, snake, snake, path);
//			printf("New: %lu\n", path.size());
			message = "Solution length increased from "+std::to_string(path.size())+" to "+std::to_string(maxLength);
			ShowSolutionLength(maxLength);
		}
		else {
			message = "NOTE: No change will make the shortest path longer";
		}
	}
	else {
		if (minLength < path.size() || (path.size() == 0 && minLength > 0))
		{
			sb = bestMin;
			snake = sb.GetStart();
			history[0] = snake;
//			bfs.GetPath(&sb, snake, snake, path);
//			printf("New: %lu\n", path.size());
			message = "Solution length increased from "+std::to_string(path.size())+" to "+std::to_string(minLength);
			ShowSolutionLength(minLength);
		}
		else {
			message = "NOTE: No change will make the shortest path shorter";
		}
	}
	messageBeginTime = globalTime;
	messageExpireTime = globalTime+10;
	UpdateLevelLink();
}

void AnalyzeObject(SnakeBird::SnakeBirdWorldObject oldobj, SnakeBird::SnakeBirdWorldObject newobj, int nodeLimit)
{
	BFS<SnakeBird::SnakeBirdState, SnakeBird::SnakeBirdAction, SnakeBird::SnakeBird> bfs;
	bfs.SetNodeLimit(nodeLimit); // max 1 million expansions
	bfs.SetVerbose(false);
	std::vector<SnakeBird::SnakeBirdState> path;

	size_t maxLength = 0;
	size_t minLength = 100000;
	SnakeBird::SnakeBird bestMin, bestMax;
	SnakeBird::SnakeBird curr = sb;
	std::vector<SnakeBird::SnakeBirdWorldObject> order;
	for (int x = 0; x < sb.GetWidth(); x++)
	{
		for (int y = 0; y < sb.GetHeight()-1; y++)
		{
			auto renderedType = sb.GetRenderedGroundType(sb.GetStart(), x, y);
			bool valid = false;
			if (renderedType == oldobj)
			{
				valid = true;
				order.push_back(newobj);
			}

			while (order.size() > 0)
			{
				sb.BeginEditing();
				sb.SetGroundType(x, y, order.back());
				sb.EndEditing();
				snake = sb.GetStart();
				//Save(sb, snake, "EPCG-");

				gRefreshBackground = true;
				order.pop_back();
				bfs.GetPath(&sb, snake, snake, path);
				if (path.size() < minLength && path.size() != 0)
				{
					minLength = path.size();
					bestMin = sb;
				}
				if (path.size() > maxLength)
				{
					maxLength = path.size();
					bestMax = sb;
				}
				sb = curr;
			}
			sb = curr;//sb.SetStart(current);
			snake = sb.GetStart();
		}
	}
	bfs.GetPath(&sb, snake, snake, path);
//	printf("Old: %lu, ", path.size());
	if (maxLength > path.size()) // found better path
	{
		sb = bestMax;
		snake = sb.GetStart();
		history[0] = snake;
		//			bfs.GetPath(&sb, snake, snake, path);
		//			printf("New: %lu\n", path.size());
		message = "Solution length increased from "+std::to_string(path.size())+" to "+std::to_string(maxLength);
		ShowSolutionLength(maxLength);
	}
	else {
		message = "NOTE: No change will make the shortest path longer";
	}
	messageBeginTime = globalTime;
	messageExpireTime = globalTime+10;
	UpdateLevelLink();
}

void ChangeMap(int x, int y, tMouseEventType o)
{
	auto renderedType = sb.GetRenderedGroundType(snake, x, y);
	switch (gEditorMode)
	{
		case SnakeBird::kGround:
		{
			switch (renderedType)
			{
				case SnakeBird::kGround:
				{
					sb.BeginEditing();
					sb.SetGroundType(x, y, SnakeBird::kEmpty);
					sb.EndEditing();
					UpdateLevelLink();
					gRefreshBackground = true;
					break;
				}
				case SnakeBird::kSpikes:
				case SnakeBird::kEmpty:
				case SnakeBird::kFruit:
				case SnakeBird::kPortal1:
				case SnakeBird::kPortal2:
				case SnakeBird::kExit:
				{
					sb.BeginEditing();
					sb.SetGroundType(x, y, SnakeBird::kEmpty);
					sb.SetGroundType(x, y, SnakeBird::kGround);
					sb.EndEditing();
					UpdateLevelLink();
					gRefreshBackground = true;
					break;
				}
				case SnakeBird::kBlock1:
				case SnakeBird::kBlock2:
				case SnakeBird::kBlock3:
				case SnakeBird::kBlock4:
					sb.BeginEditing();
					sb.RemoveBlock(gMouseX, gMouseY);
					sb.SetGroundType(gMouseX, gMouseY, SnakeBird::kGround);
					sb.EndEditing();
					UpdateLevelLink();
					lastFrameSnake = snake = sb.GetStart();
					history.clear();
					history[0] = snake;
					gRefreshBackground = true;
					break;
				default:
					break;
			}
			break;
		}
		case SnakeBird::kSpikes:
		{
			switch (renderedType)
			{
				case SnakeBird::kSpikes:
				{
					sb.BeginEditing();
					sb.SetGroundType(x, y, SnakeBird::kEmpty);
					sb.EndEditing();
					UpdateLevelLink();
					gRefreshBackground = true;
					break;
				}
				case SnakeBird::kGround:
				case SnakeBird::kEmpty:
				case SnakeBird::kFruit:
				case SnakeBird::kPortal1:
				case SnakeBird::kPortal2:
				case SnakeBird::kExit:
				{
					sb.BeginEditing();
					sb.SetGroundType(x, y, SnakeBird::kEmpty);
					sb.SetGroundType(x, y, SnakeBird::kSpikes);
					sb.EndEditing();
					UpdateLevelLink();
					gRefreshBackground = true;
					break;
				}
				case SnakeBird::kBlock1:
				case SnakeBird::kBlock2:
				case SnakeBird::kBlock3:
				case SnakeBird::kBlock4:
					sb.BeginEditing();
					sb.RemoveBlock(gMouseX, gMouseY);
					sb.SetGroundType(gMouseX, gMouseY, SnakeBird::kSpikes);
					sb.EndEditing();
					UpdateLevelLink();
					lastFrameSnake = snake = sb.GetStart();
					gRefreshBackground = true;
					break;
				default:
					break;
			}
			break;
		}
		case SnakeBird::kEmpty:
		{
			switch (renderedType)
			{
				case SnakeBird::kSpikes:
				case SnakeBird::kGround:
				case SnakeBird::kFruit:
				case SnakeBird::kPortal1:
				case SnakeBird::kPortal2:
				case SnakeBird::kExit:
				{
					sb.BeginEditing();
					sb.SetGroundType(x, y, SnakeBird::kEmpty);
					sb.EndEditing();
					UpdateLevelLink();
					gRefreshBackground = true;
					break;
				}
				case SnakeBird::kBlock1:
				case SnakeBird::kBlock2:
				case SnakeBird::kBlock3:
				case SnakeBird::kBlock4:
					sb.BeginEditing();
					sb.RemoveBlock(gMouseX, gMouseY);
					sb.SetGroundType(gMouseX, gMouseY, SnakeBird::kEmpty);
					sb.EndEditing();
					UpdateLevelLink();
					lastFrameSnake = snake = sb.GetStart();
					gRefreshBackground = true;
					break;
				default:
					break;
			}
			break;
		}
		case SnakeBird::kFruit:
		{
			switch (renderedType)
			{
				case SnakeBird::kFruit:
				{
					sb.BeginEditing();
					sb.SetGroundType(x, y, SnakeBird::kEmpty);
					sb.EndEditing();
					UpdateLevelLink();
					gRefreshBackground = true;
					break;
				}
				case SnakeBird::kEmpty:
				case SnakeBird::kGround:
				case SnakeBird::kSpikes:
				case SnakeBird::kPortal1:
				case SnakeBird::kPortal2:
				case SnakeBird::kExit:
				{
					sb.BeginEditing();
					sb.SetGroundType(x, y, SnakeBird::kEmpty);
					sb.SetGroundType(x, y, SnakeBird::kFruit);
					sb.EndEditing();
					UpdateLevelLink();
					gRefreshBackground = true;
					break;
				}
				case SnakeBird::kBlock1:
				case SnakeBird::kBlock2:
				case SnakeBird::kBlock3:
				case SnakeBird::kBlock4:
				{
					sb.BeginEditing();
					sb.RemoveBlock(gMouseX, gMouseY);
					sb.SetGroundType(gMouseX, gMouseY, SnakeBird::kEmpty);
					sb.SetGroundType(gMouseX, gMouseY, SnakeBird::kFruit);
					sb.EndEditing();
					UpdateLevelLink();
					lastFrameSnake = snake = sb.GetStart();
					gRefreshBackground = true;
					break;
				}
				default:
					break;
			}
			break;
		}
		case SnakeBird::kExit:
		{
			switch (renderedType)
			{
				case SnakeBird::kExit:
				{
					sb.BeginEditing();
					sb.SetGroundType(x, y, SnakeBird::kEmpty);
					sb.EndEditing();
					UpdateLevelLink();
					gRefreshBackground = true;
					break;
				}
				case SnakeBird::kEmpty:
				case SnakeBird::kGround:
				case SnakeBird::kSpikes:
				case SnakeBird::kPortal1:
				case SnakeBird::kPortal2:
				case SnakeBird::kFruit:
				{
					sb.BeginEditing();
					sb.SetGroundType(x, y, SnakeBird::kExit);
					sb.EndEditing();
					UpdateLevelLink();
					gRefreshBackground = true;
					break;
				}
				case SnakeBird::kBlock1:
				case SnakeBird::kBlock2:
				case SnakeBird::kBlock3:
				case SnakeBird::kBlock4:
				{
					sb.BeginEditing();
					sb.RemoveBlock(gMouseX, gMouseY);
					sb.SetGroundType(gMouseX, gMouseY, SnakeBird::kEmpty);
					sb.EndEditing();
					UpdateLevelLink();
					lastFrameSnake = snake = sb.GetStart();
					gRefreshBackground = true;
					break;
				}
				default:
					break;
			}
			break;
		}
		case SnakeBird::kPortal:
		{
			switch (renderedType)
			{
				case SnakeBird::kPortal:
				{
					sb.BeginEditing();
					sb.SetGroundType(gMouseX, gMouseY, SnakeBird::kEmpty);
					sb.EndEditing();
					UpdateLevelLink();
					gRefreshBackground = true;
					break;
				}
				case SnakeBird::kBlock1:
				case SnakeBird::kBlock2:
				case SnakeBird::kBlock3:
				case SnakeBird::kBlock4:
				{
					sb.BeginEditing();
					sb.RemoveBlock(gMouseX, gMouseY);
					sb.SetGroundType(gMouseX, gMouseY, SnakeBird::kEmpty);
					sb.SetGroundType(gMouseX, gMouseY, SnakeBird::kPortal);
					sb.EndEditing();
					UpdateLevelLink();
					lastFrameSnake = snake = sb.GetStart();
					gRefreshBackground = true;
					break;
				}
				default:
				{
					sb.BeginEditing();
					sb.SetGroundType(gMouseX, gMouseY, SnakeBird::kPortal);
					sb.EndEditing();
					UpdateLevelLink();
					gRefreshBackground = true;
					break;
				}
			}
			break;
		}
		case SnakeBird::kBlock1:
		{
			switch (renderedType) {
				case SnakeBird::kBlock1:
					sb.BeginEditing();
					sb.RemoveBlock(gMouseX, gMouseY);
					sb.SetGroundType(gMouseX, gMouseY, SnakeBird::kEmpty);
					sb.EndEditing();
					UpdateLevelLink();
					lastFrameSnake = snake = sb.GetStart();
					gRefreshBackground = true;
					break;
				case SnakeBird::kEmpty:
				case SnakeBird::kGround:
				case SnakeBird::kSpikes:
				case SnakeBird::kPortal1:
				case SnakeBird::kPortal2:
				case SnakeBird::kExit:
				case SnakeBird::kFruit:
					sb.BeginEditing();
					sb.SetGroundType(gMouseX, gMouseY, SnakeBird::kEmpty);
					sb.SetGroundType(gMouseX, gMouseY, SnakeBird::kBlock1);
					sb.EndEditing();
					UpdateLevelLink();
					lastFrameSnake = snake = sb.GetStart();
					history[0] = snake;
					gRefreshBackground = true;
				default:
					break;
			}
			break;
		}
		case SnakeBird::kSnake1:
		{
			switch (renderedType)
			{
				case SnakeBird::kSnake1:
					sb.BeginEditing();
					sb.RemoveSnake(x, y, o, 0);
					lastFrameSnake = snake = sb.GetStart();
					history.clear();
					future.clear();
					history.push_back(snake);
					actionInProgressStep.Reset();
					UpdateActiveSnake();
					sb.EndEditing();
					UpdateLevelLink();
					gRefreshBackground = true;
					break;
				case SnakeBird::kBlock1:
				case SnakeBird::kBlock2:
				case SnakeBird::kBlock3:
				case SnakeBird::kBlock4:
				{
					sb.BeginEditing();
					sb.RemoveBlock(x, y);
					if (o == kMouseDown)
					{
						sb.SetGroundType(x, y, SnakeBird::kEmpty);
						sb.AddSnakeHead(x, y, 0);
					}
					else if (o == kMouseDrag)
					{
						sb.SetGroundType(x, y, SnakeBird::kEmpty);
						sb.AddSnakeBody(x, y, 0);
					}
					lastFrameSnake = snake = sb.GetStart();
					history.clear();
					future.clear();
					history.push_back(snake);
					actionInProgressStep.Reset();
					UpdateActiveSnake();
					sb.EndEditing();
					UpdateLevelLink();
					gRefreshBackground = true;
					break;
				}
				case SnakeBird::kSnake2:
				case SnakeBird::kSnake3:
				case SnakeBird::kSnake4:
					break;
				default:
				{
					sb.BeginEditing();
					sb.SetGroundType(x, y, SnakeBird::kEmpty);
					if (o == kMouseDown)
					{
						sb.SetGroundType(x, y, SnakeBird::kEmpty);
						sb.AddSnakeHead(x, y, 0);
					}
					lastFrameSnake = snake = sb.GetStart();
					history.clear();
					if (o == kMouseDrag)
					{
						sb.SetGroundType(x, y, SnakeBird::kEmpty);
						sb.AddSnakeBody(x, y, 0);
					}
					future.clear();
					history.push_back(snake);
					actionInProgressStep.Reset();
					UpdateActiveSnake();
					sb.EndEditing();
					UpdateLevelLink();
					gRefreshBackground = true;
					break;
				}
			}
			break;
		}
		case SnakeBird::kSnake2:
		{
			switch (renderedType)
			{
				case SnakeBird::kSnake2:
					sb.BeginEditing();
					sb.RemoveSnake(x, y, o, 1);
					lastFrameSnake = snake = sb.GetStart();
					history.clear();
					future.clear();
					history.push_back(snake);
					actionInProgressStep.Reset();
					UpdateActiveSnake();
					sb.EndEditing();
					UpdateLevelLink();
					gRefreshBackground = true;
					break;
				case SnakeBird::kBlock1:
				case SnakeBird::kBlock2:
				case SnakeBird::kBlock3:
				case SnakeBird::kBlock4:
				{
					sb.BeginEditing();
					sb.RemoveBlock(x, y);
					if (o == kMouseDown)
					{
						sb.SetGroundType(x, y, SnakeBird::kEmpty);
						sb.AddSnakeHead(x, y, 0);
					}
					else if (o == kMouseDrag)
					{
						sb.SetGroundType(x, y, SnakeBird::kEmpty);
						sb.AddSnakeBody(x, y, 1);
					}
					lastFrameSnake = snake = sb.GetStart();
					history.clear();
					future.clear();
					history.push_back(snake);
					actionInProgressStep.Reset();
					UpdateActiveSnake();
					sb.EndEditing();
					UpdateLevelLink();
					gRefreshBackground = true;
					break;
				}
				case SnakeBird::kSnake1:
				case SnakeBird::kSnake3:
				case SnakeBird::kSnake4:
					break;
				default:
				{
					sb.BeginEditing();
					if (o == kMouseDown)
					{
						sb.SetGroundType(x, y, SnakeBird::kEmpty);
						sb.AddSnakeHead(x, y, 1);
					}
					else if (o == kMouseDrag)
					{
						sb.SetGroundType(x, y, SnakeBird::kEmpty);
						sb.AddSnakeBody(x, y, 1);
					}
					lastFrameSnake = snake = sb.GetStart();
					history.clear();
					future.clear();
					history.push_back(snake);
					actionInProgressStep.Reset();
					UpdateActiveSnake();
					sb.EndEditing();
					UpdateLevelLink();
					gRefreshBackground = true;
					break;
				}
			}
			break;
		}
		default:
			break;
	}
}

void ShowSolutionLength(int length)
{
	if (!assistiveEditor)
	{
		editorMessage = "";
		return;
	}
	if (length == 0)
	{
		Timer t;
		BFS<SnakeBird::SnakeBirdState, SnakeBird::SnakeBirdAction, SnakeBird::SnakeBird> bfs;
		bfs.SetNodeLimit(50000);
		bfs.SetVerbose(false);
		t.StartTimer();
		bfs.GetPath(&sb, snake, snake, future);
		t.EndTimer();
		if (future.size() > 0)
			length = future.size();
		else if (bfs.GetNodesExpanded() >= 50000)
			length = -1;
	}
    if (length > 0)
    {
		gSolutionLength = length;
		std::string path = std::to_string(length);
		std::string words = "Level is solvable in ";
		std::string moves = " moves";
		editorMessage = words + path + moves;
    }
	else if (length < 0)
	{
		gSolutionLength = 0;
        editorMessage = "Level unsolvable in current resource limits";
	}
	else if (length == 0)
    {
		gSolutionLength = 0;
        editorMessage = "Level unsolvable";
    }
}


EditorColor CanChangeMap(int x, int y)
{
	auto renderedType = sb.GetRenderedGroundType(snake, x, y);
	switch (gEditorMode)
	{
		case SnakeBird::kGround:
		{
			switch (renderedType)
			{
				case SnakeBird::kSpikes:
				case SnakeBird::kEmpty:
				case SnakeBird::kFruit:
				case SnakeBird::kPortal1:
				case SnakeBird::kPortal2:
				case SnakeBird::kExit:
				case SnakeBird::kBlock1:
				case SnakeBird::kBlock2:
				case SnakeBird::kBlock3:
				case SnakeBird::kBlock4:
					return kCanAdd;
					break;
				case SnakeBird::kGround:
					return kCanRemove;
					break;
				default:
					return kCannotAddRemove;
					break;
			}
			break;
		}
		case SnakeBird::kEmpty:
		{
			switch (renderedType)
			{
				case SnakeBird::kPortal:
				case SnakeBird::kPortal1:
				case SnakeBird::kPortal2:
				case SnakeBird::kFruit:
				case SnakeBird::kSpikes:
				case SnakeBird::kGround:
				case SnakeBird::kExit:
				case SnakeBird::kBlock1:
				case SnakeBird::kBlock2:
				case SnakeBird::kBlock3:
				case SnakeBird::kBlock4:
					return kCanAdd;
					break;
				case SnakeBird::kEmpty:
				default:
					return kCannotAddRemove;
					break;
			}
			break;
		}
		case SnakeBird::kSpikes:
		{
			switch (renderedType)
			{
				case SnakeBird::kEmpty:
				case SnakeBird::kPortal1:
				case SnakeBird::kPortal2:
				case SnakeBird::kGround:
				case SnakeBird::kFruit:
				case SnakeBird::kExit:
				case SnakeBird::kBlock1:
				case SnakeBird::kBlock2:
				case SnakeBird::kBlock3:
				case SnakeBird::kBlock4:
					return kCanAdd;
					break;
				case SnakeBird::kSpikes:
					return kCanRemove;
					break;
				default:
					return kCannotAddRemove;
					break;
			}
			break;
		}
		case SnakeBird::kFruit:
		{
			switch (renderedType)
			{
				case SnakeBird::kFruit:
					return kCanRemove;
					break;
				case SnakeBird::kEmpty:
				case SnakeBird::kPortal1:
				case SnakeBird::kPortal2:
				case SnakeBird::kGround:
				case SnakeBird::kExit:
				case SnakeBird::kBlock1:
				case SnakeBird::kBlock2:
				case SnakeBird::kBlock3:
				case SnakeBird::kBlock4:
				case SnakeBird::kSpikes:
					return kCanAdd;
					break;
				default:
					return kCannotAddRemove;
					break;
			}
			break;
		}
		case SnakeBird::kExit:
		{
			switch (renderedType)
			{
				case SnakeBird::kExit:
					return kCanRemove;
					break;
				case SnakeBird::kEmpty:
				case SnakeBird::kPortal1:
				case SnakeBird::kPortal2:
				case SnakeBird::kGround:
				case SnakeBird::kBlock1:
				case SnakeBird::kBlock2:
				case SnakeBird::kBlock3:
				case SnakeBird::kBlock4:
				case SnakeBird::kSpikes:
				case SnakeBird::kFruit:
					return kCanAdd;
					break;
				default:
					return kCannotAddRemove;
					break;
			}
			break;
		}
		case SnakeBird::kPortal:
		{
			switch (renderedType)
			{
				case SnakeBird::kPortal1:
				case SnakeBird::kPortal2:
					return kCanRemove;
					break;
				case SnakeBird::kEmpty:
				case SnakeBird::kGround:
				case SnakeBird::kExit:
				case SnakeBird::kBlock1:
				case SnakeBird::kBlock2:
				case SnakeBird::kBlock3:
				case SnakeBird::kBlock4:
				case SnakeBird::kSpikes:
				case SnakeBird::kFruit:
					if (sb.GetNumPortals() < 2)
					{
						return kCanAdd;
					}
					else
						return kCannotAddRemove;
					break;
				default:
					return kCannotAddRemove;
					break;
			}
			break;
		}
		case SnakeBird:: kBlock1:
		{
			switch (renderedType) {
				case SnakeBird::kBlock1:
				case SnakeBird::kBlock2:
				case SnakeBird::kBlock3:
				case SnakeBird::kBlock4:
					return kCanRemove;
					break;
				case SnakeBird::kEmpty:
				case SnakeBird::kPortal1:
				case SnakeBird::kPortal2:
				case SnakeBird::kGround:
				case SnakeBird::kExit:
				case SnakeBird::kSpikes:
				case SnakeBird::kFruit:
					return kCanAdd;
					break;
				default:
					break;
			}
		}
		case SnakeBird:: kSnake1:
		{
			switch (renderedType) {
				case SnakeBird::kEmpty:
				case SnakeBird::kPortal1:
				case SnakeBird::kPortal2:
				case SnakeBird::kGround:
				case SnakeBird::kExit:
				case SnakeBird::kBlock1:
				case SnakeBird::kBlock2:
				case SnakeBird::kBlock3:
				case SnakeBird::kBlock4:
				case SnakeBird::kSpikes:
				case SnakeBird::kFruit:
					return kCanAdd;
					break;
				case SnakeBird:: kSnake1:
				case SnakeBird:: kSnake2:
				case SnakeBird:: kSnake3:
				case SnakeBird:: kSnake4:
					return kCanRemove;
					break;
				default:
					break;
			}
		}
		case SnakeBird:: kSnake2:
		{
			switch (renderedType) {
				case SnakeBird::kEmpty:
				case SnakeBird::kPortal1:
				case SnakeBird::kPortal2:
				case SnakeBird::kGround:
				case SnakeBird::kExit:
				case SnakeBird::kBlock1:
				case SnakeBird::kBlock2:
				case SnakeBird::kBlock3:
				case SnakeBird::kBlock4:
				case SnakeBird::kSpikes:
				case SnakeBird::kFruit:
					return kCanAdd;
					break;
				case SnakeBird:: kSnake1:
				case SnakeBird:: kSnake2:
				case SnakeBird:: kSnake3:
				case SnakeBird:: kSnake4:
					return kCanRemove;
					break;
				default:
					break;
			}
		}
		default:
			return kCannotAddRemove;
			break;
	}
}
