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
#include "PuzzleSample.h"
#include "Timer.h"
#include "TemplateAStar.h"
#include "StephenPuzzle.h"
#include "Map2DEnvironment.h"
#include "ScreenTransition.h"
int which = 0;
SPState::StephenPuzzle *puzzle;
SPState::puzzleState s;
Map *editor;
MapEnvironment *me;
LineTransition line(20, 10);
FallingBoxTransition box(20);
ScreenTransition *trans;
//struct firework {
//	int count;
//	std::vector<Graphics::point> points;
//	rgbColor color;
//};

enum gameState {
	kSolving,
	kFadeOut,
	kFadeIn,
	kFinished
};

gameState currState = kSolving;

bool fireworks = false;
float firetime = 2;
Graphics::point p;

int main(int argc, char* argv[])
{
	InstallHandlers();
	RunHOGGUI(argc, argv, 1440, 1080);
	return 0;
}

/**
 * Allows you to install any keyboard handlers needed for program interaction.
 */
void InstallHandlers()
{
	InstallKeyboardHandler(MyDisplayHandler, "Record", "Record a movie", kAnyModifier, 'r');
	InstallKeyboardHandler(MyDisplayHandler, "Up", "", kAnyModifier, 'w');
	InstallKeyboardHandler(MyDisplayHandler, "Down", "", kAnyModifier, 's');
	InstallKeyboardHandler(MyDisplayHandler, "Left", "", kNoModifier, 'a');
	InstallKeyboardHandler(MyDisplayHandler, "Right", "", kAnyModifier, 'd');
	InstallKeyboardHandler(MyDisplayHandler, "Pick Up", "", kAnyModifier, ' ');

	InstallCommandLineHandler(MyCLHandler, "-map", "-map filename", "Selects the default map to be loaded.");
	
	InstallWindowHandler(MyWindowHandler);

	InstallMouseClickHandler(MyClickHandler);
}

void LoadMap()
{
	if (which == 11)
	{
		fireworks = true;
		return;
	}
	std::string mapName ="/Users/nathanst/hog2/maps/local/level"+std::to_string(which)+".map";
	Map *m = new Map(mapName.c_str());
	s = puzzle->LoadPuzzle(m);
	delete m;
	which++;
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

//		ReinitViewports(windowID, {-1, -1, 0.5f, 1}, kScaleToSquare);
//		AddViewport(windowID, {0.5f, -1, 1, 1}, kScaleToFill);

		puzzle = new SPState::StephenPuzzle();
		LoadMap();
		editor = new Map(1,5);
		editor->SetTerrainType(0, 0, kOutOfBounds);
		editor->SetTerrainType(0, 1, kGround);
		editor->SetTerrainType(0, 2, kTrees);
		editor->SetTerrainType(0, 3, kSwamp);
		editor->SetTerrainType(0, 4, kGrass);
		me = new MapEnvironment(editor);
	}
}

void MyFrameHandler(unsigned long windowID, unsigned int viewport, void *)
{
	auto &d = GetContext(windowID)->display;
	d.FillSquare({0,0}, 2, Colors::gray);
	
	if (viewport == 0)
	{
		puzzle->Draw(d);
		puzzle->Draw(d, s);
		std::string tmp = "Level "+std::to_string(which-1);
		d.DrawText(tmp.c_str(), {-0.0f, -0.8f}, Colors::white, 0.1, Graphics::textAlignCenter);
		
		if (fireworks && firetime > 1)
		{
			firetime = 0;
			p.x = ((random()%100)-50)/50.0f;
			p.y = ((random()%100)-50)/50.0f;
		}
		if (fireworks)
		{
			for (float t = 0; t < TWOPI; t += PI/5.0f)
			{
				float x = sin(t);
				float y = cos(t)+firetime*firetime;
				d.DrawLine({p.x+x*firetime, p.y+y*firetime}, {p.x+x*1.4f*firetime, p.y+y*1.4f*firetime}, 1.0, Colors::white);
			}
			firetime += 0.03;
		}
	}
	if (viewport == 1)
	{
		me->Draw(d);
	}

	if (currState == kFadeOut)
	{
		if (trans->Step(0.03))
		{
			LoadMap();
			currState = kFadeIn;
			trans = (random()%2)?((ScreenTransition*)&line):((ScreenTransition*)&box);
			trans->Reset(1);
		}
		trans->Draw(d);
	}
	else if (currState == kFadeIn)
	{
		if (trans->Step(-0.03))
		{
			currState = kSolving;
		}
		trans->Draw(d);
	}
	return;
	
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
		case 'w':
		{
			if (currState != kSolving) break;
			puzzle->ApplyAction(s, SPState::kN);
			if (puzzle->GoalTest(s))
			{
				currState = kFadeOut;
				trans = (random()%2)?((ScreenTransition*)&line):((ScreenTransition*)&box);
				trans->Reset(0);
			}
			break;
		}
		case 's':
		{
			if (currState != kSolving) break;
			puzzle->ApplyAction(s, SPState::kS);
			if (puzzle->GoalTest(s))
			{
				currState = kFadeOut;
				trans = (random()%2)?((ScreenTransition*)&line):((ScreenTransition*)&box);
				trans->Reset(0);
			}
			break;
		}
		case 'a':
		{
			if (currState != kSolving) break;
			puzzle->ApplyAction(s, SPState::kW);
			if (puzzle->GoalTest(s))
			{
				currState = kFadeOut;
				trans = (random()%2)?((ScreenTransition*)&line):((ScreenTransition*)&box);
				trans->Reset(0);
			}
			break;
		}
		case 'd':
		{
			if (currState != kSolving) break;
			puzzle->ApplyAction(s, SPState::kE);
			if (puzzle->GoalTest(s))
			{
				currState = kFadeOut;
				trans = (random()%2)?((ScreenTransition*)&line):((ScreenTransition*)&box);
				trans->Reset(0);
			}
			break;
		}
		case ' ':
		{
			if (currState != kSolving) break;
			if (s.carry)
				puzzle->ApplyAction(s, SPState::kDrop);
			else
				puzzle->ApplyAction(s, SPState::kPickUp);
			break;
		}
		case 'r':
			currState = kSolving;
			which = 0;
			fireworks = false;
			LoadMap();
			break;

		default:
			break;
	}
	
}


bool MyClickHandler(unsigned long windowID, int viewport, int x, int y, point3d loc, tButtonType, tMouseEventType)
{
	return false;
}
