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
#include "Map2DEnvironment.h"
#include "RandomUnits.h"
#include "TemplateAStar.h"
#include "GraphEnvironment.h"
#include "ScenarioLoader.h"
#include "BFS.h"
#include "PEAStar.h"
#include "EPEAStar.h"
#include "MapGenerators.h"
#include "FPUtil.h"
#include "CanonicalGrid.h"
#include "Witness.h"


std::vector<std::vector<Witness<3, 3>>> firstSet;
std::vector<std::vector<Witness<4, 4>>> secondSet;

//Witness<3, 3> wp33_1a;
//Witness<3, 3> wp33_1b;
//Witness<3, 3> wp33_1c;
Witness<3, 3> wp33a;
Witness<4, 4> wp44;

InteractiveWitnessState<3, 3> iws3;
InteractiveWitnessState<4, 4> iws4;

int whichPuzzle = 0;
bool redrawBackground = true;

int main(int argc, char* argv[])
{
	InstallHandlers();
	RunHOGGUI(argc, argv, 900, 300);
	return 0;
}


/**
 * Allows you to install any keyboard handlers needed for program interaction.
 */
void InstallHandlers()
{
	InstallKeyboardHandler(MyDisplayHandler, "Reset", "Reset puzzles.", kNoModifier, 'r');
	
	InstallWindowHandler(MyWindowHandler);
	
	InstallMouseClickHandler(MyClickHandler, static_cast<tMouseEventType>(kMouseMove|kMouseUp|kMouseDrag));
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
//		ReinitViewports(windowID, {-1.0f, -0.33333333f, -0.33333333f, 0.33333333f}, kScaleToFill);
//		AddViewport(windowID, {-0.33333333f, -0.33333333f, 0.33333333f, 0.33333333f}, kScaleToFill); // kTextView
//		AddViewport(windowID, {0.33333333f, -0.33333333f, 1.0f, 0.33333333f}, kScaleToFill); // kTextView
		ReinitViewports(windowID, {-1.0f, -1.0f, -0.33333333f, 1.0f}, kScaleToSquare);
		AddViewport(windowID, {-0.33333333f, -1.0f, 0.33333333f, 1.0f}, kScaleToSquare); // kTextView
		AddViewport(windowID, {0.33333333f, -1.0f, 1.0f, 1.0f}, kScaleToSquare); // kTextView

		firstSet.resize(3);
		
		Witness<3, 3> tmp33;
		submitTextToBuffer("");
		// puzzle 1
		tmp33.ClearTetrisConstraints();
		tmp33.AddTetrisConstraint(0, 2, 1);
		tmp33.AddTetrisConstraint(1, 2, 1);
		tmp33.AddTetrisConstraint(1, 1, 10);
		firstSet[0].push_back(tmp33);
		
		tmp33.ClearTetrisConstraints();
		tmp33.AddTetrisConstraint(0, 2, 3);
		tmp33.AddTetrisConstraint(1, 2, 3);
		tmp33.AddTetrisConstraint(0, 1, 2);
		firstSet[0].push_back(tmp33);

		tmp33.ClearTetrisConstraints();
		tmp33.AddTetrisConstraint(0, 2, 1);
		tmp33.AddTetrisConstraint(1, 2, 3);
		tmp33.AddTetrisConstraint(1, 1, 8);
		firstSet[0].push_back(tmp33);

		// puzzle 2
		tmp33.ClearTetrisConstraints();
		tmp33.AddTetrisConstraint(0, 2, 1);
		tmp33.AddTetrisConstraint(1, 2, 3);
		tmp33.AddTetrisConstraint(1, 1, 4);
		firstSet[1].push_back(tmp33);
		
		tmp33.ClearTetrisConstraints();
		tmp33.AddTetrisConstraint(0, 2, 8);
		tmp33.AddTetrisConstraint(0, 1, 3);
		tmp33.AddTetrisConstraint(1, 1, 1);
		firstSet[1].push_back(tmp33);
		
		tmp33.ClearTetrisConstraints();
		tmp33.AddTetrisConstraint(0, 2, 1);
		tmp33.AddTetrisConstraint(1, 1, 1);
		tmp33.AddTetrisConstraint(1, 0, 10);
		firstSet[1].push_back(tmp33);
		
		// puzzle 3
		tmp33.ClearTetrisConstraints();
		tmp33.AddTetrisConstraint(0, 2, 1);
		tmp33.AddTetrisConstraint(1, 2, 2);
		tmp33.AddTetrisConstraint(0, 1, 12);
		firstSet[2].push_back(tmp33);
		
		tmp33.ClearTetrisConstraints();
		tmp33.AddTetrisConstraint(0, 1, 1);
		tmp33.AddTetrisConstraint(2, 1, 12);
		tmp33.AddTetrisConstraint(2, 0, 2);
		firstSet[2].push_back(tmp33);
		
		tmp33.ClearTetrisConstraints();
		tmp33.AddTetrisConstraint(0, 1, 12);
		tmp33.AddTetrisConstraint(0, 0, 1);
		tmp33.AddTetrisConstraint(1, 0, 2);
		firstSet[2].push_back(tmp33);
		
		secondSet.resize(3);
		Witness<4, 4> tmp44;
		// puzzle 4
		tmp44.ClearTetrisConstraints();
		tmp44.AddTetrisConstraint(0, 3, 1);
		tmp44.AddTetrisConstraint(1, 3, 2);
		tmp44.AddTetrisConstraint(1, 2, 7);
		secondSet[0].push_back(tmp44);

		tmp44.ClearTetrisConstraints();
		tmp44.AddTetrisConstraint(0, 3, 1);
		tmp44.AddTetrisConstraint(1, 3, 2);
		tmp44.AddTetrisConstraint(3, 3, 7);
		secondSet[0].push_back(tmp44);

		tmp44.ClearTetrisConstraints();
		tmp44.AddTetrisConstraint(0, 3, 1);
		tmp44.AddTetrisConstraint(2, 2, 2);
		tmp44.AddTetrisConstraint(3, 2, 4);
		secondSet[0].push_back(tmp44);

		// puzzle 5
		tmp44.ClearTetrisConstraints();
		tmp44.AddTetrisConstraint(0, 3, 1);
		tmp44.AddTetrisConstraint(1, 3, 6);
		tmp44.AddTetrisConstraint(0, 2, 7);
		secondSet[1].push_back(tmp44);
		
		tmp44.ClearTetrisConstraints();
		tmp44.AddTetrisConstraint(0, 3, 1);
		tmp44.AddTetrisConstraint(1, 3, 2);
		tmp44.AddTetrisConstraint(1, 2, 24);
		secondSet[1].push_back(tmp44);
		
		tmp44.ClearTetrisConstraints();
		tmp44.AddTetrisConstraint(0, 3, 1);
		tmp44.AddTetrisConstraint(2, 2, 6);
		tmp44.AddTetrisConstraint(2, 3, 5);
		secondSet[1].push_back(tmp44);

		// puzzle 6
		tmp44.ClearTetrisConstraints();
		tmp44.AddTetrisConstraint(0, 3, 1);
		tmp44.AddTetrisConstraint(1, 3, 2);
		tmp44.AddTetrisConstraint(1, 1, 5);
		secondSet[2].push_back(tmp44);
		
		tmp44.ClearTetrisConstraints();
		tmp44.AddTetrisConstraint(0, 2, 1);
		tmp44.AddTetrisConstraint(0, 1, 16);
		tmp44.AddTetrisConstraint(1, 2, 3);
		secondSet[2].push_back(tmp44);
		
		tmp44.ClearTetrisConstraints();
		tmp44.AddTetrisConstraint(3, 0, 1);
		tmp44.AddTetrisConstraint(2, 2, 20);
		tmp44.AddTetrisConstraint(3, 2, 9);
		secondSet[2].push_back(tmp44);

	
	}
	
}

void MyFrameHandler(unsigned long windowID, unsigned int viewport, void *)
{
	Graphics::Display &display = getCurrentContext()->display;
	if (whichPuzzle < 3)
	{
		if (viewport == 0)
			iws3.IncrementTime();
		if (redrawBackground)
		{
			display.StartBackground();
			firstSet[whichPuzzle][viewport].Draw(display);
			display.EndBackground();
		}
		firstSet[whichPuzzle][viewport].Draw(display, iws3);
	}
	else if (whichPuzzle < 6) {
		if (viewport == 0)
			iws4.IncrementTime();
		if (redrawBackground)
		{
			display.StartBackground();
			secondSet[whichPuzzle-3][viewport].Draw(display);
			display.EndBackground();
		}
		secondSet[whichPuzzle-3][viewport].Draw(display, iws4);
	}
	else {
		if (redrawBackground)
		{
			display.StartBackground();
			display.FillRect({-1.0f, -1.0f, 1.0f, 1.0f}, Colors::white);
			display.EndBackground();
		}
		display.DrawText("Puzzles Complete!", {0.0f, 0.0f}, Colors::black, 0.2f, Graphics::textAlignCenter);
	}
	if (viewport == 2)
		redrawBackground = false;
}



void MyDisplayHandler(unsigned long windowID, tKeyboardModifier mod, char key)
{
	switch (key)
	{
		case 'r':
		{
			whichPuzzle = 0;
			submitTextToBuffer("");
			redrawBackground = true;
			iws3.Reset();
		}
			break;
		default:
			break;
	}
}

#include "SVGUtil.h"
std::string GetPuzzleSVG()
{
	Graphics::Display &d = getCurrentContext()->display;
	std::string res = MakeSVG(d, 200, 200, 0)+MakeSVG(d, 200, 200, 1)+MakeSVG(d, 200, 200, 2);
	return res;
}

bool MyClickHandler(unsigned long , int viewport, int windowX, int windowY, point3d p, tButtonType button, tMouseEventType e)
{
	if (e == kMouseDrag) // ignore movement with mouse button down
		return true;
	
	if (e == kMouseUp)
	{
		if (whichPuzzle < 3)
		{
			if (firstSet[whichPuzzle][0].Click(p, iws3))
			{
				int cnt = 0;
				for (int x = 0; x < firstSet[whichPuzzle].size(); x++)
				{
					if (firstSet[whichPuzzle][x].GoalTest(iws3.ws))
						cnt++;
				}
				if (cnt == firstSet[whichPuzzle].size())
				{
					std::string tmp = getTextBuffer();
					submitTextToBuffer((GetPuzzleSVG()+"<br/>"+tmp).c_str());
					whichPuzzle++;
					iws3.Reset();
					redrawBackground = true;
				}
				else {
					printf("Invalid solution\n");
					iws3.Reset();
				}
			}
		}
		else if (whichPuzzle < 6) {
			if (secondSet[whichPuzzle-3][0].Click(p, iws4))
			{
				int cnt = 0;
				for (int x = 0; x < secondSet[whichPuzzle-3].size(); x++)
				{
					if (secondSet[whichPuzzle-3][x].GoalTest(iws4.ws))
						cnt++;
				}
				if (cnt == secondSet[whichPuzzle-3].size())
				{
					std::string tmp = getTextBuffer();
					submitTextToBuffer((GetPuzzleSVG()+"<br/>"+tmp).c_str());
					whichPuzzle++;
					iws4.Reset();
					redrawBackground = true;
				}
				else {
					printf("Invalid solution\n");
					iws4.Reset();
				}
			}
		}
	}
	if (e == kMouseMove)
	{
//		printf("Move\n");
		if (whichPuzzle < 3)
			firstSet[whichPuzzle][viewport].Move(p, iws3);
		else if (whichPuzzle < 6)
			secondSet[whichPuzzle-3][viewport].Move(p, iws4);
	}
	
	// Don't need any other mouse support
	return true;
}


