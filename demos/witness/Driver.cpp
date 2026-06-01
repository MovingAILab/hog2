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
#include "FPUtil.h"
#include "Witness.h"
#include "SVGUtil.h"

//To turn a sequence of svg files into levels that can be loaded here:
//grep -h "<!" *.svg | sed 's/<!\-\-//g' | sed 's/\-\->//g' | sed 's/\"/\\\"/g' | sed 's/^/\"/g' | sed 's/$/\",/g' | pbcopy

#include <sys/stat.h>
bool fileExists(const char *name)
{
	struct stat buffer;
	return (stat(name, &buffer) == 0);
}

std::vector<std::string> levels =
{
	"{\"dim\":\"3x3\",\"cc\":{\"0;0;#000000\",\"0;0;#000000\",\"0;0;#000000\",\"0;0;#000000\",\"0;0;#000000\",\"0;0;#000000\",\"0;0;#000000\",\"0;0;#000000\",\"0;0;#000000\"},\"mc\":\"0001001000000000010000000000000000000000\"}",
	"{\"dim\":\"3x3\",\"cc\":{\"0;0;#000000\",\"0;0;#000000\",\"0;0;#000000\",\"0;0;#000000\",\"0;0;#000000\",\"0;0;#000000\",\"0;0;#000000\",\"0;0;#000000\",\"0;0;#000000\"},\"mc\":\"0001000000000000110000000000000000000000\"}",
	"{\"dim\":\"3x3\",\"cc\":{\"0;0;#000000\",\"0;0;#000000\",\"0;0;#000000\",\"0;0;#000000\",\"0;0;#000000\",\"0;0;#000000\",\"0;0;#000000\",\"0;0;#000000\",\"0;0;#000000\"},\"mc\":\"0000000000010000001000000000000000010000\"}",
	"{\"dim\":\"3x4\",\"cc\":{\"0;0;#002BDFFFFFFF1FFFFFFF1\",\"0;0;#2C90000\",\"0;0;#000000\",\"0;0;#000000\",\"0;0;#000000\",\"0;0;#000000\",\"0;0;#000000\",\"0;0;#000000\",\"0;0;#000000\",\"0;0;#000000\",\"0;0;#000000\",\"0;0;#000000\"},\"mc\":\"000000000101000000000000010000000000000100000001000\"}",
	"{\"dim\":\"3x4\",\"cc\":{\"0;0;#002BDFFFFFFF1FFFFFFF1\",\"0;0;#2C90000\",\"0;0;#000000\",\"0;0;#000000\",\"0;0;#000000\",\"0;0;#000000\",\"0;0;#000000\",\"0;0;#000000\",\"0;0;#000000\",\"0;0;#000000\",\"0;0;#000000\",\"0;0;#000000\"},\"mc\":\"000000010010001000000001001000000000000000000000000\"}",
	"{\"dim\":\"3x3\",\"cc\":{\"1;0;#FFFFFF\",\"1;0;#000000\",\"1;0;#FFFFFF\",\"0;0;#000000\",\"1;0;#FFFFFF\",\"0;0;#000000\",\"1;0;#000000\",\"0;0;#000000\",\"0;0;#000000\"},\"mc\":\"0000000000000000000000000000000000000000\"}",
	"{\"dim\":\"3x3\",\"cc\":{\"1;0;#FFFFFF\",\"1;0;#000000\",\"0;0;#000000\",\"0;0;#000000\",\"1;0;#FFFFFF\",\"0;0;#000000\",\"1;0;#000000\",\"1;0;#FFFFFF\",\"0;0;#000000\"},\"mc\":\"0000000000000000000000000000000000000000\"}",
	"{\"dim\":\"3x4\",\"cc\":{\"1;1076363264;#FFFFFF\",\"1;0;#000000\",\"1;0;#FFFFFF\",\"0;0;#000000\",\"0;0;#000000\",\"0;0;#000000\",\"0;0;#000000\",\"0;0;#000000\",\"1;0;#000000\",\"1;0;#FFFFFF\",\"1;0;#000000\",\"0;0;#000000\"},\"mc\":\"000000000000000000000000000000000000000000000000000\"}",
	"{\"dim\":\"3x4\",\"cc\":{\"1;1077084160;#FFFFFF\",\"1;0;#000000\",\"1;0;#000000\",\"0;0;#000000\",\"0;0;#000000\",\"1;0;#FFFFFF\",\"0;0;#000000\",\"1;0;#000000\",\"1;0;#FFFFFF\",\"1;0;#000000\",\"1;0;#FFFFFF\",\"0;0;#000000\"},\"mc\":\"000000000000000000000000000000000000000000000000000\"}",
	"{\"dim\":\"3x4\",\"cc\":{\"1;1077084160;#FFFFFF\",\"1;0;#FFFFFF\",\"1;0;#FFFFFF\",\"0;0;#000000\",\"0;0;#000000\",\"1;0;#000000\",\"0;0;#000000\",\"1;0;#FFFFFF\",\"1;0;#000000\",\"1;0;#FFFFFF\",\"1;0;#000000\",\"0;0;#000000\"},\"mc\":\"000000000000000000000000000000000000000000000000000\"}",
	"{\"dim\":\"3x4\",\"cc\":{\"1;1077084160;#FFFFFF\",\"1;0;#000000\",\"1;0;#000000\",\"0;0;#000000\",\"0;0;#000000\",\"1;0;#FFFFFF\",\"0;0;#000000\",\"1;0;#000000\",\"1;0;#000000\",\"1;0;#000000\",\"1;0;#FFFFFF\",\"0;0;#000000\"},\"mc\":\"000000000000000000000000000000000000000000000000000\"}",
	"{\"dim\":\"3x3\",\"cc\":{\"0;0;#FFFFFF\",\"0;0;#000000\",\"0;0;#000000\",\"1;0;#FFFFFF\",\"1;0;#000000\",\"0;0;#000000\",\"0;0;#000000\",\"0;0;#000000\",\"0;0;#000000\"},\"mc\":\"0000000001000100000001000000000000000000\"}",
	"{\"dim\":\"3x3\",\"cc\":{\"0;0;#FFFFFF\",\"0;0;#000000\",\"0;0;#000000\",\"0;0;#FFFFFF\",\"0;0;#000000\",\"0;0;#000000\",\"1;0;#FFFFFF\",\"1;0;#000000\",\"0;0;#000000\"},\"mc\":\"0000000000100010000000100000000000000000\"}",
	"{\"dim\":\"3x4\",\"cc\":{\"0;0;#FFFFFF\",\"0;0;#000000\",\"1;0;#000000\",\"1;0;#FFFFFF\",\"0;0;#FFFFFF\",\"0;0;#000000\",\"0;0;#000000\",\"0;0;#000000\",\"1;0;#FFFFFF\",\"1;0;#000000\",\"0;0;#000000\",\"0;0;#000000\"},\"mc\":\"000000000000000000000000000000000010000000000001000\"}",
	"{\"dim\":\"3x4\",\"cc\":{\"0;0;#FFFFFF\",\"0;0;#000000\",\"0;0;#000000\",\"1;0;#FFFFFF\",\"1;0;#FFFFFF\",\"1;0;#000000\",\"0;0;#000000\",\"0;0;#000000\",\"0;0;#000000\",\"1;0;#FFFFFF\",\"0;0;#000000\",\"0;0;#000000\"},\"mc\":\"000000000000000000000000000000000010000000000001000\"}",
	"{\"dim\":\"4x4\",\"cc\":{\"1;0;#FFFFFF\",\"1;0;#000000\",\"0;0;#000000\",\"0;0;#000000\",\"0;0;#000000\",\"0;0;#000000\",\"0;0;#000000\",\"0;0;#000000\",\"0;0;#000000\",\"1;0;#FFFFFF\",\"0;0;#000000\",\"0;0;#000000\",\"0;0;#000000\",\"0;0;#000000\",\"0;0;#000000\",\"0;0;#000000\"},\"mc\":\"10000000000000000000000000001100000000000000000000000000000000000\"}",
	"{\"dim\":\"4x4\",\"cc\":{\"0;0;#FFFFFF\",\"0;0;#FFFFFF\",\"0;0;#FFFFFF\",\"0;0;#000000\",\"0;0;#FFFFFF\",\"0;0;#FFFFFF\",\"1;1077084160;#FFFFFF\",\"0;0;#000000\",\"0;0;#FFFFFF\",\"0;0;#FFFFFF\",\"1;0;#000000\",\"0;0;#000000\",\"0;0;#FFFFFF\",\"0;0;#FFFFFF\",\"1;0;#FFFFFF\",\"0;0;#000000\"},\"mc\":\"00000000010000010000000000000001000000000000000000000000000000000\"}",
	"{\"dim\":\"3x4\",\"cc\":{\"1;1082388480;#FFFFFF\",\"0;0;#0000FF\",\"1;0;#0000FF\",\"0;0;#0000FF\",\"1;0;#000000\",\"0;0;#0000FF\",\"1;0;#FFFFFF\",\"0;0;#0000FF\",\"0;0;#0000FF\",\"0;0;#0000FF\",\"0;0;#0000FF\",\"0;0;#0000FF\"},\"mc\":\"000000000100000000000000000000000000000000000000000\"}",
	"{\"dim\":\"3x4\",\"cc\":{\"0;0;#FFFFFF\",\"0;0;#000000\",\"0;0;#0000FF\",\"1;0;#FFFFFF\",\"0;0;#FFFFFF\",\"1;0;#000000\",\"1;0;#0000FF\",\"0;0;#0000FF\",\"1;0;#FFFFFF\",\"0;0;#0000FF\",\"0;0;#0000FF\",\"0;0;#0000FF\"},\"mc\":\"000000000000000000000000000000000000000000000001000\"}",
	"{\"dim\":\"4x4\",\"cc\":{\"0;0;#FFFFFF\",\"0;0;#FFFFFF\",\"0;0;#0000FF\",\"0;0;#0000FF\",\"0;0;#FFFFFF\",\"1;0;#FFFFFF\",\"0;0;#0000FF\",\"0;0;#0000FF\",\"0;0;#FFFFFF\",\"1;0;#000000\",\"0;0;#0000FF\",\"1;0;#0000FF\",\"0;0;#FFFFFF\",\"0;0;#0000FF\",\"0;0;#0000FF\",\"1;0;#FFFFFF\"},\"mc\":\"00000010000000000000000000000000000000000000000000000000000000000\"}"
};

int width = 3;
int height = 3;

Witness<3, 3> w33;
Witness<3, 4> w34;
Witness<4, 4> w44;

InteractiveWitnessState<3, 3> iws33;
InteractiveWitnessState<3, 4> iws34;
InteractiveWitnessState<4, 4> iws44;

int whichPuzzle = 0;
bool redrawBackground = true;

int main(int argc, char* argv[])
{
	InstallHandlers();
	RunHOGGUI(argc, argv, 400, 400);
	return 0;
}


/**
 * Allows you to install any keyboard handlers needed for program interaction.
 */
void InstallHandlers()
{
	InstallKeyboardHandler(MyDisplayHandler, "Reset", "Reset puzzles.", kNoModifier, 'r');
	InstallKeyboardHandler(MyDisplayHandler, "Save", "Save puzzle", kNoModifier, 's');

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
		ReinitViewports(windowID, {-1.0f, -1.0f, 1.0f, 1.0f}, kScaleToSquare);
		submitTextToBuffer("");
	}
	
}

void MyFrameHandler(unsigned long windowID, unsigned int viewport, void *)
{
	Graphics::Display &display = getCurrentContext()->display;
	if (whichPuzzle >= levels.size())
	{
		if (redrawBackground)
		{
			display.StartBackground();
			display.FillRect({-1, -1, 1, 1}, Colors::white);
			display.EndBackground();
			redrawBackground = false;
		}
		display.DrawText("Done!", {0.0f, 0.0f}, Colors::black, 0.2f, Graphics::textAlignCenter);
	}
	else if (width == 3 && height == 3)
	{
		if (redrawBackground)
		{
			display.StartBackground();
			w33.LoadFromHashString(levels[whichPuzzle]);
			w33.Draw(display);
			display.EndBackground();
			iws33.Reset();
			redrawBackground = false;
		}
		
		iws33.IncrementTime();
		w33.Draw(display, iws33);
	}
	else if (width == 3 && height == 4)
	{
		if (redrawBackground)
		{
			display.StartBackground();
			w34.LoadFromHashString(levels[whichPuzzle]);
			w34.Draw(display);
			display.EndBackground();
			iws34.Reset();
			redrawBackground = false;
		}
		
		iws34.IncrementTime();
		w34.Draw(display, iws34);
	}
	else if (width == 4 && height == 4)
	{
		if (redrawBackground)
		{
			display.StartBackground();
			w44.LoadFromHashString(levels[whichPuzzle]);
			w44.Draw(display);
			display.EndBackground();
			iws44.Reset();
			redrawBackground = false;
		}
		
		iws44.IncrementTime();
		w44.Draw(display, iws44);
	}
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
			iws33.Reset();
			iws34.Reset();
			iws44.Reset();
		}
			break;
		case 's':
		{
			
			std::string fname = "/Users/nathanst/Desktop/SVG/witness_";
			int count = 0;
			while (fileExists((fname+std::to_string(count)+".svg").c_str()))
			{
				count++;
			}
			printf("Save to '%s'\n", (fname+std::to_string(count)+".svg").c_str());
			MakeSVG(GetContext(windowID)->display, (fname+std::to_string(count)+".svg").c_str(), 400, 400, 0);
			
		}
		default:
			break;
	}
}

#include "SVGUtil.h"
std::string GetPuzzleSVG()
{
	Graphics::Display &d = getCurrentContext()->display;
	std::string res = MakeSVG(d, 200, 200, 0);
	return res;
}

bool MyClickHandler(unsigned long , int viewport, int windowX, int windowY, point3d p, tButtonType button, tMouseEventType e)
{
	if (e == kMouseDrag) // ignore movement with mouse button down
		return true;
	if (whichPuzzle >= levels.size())
		return true;
	
	if (e == kMouseUp)
	{
		if (width == 3 && height == 3)
		{
			if (w33.Click(p, iws33))
			{
				if (w33.GoalTest(iws33.ws))
				{
					std::string tmp = getTextBuffer();
					submitTextToBuffer((GetPuzzleSVG()+"<br/>"+tmp).c_str());
					redrawBackground = true;
					whichPuzzle++;
					if (whichPuzzle < levels.size())
						w33.GetDimensionsFromHashString(levels[whichPuzzle], width, height);
				}
				else
					iws33.Reset();
			}
		}
		else if (width == 3 && height == 4)
		{
			if (w34.Click(p, iws34))
			{
				if (w34.GoalTest(iws34.ws))
				{
					std::string tmp = getTextBuffer();
					submitTextToBuffer((GetPuzzleSVG()+"<br/>"+tmp).c_str());
					redrawBackground = true;
					whichPuzzle++;
					if (whichPuzzle < levels.size())
						w34.GetDimensionsFromHashString(levels[whichPuzzle], width, height);
				}
				else
					iws34.Reset();
			}
		}
		else if (width == 4 && height == 4)
		{
			if (w44.Click(p, iws44))
			{
				if (w44.GoalTest(iws44.ws))
				{
					std::string tmp = getTextBuffer();
					submitTextToBuffer((GetPuzzleSVG()+"<br/>"+tmp).c_str());
					redrawBackground = true;
					whichPuzzle++;
					if (whichPuzzle < levels.size())
						w44.GetDimensionsFromHashString(levels[whichPuzzle], width, height);
				}
				else
					iws44.Reset();
			}
		}

	}
	if (e == kMouseMove)
	{
//		printf("Move\n");
		if (width == 3 && height == 3)
			w33.Move(p, iws33);
		else if (width == 3 && height == 4)
			w34.Move(p, iws34);
		else if (width == 4 && height == 4)
			w44.Move(p, iws44);
		
//		if (whichPuzzle < 3)
//			firstSet[whichPuzzle][viewport].Move(p, iws3);
//		else if (whichPuzzle < 6)
//			secondSet[whichPuzzle-3][viewport].Move(p, iws4);
	}
	
	// Don't need any other mouse support
	return true;
}


