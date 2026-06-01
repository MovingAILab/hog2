/*
 *  Driver.cpp
 *  hog2
 *
 *  Created by Nathan Sturtevant on 5/31/05.
 *  Modified by Nathan Sturtevant on 07/15/21.
 *
 * This file is part of HOG2. See https://github.com/nathansttt/hog2 for licensing information.
 *
 */

#include <cstring>
#include "Common.h"
#include "Driver.h"
#include "SVGUtil.h"

int main(int argc, char* argv[])
{
	setvbuf(stdout, NULL, _IONBF, 0);
	InstallHandlers();
	RunHOGGUI(argc, argv, 1000, 1000);
	return 0;
}

/**
 * Allows you to install any keyboard handlers needed for program interaction.
 */
void InstallHandlers()
{
	InstallKeyboardHandler(MyDisplayHandler, "Save", "Save screen shot", kAnyModifier, 's');
	InstallCommandLineHandler(MyCLHandler, "-save", "-save <file>", "Save shot and quit");

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
		SetNumPorts(windowID, 1);
	}
}


void MyFrameHandler(unsigned long windowID, unsigned int viewport, void *)
{
	static int frame = 0;
	frame++;
	
	Graphics::Display &d = GetContext(windowID)->display;
	d.FillSquare({0, 0}, 1, Colors::white);
	// 5x5 grid cell
	const float width = 2.0/5.0f;
	const float left = -1.0f;
	const float top = -1.0f;

	for (int t = 0; t < 25; t++)
	{
		int x = t/5;
		int y = t%5;
		Graphics::point center(x*width+left+0.5f*width, y*width+top+0.5f*width);
		switch (t)
		{
		case 0:
			d.FillCircle(center, 0.5f*width, Colors::red);
			d.DrawText("Red circle (fill)", center, Colors::white, 0.1f*width, Graphics::textAlignCenter, Graphics::textBaselineMiddle);
			break;
		case 1:
			d.FrameCircle(center, 0.5f*width, Colors::blue, 0.05f*width);
			d.DrawText("Blue circle (border)", center, Colors::black, 0.1f*width, Graphics::textAlignCenter, Graphics::textBaselineMiddle);
			break;
		case 2:
			d.FillSquare(center, 0.5f*width, Colors::purple);
			break;
		case 3:
			d.FrameSquare(center, 0.5f*width, Colors::green, 0.05f*width);
			break;
		case 4:
			d.FillNGon(center, 0.5f*width, 5, -(frame%360), Colors::purple);
			break;
		case 5:
			d.FrameNGon(center, 0.5f*width, 0.05f*width, 5, frame%360, Colors::pink);
			break;
		}
	}
}

int MyCLHandler(char *argument[], int maxNumArgs)
{
	if (strcmp(argument[0], "-save") == 0)
	{
		// save into argument[1];
		
		exit(0);
	}
	return 0;
}

void MyDisplayHandler(unsigned long windowID, tKeyboardModifier mod, char key)
{
	switch (key)
	{
		case 's':
			
			//BaselineTest();
			break;
		case 'r':
			//recording = !recording;
			break;
	}
}

bool MyClickHandler(unsigned long , int, int, point3d , tButtonType , tMouseEventType )
{
	return false;
}

#include <sys/stat.h>
bool fileExists(const char *name)
{
	struct stat buffer;
	return (stat(name, &buffer) == 0);
}

