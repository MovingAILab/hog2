//
//  CocoaHog.cpp
//  hog2 mac native demos
//
//  Created by Nathan Sturtevant on 8/2/17.
//  Copyright © 2017 NS Software. All rights reserved.
//

#include <stdio.h>
#include "CocoaHOG.h"
#include <assert.h>

recContext context;
pRecContext pContextInfo = &context;

void updateProjection(pRecContext pContextInfo, int viewPort)
{
	
}

pRecContext GetContext(unsigned long windowID)
{
	return pContextInfo;
}

pRecContext getCurrentContext()
{
	return pContextInfo;
}

void updateModelView(pRecContext pContextInfo, int currPort)
{
	
}

//GLfloat gTrackBallRotation [4] = {0.0f, 0.0f, 0.0f, 0.0f};
//pRecContext gTrackingContextInfo = NULL;

void renderScene()
{
	
}

void RunHOGGUI(int argc, char* argv[], int windowDimension)
{
	RunHOGGUI(argc, argv, windowDimension, windowDimension);
}

void RunHOGGUI(int argc, char* argv[], int xDimension, int yDimension)
{
	ProcessCommandLineArgs(argc, argv);
	context.display.windowWidth = xDimension;
	context.display.windowHeight = yDimension;
//	initialConditions(pContextInfo);
//	HandleWindowEvent(pContextInfo, kWindowCreated);
}

char *textBuffer = 0;
bool bufferVisibility = true;

void setTextBufferVisibility(bool visible)
{
	bufferVisibility = visible;
}

bool getTextBufferVisibility()
{
	return bufferVisibility;
}

void submitTextToBuffer(const char *val)
{
	if (val)
	{
		delete [] textBuffer;
		int len = strlen(val);
		textBuffer = new char[len+1];
		strncpy(textBuffer, val, len);
		textBuffer[len] = 0;
	}
}

void appendTextToBuffer(const char *next)
{
	if (textBuffer == 0)
	{
		submitTextToBuffer(next);
		return;
	}
	int oldLen = strlen(textBuffer);
	int newLen = strlen(next)+oldLen;
	char *newStr = new char[newLen+1];
	strcpy(newStr, textBuffer);
	strcpy(&newStr[oldLen], next);
	delete [] textBuffer;
	textBuffer = newStr;
}

const char *getTextBuffer()
{
	return textBuffer;
}
