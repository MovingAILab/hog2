/*
 *  $Id: main.cpp
 *  hog2
 *
 *  Created by Nathan Sturtevant on 11/02/06.
 *  Modified by Nathan Sturtevant on 06/22/21.
 *
 * This file is part of HOG2. See https://github.com/nathansttt/hog2 for licensing information.
 *
 */

#define GL_SILENCE_DEPRECATION

int hog_main(int argc, char **argv);
int main(int argc, char **argv)
{
	return hog_main(argc, argv);
}


#include "STUB_HOG.h"
#include "Trackball.h"
#include "Common.h"
//#include "TextBox.h"
#include <stdlib.h>
#include <time.h>
#include <math.h>
#include <string.h>
#include <cassert>
#include "MonoFont.h"

using namespace std;

// camera handling
//recCamera globalCamera; // has full screen size; see resizeGL()
//recCamera camera;
//	int numPorts, currPort;
bool moveAllPortsTogether;

pRecContext pContextInfo;
//bool pointpath = false;
//int ppMouseClicks = 0;
double fps = 30.0;
// window width and height
int width=100, height=100;
Graphics::rect screenRect;
Graphics::point WindowToHOG(const Graphics::point &p);
MonoFont font;

pRecContext GetContext(unsigned long windowID)
{
	return pContextInfo;
}

pRecContext getCurrentContext()
{
	return pContextInfo;
}

Graphics::point convertToGlobalHogCoordinate(int x, int y)
{
	Graphics::point p;
	p.x = 2.0*x/width-1.0;
	p.y = 2.0*y/height-1.0;
	p.z = 0;
	return p;
}


void RunHOGGUI(int argc, char** argv, int windowDimension)
{
	RunHOGGUI(argc, argv, windowDimension, windowDimension);
}

//void initCameras()
//{
//	moveAllPortsTogether = true;
//	//for (int x = 0; x < MAXPORTS; x++)
//	{
//		resetCamera(&camera);
//		for (int y = 0; y < 4; y++)
//		{
//			camera.rotations.worldRotation[y] = 0;
//			camera.rotations.cameraRotation[y] = 0.0001;
//		}
////		pContextInfo->camera[x].rotations.cameraRotation[0] = 180;
////		pContextInfo->camera[x].rotations.cameraRotation[2] = 1;
//		camera.thirdPerson = true;
//	}
//}

void RunHOGGUI(int argc, char* argv[], int xDimension, int yDimension)
{
	int mousePressCount = 0;
	srandom(unsigned(time(0)));
	ProcessCommandLineArgs(argc, argv);
	pContextInfo = new recContext;
	unsigned int xd = xDimension, yd = yDimension;
	initialConditions(pContextInfo);
	//	initCameras();
	buildGL(xDimension, yDimension);
	pContextInfo->display.windowHeight = xDimension;
	pContextInfo->display.windowWidth = yDimension;
	
	HandleWindowEvent(pContextInfo, kWindowCreated);
	//	createMenus();
	resizeGL(pContextInfo, xDimension, yDimension);
	
	while (true)
	{
		drawGL(pContextInfo);
	}
	delete pContextInfo;
}

void createMenus()
{
}

void processMenuEvents(int option)
{
}



/**
 * Called when a key is pressed, and no other keys are held down.
 */
void keyPressed(unsigned char key, int, int)
{
	//x+=y;
	bool shift = false;//(glutGetModifiers() == GLUT_ACTIVE_SHIFT);
	bool alt = false;//(glutGetModifiers() == GLUT_ACTIVE_ALT);
	bool cntrl = false;//(glutGetModifiers() == GLUT_ACTIVE_CTRL);
	DoKeyboardCommand(pContextInfo, key, shift, cntrl, alt);
}



void mouseMovedNoButton(int x, int y)
{
}

/**
 * Called when the mouse is moved with a button pressed down.
 */
void mouseMovedButton(int x, int y)
{
}


/**
 * Called when a mouse button is pressed.
 */
void mousePressedButton(int button, int state, int x, int y)
{
}


// move camera in x/y plane
static void mousePan (int x, int y, pRecContext pContextInfo)
{
}


// move camera in z axis
static void mouseDolly (int x, int y, pRecContext pContextInfo)
{
}

/**
 * Renders the scene.  Used by GLUT for it's display function.
 * Wraps the drawGL() function.
 */
void renderScene(void)
{
	assert(false);
}


/**
 * Called when the window is resized.  Specific format for GLUT.
 */
void resizeWindow(int x, int y)
{
	resizeGL(pContextInfo, x, y);
}


/**
 * Handles resizing of GL need context update and if the window dimensions change,
 * a window dimension update, reseting of viewport and an update of the projection matrix
 */
void resizeGL(pRecContext pContextInfo, int width, int height)
{
	if (!pContextInfo)
		return;
	
	pContextInfo->display.windowWidth = width;
	pContextInfo->display.windowHeight = height;
	//	globalCamera.viewOriginX = 0;//viewRect.origin.x;
	//	globalCamera.viewOriginY = 0;//viewRect.origin.y;
	//
	//	globalCamera.viewWidth = width;//(GLint)viewRect.size.width;
	//	globalCamera.viewHeight = height;//(GLint)viewRect.size.height;
	// printf("Window size: {%d, %d}\n", width, height);
	//	for (int x = 0; x < pContextInfo->numPorts; x++)
	//	{
	setPortCamera(pContextInfo, 0);
	//	}
	//		glViewport(0, 0, pContextInfo->camera.viewWidth, pContextInfo->camera.viewHeight);
}



///**
// * Draws a CString in OpenGL
// */
//void drawCStringGL (char * cstrOut, GLuint fontList)
//{
//	GLint i = 0;
//	if (!cstrOut)
//		return;
//	while (cstrOut [i])
//		glCallList (fontList + cstrOut[i++]);
//}

//TextBox *myTextBox = 0;

bool bufferVisibility = true;

void setTextBufferVisibility(bool visible)
{
	bufferVisibility = visible;
	if (bufferVisibility)
	{
//		if (myTextBox == 0 && pContextInfo->message != 0)
//		{
//			Graphics::point a(-.95, .95, -.95), b(.95, -.95, .95);
//			rgbColor rc(1, 1, 1);
//			myTextBox = new TextBox(pContextInfo->message, 120, a, b, 1000, true);
//			myTextBox->setColor(rc);
//		}
	}
	else
	{
//		delete myTextBox;
//		myTextBox = 0;
	}
}
bool getTextBufferVisibility()
{ return bufferVisibility; }

void appendTextToBuffer(const char *tempStr)
{
	int ind = int(strlen(pContextInfo->message));
	pContextInfo->message[ind] = ' ';
	snprintf(&pContextInfo->message[ind+1], 256-(ind+2), "%s", tempStr);
	
//	delete myTextBox;
//	myTextBox = 0;
	Graphics::point a(-.95, .95, -.95), b(.95, -.95, .95);
	rgbColor rc(1, 1, 1);
	if (bufferVisibility)
	{
//		myTextBox = new TextBox(pContextInfo->message, 120, a, b, 1000, true);
//		myTextBox->setColor(rc);
	}
}

void submitTextToBuffer(const char *val)
{
	strncpy(pContextInfo->message, val, 255);
//	delete myTextBox;
//	myTextBox = 0;
	Graphics::point a(-.95, .95, -.95), b(.95, -.95, .95);
	rgbColor rc(1, 1, 1);
	if (bufferVisibility)
	{
//		myTextBox = new TextBox(pContextInfo->message, 120, a, b, 1000, true);
//		myTextBox->setColor(rc);
	}
}

Graphics::point ViewportToScreen(Graphics::point where, int viewport)
{
	auto p = ViewportToGlobalHOG(pContextInfo, where, viewport);
	p.x *= screenRect.right;
	p.y *= screenRect.bottom;
	return p;
}

Graphics::rect ViewportToScreen(const Graphics::rect &loc, int viewport)
{
	auto r = ViewportToGlobalHOG(pContextInfo, loc, viewport);
	r.left *= screenRect.right;
	r.right *= screenRect.right;
	r.top *= screenRect.bottom;
	r.bottom *= screenRect.bottom;
	return r;
}

float ViewportToScreenX(float x, int v)
{
	float val = ViewportToGlobalHOGX(pContextInfo, x, v);
	return val*screenRect.right;
}

Graphics::point WindowToHOG(const Graphics::point &p)
{
	// Convert from 0,0 -> width/height range to HOG range
	// Just map to screenRect
	float xperc = p.x/pContextInfo->display.windowWidth;
	float yperc = p.y/pContextInfo->display.windowHeight;
	// return Graphics::point(screenRect.left*(1-xperc)+screenRect.right*xperc,
	//					   screenRect.top*(1-yperc)+screenRect.bottom*yperc);
	return Graphics::point(-1*(1-xperc)+1*xperc,
						   -1*(1-yperc)+1*yperc);
}


/**
 * Main OpenGL drawing function.
 */
void drawGL (pRecContext pContextInfo)
{
	if (!pContextInfo)
		return;
	pContextInfo->display.StartFrame();
	//window.clear();
	// clear our drawable
	for (int x = 0; x < pContextInfo->display.numViewports; x++)
	{
		//updateProjection(pContextInfo, x);
		setViewport(pContextInfo, x);
		//if (pContextInfo->drawing)
		{
			HandleFrame(pContextInfo, x);
		}
	}
//	if (myTextBox)
//	{
//		myTextBox->stepTime(0.1);
//		//		myTextBox->draw();
//	}
	pContextInfo->display.EndFrame();
}



/**
 * Initializes OpenGL.
 */
void buildGL(int xDim, int yDim)
{}

//void resetCamera()
//{} // no visualization, no camera
//
//// sets the camera data to initial conditions
//void resetCamera(recCamera * pCamera)
//{} // no visualization, no camera
//

//Graphics::point cameraLookingAt(int port)
//{
//	pRecContext pContextInfo = getCurrentContext();
//	if (port == -1)
//		port = pContextInfo->display.numViewports;
//	return /*pContextInfo->camera[port].viewPos-*/camera.viewDir;
//}
//

void setPortCamera(pRecContext pContextInfo, int currPort)
{} // no visualization, no camera

void setViewport(pRecContext pContextInfo, int currPort)
{
	pContextInfo->display.SetViewport(currPort);
	currPort = 0;
}

point3d GetOGLPos(pRecContext pContextInfo, int x, int y)
{
	return point3d(x, y, 0);
}

void SetZoom(int windowID, float amount)
{
	pRecContext pContextInfo = GetContext(windowID);
	//	camera.aperture = amount;
	//	updateProjection(pContextInfo, pContextInfo->display.currViewport);  // update projection matrix
}


