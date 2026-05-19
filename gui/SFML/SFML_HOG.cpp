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

#include <SFML/Window.hpp>
#include <SFML/OpenGL.hpp>
#define GL_SILENCE_DEPRECATION

extern "C" {
	void glutStrokeCharacter(void *font, int character){}
	int glutStrokeWidth(void *font, int character) {return 0;}
}

int hog_main(int argc, char **argv);
int main(int argc, char **argv)
{
	return hog_main(argc, argv);
}


#include "SFML_HOG.h"
#include "Trackball.h"
#include "Common.h"
#include "TextBox.h"
#include <stdlib.h>
#include <time.h>
#include <math.h>
#include <string.h>
#include <cassert>
#include "MonoFont.h"

using namespace std;

// camera handling
recCamera globalCamera; // has full screen size; see resizeGL()
recCamera camera;
//	int numPorts, currPort;
bool moveAllPortsTogether;


pRecContext pContextInfo;
GLint gDollyPanStartPoint[2] = {0, 0};
GLfloat gTrackBallRotation [4] = {0.0f, 0.0f, 0.0f, 0.0f};
GLboolean gDolly = GL_FALSE;
GLboolean gPan = GL_FALSE;
GLboolean gTrackball = GL_FALSE;
pRecContext gTrackingContextInfo = NULL;
int gCurrButton = -1;
//bool pointpath = false;
//int ppMouseClicks = 0;
pRecContext backup;
double fps = 30.0;
// window width and height
int width=100, height=100;
Graphics::rect screenRect;
Graphics::point WindowToHOG(const Graphics::point &p);
MonoFont font;
std::vector<Graphics::Display::lineInfo> textLines;

void Line(Graphics::point p1, Graphics::point p2, float width, int viewport);

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

void initCameras()
{
	moveAllPortsTogether = true;
	//for (int x = 0; x < MAXPORTS; x++)
	{
		resetCamera(&camera);
		for (int y = 0; y < 4; y++)
		{
			camera.rotations.worldRotation[y] = 0;
			camera.rotations.cameraRotation[y] = 0.0001;
		}
//		pContextInfo->camera[x].rotations.cameraRotation[0] = 180;
//		pContextInfo->camera[x].rotations.cameraRotation[2] = 1;
		camera.thirdPerson = true;
	}
	gTrackBallRotation [0] = gTrackBallRotation [1] = gTrackBallRotation [2] = gTrackBallRotation [3] = 0.0f;
}

void RunHOGGUI(int argc, char* argv[], int xDimension, int yDimension)
{
	int mousePressCount = 0;
	srandom(unsigned(time(0)));
	ProcessCommandLineArgs(argc, argv);
	pContextInfo = new recContext;
	unsigned int xd = xDimension, yd = yDimension;
	sf::Window window(sf::VideoMode({xd, yd}), "My window");
	window.setFramerateLimit(30); // call it once, after creating the window
	initialConditions(pContextInfo);
	initCameras();
	buildGL(xDimension, yDimension);
	pContextInfo->display.windowHeight = xDimension;
	pContextInfo->display.windowWidth = yDimension;
	
	HandleWindowEvent(pContextInfo, kWindowCreated);
	//	createMenus();
	resizeGL(pContextInfo, xDimension, yDimension);
	
	while (window.isOpen())
    {
		while (const std::optional event = window.pollEvent())
		{
			if (event->is<sf::Event::Closed>())
			{
				window.close();
				break;
			}
			else if (event->is<sf::Event::Resized>())
			{
				sf::Vector2u size = window.getSize();
				//				float ratio = static_cast<float>(size.x) / size.y;
				//				size.x = size.y * mAspectRatio; // Maintain aspect ratio
				//				mWindow->setSize(size);
				resizeWindow(size.x, size.y);
				//printf("Window now %dx%d\n", event.size.width, event.size.height);
				xDimension = size.x;
				yDimension = size.y;
				pContextInfo->display.windowHeight = yDimension;
				pContextInfo->display.windowWidth = xDimension;
				resizeGL(pContextInfo, xDimension, yDimension); // forces projection matrix update
				break;
			}
			else if (const auto* keyPressed = event->getIf<sf::Event::TextEntered>())
			{
				if (keyPressed->unicode < 128)
				{
					DoKeyboardCommand(pContextInfo, keyPressed->unicode, false, false, false);
				}
			}
			else if (const auto* keyPressed = event->getIf<sf::Event::KeyPressed>())
			{
				switch (keyPressed->code)
				{
					case sf::Keyboard::Key::Left: DoKeyboardCommand(pContextInfo, kLeftArrow, keyPressed->shift, keyPressed->control, keyPressed->alt); break;
					case sf::Keyboard::Key::Right: DoKeyboardCommand(pContextInfo, kRightArrow, keyPressed->shift, keyPressed->control, keyPressed->alt); break;
					case sf::Keyboard::Key::Up: DoKeyboardCommand(pContextInfo, kUpArrow, keyPressed->shift, keyPressed->control, keyPressed->alt); break;
					case sf::Keyboard::Key::Down: DoKeyboardCommand(pContextInfo, kDownArrow, keyPressed->shift, keyPressed->control, keyPressed->alt); break;
					default: break; // others handled by TextEntered
				}
			}
			else if (const auto* mouse = event->getIf<sf::Event::MouseButtonPressed>())
			{
				Graphics::point p = WindowToHOG(Graphics::point(mouse->position.x, mouse->position.y));//convertToGlobalHogCoordinate(mouse->position.x, mouse->position.y);
				//printf("Click (%d, %d) - {%f, %f, %f}\n", mouse->position.x, mouse->position.y, p.x, p.y, p.z);
				mousePressCount+=1;
				switch (mouse->button)
				{
						
					case sf::Mouse::Button::Right:
						//HandleMouseClick(pContextInfo, mouse->position.x, mouse->position.y, p, kRightButton, kMouseDown);
						//printf("Right (%d, %d) - {%f, %f, %f}\n", mouse->position.x, mouse->position.y, p.x, p.y, p.z);
						HandleMouse(pContextInfo, mouse->position.x, mouse->position.y, p, kRightButton, kMouseDown);
						break;
					case sf::Mouse::Button::Left:
						//printf("Left (%d, %d) - {%f, %f, %f}\n", mouse->position.x, mouse->position.y, p.x, p.y, p.z);
						HandleMouse(pContextInfo, mouse->position.x, mouse->position.y, p, kLeftButton, kMouseDown);
						break;
					case sf::Mouse::Button::Middle:
						HandleMouse(pContextInfo, mouse->position.x, mouse->position.y, p, kMiddleButton, kMouseDown);
						break;
					default: break;
				}
			}
			else if (const auto* mouse = event->getIf<sf::Event::MouseButtonReleased>())
			{
				Graphics::point p = WindowToHOG(Graphics::point(mouse->position.x, mouse->position.y));
				mousePressCount-=1;
				switch (mouse->button)
				{
					case sf::Mouse::Button::Right:
						HandleMouse(pContextInfo, mouse->position.x, mouse->position.y, p, kRightButton, kMouseUp);
						break;
					case sf::Mouse::Button::Left:
						HandleMouse(pContextInfo, mouse->position.x, mouse->position.y, p, kLeftButton, kMouseUp);
						break;
					case sf::Mouse::Button::Middle:
						HandleMouse(pContextInfo, mouse->position.x, mouse->position.y, p, kMiddleButton, kMouseUp);
						break;
					default: break;
				}
			}
			else if (const auto* mouse = event->getIf<sf::Event::MouseMoved>())
			{
				Graphics::point p = WindowToHOG(Graphics::point(mouse->position.x, mouse->position.y));
				tButtonType bType = kNoButton;
				if (sf::Mouse::isButtonPressed(sf::Mouse::Button::Left))
					bType = kLeftButton;
				else if (sf::Mouse::isButtonPressed(sf::Mouse::Button::Right))
					bType = kRightButton;
				else if (sf::Mouse::isButtonPressed(sf::Mouse::Button::Middle))
					bType = kMiddleButton;
				
				if (bType == kNoButton)
				{
					HandleMouse(pContextInfo, mouse->position.x, mouse->position.y, p, bType, kMouseMove);
				}
				else {
					HandleMouse(pContextInfo, mouse->position.x, mouse->position.y, p, bType, kMouseDrag);
				}
			}
			//std::cout << "new mouse x: " << mouse->position.x << std::endl;
			//std::cout << "new mouse y: " << mouse->position.y << std::endl;
			break;
		}
		drawGL (pContextInfo, window);
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
	globalCamera.viewOriginX = 0;//viewRect.origin.x;
	globalCamera.viewOriginY = 0;//viewRect.origin.y;
		
	globalCamera.viewWidth = width;//(GLint)viewRect.size.width;
	globalCamera.viewHeight = height;//(GLint)viewRect.size.height;
	// printf("Window size: {%d, %d}\n", width, height);
//	for (int x = 0; x < pContextInfo->numPorts; x++)
//	{
	setPortCamera(pContextInfo, 0);
//	}
	//		glViewport(0, 0, pContextInfo->camera.viewWidth, pContextInfo->camera.viewHeight);
	
	updateProjection(pContextInfo);  // update projection matrix
}



/**
 * Update the projection matrix based on camera and view info.
 * Should be called when viewport size, eye z position, or camera aperture changes.
 * Also call if far or near changes which is determined by shape size in this case.
 */
void updateProjection(pRecContext pContextInfo, int viewPort)
{
	GLdouble ratio, radians, wd2;
	int minVal, maxVal;
	if (viewPort == -1)
	{
		minVal = 0;
		maxVal = 0;//pContextInfo->numPorts-1;
	}
	else {
		minVal = maxVal = viewPort;
	}
	for (int x = minVal; x <= maxVal; x++)
	{
		camera.frust.near = 0.01;
		camera.frust.far = 20.0;
		
		radians = 0.0174532925 * camera.aperture / 2; // half aperture degrees to radians
		wd2 = camera.frust.near * tan(radians);
		//ratio = camera.viewWidth / (float) camera.viewHeight;
		ratio = pContextInfo->display.windowWidth / (float) pContextInfo->display.windowHeight;
		// printf("[%d/%d]\n", pContextInfo->camera[x].viewWidth, pContextInfo->camera[x].viewHeight);
		if (ratio >= 1.0) // wider than tall
		{
			camera.frust.left  = (ratio)*camera.frust.near/camera.viewPos.z;
			camera.frust.right = -(ratio)*camera.frust.near/camera.viewPos.z;
			camera.frust.top = -camera.frust.near/camera.viewPos.z;
			camera.frust.bottom = camera.frust.near/camera.viewPos.z;
			screenRect.left = -ratio;
			screenRect.right = ratio;
			screenRect.top = -1;
			screenRect.bottom = 1;
			// std::cout << "window coordinates: {" << screenRect << "}\n";
			// camera.frust.left  = -ratio * wd2;
			// pContextInfo->camera[x].frust.right = ratio * wd2;
			// pContextInfo->camera[x].frust.top = wd2;
			// pContextInfo->camera[x].frust.bottom = -wd2;
			
			// printf("=> l: %f r: %f t: %f b: %f\n",
			// 	   pContextInfo->camera[x].frust.left,
			// 	   pContextInfo->camera[x].frust.right,
			// 	   pContextInfo->camera[x].frust.top,
			// 	   pContextInfo->camera[x].frust.bottom);
			Graphics::rect t1, t2, r(-1, -1, 1, 1);
			t1 = ViewportToGlobalHOG(pContextInfo, r, x);
			// std::cout << "Port " << x << " {" << r << "} -> {" << t1 << "}\n";
		} else {
			ratio = 1/ratio;
			camera.frust.bottom  = (ratio)*camera.frust.near/camera.viewPos.z;
			camera.frust.top = -(ratio)*camera.frust.near/camera.viewPos.z;
			camera.frust.left = camera.frust.near/camera.viewPos.z;
			camera.frust.right = -camera.frust.near/camera.viewPos.z;
			screenRect.left = -1;
			screenRect.right = 1;
			screenRect.top = -ratio;
			screenRect.bottom = ratio;

			// std::cout << "-->window coordinates: {" << screenRect << "}\n";
			// pContextInfo->camera[x].frust.left  = -ratio * wd2;
			// pContextInfo->camera[x].frust.right = ratio * wd2;
			// pContextInfo->camera[x].frust.top = wd2;
			// pContextInfo->camera[x].frust.bottom = -wd2;
			
			// printf("--> l: %f r: %f t: %f b: %f\n",
			// 	   pContextInfo->camera[x].frust.left,
			// 	   pContextInfo->camera[x].frust.right,
			// 	   pContextInfo->camera[x].frust.top,
			// 	   pContextInfo->camera[x].frust.bottom);
			Graphics::rect t1, t2, r(-1, -1, 1, 1);
			t1 = ViewportToGlobalHOG(pContextInfo, r, x);
			// std::cout << "Port " << x << " {" << r << "} -> {" << t1 << "}\n";
		}
	}
}


// to perform cross product between 2 vectors in myGluLookAt 
void CrossProd(float x1, float y1, float z1, float x2, float y2, float z2, float res[3]) 
{ 
	res[0] = y1*z2 - y2*z1; 
	res[1] = x2*z1 - x1*z2; 
	res[2] = x1*y2 - x2*y1; 
} 

// my own implementation 
void MyGluLookAt(float eyeX, float eyeY, float eyeZ, float lookAtX, float lookAtY, float lookAtZ, float upX, float upY, float upZ) 
{ 
	// i am not using here proper implementation for vectors. 
	// if you want, you can replace the arrays with your own 
	// vector types 
	float f[3]; 
	
	// calculating the viewing vector 
	f[0] = lookAtX - eyeX; 
	f[1] = lookAtY - eyeY; 
	f[2] = lookAtZ - eyeZ; 
	
	float fMag, upMag; 
	fMag = sqrt(f[0]*f[0] + f[1]*f[1] + f[2]*f[2]); 
	upMag = sqrt(upX*upX + upY*upY + upZ*upZ); 
	
	// normalizing the viewing vector 
	if( fMag != 0) 
	{ 
		f[0] = f[0]/fMag; 
		f[1] = f[1]/fMag; 
		f[2] = f[2]/fMag; 
	} 
	
	// normalising the up vector. no need for this here if you have your 
	// up vector already normalised, which is mostly the case. 
	if( upMag != 0 ) 
	{ 
		upX = upX/upMag; 
		upY = upY/upMag; 
		upZ = upZ/upMag; 
	} 
	
	float s[3], u[3]; 
	
	CrossProd(f[0], f[1], f[2], upX, upY, upZ, s); 
	CrossProd(s[0], s[1], s[2], f[0], f[1], f[2], u); 
	
	float M[]= 
	{ 
		s[0], u[0], -f[0], 0, 
		s[1], u[1], -f[1], 0, 
		s[2], u[2], -f[2], 0, 
		0, 0, 0, 1 
	}; 
	
	glMultMatrixf(M); 
	glTranslatef(-eyeX, -eyeY, -eyeZ); 
}


/**
 * Updates the viewpoint of the model.
 */
void updateModelView(pRecContext pContextInfo, int currPort)
{
	// move view
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	
	// mouse transforms object
	if (camera.thirdPerson)
	{
		MyGluLookAt(camera.viewPos.x,
					camera.viewPos.y,
					camera.viewPos.z,
					camera.viewPos.x + camera.viewDir.x,
					camera.viewPos.y + camera.viewDir.y,
					camera.viewPos.z + camera.viewDir.z,
					camera.viewUp.x, camera.viewUp.y ,camera.viewUp.z);

		if ((gTrackingContextInfo == pContextInfo) && gTrackBallRotation[0] != 0.0f) // if we have trackball rotation to map (this IS the test I want as it can be explicitly 0.0f)
		{
			//if (pContextInfo->currPort == currPort || pContextInfo->moveAllPortsTogether)
			glRotatef (gTrackBallRotation[0], gTrackBallRotation[1], gTrackBallRotation[2], gTrackBallRotation[3]);
		}
		else {
		}
		
		// accumlated world rotation via trackball
		glRotatef (camera.rotations.worldRotation[0],
				   camera.rotations.worldRotation[1],
				   camera.rotations.worldRotation[2],
				   camera.rotations.worldRotation[3]);
	}
	// if mouse moves whole world:
	else {
		glTranslatef(camera.viewPos.x,
					 camera.viewPos.y,
					 camera.viewPos.z);
	}
	
}


/**
 * Draws a CString in OpenGL
 */
void drawCStringGL (char * cstrOut, GLuint fontList)
{
	GLint i = 0;
	if (!cstrOut)
		return;
	while (cstrOut [i])
		glCallList (fontList + cstrOut[i++]);
}

TextBox *myTextBox = 0;

bool bufferVisibility = true;

void setTextBufferVisibility(bool visible)
{
  bufferVisibility = visible;
  if (bufferVisibility)
    {
      if (myTextBox == 0 && pContextInfo->message != 0)
	{
	  Graphics::point a(-.95, .95, -.95), b(.95, -.95, .95);
	  rgbColor rc(1, 1, 1);
	  myTextBox = new TextBox(pContextInfo->message, 120, a, b, 1000, true);
	  myTextBox->setColor(rc);
	}
    }
  else 
  {
    delete myTextBox;
    myTextBox = 0;
  }
}
bool getTextBufferVisibility()
{ return bufferVisibility; }

void appendTextToBuffer(const char *tempStr)
{
	int ind = int(strlen(pContextInfo->message));
	pContextInfo->message[ind] = ' ';
	snprintf(&pContextInfo->message[ind+1], 256-(ind+2), "%s", tempStr);

	delete myTextBox;
	myTextBox = 0;
	Graphics::point a(-.95, .95, -.95), b(.95, -.95, .95);
	rgbColor rc(1, 1, 1);
	if (bufferVisibility)
	  {
	    myTextBox = new TextBox(pContextInfo->message, 120, a, b, 1000, true);
	    myTextBox->setColor(rc);
	  }
}

void submitTextToBuffer(const char *val)
{
	strncpy(pContextInfo->message, val, 255);
	delete myTextBox;
	myTextBox = 0;
	Graphics::point a(-.95, .95, -.95), b(.95, -.95, .95);
	rgbColor rc(1, 1, 1);
	if (bufferVisibility)
	  {
	    myTextBox = new TextBox(pContextInfo->message, 120, a, b, 1000, true);
	    myTextBox->setColor(rc);
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

void Bezier(const Graphics::point &p1, const Graphics::point &p2, const Graphics::point &p3, const Graphics::point &p4)
{
	for (float f = 0.2f; fless(f, 1); f += 0.2f)
	{
		Graphics::point p = BezierHelper(p1, p2, p3, p4, f);
		glVertex2f(p.x, p.y);
	}
}

void BezierLine(const Graphics::point &p1a, const Graphics::point &p2a, const Graphics::point &p3a, const Graphics::point &p4a,
				const Graphics::point &p1b, const Graphics::point &p2b, const Graphics::point &p3b, const Graphics::point &p4b)
{
	for (float f = 0.0f; flesseq(f, 1); f += 0.2f)
	{
		Graphics::point p = BezierHelper(p1a, p2a, p3a, p4a, f);
		glVertex2f(p.x, p.y);
		p = BezierHelper(p1b, p2b, p3b, p4b, f);
		glVertex2f(p.x, p.y);
	}
}


void DoDrawCommands(Graphics::Display &display, int port, sf::Window &window, std::vector<Graphics::Display::data> &commands)
{
	for (auto &i: commands)
	{
		//auto &i = display.backgroundDrawCommands[x];
		if (i.viewport != port)
			continue;
		switch (i.what)
		{
			case Graphics::Display::kFillRoundedRectangle:
				{
					glColor3f(i.rr.c.r, i.rr.c.g, i.rr.c.b);
					Graphics::rect outer = ViewportToScreen(i.rr.r.r, i.viewport);
					Graphics::rect inner = ViewportToScreen(i.rr.r.r.inset(i.rr.r.rad), i.viewport);
					glBegin(GL_TRIANGLE_FAN);
					Bezier({outer.left, inner.top}, {outer.left, outer.top},
						   {outer.left, outer.top}, {inner.left, outer.top});
					Bezier({inner.right, outer.top}, {outer.right, outer.top},
						   {outer.right, outer.top}, {outer.right, inner.top});
					Bezier({outer.right, inner.bottom}, {outer.right, outer.bottom},
						   {outer.right, outer.bottom}, {inner.right, outer.bottom});
					Bezier({inner.left, outer.bottom}, {outer.left, outer.bottom},
						   {outer.left, outer.bottom}, {outer.left, inner.bottom});
					glEnd();
					break;
				}
			case Graphics::Display::kFrameRoundedRectangle:
				{
					glColor3f(i.rr.c.r, i.rr.c.g, i.rr.c.b);
					Graphics::rect outer1 = ViewportToScreen(i.rr.r.r, i.viewport);
					Graphics::rect inner1 = ViewportToScreen(i.rr.r.r.inset(i.rr.r.rad), i.viewport);
					Graphics::rect outer2 = ViewportToScreen(i.rr.r.r.inset(i.rr.width), i.viewport);
					Graphics::rect inner2 = ViewportToScreen(i.rr.r.r.inset(i.rr.r.rad+i.rr.width), i.viewport);

					glBegin(GL_TRIANGLE_STRIP);
					BezierLine({outer1.left, inner1.top}, {outer1.left, outer1.top},
							   {outer1.left, outer1.top}, {inner1.left, outer1.top},
							   {outer2.left, inner2.top}, {outer2.left, outer2.top},
							   {outer2.left, outer2.top}, {inner2.left, outer2.top});
					BezierLine({inner1.right, outer1.top}, {outer1.right, outer1.top},
							   {outer1.right, outer1.top}, {outer1.right, inner1.top},
							   {inner2.right, outer2.top}, {outer2.right, outer2.top},
							   {outer2.right, outer2.top}, {outer2.right, inner2.top});
					BezierLine({outer1.right, inner1.bottom}, {outer1.right, outer1.bottom},
							   {outer1.right, outer1.bottom}, {inner1.right, outer1.bottom},
							   {outer2.right, inner2.bottom}, {outer2.right, outer2.bottom},
							   {outer2.right, outer2.bottom}, {inner2.right, outer2.bottom});
					BezierLine({inner1.left, outer1.bottom}, {outer1.left, outer1.bottom},
							   {outer1.left, outer1.bottom}, {outer1.left, inner1.bottom},
							   {inner2.left, outer2.bottom}, {outer2.left, outer2.bottom},
							   {outer2.left, outer2.bottom}, {outer2.left, inner2.bottom});
					glVertex2f(outer1.left, inner1.top);
					glVertex2f(outer2.left, outer2.top);
					glEnd();


					break;
				}		
			case Graphics::Display::kFillRectangle:
			{
				glColor3f(i.shape.c.r, i.shape.c.g, i.shape.c.b);
				//Graphics::rect tmp = GlobalHOGToViewport(i.shape.r, i.viewport);
				Graphics::rect tmp = ViewportToScreen(i.shape.r, i.viewport);
				//Graphics::rect tmp = ViewportToGlobalHOG(i.shape.r, i.viewport);
				//tmp *= {screenRect.right, screenRect.bottom}; // scale to screen
				glBegin(GL_QUADS);
				glVertex2f(tmp.left, tmp.bottom);
				glVertex2f(tmp.left, tmp.top);
				glVertex2f(tmp.right, tmp.top);
				glVertex2f(tmp.right, tmp.bottom);
				glEnd();
				//				epsilon -= de;
				break;
			}
		case Graphics::Display::kFrameRectangle:
			{
				glColor3f(i.shape.c.r, i.shape.c.g, i.shape.c.b);
				//Graphics::rect tmp = ViewportToGlobalHOG(i.shape.r, i.viewport);
				auto r2 = ViewportToScreen(i.shape.r.inset(i.shape.width/2), i.viewport);
				auto r1 = ViewportToScreen(i.shape.r.expand(i.shape.width/2), i.viewport);
				//tmp *= {screenRect.right, screenRect.bottom}; // scale to screen
					
				glBegin(GL_TRIANGLE_STRIP);
				//GLfloat rad = ViewportToGlobalHOGX(i.shape.width, i.viewport)/2.0;
				//GLfloat rad = ViewportToScreenX(i.shape.width, i.viewport)/2.0;
				glVertex2f(r1.left, r1.bottom);
				glVertex2f(r2.left, r2.bottom);
					
				glVertex2f(r1.left, r1.top);
				glVertex2f(r2.left, r2.top);
					
				glVertex2f(r1.right, r1.top);
				glVertex2f(r2.right, r2.top);
					
				glVertex2f(r1.right, r1.bottom);
				glVertex2f(r2.right, r2.bottom);
					
				glVertex2f(r1.left, r1.bottom);
				glVertex2f(r2.left, r2.bottom);


				glEnd();


				break;
			}
		case Graphics::Display::kFillOval:
			{
				//Graphics::rect tmp = ViewportToGlobalHOG(i.shape.r, i.viewport);
				Graphics::rect tmp = ViewportToScreen(i.shape.r, i.viewport);
				glColor3f(i.shape.c.r, i.shape.c.g, i.shape.c.b);
				DrawCircle((tmp.right+tmp.left)/2, (tmp.top+tmp.bottom)/2, fabs(tmp.top-tmp.bottom)/2.0, 64);
				break;
			}
		case Graphics::Display::kFrameOval:
			{
				//Graphics::rect tmp = ViewportToGlobalHOG(i.shape.r, i.viewport);
				Graphics::rect tmp = ViewportToScreen(i.shape.r, i.viewport);
				glColor3f(i.shape.c.r, i.shape.c.g, i.shape.c.b);
				FrameCircle((tmp.right+tmp.left)/2, (tmp.top+tmp.bottom)/2, fabs(tmp.top-tmp.bottom)/2.0, ViewportToScreenX(i.shape.width, i.viewport), 64);
				break;
			}
		case Graphics::Display::kFillNGon:
			{
				glColor3f(i.polygon.c.r, i.polygon.c.g, i.polygon.c.b);
				Graphics::rect hog(i.polygon.center, i.polygon.radius);
				auto tmp = ViewportToScreen(hog, i.viewport);
				DrawCircle((tmp.right+tmp.left)/2, (tmp.top+tmp.bottom)/2, fabs(tmp.top-tmp.bottom)/2.0, i.polygon.segments, i.polygon.rotate);
				break;
			}
		case Graphics::Display::kFrameNGon:
			{
				glColor3f(i.polygon.c.r, i.polygon.c.g, i.polygon.c.b);
				Graphics::rect hog(i.polygon.center, i.polygon.radius);
				auto tmp = ViewportToScreen(hog, i.viewport);
				FrameCircle((tmp.right+tmp.left)/2, (tmp.top+tmp.bottom)/2, fabs(tmp.top-tmp.bottom)/2.0, ViewportToScreenX(i.polygon.width, i.viewport), i.polygon.segments, i.polygon.rotate);
				break;
			}
		case Graphics::Display::kFillTriangle:
			{		
				glColor3f(i.triangle.c.r, i.triangle.c.g, i.triangle.c.b);
				Graphics::point tmp1 = ViewportToScreen(i.triangle.p1, i.viewport);
				Graphics::point tmp2 = ViewportToScreen(i.triangle.p2, i.viewport);
				Graphics::point tmp3 = ViewportToScreen(i.triangle.p3, i.viewport);

				glBegin(GL_TRIANGLES);
				glVertex3f(tmp1.x, tmp1.y, tmp1.z);
				glVertex3f(tmp2.x, tmp2.y, tmp2.z);
				glVertex3f(tmp3.x, tmp3.y, tmp3.z);
				glEnd();
				break;
			}
		case Graphics::Display::kFrameTriangle:
			{
				glColor3f(i.triangle.c.r, i.triangle.c.g, i.triangle.c.b);
				
				glBegin(GL_QUADS);
				Line(i.triangle.p1, i.triangle.p2, i.triangle.width, i.viewport);
				Line(i.triangle.p2, i.triangle.p3, i.triangle.width, i.viewport);
				Line(i.triangle.p3, i.triangle.p1, i.triangle.width, i.viewport);
				glEnd();
			}
		case Graphics::Display::kLine:
			{
				glColor3f(i.line.c.r, i.line.c.g, i.line.c.b);

				Line(i.line.start, i.line.end, i.line.width, i.viewport);
				//std::cout << i.line.start << " <=> " << i.line.end << " " << i.line.width << "\n";
				/*
				Graphics::point tmp1 = ViewportToScreen(i.line.start, i.viewport);
				Graphics::point tmp2 = ViewportToScreen(i.line.end, i.viewport);
				GLfloat xOff = tmp1.x-tmp2.x;
				GLfloat yOff = tmp2.y-tmp1.y;
				GLfloat ratio = ViewportToScreenX(0.5*i.line.width, i.viewport)/sqrt(xOff*xOff+yOff*yOff);
				glBegin(GL_QUADS);
				glVertex3f(tmp1.x-ratio*yOff, tmp1.y-ratio*xOff, tmp1.z);
				glVertex3f(tmp1.x+ratio*yOff, tmp1.y+ratio*xOff, tmp1.z);
				glVertex3f(tmp2.x+ratio*yOff, tmp2.y+ratio*xOff, tmp1.z);
				glVertex3f(tmp2.x-ratio*yOff, tmp2.y-ratio*xOff, tmp1.z);
				glEnd();
				*/
				break;
			}
		}
	}

}

void DrawLines(std::vector<Graphics::Display::lineInfo> &textLines, int viewport)
{
	for (auto i : textLines)
	{
		Graphics::point tmp1 = ViewportToScreen(i.start, viewport);
		Graphics::point tmp2 = ViewportToScreen(i.end, viewport);
		GLfloat xOff = tmp1.x-tmp2.x;
		GLfloat yOff = tmp2.y-tmp1.y;
		GLfloat wide = ViewportToScreenX(0.5f*i.width, viewport);
		GLfloat ratio = wide/sqrt(xOff*xOff+yOff*yOff);
		
		glBegin(GL_QUADS);
		glColor3f(i.c.r, i.c.g, i.c.b);
		glVertex3f(tmp1.x-ratio*yOff, tmp1.y-ratio*xOff, tmp1.z);
		glVertex3f(tmp1.x+ratio*yOff, tmp1.y+ratio*xOff, tmp1.z);
		glVertex3f(tmp2.x+ratio*yOff, tmp2.y+ratio*xOff, tmp1.z);
		glVertex3f(tmp2.x-ratio*yOff, tmp2.y-ratio*xOff, tmp1.z);
		glEnd();
		DrawCircle(tmp1.x, tmp1.y, wide, 16);
		DrawCircle(tmp2.x, tmp2.y, wide, 16);
	}
}

void Line(Graphics::point p1, Graphics::point p2, float width, int viewport)
{
	Graphics::point tmp1 = ViewportToScreen(p1, viewport);
	Graphics::point tmp2 = ViewportToScreen(p2, viewport);
	GLfloat xOff = tmp1.x-tmp2.x;
	GLfloat yOff = tmp2.y-tmp1.y;
	GLfloat ratio = ViewportToScreenX(0.5*width, viewport)/sqrt(xOff*xOff+yOff*yOff);
	glBegin(GL_QUADS);
	glVertex3f(tmp1.x-ratio*yOff, tmp1.y-ratio*xOff, tmp1.z);
	glVertex3f(tmp1.x+ratio*yOff, tmp1.y+ratio*xOff, tmp1.z);
	glVertex3f(tmp2.x+ratio*yOff, tmp2.y+ratio*xOff, tmp1.z);
	glVertex3f(tmp2.x-ratio*yOff, tmp2.y-ratio*xOff, tmp1.z);
	glEnd();
	
	// Graphics::point tmp1 = ViewportToScreen(p1, viewport);
	// Graphics::point tmp2 = ViewportToScreen(p2, viewport);
	// GLfloat xOff = tmp1.x-tmp2.x;
	// GLfloat yOff = tmp1.y-tmp2.y;
	// GLfloat ratio = 0.5f*width/sqrt(xOff*xOff+yOff*yOff);
	// glVertex3f(tmp1.x-ratio*yOff, tmp1.y-ratio*xOff, tmp1.z);
	// glVertex3f(tmp1.x+ratio*yOff, tmp1.y+ratio*xOff, tmp1.z);
	// glVertex3f(tmp2.x+ratio*yOff, tmp2.y+ratio*xOff, tmp1.z);
	// glVertex3f(tmp2.x-ratio*yOff, tmp2.y-ratio*xOff, tmp1.z);
}

void DrawGraphics(Graphics::Display &display, int port, sf::Window &window)
{
	for (auto &i : display.backgroundLineSegments)
	{
		if (i.viewport != port)
			continue;
		glColor3f(i.c.r, i.c.g, i.c.b);
		
		glBegin(GL_QUADS);
		for (int t = 0; t < i.points.size()-1; t++)
		{
			Graphics::point tmp1 = ViewportToScreen(i.points[t], i.viewport);
			Graphics::point tmp2 = ViewportToScreen(i.points[t+1], i.viewport);
			GLfloat xOff = tmp1.x-tmp2.x;
			GLfloat yOff = tmp2.y-tmp1.y;
			GLfloat ratio = 0.5f*i.size/sqrt(xOff*xOff+yOff*yOff);
			glVertex3f(tmp1.x-ratio*yOff, tmp1.y-ratio*xOff, tmp1.z);
			glVertex3f(tmp1.x+ratio*yOff, tmp1.y+ratio*xOff, tmp1.z);
			glVertex3f(tmp2.x+ratio*yOff, tmp2.y+ratio*xOff, tmp1.z);
			glVertex3f(tmp2.x-ratio*yOff, tmp2.y-ratio*xOff, tmp1.z);
		}
		glEnd();
	}
	DoDrawCommands(display, port, window, display.backgroundDrawCommands);
	for (auto &i : display.backgroundText)
	{
		if (i.viewport != port)
			continue;

		font.GetTextLines(textLines,
						  i.loc, i.s.c_str(), i.size,
						  i.c,
						  i.align, i.base);
		DrawLines(textLines, i.viewport);
	}

	
	for (auto &i : display.lineSegments)
	{
		if (i.viewport != port)
			continue;
		glColor3f(i.c.r, i.c.g, i.c.b);
		
		glBegin(GL_QUADS);
		for (int t = 0; t < i.points.size()-1; t++)
		{
			Line(i.points[t], i.points[t+1], i.size, i.viewport);
		}
		glEnd();
		
	}	
	DoDrawCommands(display, port, window, display.drawCommands);
	for (auto &i : display.text)
	{
		if (i.viewport != port)
			continue;

		font.GetTextLines(textLines,
						  i.loc, i.s.c_str(), i.size,
						  i.c,
						  i.align, i.base);
		DrawLines(textLines, i.viewport);
	}

}

/**
 * Main OpenGL drawing function.
 */
void drawGL (pRecContext pContextInfo, sf::Window &window)
{
	if (!pContextInfo)
	 return;
	pContextInfo->display.StartFrame();
	//window.clear();
	// clear our drawable
	glClear(GL_COLOR_BUFFER_BIT);
	glClear(GL_DEPTH_BUFFER_BIT);
	
	for (int x = 0; x < pContextInfo->display.numViewports; x++)
	{
		updateProjection(pContextInfo, x);
		setViewport(pContextInfo, x);
		//if (pContextInfo->drawing)
		{
			// set projection
			glMatrixMode(GL_PROJECTION);
			glLoadIdentity();
			
			glFrustum(camera.frust.left, camera.frust.right,
					  camera.frust.bottom, camera.frust.top,
					  camera.frust.near, camera.frust.far);
			// projection matrix already set
			updateModelView(pContextInfo, x);			
			HandleFrame(pContextInfo, x);
			DrawGraphics(pContextInfo->display, x, window);
		}
	}
	if (myTextBox)
	{
		myTextBox->stepTime(0.1);
		myTextBox->draw();
	}
	pContextInfo->display.EndFrame();
	//glutSwapBuffers();
	window.display();
}


/**
 * End OpenGL drawing function - for visualizing trajectory merging
 */
/*void trajectoryDrawGL (pRecContext pContextInfo)
{
 if (!pContextInfo)
        return;

	// clear our drawable
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	
	// projection matrix already set	
	updateModelView (pContextInfo);

	glDisable(GL_LIGHTING);
	pContextInfo->unitLayer->GetMap()->OpenGLDraw(kPolygons);
	if (pContextInfo->unitLayer->getMapAbstractionDisplay())
	{
		pContextInfo->unitLayer->getMapAbstractionDisplay()->OpenGLDraw();
	}
	glEnable(GL_LIGHTING);

	static double lastTime = ((double)clock()/CLOCKS_PER_SEC);
	double currTime = ((double)clock()/CLOCKS_PER_SEC);

	pContextInfo->unitLayer->advanceTime(currTime-lastTime);
	lastTime = currTime;
	if (pContextInfo->drawing)
	{
		pContextInfo->unitLayer->OpenGLDraw();
		if (pContextInfo->info) {
			glDisable(GL_LIGHTING);
			drawInfo (pContextInfo);
		}
	}
	frameCallback(pContextInfo->unitLayer);

	//glFlush();
	glutSwapBuffers();

}*/


/**
 * Initializes OpenGL.
 */
void buildGL(int xDim, int yDim)
{ 
    	if (NULL == pContextInfo)
        	return;
		
	// build context
		//CGRect viewRect = {{0.0f, 0.0f}, {0.0f, 0.0f}};
	
//	switch (pContextInfo->modeFSAA) {
//		case kFSAAOff:
//#ifndef WIN32
//			//glDisable (GL_MULTISAMPLE_ARB);
//#endif
//			break;
//		case kFSAAFast:
////			glEnable (GL_MULTISAMPLE_ARB);
////			glHint (GL_MULTISAMPLE_FILTER_HINT_NV, GL_FASTEST);
//			break;
//		case kFSAANice:
//		        #ifndef WIN32
//			//glEnable (GL_MULTISAMPLE_ARB);
//			//glHint (GL_MULTISAMPLE_FILTER_HINT_NV, GL_NICEST);
//			#endif
//			break;
//	}

	// init GL stuff here
	//glEnable(GL_DEPTH_TEST);
	glDisable(GL_DEPTH_TEST);

	glShadeModel(GL_SMOOTH);    
	glPolygonMode(GL_FRONT_AND_BACK,GL_FILL);
	glFrontFace(GL_CCW);
	glPolygonOffset (1.0, 1.0);
		
	glClearColor(0.0,0.0,0.0,0.0);

	//int scale = 2;//glutGet(GLUT_WINDOW_SCALE);
	//viewRect.size.width = xDim;//scale*glutGet(GLUT_WINDOW_WIDTH);
	//viewRect.size.height = yDim;//scale*glutGet(GLUT_WINDOW_HEIGHT);
	
	// setup viewport and prespective
	resizeGL(pContextInfo, xDim, yDim); // forces projection matrix update
		
	SetLighting();
}


void resetCamera()
{
	pRecContext pContextInfo = getCurrentContext();
	if (!pContextInfo)
		return;
//	pContextInfo->numPorts = 1;
//	for (int x = 0; x < MAXPORTS; x++)
	{
		resetCamera(&camera);
		for (int y = 0; y < 4; y++)
		{
			camera.rotations.worldRotation[y] = 0;
			camera.rotations.cameraRotation[y] = 0.0001;
		}
//		pContextInfo->camera[x].rotations.cameraRotation[0] = 180;
//		pContextInfo->camera[x].rotations.cameraRotation[2] = 1;
	}
	
	gTrackBallRotation [0] = gTrackBallRotation [1] = gTrackBallRotation [2] = gTrackBallRotation [3] = 0.0f;
	gTrackingContextInfo = 0;

	updateProjection(pContextInfo);  // update projection matrix
//	updateModelView(pContextInfo);
}

// sets the camera data to initial conditions
void resetCamera(recCamera * pCamera)
{
	pCamera->aperture = 10.0;
	
	pCamera->viewPos.x = 0.0;
	pCamera->viewPos.y = 0.0;
	pCamera->viewPos.z = -12.5;
	pCamera->viewDir.x = -pCamera->viewPos.x;
	pCamera->viewDir.y = -pCamera->viewPos.y;
	pCamera->viewDir.z = -pCamera->viewPos.z;
	
	pCamera->viewUp.x = 0;
	pCamera->viewUp.y = -1;//-.1;
	pCamera->viewUp.z = 0;//-1;

	//pCamera->viewRot.worldRotation = {0,0,0,0};
}

recVec cameraLookingAt(int port)
{
	pRecContext pContextInfo = getCurrentContext();
	if (port == -1)
		port = pContextInfo->display.numViewports;
	return /*pContextInfo->camera[port].viewPos-*/camera.viewDir;
}


void cameraLookAt(GLfloat x, GLfloat y, GLfloat z, float cameraSpeed, int port)
{
	pRecContext pContextInfo = getCurrentContext();
	if (!pContextInfo)
		return;
//	const float cameraSpeed = .1;
	if (port == -1)
		port = pContextInfo->display.currViewport;
	
	camera.viewDir.x = (1-cameraSpeed)*camera.viewDir.x + cameraSpeed*(x - camera.viewPos.x);
	camera.viewDir.y = (1-cameraSpeed)*camera.viewDir.y + cameraSpeed*(y - camera.viewPos.y);
	camera.viewDir.z = (1-cameraSpeed)*camera.viewDir.z + cameraSpeed*(z - camera.viewPos.z);
//	pContextInfo->rotations[port].objectRotation[0] *= (1-cameraSpeed);
//	pContextInfo->rotations[port].worldRotation[0] *= (1-cameraSpeed);
	updateProjection(pContextInfo);
}

void cameraMoveTo(GLfloat x, GLfloat y, GLfloat z, float cameraSpeed, int port)
{
	pRecContext pContextInfo = getCurrentContext();
	if (!pContextInfo)
		return;
//	const float cameraSpeed = .1;
//	if (port == -1)
//	{
//		port = pContextInfo->currPort;
//	}
	camera.viewPos.x = (1-cameraSpeed)*camera.viewPos.x + cameraSpeed*x;
	camera.viewPos.y = (1-cameraSpeed)*camera.viewPos.y + cameraSpeed*y;
	camera.viewPos.z = (1-cameraSpeed)*camera.viewPos.z + cameraSpeed*z;
	updateProjection(pContextInfo);
}

void cameraOffset(GLfloat x, GLfloat y, GLfloat z, float cameraSpeed, int port)
{
	pRecContext pContextInfo = getCurrentContext();
	if (!pContextInfo)
		return;
//	if (port == -1)
//	{
//		port = pContextInfo->currPort;
//	}
	camera.viewPos.x += cameraSpeed*x;
	camera.viewPos.y += cameraSpeed*y;
	camera.viewPos.z += cameraSpeed*z;
	updateProjection(pContextInfo);
}

void setPortCamera(pRecContext pContextInfo, int currPort)
{
	// Parameter no longer used
	currPort = 0;
	const double ratios[4][4][4] =
{{{0, 1, 0, 1}},
{{0, 0.5, 0, 1}, {0.5, 0.5, 0, 1}},
{{0, 0.5, 0.5, 0.5}, {0.5, 0.5, 0.5, 0.5}, {0, 1, 0, 0.5}},
{{0, 0.5, 0.5, 0.5}, {0.5, 0.5, 0.5, 0.5}, {0, 0.5, 0, 0.5}, {0.5, 0.5, 0, 0.5}}};
	
	currPort = 0; // NOTE: everyone gets the "full" screen
	//const double *val = ratios[pContextInfo->numPorts-1][currPort];
	const double *val = ratios[0][currPort];
	
	camera.viewOriginX = globalCamera.viewOriginX;
	camera.viewOriginY = globalCamera.viewOriginY;
	
	camera.viewWidth = (GLint)(val[1]*globalCamera.viewWidth);
	camera.viewHeight = (GLint)(val[3]*globalCamera.viewHeight);
	//	printf("Window %d port %d width: %d, height %d\n",
	//				 pContextInfo->windowID, currPort,
	//				 camera.viewWidth,
	//				 camera.viewHeight);
}

void setViewport(pRecContext pContextInfo, int currPort)
{
	pContextInfo->display.SetViewport(currPort);
	currPort = 0;
	
	const double ratios[4][4][4] =
	{{{0, 1, 0, 1}}, // x, width%, y, height%
		{{0, 0.5, 0, 1}, {0.5, 0.5, 0, 1}},
		{{0, 0.5, 0.5, 0.5}, {0.5, 0.5, 0.5, 0.5}, {0, 1, 0, 0.5}},
		{{0, 0.5, 0.5, 0.5}, {0.5, 0.5, 0.5, 0.5}, {0, 0.5, 0, 0.5}, {0.5, 0.5, 0, 0.5}}};
	
	const double *val = ratios[0][currPort];
	//	const double *val = ratios[pContextInfo->numPorts-1][currPort];
	
	glViewport(val[0]*globalCamera.viewWidth,
			   val[2]*globalCamera.viewHeight,
			   val[1]*globalCamera.viewWidth,
			   val[3]*globalCamera.viewHeight);
	
}

point3d GetOGLPos(pRecContext pContextInfo, int x, int y)
{
	setViewport(pContextInfo, pContextInfo->display.currViewport);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glFrustum(camera.frust.left,
			  camera.frust.right,
			  camera.frust.bottom,
			  camera.frust.top,
			  camera.frust.near,
			  camera.frust.far);
	// projection matrix already set
	updateModelView(pContextInfo, pContextInfo->display.currViewport);
	
	if (pContextInfo->display.numViewports > 1)
		HandleFrame(pContextInfo, pContextInfo->display.currViewport);
	
	GLint viewport[4];
	GLdouble modelview[16];
	GLdouble projection[16];
	GLfloat winX, winY, winZ;
	GLdouble posX, posY, posZ;
	
	glGetDoublev( GL_MODELVIEW_MATRIX, modelview );
	glGetDoublev( GL_PROJECTION_MATRIX, projection );
	glGetIntegerv( GL_VIEWPORT, viewport );
	winX = (float)x;
	winY = (float)viewport[3] - (float)y;
	glReadPixels( x, int(winY), 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &winZ );
	// if (gluUnProject( winX, winY, winZ, modelview, projection, viewport, &posX, &posY, &posZ) == GL_FALSE)
	// printf("WARNING: gluUnProject failed\n");
	//assert(!"gluUnProject missing");
	
	//	printf("Clicked (%f, %f, %f) [far plane %f near %f]\n", posX, posY, posZ,
	//		   pContextInfo->frust[pContextInfo->currPort].far+pContextInfo->camera[pContextInfo->currPort].viewPos.z,
	//		   pContextInfo->frust[pContextInfo->currPort].near+pContextInfo->camera[pContextInfo->currPort].viewPos.z);
	return point3d(posX, posY, posZ);
}



class bmp_header {
public:
	bmp_header()
	://bfType(19778),
	zero(0), bfOffBits(sizeof(bmp_header)+2), biSize(40), biPlanes(1),
	biBitCount(32), biCompression(0), biSizeImage(0), biXPelsPerMeter(2835), biYPelsPerMeter(2835),
	biClrUsed(0), biClrImportant(0) {}
	
//	uint16_t bfType;//	19778
	uint32_t bfSize; //	??	specifies the size of the file in bytes.
	uint32_t zero; // 0
	uint32_t bfOffBits;
	//	11	4	bfOffBits	1078	specifies the offset from the beginning of the file to the bitmap data.
	
	uint32_t biSize; // 40
	uint32_t biWidth;
	uint32_t biHeight;
	uint16_t biPlanes; // 0 (1??)
	uint16_t biBitCount; // 24
	uint32_t biCompression; // 0
	uint32_t biSizeImage; // 0
	uint32_t biXPelsPerMeter; // 0
	uint32_t biYPelsPerMeter; // 0
	uint32_t biClrUsed; // 0
	uint32_t biClrImportant; // 0
};

void SaveScreenshot(unsigned long windowID, const char *filename)
{
	pRecContext pContextInfo = GetContext(windowID);

	char file[strlen(filename)+5];
	sprintf(file, "%s.bmp", filename);
	FILE *f = fopen(file, "w+");

	if (f == 0) return;
	
//	3	4	bfSize	??	specifies the size of the file in bytes.
//	19	4	biWidth	100	specifies the width of the image, in pixels.
//	23	4	biHeight	100	specifies the height of the image, in pixels.

	
	uint32_t width  = globalCamera.viewWidth;
	uint32_t height  =globalCamera.viewHeight;
	long rowBytes = width * 4;
	long imageSize = rowBytes * height;
	std::vector<char> image(imageSize);
//	char image[imageSize];
	char zero[4] = {0, 0, 0, 0};
	glReadPixels(0, 0, GLsizei(width), GLsizei(height), GL_BGRA, GL_UNSIGNED_INT_8_8_8_8_REV, &image[0]);//GL_BGRA
	
	bmp_header h;
	h.biWidth = width;
	h.biHeight = height;
	int buffer = (4-width%4)%4;
	h.bfSize = sizeof(bmp_header)+2+(width+buffer)*height*4;
	h.biSizeImage = (width+buffer)*height*4;
	uint16_t bfType = 19778;
	fwrite(&bfType, sizeof(bfType), 1, f);
	fwrite(&h, sizeof(bmp_header), 1, f);
	for (int x = 0; x < height; x++)
	{
		fwrite(&image[x*width*4], sizeof(char), width*4, f);
		if (0 != width%4)
			fwrite(&zero, sizeof(char), buffer, f);
	}
	fclose(f);
}

void SetZoom(int windowID, float amount)
{
	pRecContext pContextInfo = GetContext(windowID);

	//if (pContextInfo->moveAllPortsTogether)
	{
//		for (int x = 0; x < pContextInfo->display.numViewports; x++)
//		{
//			camera.aperture = amount;
////			pContextInfo->camera[x].viewPos.z = -12.5+amount;
////			if (pContextInfo->camera[x].viewPos.z == 0.0) // do not let z = 0.0
////				pContextInfo->camera[x].viewPos.z = 0.0001;
//			updateProjection(pContextInfo, x);  // update projection matrix
//		}
//	}
//	else {
		camera.aperture = amount;
//		pContextInfo->camera[pContextInfo->currPort].viewPos.z = -12.5+amount;
//		if (pContextInfo->camera[pContextInfo->currPort].viewPos.z == 0.0) // do not let z = 0.0
//			pContextInfo->camera[pContextInfo->currPort].viewPos.z = 0.0001;
		updateProjection(pContextInfo, pContextInfo->display.currViewport);  // update projection matrix
	}
}

recVec GetHeading(unsigned long windowID, int which)
{
	recVec v;
	GetHeading(windowID, which, v.x, v.y, v.z);
	return v;
}

void GetHeading(unsigned long windowID, int which, GLdouble &hx, GLdouble &hy, GLdouble &hz)
{
	pRecContext pContextInfo = GetContext(windowID);

	double fRot[4];
	for (int x = 0; x < 4; x++)
		fRot[x] = camera.rotations.cameraRotation[x];
	// these formulas are derived from the opengl redbook 1.4 pg 700
	double xp, yp, zp, len, sa, ca;//hx, hy, hz,
	len = 1/sqrt(fRot[1]*fRot[1] +
				 fRot[2]*fRot[2] +
				 fRot[3]*fRot[3]);
	xp = fRot[1]*len;
	yp = fRot[2]*len;
	zp = fRot[3]*len;
	ca = cos(-fRot[0]*PI/180.0);
	sa = sin(-fRot[0]*PI/180.0);
	hx = (1-ca)*xp*zp+sa*yp;
	hy = (1-ca)*yp*zp-sa*xp;
	hz = ca+(1-ca)*zp*zp;
	len = 1/sqrt(hx*hx+hy*hy+hz*hz);
	hx *= len;
	hy *= len;
	hz *= len;
//	printf("Heading vector: (%1.3f, %1.3f, %1.3f)\n", hx, hy, hz);
}

