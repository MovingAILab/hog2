/*
 *  Common.h
 *  hog2
 *
 *  Created by Nathan Sturtevant on 11/02/06.
 *  Modified by Nathan Sturtevant on 02/29/20.
 *
 * This file has hog-specific functions for setting up hog events and setting up displays.
 * This current version represents a refactor where all OpenGL dependencies have been removed.
 *
 * This file is part of HOG2. See https://github.com/nathansttt/hog2 for licensing information.
 *
 */ 

#include "Graphics.h"
#include <stdio.h>
#include <cstring>
#include <cinttypes>

#ifndef COMMON_H
#define COMMON_H

#define main hog_main

// per view data
struct recContext
{
//	viewport viewports[MAXPORTS];
//	int windowHeight, windowWidth;
	
	char message[256]; // buffer for message output
	float msgTime; // message posting time for expiration
	
	Graphics::Display display;
	unsigned long windowID;
};
typedef struct recContext recContext;
typedef struct recContext * pRecContext;

void updateProjection(pRecContext pContextInfo, int viewPort = -1);
void drawGL(pRecContext pContextInfo, bool swap);

bool DoKeyboardCommand(pRecContext pContextInfo, unsigned char keyHit, bool shift, bool cntrl, bool alt);
void initialConditions(pRecContext pContextInfo);


void ProcessCommandLineArgs(int argc, char *argv[]);

extern char gDefaultMap[1024];

/* Window Viewports */
void SetNumPorts(unsigned long windowID, int count);
int GetNumPorts(unsigned long windowID);

/* Removes all active viewports and adds this one as the first.
 * rect coordinates are in HOG coordinates.
 */
void ReinitViewports(unsigned long windowID, const Graphics::rect &r, viewportType v);
/* Adds a new viewport to the existing viewports and
 * returns the new viewport numbers
 */
int AddViewport(unsigned long windowID, const Graphics::rect &r, viewportType v);
int AddViewport(unsigned long windowID, const Graphics::rect &initial, const Graphics::rect &fin, viewportType v);
void MoveViewport(unsigned long windowID, int viewport, const Graphics::rect &newLocation, float duration = 1.5, Graphics::TweenFunc func = Graphics::Tween::Linear);

/* Helper functions for converting coordinates between viewport and global hog coordinates */
Graphics::point ViewportToGlobalHOG(pRecContext pContextInfo, Graphics::point where, int viewport);
Graphics::rect  ViewportToGlobalHOG(pRecContext pContextInfo, const Graphics::rect &loc, int viewport);
Graphics::point ViewportToGlobalHOG(pRecContext pContextInfo, const Graphics::viewport &v, Graphics::point where);
float ViewportToGlobalHOGX(pRecContext pContextInfo, float x, int v);
Graphics::point GlobalHOGToViewport(pRecContext pContextInfo, const Graphics::viewport &v, Graphics::point where);
//Graphics::point GlobalHOGToViewport(Graphics::point where, int viewport);
//Graphics::rect GlobalHOGToViewport(const Graphics::rect &loc, int viewport);
//float GlobalHOGToViewportX(float x, int v);

/** Events **/

const char kLeftArrow  = 0x11;
const char kRightArrow = 0x12;
const char kUpArrow    = 0x13;
const char kDownArrow  = 0x14;

enum tKeyboardModifier {
	kNoModifier,
	kAnyModifier,
	kShiftDown,
	kControlDown,
	kAltDown // option on the mac
};

enum tButtonType {
	kLeftButton,
	kRightButton,
	kMiddleButton, // option on the mac
	kNoButton
};

enum tMouseEventType : int {
	kMouseDown = 0x1,
	kMouseUp = 0x2,
	kMouseDrag = 0x4, // option on the mac
	kMouseMove = 0x8
};

enum tWindowEventType {
	kWindowCreated,
	kWindowDestroyed
};

/**
* install a FrameCallback to be called once per frame and viewport
 * This is where you should do any drawing and/or update the unit simulation time.
 * The viewports will be processed in order (0..#, max 4), so any global processing
 * should only be done when the viewport is 0.
 * The void* data is passed back to each call.
 */
typedef void (*FrameCallback)(unsigned long windowID, unsigned int viewport, void*);

/**
* a window callback handler called when a window is created or destroyed
 */
typedef void (*WindowCallback)(unsigned long windowID, tWindowEventType);

/**
 * a joystaick callback handler is passed the same data passed in when it was installed
 */
typedef void (*JoystickCallback)(unsigned long windowID, double offsetX, double offsetY, void*);

/**
 * a mouse callback handler passes the absolute and local mouse coordinates of the click
 * returns true if the click/movement was handled; false if ignored
 */
typedef bool (*MouseCallback)(unsigned long windowID, int x, int y, point3d loc, tButtonType, tMouseEventType);
/**
 * the new callback handler also passes the viewport frame in which the click occured.
 * The loc is relative to the frame which was clicked
 */
typedef bool (*MouseCallback2)(unsigned long windowID, int viewport, int x, int y, point3d loc, tButtonType, tMouseEventType);

/**
 * a keyboard callback handler is passed the current unit simulation, the key
 * that was hit, and any modifiers that were being held down. Note that the
 * modifiers can change the character being passed.
 */
typedef void (*KeyboardCallback)(unsigned long windowID, tKeyboardModifier, char);
/**
 * A command-line callback handler takes in a char** which points to the argument
 * which matches the argument that this handler was installed for. The second
 * argument is the number of remaining arguments available. The handler
 * can process and use as many of the remaining arguments, and returns the number
 * of arguments it processed.
 */
typedef int (*CommandLineCallback)(char**, int);

///**
// * A data callback handler is called when new data is available from the GUI.
// * The handler can then read as much data as it wants.
// *
// * Implementation postponed for now:
// *   in some situations can use the text buffer for this purpose
// */
//typedef int (*DataCallback)(char**, int);

class commandLineCallbackData {
public:
	commandLineCallbackData(CommandLineCallback _CLC, const char *_argument,
													const char *_param, const char *_desc)
	:CLC(_CLC), argument(_argument), param(_param), desc(_desc) {}

	void Print()
	{ printf("%s : %s\n     %s\n", argument, param, desc); }
	
	CommandLineCallback CLC;
	const char *argument;
	const char *param;
	const char *desc;
};

class keyboardCallbackData {
public:
	keyboardCallbackData(KeyboardCallback kc, const char *_title, const char *_desc,
											 tKeyboardModifier _mod, keyboardCallbackData *_next = 0)
	:call(kc), title(_title), desc(_desc), mod(_mod), next(_next) {}
	
	KeyboardCallback call;
	const char *title;
	const char *desc;
	tKeyboardModifier mod;
	keyboardCallbackData *next;
};

class joystickCallbackData {
public:
	joystickCallbackData(JoystickCallback _jC, void *_userData)
	:jC(_jC)
	{ userData = _userData; }

	JoystickCallback jC;
	void *userData;
};

class mouseCallbackData {
public:
	mouseCallbackData(MouseCallback _mC, tMouseEventType _which)
	:mC(_mC), which(_which)
	{}
	MouseCallback mC;
	tMouseEventType which;
};

class mouseCallbackData2 {
public:
	mouseCallbackData2(MouseCallback2 _mC, tMouseEventType _which)
	:mC(_mC), which(_which)
	{}
	MouseCallback2 mC;
	tMouseEventType which;
};

class windowCallbackData {
public:
	windowCallbackData(WindowCallback _wC)
	:wC(_wC)
	{}
	WindowCallback wC;
};

class frameCallbackData {
public:
	frameCallbackData(FrameCallback _glCall, unsigned long _windowID, void *_userData)
	:glCall(_glCall)
	{ windowID = _windowID;
		userData = _userData; }
	
	FrameCallback glCall;
	unsigned long windowID;
	void *userData;
};

void InstallFrameHandler(FrameCallback jC, unsigned long windowID, void *userdata);
void RemoveFrameHandler(FrameCallback jC, unsigned long windowID, void *userdata);
void HandleFrame(pRecContext pContextInfo, int viewport);

void InstallJoystickHandler(JoystickCallback jC, void *userdata);
void RemoveJoystickHandler(JoystickCallback jC, void *userdata);
void HandleJoystickMovement(pRecContext pContextInfo, double panX, double panY);

void InstallMouseClickHandler(MouseCallback mC, tMouseEventType which = static_cast<tMouseEventType>(kMouseDown|kMouseUp|kMouseDrag));
void InstallMouseClickHandler(MouseCallback2 mC, tMouseEventType which = static_cast<tMouseEventType>(kMouseDown|kMouseUp|kMouseDrag));
void RemoveMouseClickHandler(MouseCallback mC);
void RemoveMouseClickHandler(MouseCallback2 mC);
bool HandleMouse(pRecContext pContextInfo, point3d where, tButtonType button, tMouseEventType mouse);
bool HandleMouse(pRecContext pContextInfo, int xWindow, int yWindow, point3d where, tButtonType button, tMouseEventType mouse);
bool HandleMouseClick(pRecContext pContextInfo, int x, int y, point3d where, tButtonType, tMouseEventType);
bool HandleMouseClick(pRecContext pContextInfo, int viewport, int x, int y, point3d where, tButtonType, tMouseEventType);

struct buttonData
{
	bool valid;
	bool hilighted;
	bool tracking;
	bool active;
	// todo - allow buttons to be inactivated
	unsigned long windowID;
	int viewport;
	Graphics::roundedRect r;
	std::string text;
	char hit;
	float borderSize;
	rgbColor textColor;
	rgbColor borderColor;
	rgbColor fillColor;
	rgbColor fillHitColor;
	rgbColor inactiveBorderColor;
};

void DrawButtons(unsigned long windowID, int viewport);
int CreateButton(unsigned long windowID, int viewport,
				 Graphics::roundedRect r, const char *txt, char hit, float borderSize,
				 rgbColor textColor, rgbColor lineColor, rgbColor fillColor,
				 rgbColor fillHitColor, rgbColor inactiveLineColor);
void SetButtonFillColor(int, rgbColor);
void SetButtonActive(bool, int);
void ShowButton(int identifier);
void HideButton(int identifier);

void InstallWindowHandler(WindowCallback wC);
void RemoveWindowHandler(WindowCallback wC);
void HandleWindowEvent(pRecContext pContextInfo, tWindowEventType);

void InstallKeyboardHandler(KeyboardCallback kf, const char *title, const char *description,
														tKeyboardModifier mod, unsigned char firstKey, unsigned char lastKey = 0);
void GetKeyAssignments(std::vector<char> &keys);
void GetKeyAssignmentDescriptions(std::vector<std::string> &keys);
void PrintKeyboardAssignments();
void InstallCommandLineHandler(CommandLineCallback, const char *, const char *, const char *);
void PrintCommandLineArguments();
void ProcessCommandLineArgs(int argc, char *argv[]);

void RunHOGGUI(int argc, char* argv[], int windowDimension = 1000);
void RunHOGGUI(int argc, char* argv[], int xDimension, int yDimension);

void submitTextToBuffer(const char *val);
void appendTextToBuffer(const char *);
const char *getTextBuffer();
void setTextBufferVisibility(bool);
bool getTextBufferVisibility();

/** Each window has an associated context. This gets the context currently being used. **/
// Note that this has not been well-tested recently.
pRecContext getCurrentContext();
pRecContext GetContext(unsigned long windowID);



#endif
