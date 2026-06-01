/*
 *  $Id: common.cpp
 *  hog2
 *
 *  Created by Nathan Sturtevant on 11/02/06.
 *  Modified by Nathan Sturtevant on 02/29/20.
 *
 * This file is part of HOG2. See https://github.com/nathansttt/hog2 for licensing information.
 *
 */ 

#include <iostream>
#include <vector>
#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include <cassert>

#include "Constants.h"
#include "Trackball.h"
#include "Common.h"



// For printing debug info
//static bool const verbose = false;

static unsigned long gNextWindowID = 0;
char gDefaultMap[1024] = "";
//const Graphics::point gOrigin(0.0, 0.0, 0.0);

using namespace std;

static std::vector<commandLineCallbackData *> commandLineCallbacks;
static std::vector<joystickCallbackData *> joystickCallbacks;
static std::vector<mouseCallbackData *> mouseCallbacks;
static std::vector<mouseCallbackData2 *> mouseCallbacks2;
static std::vector<windowCallbackData *> windowCallbacks;
static std::vector<frameCallbackData *> glDrawCallbacks;
//static std::vector<dataCallbackData> dataCallbacks;
static std::vector<buttonData> buttons;

void InstallSystemMouseClickHandler(MouseCallback2 mC, tMouseEventType which =
									static_cast<tMouseEventType>(kMouseDown|kMouseUp|kMouseDrag));

bool ButtonHandler(unsigned long windowID, int viewport, int windowX, int windowY, point3d loc, tButtonType button, tMouseEventType mType)
{
	for (auto &b : buttons)
	{
		if (!(b.windowID == windowID && b.viewport == viewport))
			continue;
		if (mType == kMouseDown)
		{
			if (b.active)
				b.tracking = b.hilighted = Graphics::PointInRect(loc, b.r);
			else
				b.tracking = b.hilighted = false;
		}
		if (mType == kMouseDrag && b.tracking)
			b.hilighted = Graphics::PointInRect(loc, b.r);
		if (mType == kMouseUp && b.tracking)
		{
			b.hilighted = false;
			b.tracking = false;
			if (Graphics::PointInRect(loc, b.r))
				DoKeyboardCommand(GetContext(windowID), b.hit, false, false, false);
		}
	}
	return false;
}

void DrawButton(const buttonData &b, Graphics::Display &disp)
{
	if (!b.valid)
		return;
	
	disp.FillRect(b.r, b.hilighted?b.fillHitColor:b.fillColor);
	if (b.active)
	{
		disp.FrameRect(b.r, b.borderColor, b.borderSize);
		disp.DrawText(b.text.c_str(),
					  {0.5f*b.r.r.right+0.5f*b.r.r.left,0.5f*b.r.r.top+0.5f*b.r.r.bottom},
					  b.textColor, (b.r.r.bottom-b.r.r.top)*0.7f, Graphics::textAlignCenter, Graphics::textBaselineMiddle);
	}
	else {
		disp.FrameRect(b.r, b.inactiveBorderColor, b.borderSize);
		disp.DrawText(b.text.c_str(),
					  {0.5f*b.r.r.right+0.5f*b.r.r.left,0.5f*b.r.r.top+0.5f*b.r.r.bottom},
					  b.inactiveBorderColor, (b.r.r.bottom-b.r.r.top)*0.7f, Graphics::textAlignCenter, Graphics::textBaselineMiddle);
	}
}

void DrawButtons(unsigned long windowID, int viewport)
{
	Graphics::Display &disp = GetContext(windowID)->display;
	disp.SetViewport(viewport);
	for (auto const &b : buttons)
		if (b.viewport == viewport)
			DrawButton(b, disp);
}

int CreateButton(unsigned long windowID, int viewport,
				 Graphics::roundedRect r, const char *txt, char hit, float borderSize,
				 rgbColor textColor, rgbColor lineColor, rgbColor fillColor,
				 rgbColor fillHitColor, rgbColor inactiveLineColor)
{
	bool found = false;
	if (buttons.size() == 0)
		InstallSystemMouseClickHandler(ButtonHandler); // makes button get first clicks
	buttons.push_back({true, false, false, true, windowID, viewport, r, txt, hit,
		borderSize, textColor, lineColor, fillColor, fillHitColor, inactiveLineColor});
	return buttons.size()-1;
}

void SetButtonActive(bool active, int which)
{
	buttons[which].active = active;
}

void SetButtonFillColor(int button, rgbColor c)
{
	if (button < buttons.size())
		buttons[button].fillColor = c;
}


//bool valid;
//Graphics::roundedRect r;
//std::string s;
//char hit;
//float borderSize;
//rgbColor textColor;
//rgbColor lineColor;
//rgbColor fillColor;
//rgbColor fillHitColor;

void ShowButton(int identifier)
{
	buttons[identifier].valid = true;
}

void HideButton(int identifier)
{
	buttons[identifier].valid = false;
}

static keyboardCallbackData *keyboardCallbacks[256] =
{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

const char *getModifierText(tKeyboardModifier t)
{
	switch (t)
	{
		case kNoModifier: return "";
		case kAnyModifier: return "*-";
		case kShiftDown: return "Sft-";
		case kControlDown: return "Ctl-";
		case kAltDown: return "Alt-";
		default: return "?-";
	}
}

void InstallKeyboardHandler(KeyboardCallback kf, const char *title, const char *description,
														tKeyboardModifier mod, unsigned char firstKey, unsigned char lastKey)
{
	for (int x = firstKey; ; x++)
	{
		keyboardCallbacks[x] = new keyboardCallbackData(kf, title, description, mod, keyboardCallbacks[x]);
		if (x >= lastKey)
			break;
	}
}

void GetKeyAssignments(std::vector<char> &keys)
{
	for (int x = 0; x < 256; x++)
	{
		for (keyboardCallbackData *kd = keyboardCallbacks[x]; kd; kd = kd->next)
		{
			char val = x;
			keys.push_back(val);
//			printf("%s%c\t%s\t%s\n", getModifierText(kd->mod), x, kd->title, kd->desc);
		}
	}
}

void GetKeyAssignmentDescriptions(std::vector<std::string> &keys)
{
	for (int x = 0; x < 256; x++)
	{
		for (keyboardCallbackData *kd = keyboardCallbacks[x]; kd; kd = kd->next)
		{
			//char val = x;
			keys.push_back(kd->title);
			//			printf("%s%c\t%s\t%s\n", getModifierText(kd->mod), x, kd->title, kd->desc);
		}
	}
}
void PrintKeyboardAssignments()
{
	printf("Legal Keyboard Commands\n-----------------------\n");
	for (int x = 0; x < 256; x++)
	{
		for (keyboardCallbackData *kd = keyboardCallbacks[x]; kd; kd = kd->next)
		{
			printf("%s%c\t%s\t%s\n", getModifierText(kd->mod), x, kd->title, kd->desc);
		}
	}
}


bool DoKeyboardCallbacks(pRecContext pContextInfo, unsigned char keyHit, tKeyboardModifier mod)
{
	bool called = false;
	for (keyboardCallbackData *kd = keyboardCallbacks[keyHit]; kd; kd = kd->next)
	{
		if ((mod == kd->mod) || (kd->mod == kAnyModifier))
		{
//			printf("(DEBUG) calling: %s%c\t%s\t%s\n", getModifierText(kd->mod), keyHit,
//						 kd->title, kd->desc);
			kd->call(pContextInfo->windowID, mod, keyHit);
			called = true;
		}
	}
	if (!called)
	{
		printf("Unknown keyboard command %s%c/%d\n\n", getModifierText(mod), keyHit, keyHit);
		PrintKeyboardAssignments();
	}
	return false;
}

void InstallFrameHandler(FrameCallback glCall, unsigned long windowID, void *userdata)
{
	glDrawCallbacks.push_back(new frameCallbackData(glCall, windowID, userdata));	
}

void RemoveFrameHandler(FrameCallback glCall, unsigned long windowID, void *userdata)
{
	for (unsigned int x = 0; x < glDrawCallbacks.size(); x++)
	{
		if ((glDrawCallbacks[x]->glCall == glCall) &&
				(glDrawCallbacks[x]->userData == userdata) &&
				(glDrawCallbacks[x]->windowID == windowID))
		{
			delete glDrawCallbacks[x];
			glDrawCallbacks[x] = glDrawCallbacks[glDrawCallbacks.size()-1];
			glDrawCallbacks.pop_back();
		}
	}
}

void HandleFrame(pRecContext pContextInfo, int viewport)
{
	for (unsigned int x = 0; x < glDrawCallbacks.size(); x++)
	{
		if (glDrawCallbacks[x]->windowID == pContextInfo->windowID)
			glDrawCallbacks[x]->glCall(pContextInfo->windowID, viewport, glDrawCallbacks[x]->userData);
	}
	DrawButtons(pContextInfo->windowID, viewport);
	pContextInfo->display.EndFrame(); // end last frame
	for (int x = 0; x < pContextInfo->display.numViewports; x++)
	{
		auto &vp = pContextInfo->display.viewports[x];
		if (fequal(vp.animationDuration, 0.0f))
		{
			vp.bounds = vp.finalBound;
			continue;
		}
		if (vp.bounds == vp.finalBound)
		{
			continue;
		}
		const auto percent = static_cast<float>(vp.frameCount++) / (vp.animationDuration * 30.0f);
        InterpolateRect(vp.bounds, vp.startBound, vp.finalBound, vp.tweenFunc(percent));
	}
}

void InstallJoystickHandler(JoystickCallback jC, void *userdata)
{
	joystickCallbacks.push_back(new joystickCallbackData(jC, userdata));	
}

void RemoveJoystickHandler(JoystickCallback jC, void *userdata)
{
	for (unsigned int x = 0; x < joystickCallbacks.size(); x++)
	{
		if ((joystickCallbacks[x]->jC == jC) &&
				(joystickCallbacks[x]->userData == userdata))
		{
			delete joystickCallbacks[x];
			joystickCallbacks[x] = joystickCallbacks[joystickCallbacks.size()-1];
			joystickCallbacks.pop_back();
		}
	}
}

void HandleJoystickMovement(pRecContext pContextInfo, double panX, double panY)
{
	for (unsigned int x = 0; x < joystickCallbacks.size(); x++)
	{
		//printf("Calling joystick callback %d\n", x);
		joystickCallbacks[x]->jC(pContextInfo->windowID, panX, panY, joystickCallbacks[x]->userData);
	}
}

void InstallCommandLineHandler(CommandLineCallback CLC,
															 const char *arg, const char *param, const char *usage)
{
	commandLineCallbacks.push_back(new commandLineCallbackData(CLC, arg, param, usage));
}

void PrintCommandLineArguments()
{
	printf("Valid command-line flags:\n\n");
	for (unsigned int x = 0; x < commandLineCallbacks.size(); x++)
		commandLineCallbacks[x]->Print();
	printf("\n");
}

// Process command line arguments
void ProcessCommandLineArgs(int argc, char *argv[])
{
//	printf("Processing %d command line arguments\n", argc);
//	for (int x = 0; x < argc; x++)
//	{
//		printf("%s ", argv[x]);
//        printf("\n");
//        printf("\n");
//	}
//	printf("\n");
//	initializeCommandLineHandlers();
//	 printCommandLineArguments();

	int lastval = 1;
	for (int y = 1; y < argc; )
	{
		lastval = y;
		for (unsigned int x = 0; x < commandLineCallbacks.size(); x++)
		{
			if (strcmp(commandLineCallbacks[x]->argument, argv[y]) == 0)
			{
				y += commandLineCallbacks[x]->CLC(&argv[y], argc-y);
				break;
			}
		}
		if (lastval == y)
		{
			printf("Error: unhandled command-line parameter (%s)\n\n", argv[y]);
			PrintCommandLineArguments();
			y++;
			//exit(10);
		}
	}
}

void InstallMouseClickHandler(MouseCallback mC, tMouseEventType which)
{
	mouseCallbacks.push_back(new mouseCallbackData(mC, which));
}

void InstallMouseClickHandler(MouseCallback2 mC, tMouseEventType which)
{
	mouseCallbacks2.push_back(new mouseCallbackData2(mC, which));
}

void InstallSystemMouseClickHandler(MouseCallback2 mC, tMouseEventType which)
{
	auto data = new mouseCallbackData2(mC, which);
	mouseCallbacks2.insert(mouseCallbacks2.begin(), data);
}



void RemoveMouseClickHandler(MouseCallback mC)
{
	for (unsigned int x = 0; x < mouseCallbacks.size(); x++)
	{
		if (mouseCallbacks[x]->mC == mC)
		{
			delete mouseCallbacks[x];
			mouseCallbacks[x] = mouseCallbacks[mouseCallbacks.size()-1];
			mouseCallbacks.pop_back();
		}
	}
}

void RemoveMouseClickHandler(MouseCallback2 mC)
{
	for (unsigned int x = 0; x < mouseCallbacks2.size(); x++)
	{
		if (mouseCallbacks2[x]->mC == mC)
		{
			delete mouseCallbacks2[x];
			mouseCallbacks2[x] = mouseCallbacks2[mouseCallbacks2.size()-1];
			mouseCallbacks2.pop_back();
		}
	}
}



/* New low-end mouse handler. Does the viewport computation - just requires incoming global HOG coordinates. */
bool HandleMouse(pRecContext pContextInfo, int xWindow, int yWindow, point3d where, tButtonType button, tMouseEventType mouse)
{
	for (int x = pContextInfo->display.numViewports-1; x >= 0; x--)
	{
//		if (!pContextInfo->viewports[x].active)
//			continue;
		if (!PointInRect(where, pContextInfo->display.viewports[x].bounds))
		{
			continue;
		}
		// got hit in rect
		Graphics::point res = pContextInfo->display.GlobalHOGToViewport( pContextInfo->display.viewports[x], where);
		// click handled
		if (HandleMouseClick(pContextInfo, x, xWindow, yWindow, res, button, mouse))
			return true;
	}
	return false;
}

/* New low-end mouse handler. Does the viewport computation - just requires incoming global HOG coordinates. */
bool HandleMouse(pRecContext pContextInfo, point3d where, tButtonType button, tMouseEventType mouse)
{
	for (int x = pContextInfo->display.numViewports-1; x >= 0; x--)
		//for (int x = MAXPORTS-1; x >= 0; x--)
	{
//		if (!pContextInfo->viewports[x].active)
//			continue;
		if (!PointInRect(where, pContextInfo->display.viewports[x].bounds))
			continue;
		// got hit in rect
		Graphics::point res = pContextInfo->display.GlobalHOGToViewport(pContextInfo->display.viewports[x], where);
		// click handled
		if (HandleMouseClick(pContextInfo, x, -1, -1, res, button, mouse))
			return true;
	}
	return false;
}

// this is called by the OS when it gets a click
bool HandleMouseClick(pRecContext pContextInfo, int viewport, int x, int y, point3d where,
					  tButtonType button, tMouseEventType mouse)
{
	for (unsigned int j = 0; j < mouseCallbacks2.size(); j++)
	{
		if (mouseCallbacks2[j]->which&mouse) // need to ask for event to call handler
		{
//			if (x == -1 || y == -1)
//				fprintf(stderr, "Warning: window coordinates not being pased into HandleMouseClick (%s line %d)\n",
//						__FILE__, __LINE__);
			if (mouseCallbacks2[j]->mC(pContextInfo->windowID, viewport, x, y, where,
									   button, mouse))
				return true;
		}
	}
	return HandleMouseClick(pContextInfo, x, y, where, button, mouse);
}

bool HandleMouseClick(pRecContext pContextInfo, int x, int y, point3d where,
											tButtonType button, tMouseEventType mouse)
{
	for (unsigned int j = 0; j < mouseCallbacks.size(); j++)
	{
		if (mouseCallbacks[j]->which&mouse) // need to ask for event to call handler
		{
			if (mouseCallbacks[j]->mC(pContextInfo->windowID, x, y, where,
									  button, mouse))
				return true;
		}
	}
	return false;
}

void InstallWindowHandler(WindowCallback wC)
{
	windowCallbacks.push_back(new windowCallbackData(wC));
}

void RemoveWindowHandler(WindowCallback wC)
{
	for (unsigned int x = 0; x < windowCallbacks.size(); x++)
	{
		if (windowCallbacks[x]->wC == wC)
		{
			delete windowCallbacks[x];
			windowCallbacks[x] = windowCallbacks[windowCallbacks.size()-1];
			windowCallbacks.pop_back();
		}
	}
}

void HandleWindowEvent(pRecContext pContextInfo, tWindowEventType e)
{
	for (unsigned int j = 0; j < windowCallbacks.size(); j++)
	{
		//printf("Calling window callback %d\n", x);
		windowCallbacks[j]->wC(pContextInfo->windowID, e);
	}
}

//void InstallDataHandler(DataCallback dC)
//{
//	dataCallbacks.push_back(dataCallbackData(dC));
//}
//
//void RemoveDataHandler(DataCallback dC)
//{
//	for (unsigned int x = 0; x < dataCallbacks.size(); x++)
//	{
//		if (dataCallbacks[x].dC == dC)
//		{
//			dataCallbacks[x] = dataCallbacks[dataCallbacks.size()-1];
//			dataCallbacks.pop_back();
//		}
//	}
//}


// intializes context conditions
void initialConditions(pRecContext pContextInfo)
{
	pContextInfo->windowID = gNextWindowID++;
	ReinitViewports(pContextInfo->windowID, {-1, -1, 1, 1}, kScaleToSquare);
}

bool DoKeyboardCommand(pRecContext pContextInfo, unsigned char keyHit, bool shift, bool cntrl, bool alt)
{
	DoKeyboardCallbacks(pContextInfo, tolower(keyHit), 
											shift?kShiftDown:(cntrl?kControlDown:(alt?kAltDown:kNoModifier)));
	return false;
}


//void rotateObject()
//{
//	pRecContext pContextInfo = getCurrentContext();
//	if (!pContextInfo)
//		return;
//	pContextInfo->rotations[pContextInfo->currPort].objectRotation[0] += 1;
//	pContextInfo->rotations[pContextInfo->currPort].objectRotation[1] = 0;
//	pContextInfo->rotations[pContextInfo->currPort].objectRotation[2] = 1;
//	pContextInfo->rotations[pContextInfo->currPort].objectRotation[3] = 0;
//}


// TODO: These are meant to be deprecated - but still need to verify
void SetNumPorts(unsigned long windowID, int count)
{
	pRecContext pContextInfo = GetContext(windowID);
	pContextInfo->display.SetNumViewports(count);
	pContextInfo->display.SetViewport(0);
}

int GetNumPorts(unsigned long windowID)
{
	pRecContext pContextInfo = GetContext(windowID);
	return pContextInfo->display.numViewports;
}
//
//int GetActivePort(unsigned long windowID)
//{
//	pRecContext pContextInfo = GetContext(windowID);
//	return pContextInfo->currPort;
//}
//
//void SetActivePort(unsigned long windowID, int which)
//{
//	pRecContext pContextInfo = GetContext(windowID);
//	if ((which >= 0) && (which < pContextInfo->numPorts))
//		pContextInfo->currPort = which;
//}

void ReinitViewports(unsigned long windowID, const Graphics::rect &r, viewportType v)
{
	pRecContext pContextInfo = GetContext(windowID);
	pContextInfo->display.ReinitViewports(r, v);
}

int AddViewport(unsigned long windowID, const Graphics::rect &r, viewportType v)
{
	pRecContext pContextInfo = GetContext(windowID);
	return pContextInfo->display.AddViewport(r, v);
}

int AddViewport(unsigned long windowID, const Graphics::rect &initial, const Graphics::rect &fin, viewportType v)
{
	pRecContext pContextInfo = GetContext(windowID);
	return pContextInfo->display.AddViewport(initial, fin, v);
}

void MoveViewport(unsigned long windowID, int viewport, const Graphics::rect &newLocation, float duration, Graphics::TweenFunc func)
{
	pRecContext pContextInfo = GetContext(windowID);
	pContextInfo->display.MoveViewport(viewport, newLocation, duration, std::move(func));
}

Graphics::point ViewportToGlobalHOG(pRecContext pContextInfo, const Graphics::viewport &v, Graphics::point where)
{
	return pContextInfo->display.ViewportToGlobalHOG(v, where);
}

Graphics::rect  ViewportToGlobalHOG(pRecContext pContextInfo, const Graphics::rect &loc, int viewport)
{
	return pContextInfo->display.ViewportToGlobalHOG(loc, viewport);
}

float ViewportToGlobalHOGX(pRecContext pContextInfo, float x, int v)
{
	return pContextInfo->display.ViewportToGlobalHOGX(x, v);
}

Graphics::point GlobalHOGToViewport(pRecContext pContextInfo, const Graphics::viewport &v, Graphics::point where)
{
	return pContextInfo->display.GlobalHOGToViewport(v, where);
}

Graphics::point ViewportToGlobalHOG(pRecContext pContextInfo, Graphics::point where, int viewport)
{
	return pContextInfo->display.ViewportToGlobalHOG(where, viewport);
}
