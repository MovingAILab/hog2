//
//  GUICode.h
//  hog2 glut
//
//  Created by Nathan Sturtevant on 7/29/15.
//  Copyright (c) 2015 University of Denver. All rights reserved.
//

#ifndef __hog2_glut__GUICode__
#define __hog2_glut__GUICode__

#include <stdio.h>
#include "Common.h"

bool MyClickHandler(unsigned long windowID, int, int, point3d loc, tButtonType button, tMouseEventType mType);
void MyWindowHandler(unsigned long windowID, tWindowEventType eType);
void MyFrameHandler(unsigned long windowID, unsigned int viewport, void *data);
void MyKeyboardHandler(unsigned long windowID, tKeyboardModifier, char key);
void InstallHandlers();
void AnalyzeMap(const char *, const char *, double weight);
void AnalyzeNBS(const char *, const char *, double weight);

#endif /* defined(__hog2_glut__GUICode__) */
