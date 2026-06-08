//
//  CocoaHOG.h
//  hog2 mac native demos
//
//  Created by Nathan Sturtevant on 8/2/17.
//  Copyright © 2017 NS Software. All rights reserved.
//

#ifndef CocoaHOG_h
#define CocoaHOG_h

#include "Common.h"

void updateProjection(pRecContext pContextInfo, int viewPort);
pRecContext GetContext(unsigned long windowID);
pRecContext getCurrentContext();
void updateModelView(pRecContext pContextInfo, int currPort);
bool getTextBufferVisibility();

extern GLfloat gTrackBallRotation [4];
extern pRecContext gTrackingContextInfo;

// In common.h
//void submitTextToBuffer(const char *val);
//void appendTextToBuffer(const char *);
//const char *getTextBuffer();

#endif /* CocoaHOG_h */
