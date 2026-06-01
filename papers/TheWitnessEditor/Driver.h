/*
 *  $Id: Driver.h
 *  hog2
 *
 *  Created by Nathan Sturtevant on 5/31/05.
 *  Modified by Junwen Shen on 06/29/23.
 *
 * This file is part of HOG2. See https://github.com/nathansttt/hog2 for licensing information.
 *
 */
#ifndef THE_WITNESS_EDITOR_DRIVER_H
#define THE_WITNESS_EDITOR_DRIVER_H

#include "Common.h"

void WitnessWindowHandler(unsigned long windowID, tWindowEventType eType);

void WitnessFrameHandler(unsigned long windowID, unsigned int viewport, void *data);

void WitnessKeyboardHandler(unsigned long windowID, tKeyboardModifier mod, char key);

int WitnessCLHandler(char *argument[], int maxNumArgs);

bool WitnessClickHandler(unsigned long windowID, int viewport, int x, int y, point3d p, tButtonType, tMouseEventType event);

void InstallHandlers();

#endif /* THE_WITNESS_EDITOR_DRIVER_H */
