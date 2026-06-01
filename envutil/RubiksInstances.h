//
//  RubiksInstances.h
//  hog2 glut
//
//  Created by Nathan Sturtevant on 8/26/19.
//

#ifndef RubiksInstances_h
#define RubiksInstances_h

#include <stdio.h>
#include "RubiksCube.h"
#include "RC.h"

namespace RubiksCubeInstances {

void GetKorfRubikInstance(RubiksState &start, int which);
void GetKorfRubikInstance(RCState &start, int which);

void GetSuperFlip(RubiksState &start);
void GetSuperFlip(RCState &start);

void GetDepth20(RubiksState &start, int which);

void GetRandomN(RubiksState &start, int N, int which);
void GetRandomN(RCState &start, int N, int which);

}

#endif /* RubiksInstances_h */
