//
//  PancakeInstances.h
//  hog2 glut
//
//  Created by Nathan Sturtevant on 6/23/17.
//  Copyright © 2017 University of Denver. All rights reserved.
//

#ifndef PancakeInstances_h
#define PancakeInstances_h

#include "PancakePuzzle.h"

template <int N>
bool GetPancakeInstance(PancakePuzzleState<N> &s, int instance)
{
	return false;
}

template <>
bool GetPancakeInstance(PancakePuzzleState<16> &s, int instance);
template <>
bool GetPancakeInstance(PancakePuzzleState<20> &s, int instance);
template <>
bool GetPancakeInstance(PancakePuzzleState<24> &s, int instance);
template <>
bool GetPancakeInstance(PancakePuzzleState<28> &s, int instance);

#endif /* PancakeInstances_h */
