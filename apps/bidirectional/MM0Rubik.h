//
//  MM0Rubik.h
//  hog2 glut
//
//  Created by Nathan Sturtevant on 1/19/16.
//  Copyright © 2016 University of Denver. All rights reserved.
//

#ifndef MM0Rubik_hpp
#define MM0Rubik_hpp

#include <stdio.h>
#include "RubiksCube.h"

#define NAMESPACE_OPEN(_name) namespace _name {
#define NAMESPACE_CLOSE(_name) }

namespace MM0 {
	void MM0(RubiksState &start, RubiksState &goal, const char *prefix1, const char *prefix2);
}

#endif /* MM0Rubik_hpp */
