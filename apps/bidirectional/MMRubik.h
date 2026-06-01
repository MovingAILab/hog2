//
//  MM.h
//  hog2 glut
//
//  Created by Nathan Sturtevant on 8/31/15.
//  Copyright (c) 2015 University of Denver. All rights reserved.
//

#ifndef __hog2_glut__MM__
#define __hog2_glut__MM__

#include <stdio.h>
#include "RubiksCube.h"

#define NAMESPACE_OPEN(_name) namespace _name {
#define NAMESPACE_CLOSE(_name) }

namespace MM {
	enum heuristicType {
		kNone,
		k444,
		kSmall,
		k888,
		k1997,
		k839,
		k8210
	};
	
	void MM(RubiksState &start, RubiksState &goal, const char *prefix1, const char *prefix2,
			heuristicType h, const char *heuristicloc);
	void CompareIDA(RubiksState &start, RubiksState &goal, heuristicType h, const char *heuristicloc);
}

#endif /* defined(__hog2_glut__MM__) */
