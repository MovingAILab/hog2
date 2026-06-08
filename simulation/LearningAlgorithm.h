//
//  LearningAlgorithm.h
//  hog2 glut
//
//  Created by Nathan Sturtevant on 2/23/12.
//  Copyright (c) 2012 University of Denver. All rights reserved.
//

#ifndef hog2_glut_LearningAlgorithm_h
#define hog2_glut_LearningAlgorithm_h

#include "GenericSearchAlgorithm.h"

template <class state, class action, class environment>
class LearningAlgorithm : public GenericSearchAlgorithm<state, action, environment>
{
public:
	virtual double GetAmountLearned() = 0;
};

#endif
