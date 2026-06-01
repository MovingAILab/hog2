//
//  STPInstances.h
//  hog2 glut
//
//  Created by Nathan Sturtevant on 2/27/18.
//  Copyright © 2018 University of Denver. All rights reserved.
//

#ifndef STPInstances_h
#define STPInstances_h

#include "MNPuzzle.h"

namespace STP {

	MNPuzzleState<4, 4> GetKorfInstance(int which);
	MNPuzzleState<4, 4> GetRandomInstance(int walkLength);

}
#endif /* STPInstances_h */
