/*
 *  $Id: aStar2.cpp
 *  hog2
 *
 *  Created by Nathan Sturtevant on 9/29/04.
 *  Modified by Nathan Sturtevant on 02/29/20.
 *
 * This file is part of HOG2. See https://github.com/nathansttt/hog2 for licensing information.
 *
 */

#include "AStar2.h"

using namespace GraphAbstractionConstants;

path *aStar2::GetPath(GraphAbstraction *aMap, node *from, node *to, reservationProvider *)
{
	if ((from == 0) || (to == 0) || (from == to))
		return 0;
  path *lastPath = 0;
  map = aMap;
	
  std::vector<unsigned int> eligibleNodeParents;
	
  lastPath = getAbstractPath(map->GetAbstractGraph(from->GetLabelL(kAbstractionLevel)),
														 from->GetNum(), to->GetLabelL(kParent),
														 eligibleNodeParents,
														 kTemporaryLabel,
														 to->GetNum());
  return lastPath;
}
	
