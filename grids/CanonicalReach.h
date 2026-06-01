//
//  CanonicalReach.h
//  HOG2 Demos
//
//  Created by Nathan Sturtevant on 5/15/18.
//  Copyright © 2018 NS Software. All rights reserved.
//

#ifndef CanonicalReach_h
#define CanonicalReach_h

#include <stdio.h>

#include "Constraint.h"
#include "CanonicalGrid.h"
#include "TemplateAStar.h"
#include "Map2DEnvironment.h"

class CanonicalReach : public Constraint<xyLoc> {
public:
	CanonicalReach(CanonicalGrid::CanonicalGrid *m, bool computeNow);
	double IncrementalCompute();
	bool DoneComputing();
	bool ShouldNotGenerate(const xyLoc &start, const xyLoc &parent, const xyLoc &current, double gCost, const xyLoc &goal) const;
	void Draw(Graphics::Display &display) const;

private:
	void ComputeCanonicalReach(CanonicalGrid::xyLoc start);
	
	int x, y;
	bool done;
	CanonicalGrid::CanonicalGrid *me;
	std::vector<double> reach;
	std::vector<CanonicalGrid::xyLoc> neighbors;
	std::vector<CanonicalGrid::xyLoc> result;
	TemplateAStar<CanonicalGrid::xyLoc, CanonicalGrid::tDirection, CanonicalGrid::CanonicalGrid> search;

};


#endif /* CanonicalReach_h */
