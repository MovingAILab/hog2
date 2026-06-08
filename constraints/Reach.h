//
//  Reach.h
//  HOG2 Demos
//
//  Created by Nathan Sturtevant on 5/15/18.
//  Copyright © 2018 NS Software. All rights reserved.
//

#ifndef Reach_h
#define Reach_h

#include <stdio.h>

#include "Constraint.h"
#include "Map2DEnvironment.h"
#include "TemplateAStar.h"

class Reach : public Constraint<xyLoc> {
public:
	Reach(MapEnvironment *m, bool computeNow);
	double IncrementalCompute();
	bool DoneComputing();
	virtual bool ShouldNotGenerate(const xyLoc &start, const xyLoc &parent, const xyLoc &current, double gCost, const xyLoc &goal) const;
	void Draw(Graphics::Display &display) const;
private:
	void ComputeReach(xyLoc start);
	
	int x, y;
	bool done;
	MapEnvironment *me;
	std::vector<double> reach;
	std::vector<xyLoc> neighbors;
	std::vector<xyLoc> result;

	TemplateAStar<xyLoc, tDirection, MapEnvironment> search;
};


#endif /* Reach_h */
