//
//  BoundingBox.h
//  hog2 mac native demos
//
//  Created by Nathan Sturtevant on 5/16/18.
//  Copyright © 2018 NS Software. All rights reserved.
//

#ifndef BoundingBox_h
#define BoundingBox_h

#include <stdio.h>
#include "Constraint.h"
#include "Map2DEnvironment.h"
#include "TemplateAStar.h"
#include "CanonicalGrid.h"

class BoundingBox : public Constraint<xyLoc> {
public:
	BoundingBox(MapEnvironment *m, bool computeNow);
	~BoundingBox();
	double IncrementalCompute();
	bool DoneComputing();
	virtual bool ShouldNotGenerate(const xyLoc &start, const xyLoc &parent, const xyLoc &current, double gCost, const xyLoc &goal) const;
	void Draw(Graphics::Display &display, xyLoc start, tDirection dir) const;
private:
	struct BB {
		int minx, miny, maxx, maxy;
	};
	bool InBB(const xyLoc &, const BB &) const;
	BB ComputeBB(xyLoc start, CanonicalGrid::tDirection dir);
	int GetIndex(tDirection dir) const;
	int GetIndex(CanonicalGrid::tDirection dir) const;
	int x, y;
	bool done;
	TemplateAStar<CanonicalGrid::xyLoc, CanonicalGrid::tDirection, CanonicalGrid::CanonicalGrid> canAstar;
	MapEnvironment *me;
	CanonicalGrid::CanonicalGrid *grid;
	std::vector<BB> boundingBoxes;
	std::vector<xyLoc> neighbors;
	std::vector<CanonicalGrid::tDirection> actions;
	std::vector<xyLoc> result;
	std::vector<CanonicalGrid::xyLoc> path2;

	TemplateAStar<xyLoc, tDirection, MapEnvironment> search;
};

#endif /* BoundingBox_h */
