//
//  Transit.h
//  hog2 mac native demos
//
//  Created by Nathan Sturtevant on 5/20/18.
//  Copyright © 2018 NS Software. All rights reserved.
//

#ifndef Transit_h
#define Transit_h

#include <vector>
#include <stdio.h>
#include <unordered_map>

#include "Heuristic.h"
#include "Map2DEnvironment.h"
#include "TemplateAStar.h"

class Transit : public Heuristic<xyLoc> {
public:
	Transit(Map *m, int r1, int r2, int r3, bool computeNow);
	~Transit();
	double HCost(const xyLoc &a, const xyLoc &b) const;
	double IncrementalCompute();
	bool DoneComputing();
	void Draw(Graphics::Display &display) const;
	void Draw(Graphics::Display &display, const xyLoc &l1, const xyLoc &l2) const;
	double GetPercentTransit();
private:
	struct tp {
		int firstLoc, count;
	};
	int GetIndex(const xyLoc &l) const;
	int GetIndex(const xyLoc &l1, const xyLoc &l2) const;
	// For finding the transit points for a any point in the map
	std::unordered_map<xyLoc, tp> transitLookup;
	// The array actually holding the transit points
	std::vector<xyLoc> transitPoints;

	// All unique transit points
	std::unordered_map<xyLoc, int> transitPointHash;
	std::vector<xyLoc> uniqueTransitPoints;

	// APSP between transit points
	std::vector<float> distances;
	bool apsp;
	bool done;
	
	int currRad, nearRad, farRad;
	
	void DoStateInBlock(const xyLoc &s);
	void DoOneBlock();
	void GetNextSP();

	MapEnvironment *me;
	Map *m;
	xyLoc baseLoc;
//	std::vector<xyLoc> currStates;
	std::unordered_map<xyLoc, bool> nearStates;
	std::vector<xyLoc> farStates;
	TemplateAStar<xyLoc, tDirection, MapEnvironment> search;

	int nextX, nextY;
	int nextAPSP;
};

#endif /* Transit_h */
