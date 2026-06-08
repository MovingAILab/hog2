//
//  GridHeuristics.hpp
//  HOG2 ObjC
//
//  Created by Nathan Sturtevant on 2/6/23.
//  Copyright © 2023 MovingAI. All rights reserved.
//

#ifndef GridHeuristics_hpp
#define GridHeuristics_hpp

#include <stdio.h>
#include "Heuristic.h"
#include "Map2DEnvironment.h"
#include "TemplateAStar.h"

enum tEmbeddingFunction {
	kDifferential,
	kFastMap,
	kFMUniform,
	kFMShrink80
};

enum tPlacementScheme {
	kFurthest, // Baseline for both DH and FastMap
	kHeuristicError
};

enum tMetric {
	kL1, // additive
	kLINF, // max
};

// This environment has no heursitic. The g-cost is the
// original g-cost minus the existing heursitic.
// This gives the residual costs in the FastMap embedding.
class GridEmbeddingEnvironment : public MapEnvironment {
public:
	GridEmbeddingEnvironment(Map *m, Heuristic<xyLoc> *h)
	:MapEnvironment(m), h(h) {}
	double GCost(const xyLoc &node1, const xyLoc &node2) const
	{
		//assert(fequal(h->HCost(node1, node2), h->HCost(node2, node1)));
		double g = MapEnvironment::GCost(node1, node2) - h->HCost(node1, node2);
		// Rounding issues can lead g to be very close to 0, but outside tolerances
		return std::max(g, 0.0);
	}
	double HCost(const xyLoc &node1, const xyLoc &node2) const { return 0; }
private:
	Heuristic<xyLoc> *h;
};

class GridEmbedding : public Heuristic<xyLoc>
{
public:
	GridEmbedding(MapEnvironment *e, int numDimensions, tMetric m);
	bool AddDimension(tEmbeddingFunction e, tPlacementScheme p, Heuristic<xyLoc> *h = 0);
//	bool AddDimension(xyLoc l);
	double HCost(const xyLoc &a, const xyLoc &b) const;
	
	void Draw(Graphics::Display &disp) const;
	void Draw(Graphics::Display &disp, float tween) const;
	void Draw(Graphics::Display &disp, const xyLoc &l) const;
	void DrawPivots(Graphics::Display &disp) const;
	void DrawPivots(Graphics::Display &disp, const xyLoc &l) const;
	point3d Lookup(const xyLoc &l) const;
	//	void DrawAlternate(Graphics::Display &disp, const xyLoc &l) const;
private:
	void GetConnectedComponents();
	void SelectPivots(tPlacementScheme p, int component, Heuristic<xyLoc> *h);
	void Embed(tEmbeddingFunction e, int component);
	void GetDimensionLimits();
	xyLoc GetRandomState(int component);
	xyLoc GetFurthest(xyLoc l, TemplateAStar<xyLoc, tDirection, MapEnvironment> &astar, MapEnvironment *environment);
	xyLoc GetFurthest(const std::vector<xyLoc> &l, TemplateAStar<xyLoc, tDirection, MapEnvironment> &astar, MapEnvironment *environment);
	std::vector<double> embedding;
	MapEnvironment *env;
	tMetric metric;
	int currDim, maxDim;
	std::vector<std::vector<xyLoc>> pivots;
	std::vector<std::pair<float, float>> scale;
	GridEmbeddingEnvironment residual;
	TemplateAStar<xyLoc, tDirection, MapEnvironment> s1, s2;
	std::vector<xyLoc> path;
	std::vector<uint8_t> connectedComponents;
	int numConnectedComponents;
};

#endif /* GridHeuristics_hpp */
