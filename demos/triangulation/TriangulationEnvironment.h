//
//  TriangulationEnvironment.hpp
//  Triangulation
//
//  Created by Nathan Sturtevant on 6/11/24.
//  Copyright © 2024 NS Software. All rights reserved.
//

#ifndef TriangulationEnvironment_hpp
#define TriangulationEnvironment_hpp

#include <stdio.h>
#include <iostream>
#include "Graphics.h"


class Triangulation {
public:
	Triangulation();
	void Reset();
	void AddPoint(Graphics::point p) { points.push_back(p); }
	bool Triangulate();
	//bool InsideShape(Graphics::point p);
	void Draw(Graphics::Display &display) const;
private:
	bool InsideX(Graphics::point p1, Graphics::point p2, Graphics::point test);
	float DistToLine(Graphics::point l1, Graphics::point l2, Graphics::point p);
	bool Intersect(Graphics::point l1a, Graphics::point l1b, Graphics::point l2a, Graphics::point l2b);
	bool CanConnect(Graphics::point from, Graphics::point target);
	std::vector<Graphics::point> points;
	std::vector<Graphics::point> frontier;
	struct TriangulationData {
		Graphics::triangle t;
		int neighbor[3]; // p1-p2, p2-p3, p3-p1
	};
	std::vector<TriangulationData> triangulation;
	bool triangulated;
	int next;
};

#endif /* TriangulationEnvironment_hpp */
