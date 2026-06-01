//
//  DynamicWeightedGrid.cpp
//  Dynamic Weighted Abstraction
//
//  Created by Nathan Sturtevant on 5/16/19.
//  Copyright © 2019 University of Denver. All rights reserved.
//

#include "DynamicWeightedGrid.h"
#include <cassert>
#include <algorithm>
#include <vector>
#include <random>

namespace DWG {
	
	DynamicWeightedGridEnvironment::DynamicWeightedGridEnvironment(const char *map)
	{
		FILE *f = fopen(map, "r");
		if (f == 0)
		{
			printf("Unable to open file '%s'\n", map);
			exit(0);
		}
		char format[32];
		int num = fscanf(f, "type %s\nheight %d\nwidth %d\nmap\n", format, &mHeight, &mWidth);
		if (num != 3)
		{
			printf("Bad file format: '%s'\n", map);
			exit(0);
		}
		terrain.resize(mWidth*mHeight);
		char what;
		for (uint16_t y = 0; y < mHeight; y++)
		{
			for (uint16_t x = 0; x < mWidth; x++)
			{
				fscanf(f, "%c", &what);
				terrain[y*mWidth+x] = (what-'A');
			}
			fscanf(f, "%c", &what);
			if (what != '\n')
				printf("Error loading\n");
		}
		srand(1);
		for (int x = 0; x < 20; x++)
			costs.push_back(1+0.2*x);
        std::random_device rd;
        std::mt19937 g(rd());
		// std::random_shuffle(costs.begin(), costs.end());
        std::shuffle(costs.begin(), costs.end(), g);
	}

	DynamicWeightedGridEnvironment::DynamicWeightedGridEnvironment(int width, int height)
	{
		mWidth = width;
		mHeight = height;
		
		terrain.resize(mWidth*mHeight);
		for (uint16_t y = 0; y < mHeight; y++)
		{
			for (uint16_t x = 0; x < mWidth; x++)
			{
				terrain[y*mWidth+x] = kGround;
			}
		}
		srand(1);
		for (int x = 0; x < 20; x++)
			costs.push_back(1+0.2*x);
        std::random_device rd;
        std::mt19937 g(rd());
        // std::random_shuffle(costs.begin(), costs.end());
        std::shuffle(costs.begin(), costs.end(), g);
	}
	
	void DynamicWeightedGridEnvironment::GetSuccessors(const xyLoc &nodeID, std::vector<xyLoc> &neighbors) const
	{
		neighbors.clear();
		if (nodeID.x > 0)
		{
			neighbors.push_back(nodeID);
			neighbors.back().x--;
		}
		if (nodeID.y > 0)
		{
			neighbors.push_back(nodeID);
			neighbors.back().y--;
		}
		if (nodeID.x+1 < mWidth)
		{
			neighbors.push_back(nodeID);
			neighbors.back().x++;
		}
		if (nodeID.y+1 < mHeight)
		{
			neighbors.push_back(nodeID);
			neighbors.back().y++;
		}
		if (nodeID.y > 0 && nodeID.x > 0)
		{
			neighbors.push_back(nodeID);
			neighbors.back().x--;
			neighbors.back().y--;
		}
		if (nodeID.y > 0 && nodeID.x+1 < mWidth)
		{
			neighbors.push_back(nodeID);
			neighbors.back().x++;
			neighbors.back().y--;
		}
		if (nodeID.y+1 < mHeight && nodeID.x+1 < mWidth)
		{
			neighbors.push_back(nodeID);
			neighbors.back().x++;
			neighbors.back().y++;
		}
		if (nodeID.y+1 < mHeight && nodeID.x > 0)
		{
			neighbors.push_back(nodeID);
			neighbors.back().x--;
			neighbors.back().y++;
		}
	}
	
	void DynamicWeightedGridEnvironment::GetActions(const xyLoc &nodeID, std::vector<tDirection> &actions) const
	{
		actions.clear();
		if (nodeID.x > 0)
		{
			actions.push_back(kW);
		}
		if (nodeID.y > 0)
		{
			actions.push_back(kN);
		}
		if (nodeID.x+1 < mWidth)
		{
			actions.push_back(kE);
		}
		if (nodeID.y+1 < mHeight)
		{
			actions.push_back(kS);
		}
		if (nodeID.y > 0 && nodeID.x > 0)
		{
			actions.push_back(kNW);
		}
		if (nodeID.y > 0 && nodeID.x+1 < mWidth)
		{
			actions.push_back(kNE);
		}
		if (nodeID.y+1 < mHeight && nodeID.x+1 < mWidth)
		{
			actions.push_back(kSE);
		}
		if (nodeID.y+1 < mHeight && nodeID.x > 0)
		{
			actions.push_back(kSW);
		}

	}
	
	void DynamicWeightedGridEnvironment::ApplyAction(xyLoc &s, tDirection a) const
	{
		switch (a)
		{
			case kN: s.y-=1; break;
			case kS: s.y+=1; break;
			case kE: s.x+=1; break;
			case kW: s.x-=1; break;
			case kNW: s.y-=1; s.x-=1; break;
			case kSW: s.y+=1; s.x-=1; break;
			case kNE: s.y-=1; s.x+=1; break;
			case kSE: s.y+=1; s.x+=1; break;
			default: break;
		}
	}
	
	bool DynamicWeightedGridEnvironment::InvertAction(tDirection &a) const
	{
		switch (a)
		{
			case kN: a = kS; break;
			case kNE: a = kSW; break;
			case kE: a = kW; break;
			case kSE: a = kNW; break;
			case kS: a = kN; break;
			case kSW: a = kNE; break;
			case kW: a = kE; break;
			case kNW: a = kSE; break;
			default: break;
		}
		return true;
	}
	
	tDirection DynamicWeightedGridEnvironment::GetAction(const xyLoc &s1, const xyLoc &s2) const
	{
		assert(!"Not implemented");
		return tDirection();
		
	}
	
	/** Heuristic value between two arbitrary nodes. **/
	double DynamicWeightedGridEnvironment::HCost(const xyLoc &l1, const xyLoc &l2) const
	{
		const double DIAGONAL_COST = M_SQRT2;
		double a = ((l1.x>l2.x)?(l1.x-l2.x):(l2.x-l1.x));
		double b = ((l1.y>l2.y)?(l1.y-l2.y):(l2.y-l1.y));
//		if (a == 0 && b == 0)
//			return 0;
		//return costs[terrain[l1.y*mWidth+l1.x]]*0.5+0.5*costs[terrain[l2.y*mWidth+l2.x]]-1+
		return ((a>b)?(b*DIAGONAL_COST+a-b):(a*DIAGONAL_COST+b-a));
	}
	
	double DynamicWeightedGridEnvironment::GCost(const xyLoc &node1, const xyLoc &node2) const
	{
		double base = 1.0;
		if (node1.x != node2.x && node1.y != node2.y)
			base = M_SQRT2;
		double c1 = costs[terrain[node2.y*mWidth+node2.x]];
		double c2 = costs[terrain[node1.y*mWidth+node1.x]];
		double result = base*0.5*(c1+c2);
		return result;
	}
	
	double DynamicWeightedGridEnvironment::GCost(const xyLoc &node, const tDirection &act) const
	{
		xyLoc tmp = node;
		ApplyAction(tmp, act);
		return GCost(node, tmp);
	}

	bool DynamicWeightedGridEnvironment::GoalTest(const xyLoc &node, const xyLoc &goal) const
	{
		return node == goal;
	}
	
	uint64_t DynamicWeightedGridEnvironment::GetStateHash(const xyLoc &node) const
	{
		return node.y*mWidth+node.x;
	}
	
	uint64_t DynamicWeightedGridEnvironment::GetActionHash(tDirection act) const
	{
		return act;
	}
	
	void DynamicWeightedGridEnvironment::Draw(Graphics::Display &display) const
	{
		
	}
	
	void DynamicWeightedGridEnvironment::Draw(Graphics::Display &display, const xyLoc &l) const
	{
		float x, y, r;
		GetCoordinate(l, x, y, r);
		display.FillCircle({x, y}, r, GetColor());
						   //DynamicWeightedGrid<1>::GetTerrainColor((TerrainType)terrain[l.y*mWidth+l.x]));
	}
	
	void DynamicWeightedGridEnvironment::GetCoordinate(const xyLoc &l, float &x, float &y, float &radius) const
	{
		float _scale, xOffset, yOffset;
		if (mHeight > mWidth)
		{
			_scale = 2.0/(float)(mHeight);
			xOffset = (2.0-mWidth*_scale)*0.5;
			yOffset = 0;
		}
		else {
			_scale = 2.0/(float)(mWidth);
			yOffset = (2.0-mHeight*_scale)*0.5;
			xOffset = 0;
		}
		float epsilon = _scale/2.0;
		x = -1+l.x*_scale+epsilon+xOffset;
		y = -1+l.y*_scale+epsilon+yOffset;
		//	x = (2*_x-width)*_scale+epsilon;
		//	y = (2*_y-height)*_scale+epsilon;
		radius = epsilon;
	}
}
