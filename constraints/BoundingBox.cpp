//
//  BoundingBox.cpp
//  hog2 mac native demos
//
//  Created by Nathan Sturtevant on 5/16/18.
//  Copyright © 2018 NS Software. All rights reserved.
//

#include "BoundingBox.h"
#include <deque>

BoundingBox::BoundingBox(MapEnvironment *m, bool computeNow)
{
	x = y = 0;
	done = false;
	me = m;
	grid = new CanonicalGrid::CanonicalGrid(m->GetMap());
	boundingBoxes.resize(m->GetMap()->GetMapWidth()*m->GetMap()->GetMapHeight()*8);
}

BoundingBox::~BoundingBox()
{
	delete grid;
}

double BoundingBox::IncrementalCompute()
{
	if (done)
		return 1.0;
	
	Map *m = me->GetMap();
	
	if (m->GetTerrainType(x, y) == kGround)
	{
		xyLoc l(x, y);
		grid->GetActions({l.x, l.y}, actions);
		for (auto a : actions)
		{
			boundingBoxes[me->GetStateHash(l)*8+GetIndex(a)] = ComputeBB(l, a);
		}
//		boundingBoxes[me->GetStateHash(l)*8+GetIndex(kS)] = ComputeBB(l, CanonicalGrid::kS);
//		boundingBoxes[me->GetStateHash(l)*8+GetIndex(kE)] = ComputeBB(l, CanonicalGrid::kE);
//		boundingBoxes[me->GetStateHash(l)*8+GetIndex(kW)] = ComputeBB(l, CanonicalGrid::kW);
//		boundingBoxes[me->GetStateHash(l)*8+GetIndex(kNE)] = ComputeBB(l, CanonicalGrid::kNE);
//		boundingBoxes[me->GetStateHash(l)*8+GetIndex(kSE)] = ComputeBB(l, CanonicalGrid::kSE);
//		boundingBoxes[me->GetStateHash(l)*8+GetIndex(kNW)] = ComputeBB(l, CanonicalGrid::kNW);
//		boundingBoxes[me->GetStateHash(l)*8+GetIndex(kSW)] = ComputeBB(l, CanonicalGrid::kSW);
	}
	x++;
	if (x >= m->GetMapWidth())
	{
		y++;
		x = 0;
	}
	if (y >= m->GetMapHeight())
	{
		x = y = 0;
		done = true;
		//		for (auto &f : reach)
		//			printf("%1.1f ", f);
		return 1.0;
	}
	
	// Not yet complete
	return double(y*m->GetMapWidth()+x)/double(m->GetMapWidth()*m->GetMapHeight());

}

bool BoundingBox::DoneComputing()
{
	return done == true;
}

bool BoundingBox::ShouldNotGenerate(const xyLoc &start, const xyLoc &parent, const xyLoc &current, double gCost, const xyLoc &goal) const
{
	tDirection dir = me->GetAction(parent, current);
	int index = GetIndex(dir);
	const BB &b = boundingBoxes[me->GetStateHash(parent)*8+index];
	std::cout << "BB for " << parent << " and action " << dir << " has bb ";
	std::cout << b.minx << ", " << b.miny << ", " << b.maxx << ", " << b.maxx << " with goal " << goal << " is ";
	bool in = InBB(goal, b);
	if (in)
		std::cout << "valid for generation\n";
	else
		std::cout << "invalid for generation (pruning)\n";
	return in == false;
}

void BoundingBox::Draw(Graphics::Display &display, xyLoc start, tDirection dir) const
{
	if (start.x >= me->GetMap()->GetMapWidth() || start.y >= me->GetMap()->GetMapHeight())
		return;
	int index = GetIndex(dir);
	const BB& bb = boundingBoxes[me->GetStateHash(start)*8+index];

	double t, l, r, b, z, rad, tmp;
	me->GetMap()->GetCoord(bb.minx, bb.miny, l, t, z, rad);
	me->GetMap()->GetCoord(bb.maxx, bb.maxy, r, b, z, rad);
	display.FrameRect({static_cast<float>(l-rad), static_cast<float>(t-rad), static_cast<float>(r+rad), static_cast<float>(b+rad)}, Colors::orange, 1);
}

bool BoundingBox::InBB(const xyLoc &s, const BB &b) const
{
	bool res = (s.x >= b.minx && s.y >= b.miny && s.x <= b.maxx && s.y <= b.maxy);
	
	return res;
}

BoundingBox::BB BoundingBox::ComputeBB(xyLoc start, CanonicalGrid::tDirection dir)
{
	std::string str;
	int minx = 1000, miny = 1000;
	int maxx = 0, maxy = 0;
	CanonicalGrid::xyLoc cStart;
	cStart.x = start.x;
	cStart.y = start.y;
	
	canAstar.SetWeight(1.0);
	canAstar.SetStopAfterGoal(false);
	canAstar.GetPath(grid, cStart, cStart, path2);
	
	std::deque<CanonicalGrid::xyLoc> queue;

	grid->ApplyAction(cStart, dir);
	if (me->GetMap()->GetTerrainType(cStart.x, cStart.y) != kGround)
		return {0,0,0,0};
	queue.push_back(cStart);
	std::vector<CanonicalGrid::xyLoc> v;
	std::vector<xyLoc> l;
	while (!queue.empty())
	{
		CanonicalGrid::xyLoc next = queue.front();
		queue.pop_front();
		
//		grid->SetColor(0.0, 0.5, 1.0);
//		me->SetColor(0.0, 0.5, 1.0);
		AStarOpenClosedDataWithF<CanonicalGrid::xyLoc> data;
		if (canAstar.GetClosedItem(next, data))
		{
			grid->GetSuccessors(data.data, v);
//			grid->OpenGLDraw(data.data);
			
			if (data.data.x > maxx)
				maxx = data.data.x;
			if (data.data.x < minx)
				minx = data.data.x;
			if (data.data.y > maxy)
				maxy = data.data.y;
			if (data.data.y < miny)
				miny = data.data.y;
			
			for (auto &s : v)
			{
				double g;
				bool success = canAstar.GetClosedListGCost(s, g);
				//printf("Cost in closed: %f; cost through parent %f (%f+%f)\n", g, data.g+grid->GCost(data.data, s), data.g, grid->GCost(data.data, s));
				if (success && fequal(g, data.g+grid->GCost(data.data, s)))
					queue.push_back(s);
			}
		}
	}
//	std::cout << dir << " from " << start << " gets to (" << minx << ", " << miny << ", " << maxx << ", " << maxy << ")\n";
	return {minx, miny, maxx, maxy};
}

int BoundingBox::GetIndex(tDirection dir) const
{
	switch (dir)
	{
		case kN: return 0;
		case kS: return 1;
		case kE: return 2;
		case kW: return 3;
		case kNE: return 4;
		case kSE: return 5;
		case kNW: return 6;
		case kSW: return 7;
	}
	assert(!"Should not get here");
}

int BoundingBox::GetIndex(CanonicalGrid::tDirection dir) const
{
	switch (dir)
	{
		case CanonicalGrid::kN: return 0;
		case CanonicalGrid::kS: return 1;
		case CanonicalGrid::kE: return 2;
		case CanonicalGrid::kW: return 3;
		case CanonicalGrid::kNE: return 4;
		case CanonicalGrid::kSE: return 5;
		case CanonicalGrid::kNW: return 6;
		case CanonicalGrid::kSW: return 7;
	}
	assert(!"Should not get here");
}
