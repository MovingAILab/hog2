//
//  Map2DSectorAbstractionEnvironment.cpp
//  hog2 mac native demos
//
//  Created by Nathan Sturtevant on 5/23/18.
//  Copyright © 2018 NS Software. All rights reserved.
//

#include "Map2DSectorAbstractionEnvironment.h"

bool operator==(const abstractGridState &s1, const abstractGridState &s2)
{
	return s1.sector == s2.sector && s1.region == s2.region;
}

bool operator==(const abstractMove &a1, const abstractMove &a2)
{
	return a1.direction == a2.direction && a1.region == a2.region;
}

Map2DSectorAbstraction::Map2DSectorAbstraction(Map *m, int sectorSize)
{
	map = m;
	me = new MapEnvironment(map);
	msa = new MinimalSectorAbstraction(m, sectorSize);
}

Map2DSectorAbstraction::~Map2DSectorAbstraction()
{
	delete me;
	delete msa;
}

void Map2DSectorAbstraction::GetSuccessors(const abstractGridState &nodeID, std::vector<abstractGridState> &neighbors) const
{
	neighbors.clear();
	msa->GetNeighbors(nodeID.sector, nodeID.region, actions);
	abstractGridState g;
	for (tempEdgeData &a : actions)
	{
		g.sector = msa->GetAdjacentSector(nodeID.sector, a.direction);
		g.region = a.to;
		neighbors.push_back(g);
	}
}

void Map2DSectorAbstraction::GetActions(const abstractGridState &nodeID, std::vector<abstractMove> &acts) const
{
	acts.clear();
	msa->GetNeighbors(nodeID.sector, nodeID.region, actions);
	for (tempEdgeData &a : actions)
	{
		acts.push_back({a.direction, a.to});
	}
}


void Map2DSectorAbstraction::ApplyAction(abstractGridState &s, abstractMove a) const
{
	s.sector = msa->GetAdjacentSector(s.sector, a.direction);
	s.region = a.region;
}

xyLoc Map2DSectorAbstraction::GetState(const abstractGridState &loc) const
{
	unsigned int x, y;
	msa->GetXYLocation(loc.sector, loc.region, x, y);
	return xyLoc(x, y);
}

abstractGridState Map2DSectorAbstraction::GetAbstractState(const xyLoc &loc) const
{
	abstractGridState g;
	g.sector = msa->GetSector(loc.x, loc.y);
	g.region = msa->GetRegion(loc.x, loc.y);
	return g;
}

/** Heuristic value between two arbitrary nodes. **/
double Map2DSectorAbstraction::HCost(const abstractGridState &node1, const abstractGridState &node2) const
{
	xyLoc l1, l2;
	
	l1 = GetState(node1);
	l2 = GetState(node2);
	
	int dx = std::abs(l1.x-l2.x);
	int dy = std::abs(l1.y-l2.y);
	return std::max(dx, dy)+std::min(dx, dy)*(ROOT_TWO-1.0);
}


double Map2DSectorAbstraction::GCost(const abstractGridState &node1, const abstractGridState &node2) const
{
	return HCost(node1, node2);
}

double Map2DSectorAbstraction::GCost(const abstractGridState &node, const abstractMove &act) const
{
	assert(!"Not implemented");
}

bool Map2DSectorAbstraction::GoalTest(const abstractGridState &node, const abstractGridState &goal) const
{
	return (node.sector == goal.sector && node.region == goal.region);
}


uint64_t Map2DSectorAbstraction::GetStateHash(const abstractGridState &node) const
{
	return (node.sector<<16)|node.region;
}


uint64_t Map2DSectorAbstraction::GetActionHash(abstractMove act) const
{
	return (act.direction<<16)|act.region;
}


void Map2DSectorAbstraction::Draw(Graphics::Display &display) const
{
	msa->Draw(display);
}

void Map2DSectorAbstraction::Draw(Graphics::Display &display, const abstractGridState&g) const
{
	xyLoc l = GetState(g);
//	me->SetColor(GetColor());
//	me->Draw(display, l);
//	rgbColor c = GetColor();


	double px, py, t, rad;
	map->GetCoord(l.x, l.y, px, py, t, rad);
	
	//if (map->GetTerrainType(l.x, l.y) == kGround)
	{
		rad*=3;
//		Graphics::rect r;
//		r.left = px-rad;
//		r.top = py-rad;
//		r.right = px+rad;
//		r.bottom = py+rad;
		
		//s += SVGDrawCircle(l.x+0.5+1, l.y+0.5+1, 0.5, c);
//		display.FillCircle(r, GetColor());
		display.FillCircle({static_cast<float>(px), static_cast<float>(py)}, rad, GetColor());
		//stroke-width="1" stroke="pink" />
	}

	xyLoc curr = GetState(g);
	xyLoc next;
	GetSuccessors(g, nbr);
	me->SetColor(Colors::red);
	for (auto &n : nbr)
	{
		next = GetState(n);
		me->DrawLine(display, curr, next, 3);
	}
}

void Map2DSectorAbstraction::DrawLine(Graphics::Display &display, const abstractGridState &x, const abstractGridState &y, float width) const
{
	xyLoc l1 = GetState(x);
	xyLoc l2 = GetState(y);
	me->SetColor(GetColor());
	me->DrawLine(display, l1, l2, width);
}


