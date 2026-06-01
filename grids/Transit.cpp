//
//  Transit.cpp
//  hog2 mac native demos
//
//  Created by Nathan Sturtevant on 5/20/18.
//  Copyright © 2018 NS Software. All rights reserved.
//

#include "Transit.h"


Transit::Transit(Map *m, int r1, int r2, int r3, bool computeNow)
:m(m), currRad(r1), nearRad(r2), farRad(r3)
{
	nextX = nextY = 0;
	done = false;
	apsp = false;
	me = new MapEnvironment(m);
	me->SetEightConnected();
	me->SetDiagonalCost(ROOT_TWO);
	if (computeNow)
	{
		while (!DoneComputing())
		{
			IncrementalCompute();
		}
	}
}

Transit::~Transit()
{
	delete me;
}

double Transit::HCost(const xyLoc &a, const xyLoc &b) const
{
	return 0;
}

double Transit::IncrementalCompute()
{
	if (done)
		return 1.0;

	if (!apsp)
	{
		baseLoc = xyLoc(nextX, nextY);
		DoOneBlock();
		
		nextX+=currRad;
		if (nextX >= m->GetMapWidth())
		{
			nextY+=currRad;
			nextX = 0;
		}
		if (nextY >= m->GetMapHeight())
		{
			nextX = nextY = 0;
			apsp = false;
			done = true;
			nextAPSP = 0;
			distances.resize(uniqueTransitPoints.size()*uniqueTransitPoints.size());
			return 1.0;
		}
		// Not yet complete
		return double(nextY*m->GetMapWidth()+nextX)/double(m->GetMapWidth()*m->GetMapHeight());
	}
	
	if (apsp)
	{
		GetNextSP();
		nextAPSP++;
		if (nextAPSP == uniqueTransitPoints.size())
		{
			nextX = nextY = 0;
			nextAPSP = 0;
			done = true;
			return 1.0;
		}
		return (double)nextAPSP/(double)uniqueTransitPoints.size();
	}
	return 0.0;
}

bool Transit::DoneComputing()
{
	return done;
}

double Transit::GetPercentTransit()
{
	double val = uniqueTransitPoints.size();
	int ground = 0;
	for (int x = 0; x < m->GetMapWidth(); x++)
		for (int y = 0; y < m->GetMapHeight(); y++)
			ground += (m->GetTerrainType(x, y)==kGround)?1:0;
	return val/ground;
}


void Transit::Draw(Graphics::Display &display) const
{
	if (done == false && apsp == false)
	{
		me->SetColor(Colors::yellow);
		// For each location in the local radius
		for (int y = 0; y < currRad; y++)
		{
			for (int x = 0; x < currRad; x++)
			{
				xyLoc tmp(baseLoc.x+x, baseLoc.y+y);
				me->DrawAlternate(display, tmp);
			}
		}
		
		me->SetColor(Colors::blue);
		for (const xyLoc &l : farStates)
			me->Draw(display, l);
		
		me->SetColor(Colors::green);
		for (auto i = nearStates.begin(); i != nearStates.end(); i++)
			me->Draw(display, i->first);
	}

	me->SetColor(Colors::red);
	for (const xyLoc &l : uniqueTransitPoints)
		me->Draw(display, l);

	if (done == false && apsp == true)
	{
		me->SetColor(Colors::green);
		me->Draw(display, uniqueTransitPoints[nextAPSP]);
	}
}

void Transit::Draw(Graphics::Display &display, const xyLoc &l1, const xyLoc &l2) const
{
	xyLoc bestl1, bestl2;
	float totalDistance = 100000;
	tp i1, i2;
	auto tmp = transitLookup.find(l1);
	if (tmp != transitLookup.end())
	{
		i1 = tmp->second;
	}
	else {
		printf("Lookup failure(%s, %d)\n", __FILE__, __LINE__);
		return; // failure
	}
	tmp = transitLookup.find(l2);
	if (tmp != transitLookup.end())
	{
		i2 = tmp->second;
	}
	else {
		printf("Lookup failure(%s, %d)\n", __FILE__, __LINE__);
		return; // failure
	}
	
	for (int a1 = 0; a1 < i1.count; a1++)
	{
		for (int a2 = 0; a2 < i2.count; a2++)
		{
			auto tp1 = transitPoints[i1.firstLoc+a1];
			auto tp2 = transitPoints[i2.firstLoc+a2];
			int h1, h2;
			auto tmp2 = transitPointHash.find(tp1);
			if (tmp2 != transitPointHash.end())
			{
				h1 = tmp2->second;
			}
			else {
				printf("Lookup failure(%s, %d)\n", __FILE__, __LINE__);
				return;
			}
			tmp2 = transitPointHash.find(tp2);
			if (tmp2 != transitPointHash.end())
			{
				h2 = tmp2->second;
			}
			else {
				printf("Lookup failure(%s, %d)\n", __FILE__, __LINE__);
				return;
			}
			auto u1 = uniqueTransitPoints[h1];
			auto u2 = uniqueTransitPoints[h2];
			int index = GetIndex(u1, u2);
			float newDist = distances[index];
			if (newDist < totalDistance)
			{
				totalDistance = newDist;
				auto tmp3 = transitPointHash.find(transitPoints[i1.firstLoc+a1]);
				if (tmp3 != transitPointHash.end())
				{
					bestl1 = uniqueTransitPoints[tmp3->second];
				}
				else {
					printf("Lookup failure(%s, %d)\n", __FILE__, __LINE__);
					return;
				}
				auto tmp4 = transitPointHash.find(transitPoints[i2.firstLoc+a2]);
				if (tmp4 != transitPointHash.end())
				{
					bestl2 = uniqueTransitPoints[tmp4->second];
				}
				else {
					printf("Lookup failure(%s, %d)\n", __FILE__, __LINE__);
					return;
				}
			}
		}
	}
	me->SetColor(Colors::blue);
	for (int a1 = 0; a1 < i1.count; a1++)
		me->Draw(display, transitPoints[i1.firstLoc+a1]);
	for (int a2 = 0; a2 < i2.count; a2++)
		me->Draw(display, transitPoints[i2.firstLoc+a2]);

	me->SetColor(Colors::blue);
	me->DrawLine(display, bestl1, bestl2);
}

//struct tp {
//	int firstLoc, count;
//};
int Transit::GetIndex(const xyLoc &l) const
{
	return l.y*m->GetMapWidth() + l.y;
}

int Transit::GetIndex(const xyLoc &l1, const xyLoc &l2) const
{
	auto i = transitPointHash.find(l1);
	int i1 = i->second;
	i = transitPointHash.find(l2);
	int i2 = i->second;
//	int i1 = transitPointHash[l1];
//	int i2 = transitPointHash[l2];
	return i1*uniqueTransitPoints.size()+i2;
}

// For finding the transit points for a any point in the map
//std::unordered_map<xyLoc, tp> transitLookup;
//// The array actually holding the transit points
//std::vector<xyLoc> transitPoints;

// All unique transit points
//std::vector<xyLoc> uniqueTransitPoints;
//std::vector<float> distances;

void Transit::DoStateInBlock(const xyLoc &s)
{
//	if (m->GetTerrainType(s.x, s.y) != kGround)
//		return;
	
//	hd.clear();
//	farStates.clear();
	
	std::vector<xyLoc> v;
	search.SetStopAfterGoal(false);
	ZeroHeuristic<xyLoc> z;
	search.SetHeuristic(&z);
	bool init = false;
//	search.InitializeSearch(me, s, s, v);
	for (int y = 0; y < currRad; y++)
	{
		for (int x = 0; x < currRad; x++)
		{
			// Find transit points at near radius
			xyLoc tmp(s.x+x, s.y+y);
			if (me->GetMap()->GetTerrainType(s.x+x, s.y+y) != kGround)
				continue;
			
			if (!init)
			{
				search.InitializeSearch(me, tmp, tmp, v);
				init = true;
			}
			else {
				search.AddAdditionalStartState(tmp);
			}
			//if (x != 0 || y != 0)
		}
	}
	if (!init) // no states to handle
		return;
	
	// Search out to find all states at far radius
	// [more complicated that it seems, because they may not all reachable]
	while (true)
	{
		double cost;
		bool success = search.GetOpenListGCost(search.CheckNextNode(), cost);
		// TODO: The 2 factor will fail in a maze where there are long paths; need better check
		if (!success || cost > 2*ROOT_TWO*farRad+1)
			break;
		if (search.DoSingleSearchStep(v))
			break;
		if (search.GetNumOpenItems() == 0)
			break;
	}
	std::unordered_map<xyLoc, bool> newTransitPoints;
	for (xyLoc &t : farStates)
	{
		search.ExtractPathToStart(t, v);
		std::reverse(v.begin(), v.end());
		for (const xyLoc &l : v)
		{
			// only need the first state on the path
			if (nearStates.find(l) != nearStates.end())
			{
				// l is a transit point
				if (transitPointHash.find(l) == transitPointHash.end())
				{
					transitPointHash[l] = uniqueTransitPoints.size();
					uniqueTransitPoints.push_back(l);
					newTransitPoints[l] = true;
				}
				break;
			}
		}
	}
	transitLookup[s].firstLoc = transitPoints.size();
	transitLookup[s].count = newTransitPoints.size();
	for (auto i = newTransitPoints.begin(); i != newTransitPoints.end(); i++)
	{
		transitPoints.push_back(i->first);
	}
}

void Transit::DoOneBlock()
{
	farStates.clear();
	nearStates.clear();
	
	// center of computation (may be off map)
	xyLoc s(baseLoc);
	s.x += (currRad+1)/2;
	s.y += (currRad+1)/2;

	
	// Get far states
	// TODO: make this more efficient - writing quickly; needs to be correct
	for (int y = 0; y < m->GetMapHeight(); y++)
	{
		if (y < s.y-farRad || y > s.y+farRad)
			continue;
		for (int x = 0; x < m->GetMapWidth(); x++)
		{
			if ((x < s.x-farRad) || (x > s.x+farRad))
				continue;
			
			if (m->GetTerrainType(x, y) == kGround &&
				((x == s.x-farRad) ||
				 (x == s.x+farRad) ||
				 (y == s.y-farRad) ||
				 (y == s.y+farRad)))
			{
				farStates.push_back(xyLoc(x, y));
			}
		}
	}
	// TODO: make this more efficient - writing quickly; needs to be correct
	for (int y = 0; y < m->GetMapHeight(); y++)
	{
		if (y < s.y-nearRad || y > s.y+nearRad)
			continue;
		for (int x = 0; x < m->GetMapWidth(); x++)
		{
			if ((x < s.x-nearRad) || (x > s.x+nearRad))
				continue;
			
			// only need the first state on the path
			if (m->GetTerrainType(x, y) == kGround && (x == s.x-nearRad || x == s.x+nearRad || y == s.y-nearRad || y == s.y+nearRad))
			{
				nearStates[xyLoc(x, y)] = true;
			}

		}
	}
	
	// For each location in the local radius
	DoStateInBlock(baseLoc);
//	for (int y = 0; y < currRad; y++)
//	{
//		for (int x = 0; x < currRad; x++)
//		{
//			// Find transit points at near radius
//			DoStateInBlock(xyLoc(baseLoc.x+x, baseLoc.y+y));
//		}
//	}
}

void Transit::GetNextSP()
{
	std::vector<xyLoc> path;
	search.SetHeuristic(0);
	search.SetStopAfterGoal(false);
	search.GetPath(me, uniqueTransitPoints[nextAPSP], uniqueTransitPoints[nextAPSP], path);
	for (int x = 0; x < search.GetNumItems(); x++)
	{
		distances[GetIndex(uniqueTransitPoints[nextAPSP], search.GetItem(x).data)] = search.GetItem(x).g;
		distances[GetIndex(search.GetItem(x).data, uniqueTransitPoints[nextAPSP])] = search.GetItem(x).g;
	}
}


//xyLoc baseLoc;
//std::vector<xyLoc> currStates;
//std::vector<xyLoc> nearStates;
//std::vector<xyLoc> farStates;

