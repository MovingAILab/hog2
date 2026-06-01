//
//  RAJPS.cpp
//  hog2 glut
//
//  Created by Nathan Sturtevant on 2/2/16.
//  Copyright © 2016 University of Denver. All rights reserved.
//

#include "RAJPS.h"
#include <algorithm>

RAJPS::RAJPS()
{
	env = 0;
	weight = 1.0;
	jumpLimit = -1;
}

bool RAJPS::InitializeSearch(RoboticArm *env, const armAngles& from, const armAngles& to, std::vector<armAngles> &thePath)
{
	nodesExpanded = nodesTouched = 0;
	this->env = env;
	this->to = to;
	openClosedList.Reset();
	xyArmParent f;
	f.loc = from;
	f.parent = 0xFF;
	openClosedList.AddOpenNode(f, env->GetStateHash(from), 0, weight*env->HCost(from, to));
	if (from == to)
		return false;
	return true;
}

bool RAJPS::DoSingleSearchStep(std::vector<armAngles> &thePath)
{
	if (openClosedList.OpenSize() > 0)
	{
		nodesExpanded++;
		uint64_t next = openClosedList.Close();
		xyArmParent nextState = openClosedList.Lookat(next).data;
		//std::cout << nextState.loc << "\n";
		
		// if found goal
		if (nextState.loc == to)
		{
			//printf("Found optimal path cost %1.6f\n", openClosedList.Lookat(next).g);
			thePath.resize(0);
			ExtractPathToStartFromID(next, thePath);
			reverse(thePath.begin(), thePath.end());
			
			return true;
			// return path
		}
		
		successors.resize(0);
		GetJPSSuccessors(nextState, to);
		
		for (const auto &s : successors)
		{
			uint64_t theID;
			uint64_t hash = env->GetStateHash(s.s.loc);
			dataLocation l = openClosedList.Lookup(hash, theID);
			switch (l)
			{
				case kClosedList:
				{
					break;
				}
				case kNotFound:
				{
					openClosedList.AddOpenNode(s.s,
											   hash,
											   openClosedList.Lookup(next).g+s.cost,
											   weight*env->HCost(s.s.loc, to),
											   next);
					break;
				}
				case kOpenList:
				{
					if (openClosedList.Lookup(next).g+s.cost < openClosedList.Lookup(theID).g)
					{
						openClosedList.Lookup(theID).parentID = next;
						openClosedList.Lookup(theID).g = openClosedList.Lookup(next).g+s.cost;
						openClosedList.Lookup(theID).data.parent = s.s.parent;
						openClosedList.KeyChanged(theID);
					}
					break;
				}
			}
		}
	}
	else {
		return true;
	}
	// path not found
	return false;
}

void RAJPS::GetPath(RoboticArm *env, const armAngles &from, const armAngles &to, std::vector<armAngles> &path)
{
	InitializeSearch(env, from, to, path);
	while (DoSingleSearchStep(path) == false)
	{}
}

void RAJPS::GetPath(RoboticArm *env, const armAngles &from, const armAngles &to, std::vector<armRotations> &path)
{
	
}

void RAJPS::GetJPSSuccessors(const xyArmParent &s, const armAngles &goal)
{
	// write this and return g-cost too
	GetJPSSuccessors(s.loc.x, s.loc.y, s.parent, goal, 0);
}

bool RAJPS::Passable(armAngles a)
{
	return env->LegalState(a);
	//return env->GetMap()->GetTerrainType(x, y) == kGround;
}

void RAJPS::GetJPSSuccessors(xyArmParent s, uint8_t parent, const armAngles &goal, double cost)
{
	if (s.loc.angles = goal.angles)
	{
		successors.push_back(jpsArmSuccessor(s.loc, 0, cost));
		return;
	}
	nodesTouched++;
//	int w = env->GetMap()->GetMapWidth();
//	int h = env->GetMap()->GetMapHeight();
	bool n1 = false, s1 = false, e1 = false, w1 = false;
	if (parent&kN) // action that got me here
	{
		if (y != 0 && Passable(x, (y-1)))
		{
			bool a = false, b = false;
			if (x != 0 && !Passable(x-1, (y)) && Passable(x-1, (y-1)))
				a = true;
			if (x != w-1 && !Passable(x+1, (y)) && Passable(x+1, (y-1)))
				b = true;
			
			if (a && b)
			{
				successors.push_back(jpsSuccessor(x, y-1, armRotations(kNW|kNE), cost+1));
			}
			else if (a)
			{
				successors.push_back(jpsSuccessor(x, y-1, armRotations(kNW), cost+1));
			}
			else if (b)
			{
				successors.push_back(jpsSuccessor(x, y-1, armRotations(kNE), cost+1));
			}
			else {
				if (cost >= jumpLimit)
				{
					successors.push_back(jpsSuccessor(x, y-1, armRotations(kN), cost+1));
				}
				else {
					GetJPSSuccessors(x, y-1, kN, goal, cost+1);
				}
			}
			n1 = true;
		}
	}
	if (parent&kW) // action that got me here
	{
		if (x != 0 && Passable(x-1, (y)))
		{
			bool a = false, b = false;
			if (y != 0 && !Passable(x, (y-1)) && Passable(x-1, (y-1)))
				a = true;
			if (y != h-1 && !Passable(x, (y+1)) && Passable(x-1, (y+1)))
				b = true;
			
			if (a && b)
			{
				successors.push_back(jpsSuccessor(x-1, y, armRotations(kNW|kSW), cost+1));
			}
			else if (a)
			{
				successors.push_back(jpsSuccessor(x-1, y, armRotations(kNW), cost+1));
			}
			else if (b)
			{
				successors.push_back(jpsSuccessor(x-1, y, armRotations(kSW), cost+1));
			}
			else {
				if (cost >= jumpLimit)
				{
					successors.push_back(jpsSuccessor(x-1, y, armRotations(kW), cost+1));
				}
				else {
					GetJPSSuccessors(x-1, y, kW, goal, cost+1);
				}
				
			}
			e1 = true;
		}
	}
	if (parent&kS) // action that got me here
	{
		if (y != h-1 && Passable(x, (y+1)))
		{
			bool a = false, b = false;
			if (x != 0 && !Passable(x-1, (y)) && Passable(x-1, (y+1)))
				a = true;
			if (x != w-1 && !Passable(x+1, (y)) && Passable(x+1, (y+1)))
				b = true;
			
			if (a && b)
			{
				successors.push_back(jpsSuccessor(x, y+1, armRotations(kSE|kSW), cost+1));
			}
			else if (a)
			{
				successors.push_back(jpsSuccessor(x, y+1, armRotations(kSW), cost+1));
			}
			else if (b)
			{
				successors.push_back(jpsSuccessor(x, y+1, armRotations(kSE), cost+1));
			}
			else {
				if (cost >= jumpLimit)
				{
					successors.push_back(jpsSuccessor(x, y+1, armRotations(kS), cost+1));
				}
				else {
					GetJPSSuccessors(x, y+1, kS, goal, cost+1);
				}
			}
			s1 = true;
		}
	}
	if (parent&kE) // action that got me here
	{
		if (x != w-1 && Passable(x+1, (y)))
		{
			bool a = false, b = false;
			
			if (y != 0 && !Passable(x, (y-1)) && Passable(x+1, (y-1)))
				a = true;
			if (y != h-1 && !Passable(x, (y+1)) && Passable(x+1, (y+1)))
				b = true;
			
			if (a && b)
			{
				successors.push_back(jpsSuccessor(x+1, y, armRotations(kNE|kSE), cost+1));
			}
			else if (a)
			{
				successors.push_back(jpsSuccessor(x+1, y, armRotations(kNE), cost+1));
			}
			else if (b)
			{
				successors.push_back(jpsSuccessor(x+1, y, armRotations(kSE), cost+1));
			}
			else {
				if (cost >= jumpLimit)
				{
					successors.push_back(jpsSuccessor(x+1, y, armRotations(kE), cost+1));
				}
				else {
					GetJPSSuccessors(x+1, y, kE, goal, cost+1);
				}
			}
			w1 = true;
		}
	}
	if (parent&kNW)
	{
		if (x != 0 && y != 0 && Passable(x-1, (y-1)) && n1 && e1)
		{
			if (cost >= jumpLimit)
			{
				successors.push_back(jpsSuccessor(x-1, y-1, armRotations(kNW), cost+ROOT_TWO));
			}
			else {
				GetJPSSuccessors(x-1, y-1, kNW, goal, cost+ROOT_TWO);
			}
		}
	}
	if (parent&kNE)
	{
		if (x != w-1 && y != 0 && Passable(x+1, (y-1)) && n1 && w1)
		{
			if (cost >= jumpLimit)
			{
				successors.push_back(jpsSuccessor(x+1, y-1, armRotations(kNE), cost+ROOT_TWO));
			}
			else {
				GetJPSSuccessors(x+1, y-1, kNE, goal, cost+ROOT_TWO);
			}
		}
	}
	if (parent&kSW)
	{
		if (x != 0 && y != h-1 && Passable(x-1, (y+1)) && s1 && e1)
		{
			if (cost >= jumpLimit)
			{
				successors.push_back(jpsSuccessor(x-1, y+1, armRotations(kSW), cost+ROOT_TWO));
			}
			else {
				GetJPSSuccessors(x-1, y+1, kSW, goal, cost+ROOT_TWO);
			}
		}
	}
	if (parent&kSE)
	{
		if (x != w-1 && y != h-1 && Passable(x+1, (y+1)) && s1 && w1)
		{
			if (cost >= jumpLimit)
			{
				successors.push_back(jpsSuccessor(x+1, y+1, armRotations(kSE), cost+ROOT_TWO));
			}
			else {
				GetJPSSuccessors(x+1, y+1, kSE, goal, cost+ROOT_TWO);
			}
		}
	}
}

void RAJPS::ExtractPathToStartFromID(uint64_t node, std::vector<armAngles> &thePath)
{
	do {
		thePath.push_back(openClosedList.Lookup(node).data.loc);
		node = openClosedList.Lookup(node).parentID;
	} while (openClosedList.Lookup(node).parentID != node);
	thePath.push_back(openClosedList.Lookup(node).data.loc);
}

uint64_t RAJPS::GetNodesExpanded() const
{
	return nodesExpanded;
}

uint64_t RAJPS::GetNodesTouched() const
{
	return nodesTouched;
}

void RAJPS::LogFinalStats(StatCollection *stats)
{
	
}

void RAJPS::OpenGLDraw() const
{
	double transparency = 1.0;
	if (openClosedList.size() == 0)
		return;
	uint64_t top = -1;
	
	if (openClosedList.OpenSize() > 0)
	{
		top = openClosedList.Peek();
	}
	for (unsigned int x = 0; x < openClosedList.size(); x++)
	{
		const AStarOpenClosedData<xyArmParent> &data = openClosedList.Lookat(x);
		if (x == top)
		{
			env->SetColor(1.0, 1.0, 0.0, transparency);
			env->OpenGLDraw(data.data.loc);
		}
		if ((data.where == kOpenList) && (data.reopened))
		{
			env->SetColor(0.0, 0.5, 0.5, transparency);
			env->OpenGLDraw(data.data.loc);
		}
		else if (data.where == kOpenList)
		{
			env->SetColor(0.0, 1.0, 0.0, transparency);
			env->OpenGLDraw(data.data.loc);
		}
		else if ((data.where == kClosedList) && (data.reopened))
		{
			env->SetColor(0.5, 0.0, 0.5, transparency);
			env->OpenGLDraw(data.data.loc);
		}
		else if (data.where == kClosedList)
		{
			env->SetColor(1.0, 0.0, 0.0, transparency);
			env->OpenGLDraw(data.data.loc);
		}
	}
}

void RAJPS::OpenGLDraw(const RoboticArm *env) const {}
