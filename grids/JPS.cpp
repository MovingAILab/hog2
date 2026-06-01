//
//  JPS.cpp
//  hog2 glut
//
//  Created by Nathan Sturtevant on 12/28/15.
//  Copyright © 2015 University of Denver. All rights reserved.
//

#include "JPS.h"
#include <string>
#include <algorithm>
#include "Graphics.h"


/**
 * @brief Constructor for JPS.
 * Initializes the jump point map based on the terrain of the input map.
 * Marks jump points and boundaries.
 * @param m Pointer to the map object.
 */
JPS::JPS(Map *m)
{
	env = 0;
	weight = 1.0;
	jumpLimit = -1;

	w = m->GetMapWidth();
	h = m->GetMapHeight();
	
	jumpPoints.resize(m->GetMapWidth()*m->GetMapHeight());
	for (int y = 0; y < m->GetMapHeight(); y++)
	{
		for (int x = 0; x < m->GetMapWidth(); x++)
		{
			if (m->GetTerrainType(x, y) != kGround)
			{
				SetJumpPoint(x, y);
				if ((m->GetTerrainType(x+1, y) == kGround) &&
					(m->GetTerrainType(x, y+1) == kGround))
					SetJumpPoint(x+1, y+1);
				if ((m->GetTerrainType(x+1, y) == kGround) &&
					(m->GetTerrainType(x, y-1) == kGround))
					SetJumpPoint(x+1, y-1);
				if ((m->GetTerrainType(x-1, y) == kGround) &&
					(m->GetTerrainType(x, y+1) == kGround))
					SetJumpPoint(x-1, y+1);
				if ((m->GetTerrainType(x-1, y) == kGround) &&
					(m->GetTerrainType(x, y-1) == kGround))
					SetJumpPoint(x-1, y-1);
			}
			if (y == 0 || y == h-1 || x == 0 || x == w-1)
				SetJumpPoint(x, y);
		}
	}
//	for (int y = 0; y < m->GetMapHeight(); y++)
//	{
//		for (int x = 0; x < m->GetMapWidth(); x++)
//		{
//			if (m->GetTerrainType(x, y) != kGround)
//				printf("x");
//			else if (JumpPoint(x, y))
//				printf("j");
//			else
//				printf(" ");
//		}
//		printf("\n");
//	}
}

/**
 * @brief Initializes the search for a path from 'from' to 'to'.
 * Prepares the open/closed list and sets up the initial node.
 * @param env Pointer to the map environment.
 * @param from Start location.
 * @param to Goal location.
 * @param thePath Reference to the path vector to be filled.
 * @return False if the start and goal are the same, true otherwise.
 */
bool JPS::InitializeSearch(MapEnvironment *env, const xyLoc& from, const xyLoc& to, std::vector<xyLoc> &thePath)
{
	nodesExpanded = nodesTouched = 0;
	thePath.resize(0);
	this->env = env;
	this->to = to;
	Map *t = env->GetMap();
	//openClosedList.Reset();
	openClosedList.Reset(t->GetMapWidth()*t->GetMapHeight());
	xyLocParent f;
	f.loc = from;
	f.parent = 0xFF;
	openClosedList.AddOpenNode(f, env->GetStateHash(from), 0, weight*env->HCost(from, to));
	if (from == to)
		return false;
	return true;
}

/**
 * @brief Performs a single step of the JPS search.
 * Expands the next node, generates successors, and updates the open/closed list.
 * @param thePath Reference to the path vector to be filled if the goal is found.
 * @return True if the goal is found or the open list is empty, false otherwise.
 */
bool JPS::DoSingleSearchStep(std::vector<xyLoc> &thePath)
{
	if (openClosedList.OpenSize() > 0)
	{
		nodesExpanded++;
		uint64_t next = openClosedList.Close();
		xyLocParent nextState = openClosedList.Lookat(next).data;
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

/**
 * @brief Computes the full path from 'from' to 'to' using JPS.
 * Fills 'path' with the resulting sequence of locations.
 * @param env Pointer to the map environment.
 * @param from Start location.
 * @param to Goal location.
 * @param path Reference to the path vector to be filled.
 */
void JPS::GetPath(MapEnvironment *env, const xyLoc &from, const xyLoc &to, std::vector<xyLoc> &path)
{
	path.resize(0);
	InitializeSearch(env, from, to, path);
	while (DoSingleSearchStep(path) == false)
	{}
}

/**
 * @brief Placeholder for direction-based path output (not implemented).
 * @param env Pointer to the map environment.
 * @param from Start location.
 * @param to Goal location.
 * @param path Reference to the direction path vector to be filled.
 */
void JPS::GetPath(MapEnvironment *env, const xyLoc &from, const xyLoc &to, std::vector<tDirection> &path)
{
}

/**
 * @brief Finds all JPS successors for a given state.
 * Updates the 'successors' vector with valid moves.
 * @param s The current state with parent direction.
 * @param goal The goal location.
 */
void JPS::GetJPSSuccessors(const xyLocParent &s, const xyLoc &goal)
{
	// write this and return g-cost too
	GetJPSSuccessors(s.loc.x, s.loc.y, s.parent, goal, 0);
	// GetJPSSuccessors counts the current state plus all states generated but not put into the
	// successor list. So, we subtract 1 here to account for the current state, and add the final
	// successors, as they aren't counted
	nodesTouched--;
	nodesTouched += successors.size();
}

/**
 * @brief Checks if the cell at (x, y) is passable (ground terrain).
 * @param x X coordinate.
 * @param y Y coordinate.
 * @return True if the cell is passable, false otherwise.
 */
bool JPS::Passable(int x, int y)
{
	return env->GetMap()->GetTerrainType(x, y) == kGround;
}

/**
 * @brief Marks the cell at (x, y) as a jump point if within bounds.
 * @param x X coordinate.
 * @param y Y coordinate.
 */
void JPS::SetJumpPoint(int x, int y)
{
	if (x >= 0 && x < w && y >= 0 && y < h)
		jumpPoints[(y+0)*w+(x+0)] = true;
}

/**
 * @brief Returns true if the cell at (x, y) is a jump point.
 * @param x X coordinate.
 * @param y Y coordinate.
 * @return True if the cell is a jump point, false otherwise.
 */
bool JPS::JumpPoint(int x, int y)
{
	return jumpPoints[(y+0)*w+(x+0)];
}


/**
 * @brief Recursively generates JPS successors from (x, y) in the direction of 'parent'.
 * Adds valid successors to the 'successors' vector.
 * @param x X coordinate.
 * @param y Y coordinate.
 * @param parent Direction from which the node was reached.
 * @param goal The goal location.
 * @param cost The current path cost.
 */
void JPS::GetJPSSuccessors(int x, int y, uint8_t parent, const xyLoc &goal, double cost)
{
	if (goal.x == x && goal.y == y)
	{
		successors.push_back(jpsSuccessor(x, y, 0, cost));
		return;
	}
	nodesTouched++;
	bool n1 = false, s1 = false, e1 = false, w1 = false;
	if (parent&kN) // action that got me here
	{
		if (y != 0 && Passable(x, (y-1)))
		{
			bool a = false, b = false;
			uint8_t next = 0;
			if (JumpPoint(x, y-1))
			{
				if (x != 0 && !Passable(x-1, (y)) && Passable(x-1, (y-1)))
					next |= kNW;
					//a = true;
				if (x != w-1 && !Passable(x+1, (y)) && Passable(x+1, (y-1)))
					//b = true;
					next |= kNE;
			}
			
			if (next)
				successors.push_back(jpsSuccessor(x, y-1, tDirection(next), cost+1));
			else {
				if (cost >= jumpLimit)
				{
					successors.push_back(jpsSuccessor(x, y-1, tDirection(kN), cost+1));
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
			uint8_t next = 0;
			if (JumpPoint(x-1, y))
			{
				if (y != 0 && !Passable(x, (y-1)) && Passable(x-1, (y-1)))
					//a = true;
					next = kNW;
				if (y != h-1 && !Passable(x, (y+1)) && Passable(x-1, (y+1)))
					//b = true;
					next |= kSW;
			}

			if (next)
				successors.push_back(jpsSuccessor(x-1, y, tDirection(next), cost+1));
			else {
				if (cost >= jumpLimit)
				{
					successors.push_back(jpsSuccessor(x-1, y, tDirection(kW), cost+1));
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
			uint8_t next = 0;
			bool a = false, b = false;
			if (JumpPoint(x, y+1))
			{
				if (x != 0 && !Passable(x-1, (y)) && Passable(x-1, (y+1)))
					//a = true;
					next = kSW;
				if (x != w-1 && !Passable(x+1, (y)) && Passable(x+1, (y+1)))
					//b = true;
					next |= kSE;
			}
			
			if (next)
				successors.push_back(jpsSuccessor(x, y+1, tDirection(next), cost+1));
			else {
				if (cost >= jumpLimit)
				{
					successors.push_back(jpsSuccessor(x, y+1, tDirection(kS), cost+1));
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
			uint8_t next = 0;
			bool a = false, b = false;
			
			if (JumpPoint(x+1, y))
			{
				if (y != 0 && !Passable(x, (y-1)) && Passable(x+1, (y-1)))
					//a = true;
					next = kNE;
				if (y != h-1 && !Passable(x, (y+1)) && Passable(x+1, (y+1)))
					//b = true;
					next |= kSE;
			}
			
			if (next)
				successors.push_back(jpsSuccessor(x+1, y, tDirection(next), cost+1));
			else {
				if (cost >= jumpLimit)
				{
					successors.push_back(jpsSuccessor(x+1, y, tDirection(kE), cost+1));
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
				successors.push_back(jpsSuccessor(x-1, y-1, tDirection(kNW), cost+ROOT_TWO));
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
				successors.push_back(jpsSuccessor(x+1, y-1, tDirection(kNE), cost+ROOT_TWO));
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
				successors.push_back(jpsSuccessor(x-1, y+1, tDirection(kSW), cost+ROOT_TWO));
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
				successors.push_back(jpsSuccessor(x+1, y+1, tDirection(kSE), cost+ROOT_TWO));
			}
			else {
				GetJPSSuccessors(x+1, y+1, kSE, goal, cost+ROOT_TWO);
			}
		}
	}
}

/**
 * @brief Extracts the path from the goal node back to the start node.
 * Fills 'thePath' with the sequence of locations.
 * @param node The node ID to start backtracking from.
 * @param thePath Reference to the path vector to be filled.
 */
void JPS::ExtractPathToStartFromID(uint64_t node, std::vector<xyLoc> &thePath)
{
	do {
		thePath.push_back(openClosedList.Lookup(node).data.loc);
		node = openClosedList.Lookup(node).parentID;
	} while (openClosedList.Lookup(node).parentID != node);
	thePath.push_back(openClosedList.Lookup(node).data.loc);
}

/**
 * @brief Returns the number of nodes expanded during the search.
 * @return Number of nodes expanded.
 */
uint64_t JPS::GetNodesExpanded() const
{
	return nodesExpanded;
}

/**
 * @brief Returns the number of nodes touched during the search.
 * @return Number of nodes touched.
 */
uint64_t JPS::GetNodesTouched() const
{
	return nodesTouched;
}

/**
 * @brief Logs final statistics to the provided StatCollection (currently empty).
 * @param stats Pointer to the statistics collection object.
 */
void JPS::LogFinalStats(StatCollection *stats)
{
	
}

/**
 * @brief Generates an SVG string visualizing the search process.
 * @return SVG string representing the search.
 */
std::string JPS::SVGDraw()
{
	std::string s;
	double transparency = 1.0;
	if (openClosedList.size() == 0)
		return s;
	uint64_t top = -1;
	
	if (openClosedList.OpenSize() > 0)
	{
		top = openClosedList.Peek();
	}
	for (unsigned int x = 0; x < openClosedList.size(); x++)
	{
		const auto &data = openClosedList.Lookat(x);
		if (data.round != openClosedList.GetRound())
			continue;

		env->SetColor(1.0, 1.0, 1.0);
		s += env->SVGDrawLine(data.data.loc, openClosedList.Lookat(data.parentID).data.loc, 3);
		env->SetColor(0.0, 0.0, 0.0);
		s += env->SVGDrawLine(data.data.loc, openClosedList.Lookat(data.parentID).data.loc, 2);
	}
	for (unsigned int x = 0; x < openClosedList.size(); x++)
	{
		const auto &data = openClosedList.Lookat(x);
		if (data.round != openClosedList.GetRound())
			continue;

		//if (x == top)
		if (env->GetStateHash(data.data.loc) == top)
		{
			env->SetColor(1.0, 1.0, 0.0, transparency);
			s+=env->SVGDraw(data.data.loc);
		}
		else if ((data.where == kOpenList) && (data.reopened))
		{
			env->SetColor(0.0, 0.5, 0.5, transparency);
			s+=env->SVGDraw(data.data.loc);
		}
		else if (data.where == kOpenList)
		{
			env->SetColor(0.0, 1.0, 0.0, transparency);
			s+=env->SVGDraw(data.data.loc);
		}
		else if ((data.where == kClosedList) && (data.reopened))
		{
			env->SetColor(0.5, 0.0, 0.5, transparency);
			s+=env->SVGDraw(data.data.loc);
		}
		else if (data.where == kClosedList)
		{
			env->SetColor(1.0, 0.0, 0.0, transparency);
			s+=env->SVGDraw(data.data.loc);
		}
	}
	return s;
}


/**
 * @brief Draws the search process using OpenGL.
 */
//void JPS::OpenGLDraw() const
//{
//	double transparency = 1.0;
//	if (openClosedList.size() == 0)
//		return;
//	uint64_t top = -1;
//	
//	if (openClosedList.OpenSize() > 0)
//	{
//		top = openClosedList.Peek();
//	}
//	for (unsigned int x = 0; x < openClosedList.size(); x++)
//	{
//		const auto &data = openClosedList.Lookat(x);
//		if (data.round != openClosedList.GetRound())
//			continue;
//		glLineWidth(2);
//		env->SetColor(1.0, 1.0, 1.0);
//		env->GLDrawLine(data.data.loc, openClosedList.Lookat(data.parentID).data.loc);
//		glLineWidth(3);
//		env->SetColor(0.0, 0.0, 0.0);
//		env->GLDrawLine(data.data.loc, openClosedList.Lookat(data.parentID).data.loc);
//		glLineWidth(1);
//
//		if (x == top)
//		{
//			env->SetColor(1.0, 1.0, 0.0, transparency);
//			env->OpenGLDraw(data.data.loc);
//		}
//		if ((data.where == kOpenList) && (data.reopened))
//		{
//			env->SetColor(0.0, 0.5, 0.5, transparency);
//			env->OpenGLDraw(data.data.loc);
//		}
//		else if (data.where == kOpenList)
//		{
//			env->SetColor(0.0, 1.0, 0.0, transparency);
//			env->OpenGLDraw(data.data.loc);
//		}
//		else if ((data.where == kClosedList) && (data.reopened))
//		{
//			env->SetColor(0.5, 0.0, 0.5, transparency);
//			env->OpenGLDraw(data.data.loc);
//		}
//		else if (data.where == kClosedList)
//		{
//			env->SetColor(1.0, 0.0, 0.0, transparency);
//			env->OpenGLDraw(data.data.loc);
//		}
//	}
//}
//
///**
// * @brief Overload for OpenGLDraw (not implemented).
// * @param env Pointer to the map environment.
// */
//void JPS::OpenGLDraw(const MapEnvironment *env) const {}

/**
 * @brief Draws the search process using a Graphics::Display object.
 * @param disp Reference to the display object.
 */
void JPS::Draw(Graphics::Display &disp) const
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
		const auto &data = openClosedList.Lookat(x);
		if (data.round != openClosedList.GetRound())
			continue;

		if (jumpLimit != 0)
		{
			env->SetColor(Colors::red);
			env->DrawLine(disp, data.data.loc, openClosedList.Lookat(data.parentID).data.loc, 4);
			env->SetColor(Colors::white);
			env->DrawLine(disp, data.data.loc, openClosedList.Lookat(data.parentID).data.loc, 2);
		}
//		env->SetColor(0.0, 0.0, 0.0);
//		env->DrawLine(disp, data.data.loc, openClosedList.Lookat(data.parentID).data.loc);
		
		if (x == top)
		{
			env->SetColor(1.0, 1.0, 0.0, transparency);
			env->Draw(disp, data.data.loc);
		}
		else if ((data.where == kOpenList) && (data.reopened))
		{
			env->SetColor(0.0, 0.5, 0.5, transparency);
			env->Draw(disp, data.data.loc);
		}
		else if (data.where == kOpenList)
		{
			env->SetColor(0.0, 1.0, 0.0, transparency);
			env->Draw(disp, data.data.loc);
		}
		else if ((data.where == kClosedList) && (data.reopened))
		{
			env->SetColor(0.5, 0.0, 0.5, transparency);
			env->Draw(disp, data.data.loc);
		}
		else if (data.where == kClosedList)
		{
			env->SetColor(1.0, 0.0, 0.0, transparency);
			env->Draw(disp, data.data.loc);
		}
	}
}
