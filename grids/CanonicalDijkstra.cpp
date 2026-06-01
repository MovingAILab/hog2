//
//  CanonicalDijkstra.cpp
//  hog2 glut
//
//  Created by Nathan Sturtevant on 1/31/16.
//  Copyright © 2016 University of Denver. All rights reserved.
//

#include "CanonicalDijkstra.h"

const double gDiagCost = 1.5;

CanonicalDijkstra::CanonicalDijkstra()
{
	jumpLimit = 16;
	env = 0;
}

bool CanonicalDijkstra::InitializeSearch(MapEnvironment *env, const xyLoc& from, const xyLoc& to, std::vector<xyLoc> &thePath)
{
	nodesExpanded = nodesTouched = 0;
	this->env = env;
	this->to = to;
	openClosedList.Reset(env->GetMap()->GetMapWidth()*env->GetMap()->GetMapHeight());
	xyLocParent f;
	f.loc = from;
	f.parent = 0xFF;
	openClosedList.AddOpenNode(f, env->GetStateHash(from), 0, 0);
	if (from == to)
		return false;
	return true;
}

bool CanonicalDijkstra::DoSingleSearchStep(std::vector<xyLoc> &thePath)
{
	if (openClosedList.OpenSize() > 0)
	{
		nodesExpanded++;
		uint64_t next = openClosedList.Close();
		xyLocParent nextState = openClosedList.Lookat(next).data;
		//std::cout << nextState.loc << "\n";
		
		// no goal check (would normally be here)
		
		successors.resize(0);
		GetJPSSuccessors(nextState, next, openClosedList.Lookat(next).g);
		
		for (const auto &s : successors)
		{
			uint64_t theID;
			uint64_t hash = env->GetStateHash(s.s.loc);
			dataLocation l = openClosedList.Lookup(hash, theID);
			switch (l)
			{
				case kClosedList:
				{
					//					printf("(%d, %d) on closed g: %f [might be %f]\n", s.s.loc.x,  s.s.loc.y,
					//						   openClosedList.Lookup(theID).g, openClosedList.Lookup(next).g+s.cost);
					if (fless(openClosedList.Lookup(next).g+s.cost, openClosedList.Lookup(theID).g))
					{
						openClosedList.Lookup(theID).parentID = next;
						openClosedList.Lookup(theID).g = openClosedList.Lookup(next).g+s.cost;
						openClosedList.Reopen(theID);
						//std::cout << "Reopening " << openClosedList.Lookup(theID).data.loc << "\n";
						// This line isn't normally needed, but in some state spaces we might have
						// equality but different meta information, so we need to make sure that the
						// meta information is also copied, since this is the most generic A* implementation
						openClosedList.Lookup(theID).data = s.s;
					}
					
					break;
				}
				case kNotFound:
				{
					//					printf("(%d, %d) added to open\n", s.s.loc.x,  s.s.loc.y);
					openClosedList.AddOpenNode(s.s,
											   hash,
											   openClosedList.Lookup(next).g+s.cost,
											   0,
											   next);
					break;
				}
				case kOpenList:
				{
					if (openClosedList.Lookup(next).g+s.cost < openClosedList.Lookup(theID).g)
					{
						//						printf("(%d, %d) updated on open\n", s.s.loc.x,  s.s.loc.y);
						openClosedList.Lookup(theID).parentID = next;
						openClosedList.Lookup(theID).g = openClosedList.Lookup(next).g+s.cost;
						openClosedList.Lookup(theID).data.parent = s.s.parent;
						openClosedList.KeyChanged(theID);
					}
					else {
						//						printf("(%d, %d) untouched on open\n", s.s.loc.x,  s.s.loc.y);
					}
					break;
				}
			}
		}
	}
	else {
		return true;
	}
	return false;
}

void CanonicalDijkstra::GetPath(MapEnvironment *env, const xyLoc &from, const xyLoc &to, std::vector<xyLoc> &path)
{
	InitializeSearch(env, from, to, path);
	while (DoSingleSearchStep(path) == false)
	{}
}

void CanonicalDijkstra::GetPath(MapEnvironment *env, const xyLoc &from, const xyLoc &to, std::vector<tDirection> &path)
{
	
}

void CanonicalDijkstra::GetJPSSuccessors(const xyLocParent &s, uint64_t pid, double pg)
{
	// write this and return g-cost too
	GetJPSSuccessors(s.loc.x, s.loc.y, s.parent, 0, pid, pg, true);
}

bool CanonicalDijkstra::Passable(int x, int y)
{
	return env->GetMap()->GetTerrainType(x, y) == kGround;
}

void CanonicalDijkstra::GetJPSSuccessors(int x, int y, uint8_t parent, double cost, uint64_t pid, double pg, bool first)
{
	// If on closed with lower g-cost, return
	// else add to closed with current g-cost
	if (!first)
	{
		xyLocParent s = {{static_cast<uint16_t>(x), static_cast<uint16_t>(y)}, parent};
		uint64_t hash = env->GetStateHash(s.loc);
		uint64_t theID;
		dataLocation l = openClosedList.Lookup(hash, theID);
		if (l == kNotFound) // add to closed
		{
			//			printf("Setting (%d, %d) to g:%f\n", x, y, pg+cost);
			openClosedList.AddClosedNode(s, hash, pg+cost, 0, pid);
			//			printf("Value set to: %f\n", GetClosedGCost(s.loc));
		}
		else if (fless(pg+cost, openClosedList.Lookat(theID).g)) // update on closed
		{
			//			printf("Updating (%d, %d) to g:%f\n", x, y, pg+cost);
			openClosedList.Lookup(theID).g = pg+cost;
			openClosedList.Lookup(theID).reopened = true;
		}
		else { // stop!
			//			printf("Skipping (%d, %d) to g:%f cs \n", x, y, pg+cost, openClosedList.Lookat(theID).g);
			return;
		}
	}
	
	nodesTouched++;
	int w = env->GetMap()->GetMapWidth();
	int h = env->GetMap()->GetMapHeight();
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
				successors.push_back(jpsSuccessor(x, y-1, tDirection(kNW|kNE), cost+1));
			}
			else if (a)
			{
				successors.push_back(jpsSuccessor(x, y-1, tDirection(kNW), cost+1));
			}
			else if (b)
			{
				successors.push_back(jpsSuccessor(x, y-1, tDirection(kNE), cost+1));
			}
			else {
				GetJPSSuccessors(x, y-1, kN, cost+1, pid, pg);
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
				successors.push_back(jpsSuccessor(x-1, y, tDirection(kNW|kSW), cost+1));
			}
			else if (a)
			{
				successors.push_back(jpsSuccessor(x-1, y, tDirection(kNW), cost+1));
			}
			else if (b)
			{
				//neighbors.push_back(xyLoc(x-1, y, kSW));
				successors.push_back(jpsSuccessor(x-1, y, tDirection(kSW), cost+1));
			}
			else {
				GetJPSSuccessors(x-1, y, kW, cost+1, pid, pg);
				
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
				successors.push_back(jpsSuccessor(x, y+1, tDirection(kSE|kSW), cost+1));
			}
			else if (a)
			{
				successors.push_back(jpsSuccessor(x, y+1, tDirection(kSW), cost+1));
			}
			else if (b)
			{
				successors.push_back(jpsSuccessor(x, y+1, tDirection(kSE), cost+1));
			}
			else {
				GetJPSSuccessors(x, y+1, kS, cost+1, pid, pg);
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
				successors.push_back(jpsSuccessor(x+1, y, tDirection(kNE|kSE), cost+1));
			}
			else if (a)
			{
				successors.push_back(jpsSuccessor(x+1, y, tDirection(kNE), cost+1));
			}
			else if (b)
			{
				successors.push_back(jpsSuccessor(x+1, y, tDirection(kSE), cost+1));
			}
			else {
				GetJPSSuccessors(x+1, y, kE, cost+1, pid, pg);
				
			}
			w1 = true;
		}
	}
	if (parent&kNW)
	{
		if (x != 0 && y != 0 && Passable(x-1, (y-1)) && n1 && e1)
		{
			GetJPSSuccessors(x-1, y-1, kNW, cost+gDiagCost, pid, pg);
			
		}
	}
	if (parent&kNE)
	{
		if (x != w-1 && y != 0 && Passable(x+1, (y-1)) && n1 && w1)
		{
			GetJPSSuccessors(x+1, y-1, kNE, cost+gDiagCost, pid, pg);
			
		}
	}
	if (parent&kSW)
	{
		if (x != 0 && y != h-1 && Passable(x-1, (y+1)) && s1 && e1)
		{
			GetJPSSuccessors(x-1, y+1, kSW, cost+gDiagCost, pid, pg);
		}
	}
	if (parent&kSE)
	{
		if (x != w-1 && y != h-1 && Passable(x+1, (y+1)) && s1 && w1)
		{
			GetJPSSuccessors(x+1, y+1, kSE, cost+gDiagCost, pid, pg);
		}
	}
}

void CanonicalDijkstra::ExtractPathToStartFromID(uint64_t node, std::vector<xyLoc> &thePath)
{
	do {
		thePath.push_back(openClosedList.Lookup(node).data.loc);
		node = openClosedList.Lookup(node).parentID;
	} while (openClosedList.Lookup(node).parentID != node);
	thePath.push_back(openClosedList.Lookup(node).data.loc);
}

uint64_t CanonicalDijkstra::GetNodesExpanded() const
{
	return nodesExpanded;
}

uint64_t CanonicalDijkstra::GetNodesTouched() const
{
	return nodesTouched;
}

double CanonicalDijkstra::GetClosedGCost(xyLoc s)
{
	uint64_t hash = env->GetStateHash(s);
	uint64_t theID;
	dataLocation l = openClosedList.Lookup(hash, theID);
	if (l == kNotFound) // add to closed
	{
		return -1;
	}
	return openClosedList.Lookat(theID).g;
}


void CanonicalDijkstra::LogFinalStats(StatCollection *stats)
{
	
}

//void CanonicalDijkstra::OpenGLDraw() const
//{
//	double transparency = 1.0;
//	if (openClosedList.size() == 0)
//		return;
//	uint64_t top = -1;
//	double maxGCost = 0;
//	
//	for (unsigned int x = 0; x < openClosedList.size(); x++)
//	{
//		const auto &data = openClosedList.Lookat(x);
//		if (data.round == openClosedList.GetRound())
//			maxGCost = std::max(maxGCost, data.g);
//	}
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
//			env->SetColor(data.g/maxGCost, 0.0, 1.0, transparency);
//			env->OpenGLDraw(data.data.loc);
//		}
//		else if (data.where == kClosedList)
//		{
//			env->SetColor(data.g/maxGCost, 0.0, 0.0, transparency);
//			env->OpenGLDraw(data.data.loc);
//		}
//	}
//}
//
//void CanonicalDijkstra::OpenGLDraw(const MapEnvironment *env) const {}

void CanonicalDijkstra::Draw(Graphics::Display &disp) const
{
	double transparency = 1.0;
	if (openClosedList.size() == 0)
		return;
	uint64_t top = -1;
	double maxGCost = 0;
	
	for (unsigned int x = 0; x < openClosedList.size(); x++)
	{
		const auto &data = openClosedList.Lookat(x);
		if (data.round == openClosedList.GetRound())
			maxGCost = std::max(maxGCost, data.g);
	}
	
	if (openClosedList.OpenSize() > 0)
	{
		top = openClosedList.Peek();
	}
	for (unsigned int x = 0; x < openClosedList.size(); x++)
	{
		const auto &data = openClosedList.Lookat(x);
		if (data.round != openClosedList.GetRound())
			continue;
		
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
			env->SetColor(data.g/maxGCost, 0.0, 1.0, transparency);
			env->Draw(disp, data.data.loc);
		}
		else if (data.where == kClosedList)
		{
			env->SetColor(data.g/maxGCost, 0.0, 0.0, transparency);
			env->Draw(disp, data.data.loc);
		}
	}
}


std::string CanonicalDijkstra::SVGDraw() const
{
	std::string s;
	double transparency = 1.0;
	if (openClosedList.size() == 0)
		return s;
	uint64_t top = -1;
	double maxGCost = 0;
	
	for (unsigned int x = 0; x < openClosedList.size(); x++)
	{
		const auto &data = openClosedList.Lookat(x);
		if (data.round == openClosedList.GetRound())
			maxGCost = std::max(maxGCost, data.g);
	}
	
	if (openClosedList.OpenSize() > 0)
	{
		top = openClosedList.Peek();
	}
	for (unsigned int x = 0; x < openClosedList.size(); x++)
	{
		const auto &data = openClosedList.Lookat(x);
		if (data.round != openClosedList.GetRound())
			continue;

		if (x == top)
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
			env->SetColor(data.g/maxGCost, 0.0, 1.0, transparency);
			s+=env->SVGDraw(data.data.loc);
		}
		else if (data.where == kClosedList)
		{
			env->SetColor(data.g/maxGCost, 0.0, 0.0, transparency);
			s+=env->SVGDraw(data.data.loc);
		}
	}
	return s;
}

std::string CanonicalDijkstra::SVGDrawDetailed() const
{
	std::string s;
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

		if (x == top)
		{
			env->SetColor(1.0, 1.0, 0.0);
			s+=env->SVGDraw(data.data.loc);
		}
		else if (data.where == kOpenList)
		{
			env->SetColor(0.0, 1.0, 0.0, 0.2);
			s+=env->SVGDraw(data.data.loc);
		}

		env->SetColor(0.0, 0.0, 0.0);
		char d[32];
		sprintf(d, "%1.1f", data.g);
		s+=env->SVGLabelState(data.data.loc, d, 0.4, -0.2, -0.7);
	}
	return s;

}
