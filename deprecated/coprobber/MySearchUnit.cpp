#include <iostream>
#include "MySearchUnit.h"

using namespace GraphAbstractionConstants;
using namespace std;

static const bool verbose = false;

MySearchUnit::MySearchUnit( int _x, int _y, int _targetUnit, SearchAlgorithm *alg)
	:AbsMapUnit(_x, _y),
	nodesExpanded(0),
	nodesTouched(0),
	algorithm(alg),
	s_algorithm(0),
	spread_cache(0),
	targetUnit(_targetUnit),
	targetTime(0),
	onTarget(false)
{ }

MySearchUnit::~MySearchUnit()
{
	if (algorithm)
		delete algorithm;
	algorithm = 0;
	if (spread_cache)
		delete spread_cache;
	spread_cache = 0;
}

bool MySearchUnit::getCachedMove(tDirection &dir)
{
	if (moves.size() > 0)
	{
		if (s_algorithm && (spread_cache == 0))
		{
			spread_cache = s_algorithm->think();
			nodesExpanded+=algorithm->GetNodesExpanded();
			nodesTouched+=algorithm->GetNodesTouched();
		}
		dir = moves.back();
		moves.pop_back();
		//		if (verbose)
		//			printf("SU %p: returning cached move 0x%X\n", this, (int)dir);
		return true;
	}
	return false;
}

bool MySearchUnit::makeMove(MapProvider *mp, reservationProvider *rp, AbsMapSimulationInfo *simInfo, tDirection &res)
{
	if (getCachedMove(res))
		return true;

	Map *map = mp->GetMap();
	MapAbstraction *aMap = mp->GetMapAbstraction();
	PublicUnitInfo<xyLoc,tDirection,AbsMapEnvironment> pui;

	// if we have a cache to be used, use it!
	if (spread_cache)
	{
		addPathToCache(spread_cache);
		node *next_start = spread_cache->tail()->n;
		int targx, targy;
		simInfo->GetPublicUnitInfo( targetUnit, pui );
		xyLoc l = pui.currentState;
		targx = l.x;
		targy = l.y;
		
		// Get a path by path-planning
		node *to = aMap->GetAbstractGraph(0)->GetNode(map->GetNodeNum(targx, targy));
		
		s_algorithm->setTargets(mp->GetMapAbstraction(), next_start, to, rp);
		delete spread_cache;
		spread_cache = 0;
		res = moves.back();
		moves.pop_back();
		return true;
	}
	
	// Check if we have a target defined
	if (targetUnit < 0)
	{
		if (verbose)
			printf("SU %s: No target, doing nothing\n", this->GetName());
		return false;
	}
	
	// Get the position of the target
	int targx, targy;
	simInfo->GetPublicUnitInfo( targetUnit, pui );
	xyLoc l = pui.currentState;
	targx = l.x;
	targy = l.y;
		
	// Get a path by path-planning
	Graph *g0 = aMap->GetAbstractGraph(0);
	// Get the start and goal nodes
	node *from = g0->GetNode(map->GetNodeNum(loc.x, loc.y));
	node *to = g0->GetNode(map->GetNodeNum(targx, targy));
	
	if (from == to)
	{
		if (!onTarget)
		{
			if (verbose)
			{
				printf("STAY ON TARGET!\n");
				printf("%p target time %1.4f\n", (void*)this, targetTime);
			}
			if (simInfo)
				targetTime = simInfo->GetSimulationTime();
		}
		onTarget = true;
//		return kStay;
	}
	else
		onTarget = false;
//	if (verbose)
//		printf("SU %p: Getting new path\n", this);
	path *p;
	p = algorithm->GetPath(aMap, from, to, rp);
	nodesExpanded+=algorithm->GetNodesExpanded();
	nodesTouched+=algorithm->GetNodesTouched();

	// returning an empty path means there is no path between the start and goal
	if (p == NULL)
	{
		if (verbose)
			printf("SU %s: Path returned NIL\n", this->GetName());
		return false;
	}
		
	if (!(p->n && p->next && p->next->n && (loc.x == p->n->GetLabelL(kFirstData)) 
				 && (loc.y == p->n->GetLabelL(kFirstData+1))))
	{
		if (p->n)
			std::cout << *p->n << std::endl;
		if ((p->next) && (p->next->n))
			std::cout << *p->next->n << std::endl;
		std::cout << loc.x << ", " << loc.y << std::endl;
	}

	// a valid path must have at least 2 nodes and start where the unit is located
	assert(p->n && p->next && p->next->n && (loc.x == p->n->GetLabelL(kFirstData)) 
				 && (loc.y == p->n->GetLabelL(kFirstData+1)));
	
	addPathToCache(p);
	if (s_algorithm)
	{
		node *next_start = p->tail()->n;
		s_algorithm->setTargets(mp->GetMapAbstraction(), next_start, to, rp);
	}
	delete p;

	assert(moves.size() > 0);

	res = moves.back();
	moves.pop_back();
//	if (verbose)
//		printf("SU %p: returning move 0x%X\n", this, (int)dir);
	return true;
}

void MySearchUnit::addPathToCache(path *p)
{
	// we are at the last move; abort recursion
	if (p->next == NULL)
		return;
	// there is another move; add it first to cache
	if (p->next->next)
		MySearchUnit::addPathToCache(p->next);

	// ----- Ok, we have a path starting at (x,y) [the current location] and
	// having at least one more state ----------------------------------------
	
	// Take the first move off the path and execute it
	int result = kStay;
	
	// Decide on the horizontal move
	switch ((p->n->GetLabelL(kFirstData)-p->next->n->GetLabelL(kFirstData)))
	{
		case -1: result = kE; break;
		case 0: break;
		case 1: result = kW; break;
		default :
			printf("SU: %s : The (x) nodes in the path are not next to each other!\n",
						 this->GetName());
			printf("Distance is %ld\n",
						 p->n->GetLabelL(kFirstData)-p->next->n->GetLabelL(kFirstData));
			std::cout << *p->n << "\n" << *p->next->n << "\n";
			exit(10); break;
	}
	
	// Tack the vertical move onto it
	// Notice the exploit of particular encoding of kStay, kE, etc. labels
	switch ((p->n->GetLabelL(kFirstData+1)-p->next->n->GetLabelL(kFirstData+1)))
	{
		case -1: result = result|kS; break;
		case 0: break;
		case 1: result = result|kN; break;
		default :
			printf("SU: %s : The (y) nodes in the path are not next to each other!\n",
						 this->GetName());
			printf("Distance is %ld\n",
						 p->n->GetLabelL(kFirstData+1)-p->next->n->GetLabelL(kFirstData+1));
			std::cout << *p->n << "\n" << *p->next->n << "\n";
			exit(10); break;
	}
	moves.push_back((tDirection)result);
}

void MySearchUnit::updateLocation(int _x, int _y, bool success, AbsMapSimulationInfo *)
{
	if (!success)
	{
		moves.clear();
		delete spread_cache;
		spread_cache = 0;
		if (verbose)
			printf("SU %s: clearing cached moves, (%d,%d)\n", this->GetName(),_x,_y);
	}
	loc.x = _x; loc.y = _y;
}

void MySearchUnit::OpenGLDraw(int , AbsMapEnvironment *ame, AbsMapSimulationInfo *si)
{
	GLdouble xx, yy, zz, rad;
	Map *map = ame->GetMap();

	int posx = loc.x, posy = loc.y;
	map->GetOpenGLCoord(posx, posy, xx, yy, zz, rad);
	glColor3f(r, g, b);
	glBegin(GL_LINE_STRIP);
	glVertex3f(xx, yy, zz-rad/2);
	for (int t = moves.size()-1; t >= 0; t--)
	{
		posx += ((moves[t]&kE)?1:0) - ((moves[t]&kW)?1:0);
		posy += ((moves[t]&kS)?1:0) - ((moves[t]&kN)?1:0);
		
		map->GetOpenGLCoord(posx, posy, xx, yy, zz, rad);

		glVertex3f(xx, yy, zz-rad/2);
	}
	glEnd();
	
	// draw object
  map->GetOpenGLCoord(loc.x, loc.y, xx, yy, zz, rad);
	if (onTarget)
	{
		double perc = (1.0-sqrt(sqrt(abs(sin(targetTime+0.25*si->GetSimulationTime())))));
		glColor3f(r*perc, g*perc, b*perc);
	}
	else
		glColor3f(r, g, b);
	DrawSphere(xx, yy, zz, rad);

/*
	// draw target
	if (targetUnit >= 0)
	{
		si->GetPublicUnitInfo( targetUnit, pui );
		xyLoc tloc = pui.currentState;
		map->GetOpenGLCoord(tloc.x, tloc.y, xx, yy, zz, rad);

		double perc = (1.0-sqrt(sqrt(abs(sin(targetTime+0.25*si->GetSimulationTime())))));
		glColor3f(r*perc, g*perc, b*perc);

		DrawPyramid(xx, yy, zz, 1.1*rad, 0.75*rad);
	}
*/
}

void MySearchUnit::LogStats(StatCollection *stats)
{
	if (((nodesExpanded == 0) && (nodesTouched != 0)) ||
			((nodesExpanded != 0) && (nodesTouched == 0)))
	{
		printf("Error; somehow nodes touched/expanded are inconsistent. t:%d e:%d\n",
					 nodesTouched, nodesExpanded);
	}
	// printf("MySearchUnit::LogStats(nodesExpanded=%d, nodesTouched=%d)\n",nodesExpanded,nodesTouched);
	if (nodesExpanded != 0)
		stats->AddStat("nodesExpanded", GetName(), (long)nodesExpanded);
	if (nodesTouched != 0)
		stats->AddStat("nodesTouched", GetName(), (long)nodesTouched);
	nodesExpanded = nodesTouched = 0;
}

void MySearchUnit::LogFinalStats(StatCollection *stats)
{
	algorithm->LogFinalStats(stats);
}

