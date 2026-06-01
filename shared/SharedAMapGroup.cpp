/*
 *  $Id: SharedAMapGroup.cpp
 *  hog2
 *
 *  Created by Nathan Sturtevant on 12/16/04.
 *  Modified by Nathan Sturtevant on 02/29/20.
 *
 * This file is part of HOG2. See https://github.com/nathansttt/hog2 for licensing information.
 *
 */

#include "SharedAMapGroup.h"
#include "Unit.h"
#include "UnitGroup.h"
#include "MapCliqueAbstraction.h"
#include "MapFlatAbstraction.h"

using namespace GraphAbstractionConstants;

SharedAMapGroup::SharedAMapGroup(MapProvider *mp)
:aMap(0), seen(0)
{
	visRadius = 2;
	sawNewLand = true;
	newTileCount = 0;
	newTileCountPerTrial = 0;

	Map *m = mp->GetMap();
	map = new Map(m->GetMapWidth(), m->GetMapHeight());
	aMap = new MapCliqueAbstraction(map);
	//aMap = new MapFlatAbstraction(map);
	seen = new BitVector(m->GetMapWidth() * m->GetMapHeight());
	
}

SharedAMapGroup::~SharedAMapGroup()
{
	delete aMap;
	delete seen;
}

//void SharedAMapGroup::updateLocation(BaseAbsMapUnit *u, MapProvider *mp, int _x, int _y, bool success, AbsMapSimulationInfo *simInfo)
void SharedAMapGroup::UpdateLocation(Unit<xyLoc, tDirection, AbsMapEnvironment> *u, AbsMapEnvironment *mp, xyLoc &loc, bool success, AbsMapSimulationInfo *si)
{
	Map *worldMap = mp->GetMap();
	int rad = visRadius; // how far we can see -- square
	int x, y, num;
	u->UpdateLocation(mp, loc, success, si);
	xyLoc l;
	u->GetLocation(l);
	x = l.x; y = l.y;
	bool changed = false;
	for (int x1 = x-rad; x1 <= x+rad; x1++)
	{
		if ((x1 < 0) || (x1 >= map->GetMapWidth()))
			continue;
		for (int y1 = y-rad; y1 <= y+rad; y1++)
		{
			if ((y1 < 0) || (y1 >= map->GetMapHeight()))
				continue;
			
			// if we haven't observed a particular tile, we need to look at it
			if (!seen->Get(y1*map->GetMapWidth()+x1))
			{
				// count a new tile!
				newTileCount++;
				// if it's different that we think, we just remove it.
				// in the future we'll need to handle water & splits here
				num = map->GetNodeNum(x1, y1);
				Graph *g = aMap->GetAbstractGraph(0);
				if (!worldMap->AdjacentEdges(x1, y1, kLeftEdge))
				{ edge *e = g->FindEdge(num, map->GetNodeNum(x1-1, y1)); aMap->RemoveEdge(e, 0); }
				if (!worldMap->AdjacentEdges(x1, y1, kRightEdge))
				{ edge *e = g->FindEdge(num, map->GetNodeNum(x1+1, y1)); aMap->RemoveEdge(e, 0); }
				if (!worldMap->AdjacentEdges(x1, y1, kTopEdge))
				{ edge *e = g->FindEdge(num, map->GetNodeNum(x1, y1-1)); aMap->RemoveEdge(e, 0); }
				if (!worldMap->AdjacentEdges(x1, y1, kBottomEdge))
				{ edge *e = g->FindEdge(num, map->GetNodeNum(x1, y1+1)); aMap->RemoveEdge(e, 0); }

				if (!worldMap->AdjacentCorners(x1, y1, kTopLeft))
				{ edge *e = g->FindEdge(num, map->GetNodeNum(x1-1, y1-1)); aMap->RemoveEdge(e, 0); }
				if (!worldMap->AdjacentCorners(x1, y1, kTopRight))
				{ edge *e = g->FindEdge(num, map->GetNodeNum(x1+1, y1-1)); aMap->RemoveEdge(e, 0); }
				if (!worldMap->AdjacentCorners(x1, y1, kBottomLeft))
				{ edge *e = g->FindEdge(num, map->GetNodeNum(x1-1, y1+1)); aMap->RemoveEdge(e, 0); }
				if (!worldMap->AdjacentCorners(x1, y1, kBottomRight))
				{ edge *e = g->FindEdge(num, map->GetNodeNum(x1+1, y1+1)); aMap->RemoveEdge(e, 0); }
				
				//aMap->RemoveNode(me);
				map->SetTerrainType(x1, y1, (tTerrain)worldMap->GetTerrainType(x1, y1));
				changed = true;
				
				sawNewLand = true;
				seen->Set(y1*map->GetMapWidth()+x1, true);
			}
		}
	}
	if (changed)
		aMap->RepairAbstraction();
}

void SharedAMapGroup::OpenGLDraw(const AbsMapEnvironment *, const AbsMapSimulationInfo *) const
{
	glBegin(GL_QUADS);
	glColor3f(.25, .25, .25); // kOutOfBounds
	glNormal3f(0, 0, 1);
	GLdouble coverage = 0.9;
	for (int x = 0; x < map->GetMapWidth(); x++)
	{
		for (int y = 0; y < map->GetMapHeight(); y++)
		{
			GLdouble a, b, c, radius;
			if (!seen->Get(y*map->GetMapWidth()+x))
			{
				map->GetOpenGLCoord(x, y, a, b, c, radius);
				glVertex3f(a+coverage*radius, b+coverage*radius, c-4*radius);
				glVertex3f(a+coverage*radius, b-coverage*radius, c-4*radius);
				glVertex3f(a-coverage*radius, b-coverage*radius, c-4*radius);
				glVertex3f(a-coverage*radius, b+coverage*radius, c-4*radius);
			}
		}
	}
	glEnd();
}

MapAbstraction *SharedAMapGroup::GetMapAbstraction()
{
	return aMap;
}

Map *SharedAMapGroup::GetMap() const
{
	return map;
}

/** Is the group done with their exploration? */
bool SharedAMapGroup::Done()
{
	return (!sawNewLand);
}

void SharedAMapGroup::LogStats(StatCollection *stats)
//void SharedAMapGroup::printRoundStats(unit *, FILE *f)
{
	if (newTileCount != 0)
	{
		stats->SumStat("newTilesExplored", GetName(), (long)newTileCount);
		newTileCountPerTrial += newTileCount;
		newTileCount = 0;
	}
	//fprintf(f,"%8.2f", (float)newTileCount);
}

/** Lets the unit group do what it needs to reset a trial */
void SharedAMapGroup::StartNewTrial(StatCollection *stats)
{
	sawNewLand = false;
	newTileCount = 0;
	newTileCountPerTrial = 0;
	stats->AddStat("newTilesExplored", GetName(), (long)newTileCount);
}

void SharedAMapGroup::SetVisibilityRadius(int _visibility)
{
	visRadius = _visibility;
}

int SharedAMapGroup::GetVisibilityRadius()
{
	return visRadius;
}

bool SharedAMapGroup::Explored(int x, int y)
{
	return seen->Get(y*map->GetMapWidth()+x);
}

bool SharedAMapGroup::Explored(unsigned int _node)
{
	node *loc = aMap->GetAbstractGraph(0)->GetNode(_node);
	return Explored((unsigned int)loc->GetLabelL(kFirstData), 
									(unsigned int)loc->GetLabelL(kFirstData+1));
}
