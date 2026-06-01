/*
 *  $Id: rewardUnit.cpp
 *  hog2
 *
 *  Created by Nathan Sturtevant on 2/16/05.
 *  Modified by Nathan Sturtevant on 02/29/20.
 *
 * This file is part of HOG2. See https://github.com/nathansttt/hog2 for licensing information.
 *
 */

#include "RewardUnit.h"

rewardUnit::rewardUnit(int _x, int _y)
:unit(_x, _y)
{
	setObjectType(kIncidentalUnit);
}

void rewardUnit::OpenGLDraw(const MapProvider *mp, const SimulationInfo *) const
{
	Map *map = mp->GetMap();
	GLdouble xx, yy, zz, rad;
	map->GetOpenGLCoord(x, y, xx, yy, zz, rad);
	glColor3f(r, g, b);
	drawTriangle(xx, yy, zz, rad);
}



rewardSeekingUnit::rewardSeekingUnit(int _x, int _y)
:rewardUnit(_x, _y)
{
	setObjectType(kWorldObject);
}

void rewardSeekingUnit::receiveReward(double amount)
{
	printf("%s got reward %3.2f!!\n", GetName(), amount);
}

tDirection rewardSeekingUnit::makeMove(MapProvider *mp, reservationProvider *, SimulationInfo *)
{
	MapAbstraction *aMap = mp->GetMapAbstraction();
	if (moves.size() > 0)
	{
		tDirection dir = moves.back();
		moves.pop_back();
		//		if (verbose)
		//			printf("SU %p: returning cached move 0x%X\n", this, (int)dir);
		return dir;
	}

	// add your logic code here...
	static int nextTarget = 0;
	goToRewardLoc(aMap, nextTarget);
	nextTarget = (nextTarget+1)%rewardLocs.size();
	return kStay;
}

double rewardSeekingUnit::goToRewardLoc(MapAbstraction *aMap, int which)
{
	double pathCost=-1;
	path *p;
	node *from, *to;
	from = aMap->GetNodeFromMap(x, y); // gets my location
	int tox, toy;
	rewardLocs[which]->getLocation(tox, toy);
	to = aMap->GetNodeFromMap(tox, toy);
	p = a.GetPath(aMap, from, to);
	if (p)
	{
		pathCost = aMap->distance(p);
		addPathToCache(p);
	}
	return pathCost;
}

void rewardSeekingUnit::OpenGLDraw(const MapProvider *mp, const SimulationInfo *) const
{
	GLdouble xx, yy, zz, rad;
	Map *map = mp->GetMap();
	int posx = x, posy = y;
	map->GetOpenGLCoord(posx, posy, xx, yy, zz, rad);
	glColor3f(r, g, b);
	glBegin(GL_LINE_STRIP);
	glVertex3f(xx, yy+rad/2, zz);
	for (int t = moves.size()-1; t >= 0; t--)
	{
		posx += ((moves[t]&kE)?1:0) - ((moves[t]&kW)?1:0);
		posy += ((moves[t]&kS)?1:0) - ((moves[t]&kN)?1:0);
		
		map->GetOpenGLCoord(posx, posy, xx, yy, zz, rad);
		
		glVertex3f(xx, yy+rad/2, zz);
	}
	glEnd();
	
	map->GetOpenGLCoord(x, y, xx, yy, zz, rad);
	glColor3f(r, g, b);
	drawSphere(xx, yy, zz, rad);
}

void rewardSeekingUnit::addRewardLocation(rewardUnit *ru)
{
	rewardLocs.push_back(ru);
}

void rewardSeekingUnit::addPathToCache(path *p)
{
	// we are at the last move; abort recursion
	if ((p == NULL) || (p->next == NULL))
		return;
	// there is another move; add it first to cache
	if (p->next->next)
		addPathToCache(p->next);
	
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
