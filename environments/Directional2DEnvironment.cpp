/*
 *  Directional2DEnvironment.cpp
 *  hog2
 *
 *  Created by Nathan Sturtevant on 2/20/08.
 *  Copyright 2008 __MyCompanyName__. All rights reserved.
 *
 */

#include "Directional2DEnvironment.h"
#include "FPUtil.h"
#include "TemplateAStar.h"

#define TABLE_SIZE 15
#define ROW_SIZE (2*TABLE_SIZE+1)

#pragma message("ABS functions might have been removed; need to verify to make sure they weren't needed")
Directional2DEnvironment::Directional2DEnvironment(Map *_m, model envType, heuristicType heuristic)
{
	motionModel = envType;//kHumanoid;
	//hType = kExtendedPerimeterHeuristic;
	//hType = kPerimeterHeuristic;
	//hType = kOctileHeuristic
	
	hType = heuristic;
	BuildAngleTables();
//	heuristics.resize(4);
//	for (int x = 0; x < 4; x++)
//	{
//		heuristics[x].speed = 0;
//		heuristics[x].rotation = x;
//		BuildHTable(heuristics[x]);
//	}
	SetColor(1.0, 0.25, 0.25, 1.0);
	map = _m;
	checkLegal = true;
	test = 0;
}

Directional2DEnvironment::~Directional2DEnvironment()
{
	delete map;
}

int Directional2DEnvironment::GetNumAngles()
{
	switch (motionModel)
	{
		case kVehicle: return 16;
		case kTank: return 24;
		case kBetterTank: return 24;
		case kHumanoid: return 16;
	}
	assert(!"Unknown motion model");
	return 0;
}

void Directional2DEnvironment::GetSuccessors(const xySpeedHeading &loc, std::vector<xySpeedHeading> &neighbors) const
{
	neighbors.resize(0);
	std::vector<deltaSpeedHeading> acts;
	GetActions(loc, acts);
	for (unsigned int x = 0; x < acts.size(); x++)
	{
		xySpeedHeading newLoc(loc);
		ApplyAction(newLoc, acts[x]);
		neighbors.push_back(newLoc);
	}
}

void Directional2DEnvironment::GetActions(const xySpeedHeading &loc, std::vector<deltaSpeedHeading> &actions) const
{
	if (motionModel == kVehicle)
	{
		deltaSpeedHeading sh;
		if (loc.speed <= 1)
		{
			// stop move
			sh.speed = 0-loc.speed;
			sh.turn = 0;
			actions.push_back(sh);
//TODO: Backwards moves disabled!
//			// reverse moves
//			sh.speed = -1-loc.speed;
//			sh.turn = 0;
//			if (Legal(loc, sh))
//				actions.push_back(sh);
//			sh.turn = 1;
//			if (Legal(loc, sh))
//				actions.push_back(sh);
//			sh.turn = -1;
//			if (Legal(loc, sh))
//				actions.push_back(sh);
//			sh.turn = 2;
//			if (Legal(loc, sh))
//				actions.push_back(sh);
//			sh.turn = -2;
//			if (Legal(loc, sh))
//				actions.push_back(sh);
		}

		if (loc.speed >= 1) // medium moves
		{
			sh.speed = 2-loc.speed;
			sh.turn = 0;
			if (Legal(loc, sh))
				actions.push_back(sh);
			sh.turn = 1;
			if (Legal(loc, sh))
				actions.push_back(sh);
			sh.turn = -1;
			if (Legal(loc, sh))
				actions.push_back(sh);
		}
		if (loc.speed > 1) // fast moves
		{
			sh.speed = 3-loc.speed;
			sh.turn = 0;
			if (Legal(loc, sh))
				actions.push_back(sh);
//			sh.turn = 1;
//			if (Legal(loc, sh))
//				actions.push_back(sh);
//			sh.turn = -1;
//			if (Legal(loc, sh))
//				actions.push_back(sh);
		}
		// slow moves
		if (loc.speed < 3)
		{
			sh.speed = 1-loc.speed;
			sh.turn = -3;
			if (Legal(loc, sh))
				actions.push_back(sh);
			sh.turn = -2;
			if (Legal(loc, sh))
				actions.push_back(sh);
			sh.turn = -1;
			if (Legal(loc, sh))
				actions.push_back(sh);
			sh.turn = 0;
			if (Legal(loc, sh))
				actions.push_back(sh);
			sh.turn = 1;
			if (Legal(loc, sh))
				actions.push_back(sh);
			sh.turn = 2;
			if (Legal(loc, sh))
				actions.push_back(sh);
			sh.turn = 3;
			if (Legal(loc, sh))
				actions.push_back(sh);
		}
	}
	else if (motionModel == kHumanoid)
	{
		deltaSpeedHeading sh;

		for (int x = 0; x <= 2; x++)
		{
			sh.speed = x-loc.speed;
			sh.turn = 0;
			if (Legal(loc, sh))
				actions.push_back(sh);

			for (int y = 1; y <= 3; y++)
			{
				sh.turn = y;
				if (Legal(loc, sh))
				actions.push_back(sh);
				sh.turn = -y;
				if (Legal(loc, sh))
				actions.push_back(sh);
			}
		}
	}
	else if (motionModel == kTank)
	{
		deltaSpeedHeading sh;
		
		for (int x = -1; x <= 1; x++)
		{
			sh.speed = x-loc.speed;
			sh.turn = 0;
			if (Legal(loc, sh))
				actions.push_back(sh);
			
			for (int y = 1; y <= 3; y++)
			{
				sh.turn = y;
				if (Legal(loc, sh))
					actions.push_back(sh);
				sh.turn = -y;
				if (Legal(loc, sh))
					actions.push_back(sh);
			}
		}
	}
	else if (motionModel == kBetterTank)
	{
		deltaSpeedHeading sh;
		
		for (int x = -1; x <= 1; x++)
		{
			sh.speed = x;
			sh.turn = 0;
			if (Legal(loc, sh) && (x != 0))
				actions.push_back(sh);
			
			for (int y = 1; y <= 3; y++)
			{
				sh.turn = y;
				if (Legal(loc, sh))
					actions.push_back(sh);
				sh.turn = -y;
				if (Legal(loc, sh))
					actions.push_back(sh);
			}
		}
	}
	else { assert(false); }
	if (actions.size() == 0) // always have 1 action(?)
	{
		deltaSpeedHeading act;
		act.speed = -loc.speed;
		act.turn = 0;
		actions.push_back(act);
	}
}

deltaSpeedHeading Directional2DEnvironment::GetAction(const xySpeedHeading &one, const xySpeedHeading &two) const
{
	std::vector<deltaSpeedHeading> acts;
	xySpeedHeading modified;
	GetActions(one, acts);
	for (unsigned int x = 0; x < acts.size(); x++)
	{
		modified = one;
		ApplyAction(modified, acts[x]);
		//if (two == modified)
		if (fequal(two.x, modified.x) && fequal(two.y, modified.y) &&
			(two.rotation == modified.rotation) && (two.speed == modified.speed))
		{
			return acts[x];
		}
	}
	assert(false);
	return acts[0];
}

bool Directional2DEnvironment::InvertAction(deltaSpeedHeading &act) const
{
	if (motionModel == kBetterTank)
	{
		act.turn = -act.turn;
		act.speed = -act.speed;
		return true;
	}
	assert(false);
	return false;
}

void Directional2DEnvironment::RotateCCW(xySpeedHeading &s, unsigned int rotation) const
{
//	GLdouble yoffset = mySin(l.rotation)*s.x;//sin(TWOPI*rot/16)*rad;
//	GLdouble xoffset = myCos(l.rotation)*s.;//cos(TWOPI*rot/16)*rad;
	float x1 = s.x;
	s.x = roundf(s.x*myCos(rotation) + s.y*mySin(rotation));
	s.y = roundf(s.y*myCos(rotation) - x1*mySin(rotation));
	s.rotation = ((((motionModel==kTank)||(motionModel==kBetterTank))?24:16)+s.rotation-rotation)%(((motionModel==kTank)||(motionModel==kBetterTank))?24:16);
}

void Directional2DEnvironment::ApplyAction(xySpeedHeading &s, deltaSpeedHeading dir) const
{
	if ((motionModel != kTank) && (motionModel != kBetterTank))
	{
		float xoffset[16] = {1.0,  2.0,  1.0,  1.0,
							 0.0, -1.0, -1.0, -2.0,
							-1.0, -2.0, -1.0, -1.0,
							 0.0,  1.0,  1.0,  2.0 };
		float yoffset[16] = {0.0,  1.0,  1.0,  2.0,
							 1.0,  2.0,  1.0,  1.0,
							 0.0, -1.0, -1.0, -2.0,
							-1.0, -2.0, -1.0, -1.0};
		s.speed += dir.speed;
		s.rotation = (16+s.rotation+dir.turn)%16;
		
		if (s.speed > 0)
		{
			s.x += xoffset[s.rotation];
			s.y += yoffset[s.rotation];
		}
		else if (s.speed < 0)
		{
			s.x -= xoffset[s.rotation];
			s.y -= yoffset[s.rotation];
	//		float tmp = myCos(s.rotation);//*s.speed;
	//		s.x += tmp;
	//		tmp = mySin(s.rotation);//*s.speed;
	//		s.y += tmp;
		}
		assert(s.speed >= -1 && s.speed <= 3 && s.rotation >= 0 && s.rotation < 16);
	}
	else if (motionModel == kTank)
	{
		float xoffset[24] = {1.0,  3.0,  3.0, 1.0,  2.0, 1.0,
			                 0.0, -1.0, -2.0,-1.0, -3.0,-3.0,
			                -1.0, -3.0, -3.0,-1.0, -2.0,-1.0,
		                     0.0,  1.0,  2.0, 1.0,  3.0, 3.0 };
		float yoffset[24] = {0.0,  1.0,  2.0, 1.0,  3.0, 3.0,
			                 1.0,  3.0,  3.0, 1.0,  2.0, 1.0,
			                 0.0, -1.0, -2.0,-1.0, -3.0,-3.0,
							-1.0, -3.0, -3.0,-1.0, -2.0,-1.0};
		s.speed += dir.speed;
		s.rotation = (24+s.rotation+dir.turn)%24;
		
		if (s.speed > 0)
		{
			s.x += xoffset[s.rotation];
			s.y += yoffset[s.rotation];
		}
		else if (s.speed < 0)
		{
			s.x -= xoffset[s.rotation];
			s.y -= yoffset[s.rotation];
			//		float tmp = myCos(s.rotation);//*s.speed;
			//		s.x += tmp;
			//		tmp = mySin(s.rotation);//*s.speed;
			//		s.y += tmp;
		}
		assert(s.speed >= -1 && s.speed <= 1 && s.rotation >= 0 && s.rotation < 24);
	}
	else if (motionModel == kBetterTank)
	{
		float xoffset[24] = {1.0,  3.0,  3.0, 1.0,  2.0, 1.0,
			                 0.0, -1.0, -2.0,-1.0, -3.0,-3.0,
			                -1.0, -3.0, -3.0,-1.0, -2.0,-1.0,
			                 0.0,  1.0,  2.0, 1.0,  3.0, 3.0 };
		float yoffset[24] = {0.0,  1.0,  2.0, 1.0,  3.0, 3.0,
			                 1.0,  3.0,  3.0, 1.0,  2.0, 1.0,
			                 0.0, -1.0, -2.0,-1.0, -3.0,-3.0,
			                -1.0, -3.0, -3.0,-1.0, -2.0,-1.0};
		s.speed = 0;
		
		if (dir.speed > 0)
		{
			s.rotation = (24+s.rotation+dir.turn)%24;
			s.x += xoffset[s.rotation];
			s.y += yoffset[s.rotation];
		}
		else if (dir.speed < 0)
		{
			s.x -= xoffset[s.rotation];
			s.y -= yoffset[s.rotation];
			s.rotation = (24+s.rotation+dir.turn)%24;
			//		float tmp = myCos(s.rotation);//*s.speed;
			//		s.x += tmp;
			//		tmp = mySin(s.rotation);//*s.speed;
			//		s.y += tmp;
		}
		else {
			s.rotation = (24+s.rotation+dir.turn)%24;
		}
		assert(s.speed >= -1 && s.speed <= 1 && s.rotation >= 0 && s.rotation < 24);
	}
	else { // old tank code
		assert(false);
		s.speed += dir.speed;
		s.rotation = (24+s.rotation+dir.turn)%24;
		if (s.speed != 0)
		{
			float tmp = myCos(s.rotation)*1.4;//*s.speed;
			s.x += tmp;
			tmp = mySin(s.rotation)*1.4;//*s.speed;
			s.y += tmp;
		}		
	}
}

void Directional2DEnvironment::UndoAction(xySpeedHeading &s, deltaSpeedHeading dir) const
{	
	if ((motionModel != kTank) && (motionModel != kBetterTank))
	{
		float xoffset[16] = {1.0,  2.0,  1.0,  1.0,
							 0.0, -1.0, -1.0, -2.0,
							-1.0, -2.0, -1.0, -1.0,
							 0.0,  1.0,  1.0,  2.0 };
		float yoffset[16] = {0.0,  1.0,  1.0,  2.0,
							 1.0,  2.0,  1.0,  1.0,
							 0.0, -1.0, -1.0, -2.0,
							-1.0, -2.0, -1.0, -1.0};
		if (s.speed > 0)
		{
			s.x -= xoffset[s.rotation];
			s.y -= yoffset[s.rotation];
		}
		else if (s.speed < 0)
		{
			s.x += xoffset[s.rotation];
			s.y += yoffset[s.rotation];
				
			//		float tmp = myCos(s.rotation);//*s.speed;
			//		s.x += tmp;
			//		tmp = mySin(s.rotation);//*s.speed;
			//		s.y += tmp;
		}
		s.rotation = (16+s.rotation-dir.turn)%16;
		s.speed -= dir.speed;
	}
	else if (motionModel == kTank) {
		float xoffset[24] = {1.0,  3.0,  3.0, 1.0,  2.0, 1.0,
			0.0, -1.0, -2.0,-1.0, -3.0,-3.0,
			-1.0, -3.0, -3.0,-1.0, -2.0,-1.0,
			0.0,  1.0,  2.0, 1.0,  3.0, 3.0 };
		float yoffset[24] = {0.0,  1.0,  2.0, 1.0,  3.0, 3.0,
			1.0,  3.0,  3.0, 1.0,  2.0, 1.0,
			0.0, -1.0, -2.0,-1.0, -3.0,-3.0,
			-1.0, -3.0, -3.0,-1.0, -2.0,-1.0};
		
		if (s.speed > 0)
		{
			s.x -= xoffset[s.rotation];
			s.y -= yoffset[s.rotation];
		}
		else if (s.speed < 0)
		{
			s.x += xoffset[s.rotation];
			s.y += yoffset[s.rotation];
			
			//		float tmp = myCos(s.rotation);//*s.speed;
			//		s.x += tmp;
			//		tmp = mySin(s.rotation);//*s.speed;
			//		s.y += tmp;
		}
		s.rotation = (24+s.rotation-dir.turn)%24;
		s.speed -= dir.speed;
		assert(s.speed >= -1 && s.speed <= 1 && s.rotation >= 0 && s.rotation < 24);
	}
	else if (motionModel == kBetterTank) {
		float xoffset[24] = {1.0,  3.0,  3.0, 1.0,  2.0, 1.0,
			0.0, -1.0, -2.0,-1.0, -3.0,-3.0,
			-1.0, -3.0, -3.0,-1.0, -2.0,-1.0,
			0.0,  1.0,  2.0, 1.0,  3.0, 3.0 };
		float yoffset[24] = {0.0,  1.0,  2.0, 1.0,  3.0, 3.0,
			1.0,  3.0,  3.0, 1.0,  2.0, 1.0,
			0.0, -1.0, -2.0,-1.0, -3.0,-3.0,
			-1.0, -3.0, -3.0,-1.0, -2.0,-1.0};
		
		if (dir.speed > 0)
		{
			s.x -= xoffset[s.rotation];
			s.y -= yoffset[s.rotation];
			s.rotation = (24+s.rotation-dir.turn)%24;
		}
		else if (dir.speed < 0)
		{
			s.rotation = (24+s.rotation-dir.turn)%24;
			s.x += xoffset[s.rotation];
			s.y += yoffset[s.rotation];
			
			//		float tmp = myCos(s.rotation);//*s.speed;
			//		s.x += tmp;
			//		tmp = mySin(s.rotation);//*s.speed;
			//		s.y += tmp;
		}
		else {
			s.rotation = (24+s.rotation-dir.turn)%24;
		}
		s.speed = 0;
		assert(s.speed >= -1 && s.speed <= 1 && s.rotation >= 0 && s.rotation < 24);
	}
	else { // tank!
		assert(false);
		if (s.speed != 0)
		{
			float tmp = myCos(s.rotation)*1.4;//*s.speed;
			s.x -= tmp;
			tmp = mySin(s.rotation)*1.4;//*s.speed;
			s.y -= tmp;
		}		
		s.rotation = (24+s.rotation-dir.turn)%24;
		s.speed -= dir.speed;
	}
}

double Directional2DEnvironment::HCost(const xySpeedHeading &l1, const xySpeedHeading &l2) const
{
	float dist = sqrt((l1.x-l2.x)*(l1.x-l2.x)+(l1.y-l2.y)*(l1.y-l2.y));
	if (motionModel == kHumanoid)
		dist = ceil(dist/2.0);
	else if (motionModel == kVehicle)
		dist = ceil(dist/3.0);

	return max(dist, LookupStateHeuristic(l1, l2));
//	double val = ceil(dist/3.0)+(3-l1.speed)+(3-l2.speed);
//	int angle = (16+l1.rotation-l2.rotation)%16;
//	return val+angle;
}

bool Directional2DEnvironment::Legal(const xySpeedHeading &node1, const deltaSpeedHeading &act) const
{
	if (!checkLegal)
		return true;
//	int newDir = node1.rotation+act.turn;
//	int x1 = floor(node1.x);
//	int y1 = floor(node1.y);
	xySpeedHeading next=node1;
	ApplyAction(next, act);
	if ((motionModel != kTank) && (motionModel != kBetterTank))
	{
		return ((map->GetTerrainType(node1.x, node1.y) == kGround) &&
				(map->GetTerrainType(next.x, next.y) == kGround) &&
				(map->GetTerrainType((node1.x+next.x)/2, (node1.y+next.y)/2) == kGround));
	}
	else {
		return ((map->GetTerrainType(node1.x, node1.y) == kGround) &&
				(map->GetTerrainType(next.x, next.y) == kGround) &&
				(map->GetTerrainType(node1.x+(-node1.x+next.x)/3, node1.y+(-node1.y+next.y)/3) == kGround) &&
				(map->GetTerrainType(node1.x+2*(-node1.x+next.x)/3, node1.y+2*(-node1.y+next.y)/3) == kGround));
	}
}

double Directional2DEnvironment::GCost(const xySpeedHeading &a, const deltaSpeedHeading &b) const
{
	if ((motionModel != kTank) && (motionModel != kBetterTank))
	{
		//	return 1.0;
		float val =  a.speed+b.speed;
		if (val == 0)
			return 1.0;

		double dist[4] = {1.0, 2.24, 1.42, 2.24};
	//	double dist[4] = {1.0, 1.5, 1.42, 1.5};
		return dist[(16+a.rotation+b.turn)%4]/fabs(val);
	}
	else if (motionModel == kTank) {
		float val =  a.speed+b.speed;
		if (val == 0)
			return 1.0;
		double dist[6] = {1.0, 3.16, 3.61, 1.42, 3.61, 3.16};
		return dist[(24+a.rotation+b.turn)%6]/fabs(val);
	}
	else if (motionModel == kBetterTank) {
		float val =  b.speed;		
		double result;
		if (val == 0)
			return 1.0;
		double dist[6] = {1.0, 3.25, 3.75, 1.50, 3.75, 3.25};
		if (val > 0)
		{
			result = dist[(24+a.rotation+b.turn)%6];
		}
		else {
			result = dist[(a.rotation)%6];
		}
//		std::cout << "GCost between " << a << " given action " << b << " is " << result << std::endl;
		return result;
	}
	
	if (fequal(a.speed+b.speed, 0))
		return 1.0;
	return 1.4/std::abs(a.speed+b.speed);
//	return 1.0/fabs(val);
}

double Directional2DEnvironment::GCost(const xySpeedHeading &a, const xySpeedHeading &b) const
{
	if ((motionModel != kTank) && (motionModel != kBetterTank))
	{
		if (b.speed == 0)
			return 1.0;

	//	float dist[4] = {1.0f, 2.24f, 1.42f, 2.24f};
		double dist[4] = {1.0, 2.24, 1.42, 2.24};
	//	double dist[4] = {1.0, 1.5, 1.42, 1.5};
		double ret = dist[(b.rotation)%4]/std::abs(b.speed);
	//	printf("rot: %d; speed: %d; val: %f\n", b.rotation, b.speed, ret);
		return ret;
	}
	else if (motionModel == kTank)
	{
		if (b.speed == 0)
			return 1.0;
		
		double dist[6] = {1.0, 3.16, 3.61, 1.42, 3.61, 3.16};
		double ret;
		//		if (b.speed > 0)
		ret = dist[(b.rotation)%6]/std::abs(b.speed);
		//		else
		//			ret = dist[(b.rotation)%6]/(fabs(b.speed)*0.9);
		//	printf("rot: %d; speed: %d; val: %f\n", b.rotation, b.speed, ret);
		return ret;
	}
	else if (motionModel == kBetterTank)
	{
		deltaSpeedHeading s = GetAction(a, b);
		return GCost(a, s);
//		double dist[6] = {1.0, 3.16, 3.61, 1.42, 3.61, 3.16};
//		double ret;
//		if (b.speed-a.speed > 0)
//			ret = dist[(b.rotation)%6];///fabs(b.speed-a.speed);
//		else
//			ret = dist[(a.rotation)%6];///fabs(b.speed-a.speed);
//		//	printf("rot: %d; speed: %d; val: %f\n", b.rotation, b.speed, ret);
//		return ret;
	}
	assert(false);
	return 0;
}

bool Directional2DEnvironment::GoalTest(const xySpeedHeading &node, const xySpeedHeading &goal) const
{
	if (test && test->goalTest(node))
		return true;
	return (node == goal);
//	return (((floor(node.x) == floor(goal.x)) && (floor(node.y) == floor(goal.y))) &&
//			(node.speed == 0) && (node.rotation == goal.rotation));
}

uint64_t Directional2DEnvironment::GetStateHash(const xySpeedHeading &node) const
{
	// rotation is 0..15
	// speed is 0..3
	//	float x;	float y;	uint8_t speed;	uint8_t rotation;
	if (motionModel == kBetterTank)
	{
		return (uint64_t)((uint64_t)node.x<<32)|((uint64_t)node.y<<16)|((uint64_t)node.rotation);
	}
	uint64_t hval = (((int)node.x*4)<<16)|((int)node.y*4);
	hval = (uint64_t)((uint64_t)hval<<32)|((uint64_t)node.rotation<<8)|(uint64_t)(node.speed+4);
	return hval;
}

uint64_t Directional2DEnvironment::GetActionHash(deltaSpeedHeading act) const
{
	return ((act.turn+4)<<8)+(act.speed+4);
}

//void Directional2DEnvironment::OpenGLDraw() const
//{
////	std::cout<<"drawing\n";
//	map->OpenGLDraw();
//}
//
//
//void Directional2DEnvironment::OpenGLDraw(const xySpeedHeading& oldState, const xySpeedHeading &newState, float perc) const
//{
//	int DEG = 16;
//	if ((motionModel == kTank) || (motionModel == kBetterTank))
//		DEG = 24;
//	GLfloat r, g, b, t;
//	GetColor(r, g, b, t);
//
//	GLdouble xx, yy, zz, rad;
//	map->GetCoord(perc*newState.x + (1-perc)*oldState.x, perc*newState.y + (1-perc)*oldState.y, xx, yy, zz, rad);
//	
//	float rot = (1-perc)*oldState.rotation+perc*newState.rotation;
//	if ((oldState.rotation >= DEG-4) && (newState.rotation <= 5))
//	{
//		rot = (1-perc)*oldState.rotation+perc*(newState.rotation+DEG);
//		if (rot >= DEG)
//			rot -= DEG;
//	}
//	else if ((newState.rotation >= DEG-4) && (oldState.rotation <= 5))
//	{
//		rot = (1-perc)*(oldState.rotation+DEG)+perc*(newState.rotation);
//		if (rot >= DEG)
//			rot -= DEG;
//	}
//	GLdouble yoffset = sin(TWOPI*rot/DEG)*rad;
//	GLdouble xoffset = cos(TWOPI*rot/DEG)*rad;
//
//	glBegin(GL_TRIANGLES);
//	glColor4f(r, g, b/2, t);
//	glVertex3f(xx+xoffset, yy+yoffset, zz);
//	glColor4f(r, g/2, b, t);
//	glVertex3f(xx-xoffset, yy-yoffset, zz-rad);
//	glColor4f(r, g, b/2, t);
//	glVertex3f(xx-xoffset+0.5*yoffset, yy-yoffset-0.5*xoffset, zz);
//	
//	glColor4f(r, g/2, b, t);
//	glVertex3f(xx+xoffset, yy+yoffset, zz);
//	glColor4f(r, g, b/2, t);
//	glVertex3f(xx-xoffset, yy-yoffset, zz-rad);
//	glColor4f(r, g/2, b, t);
//	glVertex3f(xx-xoffset-0.5*yoffset, yy-yoffset+0.5*xoffset, zz);
//	glEnd();
//}
//
//
//void Directional2DEnvironment::OpenGLDraw(const xySpeedHeading &l) const
//{
//	GLdouble xx, yy, zz, rad;
//	GLfloat r, g, b, t;
//	GetColor(r, g, b, t);
//	map->GetCoord(l.x, l.y, xx, yy, zz, rad);
//
//	GLdouble yoffset = mySin(l.rotation)*rad;//sin(TWOPI*rot/16)*rad;
//	GLdouble xoffset = myCos(l.rotation)*rad;//cos(TWOPI*rot/16)*rad;
//
////	glColor3f(0, 0, 1.0);
////	glBegin(GL_LINE_STRIP);
////	glVertex3f(xx-rad, yy-rad, zz-rad);
////	glVertex3f(xx-rad, yy-rad, zz);
////	glEnd();
//	
//	glBegin(GL_TRIANGLES);
//	Graphics::point surfaceNormal;
//	surfaceNormal.x = (((-0.5*xoffset) * (-rad)) - ((+rad) - (-2*yoffset)));
//	surfaceNormal.y = (((rad) * (-2*xoffset)) - ((0.5*yoffset) - (rad)));
//	surfaceNormal.z = (((0.5*yoffset) * (-2*yoffset)) - ((-0.5*xoffset) - (-2*xoffset)));
//	surfaceNormal.normalise();
//	glNormal3f(surfaceNormal.x, surfaceNormal.y, surfaceNormal.z);
//	glColor4f(r, g, b/2, t);
//	glVertex3f(xx+xoffset, yy+yoffset, zz);
//	glColor4f(r, g/2, b, t);
//	glVertex3f(xx-xoffset, yy-yoffset, zz-rad);
//	glColor4f(r, g, b/2, t);
//	glVertex3f(xx-xoffset+0.5*yoffset, yy-yoffset-0.5*xoffset, zz);
//	
//	surfaceNormal.x = (((+0.5*xoffset) * (-rad)) - ((+rad) - (-2*yoffset)));
//	surfaceNormal.y = (((rad) * (-2*xoffset)) - ((-0.5*yoffset) - (rad)));
//	surfaceNormal.z = (((-0.5*yoffset) * (-2*yoffset)) - ((+0.5*xoffset) - (-2*xoffset)));
//	surfaceNormal.normalise();
//	glNormal3f(surfaceNormal.x, surfaceNormal.y, surfaceNormal.z);
//	glColor4f(r, g/2, b, t);
//	glVertex3f(xx+xoffset, yy+yoffset, zz);
//	glColor4f(r, g, b/2, t);
//	glVertex3f(xx-xoffset, yy-yoffset, zz-rad);
//	glColor4f(r, g/2, b, t);
//	glVertex3f(xx-xoffset-0.5*yoffset, yy-yoffset+0.5*xoffset, zz);
//	glEnd();	
//}
//
//
//void Directional2DEnvironment::OpenGLDraw(const xySpeedHeading& l, const deltaSpeedHeading &) const
//{
//	GLdouble xx, yy, zz, rad;
//	map->GetCoord(l.x, l.y, xx, yy, zz, rad);
//	glColor3f(0.5f, 0.5f, 0.5);
//	DrawSphere(xx-rad+l.x, yy-rad+l.y, zz, rad);
//}
//
//
//void Directional2DEnvironment::GLDrawLine(const xySpeedHeading &a, const xySpeedHeading &b) const
//{
//	GLdouble xx, yy, zz, rad;
//	map->GetCoord(a.x, a.y, xx, yy, zz, rad);
//	
//	GLfloat rr, gg, bb, t;
//	GetColor(rr, gg, bb, t);
//	glColor4f(rr, gg, bb, t);
//	
//	glBegin(GL_LINES);
//	glVertex3f(xx, yy, zz-rad/2);
//	map->GetCoord(b.x, b.y, xx, yy, zz, rad);
//	glVertex3f(xx, yy, zz-rad/2);
//	glEnd();
//}

void Directional2DEnvironment::GetNextState(const xySpeedHeading &, deltaSpeedHeading , xySpeedHeading &) const
{
	assert(false);
}

//int Directional2DEnvironment::GetModelUniqueAngles()
//{
//	const int angles[3] = {3, 3, 4};
//	return angles[motionModel];
//}

void Directional2DEnvironment::BuildHTable(dirHeuristicTable &t)
{
	const int speedCnt[3] = {3, 5, 3}; // number of possible speeds
	const int speeds[3] = {0, 1, 1}; // offset to make all speeds positive
	const int angles[3] = {16, 16, 24};
	checkLegal = false;
	// rotation is 0..15
	// speed is 0..3

	// 9x9
	//std::vector<std::vector<int> > hTable;
//	hTable.resize(GetModelUniqueAngles());
	t.hTable.resize(ROW_SIZE*ROW_SIZE);
	for (unsigned int x = 0; x < t.hTable.size(); x++)
	{
		t.hTable[x].resize(speedCnt[motionModel]*angles[motionModel]); // 64 = 4 speeds * 16 angles
		for (unsigned int y = 0; y < t.hTable[x].size(); y++)
		{
			t.hTable[x][y] = -1;
		}
	}
	// goal is in the middle
//	hTable[(hTable.size()-1)/2][0] = 0;
	xySpeedHeading base(0, 0);
	base.speed = t.speed;
	base.rotation = t.rotation;
	int i1, i2;
	if (LookupStateHashIndex(base, i1, i2))
	{
		t.hTable[i1][i2] = 0;
	}
	else {
		assert(!"Directional2DEnvironment::BuildHTable - Illegal base state");
	}
	
//	int i2 = 0+((0+speeds[motionModel])*angles[motionModel]);
//	t.hTable[(t.hTable.size()-1)/2][i2] = 0;
	
//	if (motionModel == kTank)
//	{
//		BuildAStarTable();
//		checkLegal = true;
//		return;
//	}
	
	//for (int t = 0; t <= TABLE_SIZE*TABLE_SIZE; t++)
	int updates = 1;
	while (updates != 0)
	{
//		printf("%d updates\n", updates);
		updates = 0;
		for (unsigned int x = 0; x < t.hTable.size(); x++)
		{
			for (unsigned int y = 0; y < t.hTable[x].size(); y++)
			{
				xySpeedHeading s;
				s.x = (float)(x%ROW_SIZE)-(TABLE_SIZE-0.5);//3.5f;
				s.y = (float)(x/ROW_SIZE)-(TABLE_SIZE-0.5);//3.5f;
				s.rotation = y%angles[motionModel]; ////y&0xF;
				s.speed = y/angles[motionModel]-speeds[motionModel]; ////(y>>4)&0x3;
				//printf("Unhashing rot: %d speed: %d from %d\n", s.rotation, s.speed, y);
				
//				std::cout << "Getting successors of " << s << ":\n";

				std::vector<xySpeedHeading> succ;
				GetSuccessors(s, succ);
				for (unsigned int z = 0; z < succ.size(); z++)
				{
					float val = LookupStateHash(succ[z], t);
//					std::cout << succ[z] << " has val " << val << "\n";
					if (val != -1)
					{
						if (t.hTable[x][y] == -1)
						{
//							int x1 = floorf(s.x);
//							int y1 = floorf(s.y);
//							int index1 = (x1+TABLE_SIZE)+(y1+TABLE_SIZE)*(ROW_SIZE);
//							int index2 = (s.rotation&0xF)|((s.speed&0x3)<<4);
//							std::cout << "{" << index1 << ", " << index2 << "} ";
							
							t.hTable[x][y] = val+GCost(s, succ[z]);
							updates++;
//							std::cout << "Assigning " << val+GCost(s, succ[z]) << " to " << s <<
//							" (" << x << ", " << y << ")  from " << succ[z] << std::endl;
						}
						else {
							if (fless(val+GCost(s, succ[z]), t.hTable[x][y]))
							{
//								std::cout << "Updating " << val+GCost(s, succ[z]) << " to " << s <<
//								" (" << x << ", " << y << ")  from " << succ[z] << std::endl;
								t.hTable[x][y] = std::min(t.hTable[x][y], (float)(val+GCost(s, succ[z])));
								updates++;
							}
						}
					}
				}
//				if ((t == 15) && (hTable[x][y] == -1))
//					hTable[x][y] = 16;
			}
		}		
	}

//	for (unsigned int x = 0; x < hTable.size(); x++)
//	{
//		for (unsigned int y = 0; y < hTable[x].size(); y++)
//		{
//			xySpeedHeading s;
//			s.x = (float)(x%ROW_SIZE)-(TABLE_SIZE-0.5);//3.5f;
//			s.y = (float)(x/ROW_SIZE)-(TABLE_SIZE-0.5);//3.5f;
//			s.rotation = y%angles[motionModel]; ////y&0xF;
//			s.speed = y/angles[motionModel]-speeds[motionModel]; ////(y>>4)&0x3;
//			std::cout << s << " has value of: " << hTable[x][y] << std::endl;
//		}
//		std::cout << "---------------" << std::endl;
//	}
	checkLegal = true;
}

bool Directional2DEnvironment::LookupStateHashIndex(const xySpeedHeading &s,
													int &index1, int &index2) const
{
	const int speeds[4] = {0, 1, 1, 1}; // offset to make all speeds positive
	const int angles[4] = {16, 16, 24, 24};
	
	int x = floorf(s.x);
	int y = floorf(s.y);
	
	if ((x < -TABLE_SIZE) || (x > TABLE_SIZE) || (y < -TABLE_SIZE) || (y > TABLE_SIZE))
	{
		return false;
	}
	index1 = (x+TABLE_SIZE)+(y+TABLE_SIZE)*(ROW_SIZE);
	index2 = (s.rotation)+((s.speed+speeds[motionModel])*angles[motionModel]);
	return true;
}

float Directional2DEnvironment::LookupStateHash(const xySpeedHeading &s, const dirHeuristicTable &t) const
{
	const int speeds[4] = {0, 1, 1, 1}; // offset to make all speeds positive
	const int angles[4] = {16, 16, 24, 24};

	int x = floorf(s.x);
	int y = floorf(s.y);
	
	if ((x < -TABLE_SIZE) || (x > TABLE_SIZE) || (y < -TABLE_SIZE) || (y > TABLE_SIZE))
	{
		return -1;
	}
	int index1 = (x+TABLE_SIZE)+(y+TABLE_SIZE)*(ROW_SIZE);
	int index2 = (s.rotation)+((s.speed+speeds[motionModel])*angles[motionModel]);
	//printf("Hashing rot: %d speed: %d to %d\n", s.rotation, s.speed, index2);
	////int index2 = (s.rotation&0xF)|((s.speed&0x3)<<4);
	return t.hTable[index1][index2];
}

float Directional2DEnvironment::LookupStateHeuristic(const xySpeedHeading &s1, const xySpeedHeading &s2) const
{
	const int angles[4] = {16, 16, 24, 24};
	const int angles90[4] = {4, 4, 6, 6};

	if (hType == kOctileHeuristic)
		return 0;

//	if (s2.speed != 0)
//		return 0;
	
	xySpeedHeading s3;
	s3 = s1;
	s3.x = floor(s3.x) - floor(s2.x);
	s3.y = floor(s3.y) - floor(s2.y);

	int whichHeuristic = -1;
	for (unsigned int x = 0; x < heuristics.size(); x++)
	{
		if (heuristics[x].rotation == s2.rotation)
		{
			//printf("Found exact match\n");
			whichHeuristic = x;
			break;
		}
		else if ((heuristics[x].rotation%angles90[motionModel]) == s2.rotation%angles90[motionModel])
		{
			//printf("Found 90 degree match [%d] Our goal: %d\n", heuristics[x].rotation, s2.rotation);
			
			// rotate so they match
			int rotation = -heuristics[x].rotation+s2.rotation;
			if (rotation < 0)
				rotation += angles[motionModel];
			//std::cout << "Before rotation: " << s3 << std::endl;
			RotateCCW(s3, rotation);
			//std::cout << "After rotation: " << s3 << std::endl;
			//assert(heuristics[x].rotation == s3.rotation);
			whichHeuristic = x;
			break;
		}
	}
	if (whichHeuristic == -1)
	{
		assert(!"No heuristic built");
		return 0;
//		// old code that breaks const correctness
//		int which = heuristics.size();
//		heuristics.resize(which+1);
//		// build new heuristic
//		heuristics[which].speed = 0;
//		heuristics[which].rotation = s2.rotation;
//		BuildHTable(heuristics[which]);
//		whichHeuristic = which;
		
	}
	
//	std::cout << "Heuristic looking up: " << s3 << std::endl;
	float val = LookupStateHash(s3, heuristics[whichHeuristic]);
	if (val != -1)
	{
//		printf("Looked up %d\n", val);
		return val;
	}
	if (hType == kPerimeterHeuristic)
		return 0;

	s3.x/=2;
	s3.y/=2;
	val = LookupStateHash(s3, heuristics[whichHeuristic]);
	if (val != -1)
	{
		if (motionModel != kVehicle)
			return 2.0*val-2;
		else
			return 1.5*val;
	}

	return 0;
}

//void Directional2DEnvironment::BuildAStarTable()
//{
//	//const int speedCnt[3] = {3, 5, 3}; // offset to make all speeds positive
//	const int speeds[3] = {0, 1, 1}; // offset to make all speeds positive
//	const int angles[3] = {16, 16, 24};
//	
//	heuristicType h = hType;
//	hType = kOctileHeuristic;
//
//	TemplateAStar<xySpeedHeading, deltaSpeedHeading, Directional2DEnvironment> a;
//	std::vector<xySpeedHeading> path;
//	xySpeedHeading goal;
//	xySpeedHeading start;
//	goal.x = 0.5;
//	goal.y = 0.5;
//	goal.rotation = angles[motionModel]/2;
//	goal.speed = 0;
//	start.x = 0.5;
//	start.y = 0.5;
//	start.rotation = 0;
//	start.speed = 5;
//	a.SetStopAfterGoal(false);
//	a.InitializeSearch(this, goal, start, path);
//	while (!a.DoSingleSearchStep(path))
//	{ static int cnt = 0; if ((++cnt)%1000 == 0) printf("%d\n", cnt); if (cnt > 100000) break; }
//
//	for (unsigned int x = 0; x < hTable.size(); x++)
//	{
//		printf("Entry %d of %d\n", x, (int)hTable.size());
//		for (unsigned int y = 0; y < hTable[x].size(); y++)
//		{
//			//printf("Sub-entry %d of %d\n", y, (int)hTable[x].size());
//			xySpeedHeading s;
//			s.x = (float)(x%ROW_SIZE)-(TABLE_SIZE-0.5);//3.5f;
//			s.y = (float)(x/ROW_SIZE)-(TABLE_SIZE-0.5);//3.5f;
//			// reverse rotation for reverse search
//			s.rotation = (angles[motionModel]/2 + y%angles[motionModel])%angles[motionModel]; ////y&0xF;
//			s.speed = y/angles[motionModel]-speeds[motionModel]; ////(y>>4)&0x3;
//			double cost;
//			a.GetClosedListGCost(s, cost);
//
//			//a.GetPath(this, s, goal, path);
//			
//			hTable[x][y] = cost; //GetPathLength(path);
//		}
//	}		
//	
//	
//	hType = h;
//}

void Directional2DEnvironment::BuildAngleTables()
{
	int DEG = 16;
	if ((motionModel == kTank) || (motionModel == kBetterTank))
		DEG = 24;
	for (int x = 0; x < DEG; x++)
	{
		sinTable.push_back(sin(TWOPI*((float)x)/(float)DEG));
		//printf("sin(%d) = %f\n", x, sinTable.back());
		cosTable.push_back(cos(TWOPI*((float)x)/(float)DEG));
		//printf("cos(%d) = %f\n", x, cosTable.back());
	}
}

float Directional2DEnvironment::mySin(int dir) const
{
	return sinTable[dir];
}

float Directional2DEnvironment::myCos(int dir) const
{
	return cosTable[dir];
}

