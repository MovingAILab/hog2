/*
 *  RoboticArm.cpp
 *  hog2
 *
 *  Created by Nathan Sturtevant on 11/15/08.
 *  Copyright 2008 __MyCompanyName__. All rights reserved.
 *
 */

#include "RoboticArm.h"
#include "TemplateAStar.h"
#include "Constants.h"
#include <string.h>

tRotation armRotations::GetRotation(int which) const
{
	switch ((rotations>>(2*which))&0x3)
	{
		case 1: return kRotateCCW;
		case 0: return kNoRotation;
		case 2: return kRotateCW;
	}
	return kNoRotation;
}

void armRotations::SetRotation(int which, tRotation dir)
{
	int value = 0;
	switch (dir)
	{
		case kRotateCW: value = 2; break;
		case kRotateCCW: value = 1; break;
		case kNoRotation: value = 0; break;
	}
	rotations = (rotations&(~((0x3)<<(2*which))))|(value<<(2*which));
}

int armAngles::GetAngle(int which) const
{
	return (angles>>(10*which))&0x3FF;
}

void armAngles::SetAngle(int which, int value)
{
	value %= 1024; if ( value < 0 ) { value += 1024; }
	value -= value & 1;
	uint64_t mask = 1023ll<<(10*which);
	uint64_t val = ((uint64_t)value)<<(10*which);
	angles = (angles&~mask)|val;
}

int armAngles::GetNumArms() const
{
	return angles>>60;
}

void armAngles::SetNumArms(int count)
{
	uint64_t mask = 0xFFFFFFFFFFFFFFFull;
	uint64_t newCnt = count;
	newCnt = newCnt << 60;
	assert(count <= 6);
	angles = newCnt|(angles&mask);
}

void armAngles::SetGoal(float x, float y)
{
	uint64_t fixedDecx, fixedFracx;
	uint64_t fixedDecy, fixedFracy;
	x += 512; // avoid unsigned issues...
	y += 512;
	fixedDecx = ((uint32_t)x)&0x3FF; // 10 bits
	fixedFracx = (int)((x-(double)fixedDecx)*1024.0*1024.0); // 20 bits
	fixedDecy = ((uint32_t)y)&0x3FF; // 10 bits
	fixedFracy = (int)((y-(double)fixedDecy)*1024.0*1024.0); // 20 bits
	angles = (0xFll<<60)|(fixedDecx<<50)|(fixedFracx<<30)|(fixedDecy<<20)|fixedFracy;
}

void armAngles::GetGoal(float &x, float &y) const
{
	assert(IsGoalState());
	
	x = ((angles>>50)&0x3FF) + ((angles>>30)&0xFFFFF)/(1024.0*1024.0);
	y = ((angles>>20)&0x3FF) + ((angles)&0xFFFFF)/(1024.0*1024.0);
	x-= 512;
	y-= 512;
}


bool armAngles::IsGoalState() const
{
	return ((angles>>60) == 0xF);
}


RoboticArm::RoboticArm(int dof, double armlength, double fTolerance)
:DOF(dof), armLength(armlength), tolerance(fTolerance)
{
	BuildSinCosTables();
	ce = new ConfigEnvironment();
}

RoboticArm::~RoboticArm()
{
	delete ce;
}

void RoboticArm::GetTipPosition(const armAngles &s, double &x, double &y ) const
{
	static std::vector<line2d> armSegs;
	GenerateLineSegments( s, armSegs );
	Graphics::point a = armSegs.back().end;
	x = a.x;
	y = a.y;
}

int RoboticArm::TipPositionIndex(armAngles &s,
								 const double minX, const double minY,
								 const double width )
{
	int idx;
	double x, y;
	GetTipPosition(s, x, y);
	// if we had a guarantee that width was a multiple of
	// tolerance, we could do a bunch of simplification
	idx = (int)floor( ( y - minY ) / GetTolerance() );
	idx *= (int)floor( width / GetTolerance() );
	idx += (int)floor( ( x - minX ) / GetTolerance() );
	
	return idx;
}

void RoboticArm::GetSuccessors(const armAngles &nodeID, std::vector<armAngles> &neighbors) const
{
	neighbors.resize(0);
//	for (int x = 0; x < nodeID.GetNumArms(); x++)
//	{
//		armAngles s = nodeID;
//		armRotations a;
//		a.SetRotation(x, kRotateCW);
//		ApplyAction(s, a);
//
//		if (LegalState(s))
//			neighbors.push_back(s);
//		
//		s = nodeID;
//		a.SetRotation(x, kRotateCCW);
//		ApplyAction(s, a);
//
//		if (LegalState(s))
//			neighbors.push_back(s);
//	}
	int maxVal = pow(3.0, nodeID.GetNumArms());

	double tipx, tipy;
	GetTipPosition(nodeID, tipx, tipy);

	for (int x = 1; x < maxVal; x++)
	{
		armAngles s = nodeID;
		armRotations a;
		int tmp = x;
		for (int y = 0; y < s.GetNumArms(); y++)
		{
			switch (tmp%3)
			{
				case 0: a.SetRotation(y, kNoRotation); break;
				case 1: a.SetRotation(y, kRotateCW); break;
				case 2: a.SetRotation(y, kRotateCCW); break;
			}
			tmp/=3;
		}
		
		ApplyAction(s, a);

		if (LegalAction(s, tipx, tipy))
			neighbors.push_back(s);
	}
}

void RoboticArm::GetActions(const armAngles &nodeID, std::vector<armRotations> &actions) const
{
	actions.resize(0);
	for (int x = 0; x < nodeID.GetNumArms(); x++)
	{
		armAngles s = nodeID;
		armRotations a;
		a.SetRotation(x, kRotateCW);
		ApplyAction(s, a);
		
		if (LegalState(s))
			actions.push_back(a);
		
		s = nodeID;
		a.SetRotation(x, kRotateCCW);
		ApplyAction(s, a);
		
		if (LegalState(s))
			actions.push_back(a);
	}

//	actions.resize(0);
//	for (int x = 0; x < nodeID.GetNumArms(); x++)
//	{
//		armAngles a(nodeID);
//		a.SetAngle(x, nodeID.GetAngle(x)+2);
//		if (LegalState(a))
//		{
//			armRotations rot;
//			rot.SetRotation(x, kRotateCW);
//			actions.push_back(rot);
//		}
//		a.SetAngle(x, nodeID.GetAngle(x)-2);
//		if (LegalState(a))
//		{
//			armRotations rot;
//			rot.SetRotation(x, kRotateCCW);
//			actions.push_back(rot);
//		}
//	}
}

armRotations RoboticArm::GetAction(const armAngles &s1, const armAngles &s2) const
{
	armRotations ar;
	for (int x = 0; x < s1.GetNumArms(); x++)
	{
		ar.SetRotation(x, (tRotation)(s2.GetAngle(x)-s1.GetAngle(x)));
	}
	return ar;
}

void RoboticArm::ApplyAction(armAngles &s, armRotations dir) const
{
	armAngles newState = s;
	for (int x = 0; x < newState.GetNumArms(); x++) {
		newState.SetAngle(x, newState.GetAngle(x)+2*dir.GetRotation(x));
	}
	//if (LegalState(newState))
	s = newState;
}

bool RoboticArm::InvertAction(armRotations &a) const
{
	for (int x = 0; x < 6; x++)
		a.SetRotation(x, (tRotation)(-(int)a.GetRotation(x)));
	return true;
}


void RoboticArm::AddObstacle(line2d obs)
{
	obstacles.push_back(obs);
	ce->AddObstacle(obs);
//	printf("Found solution %d moves; length %f, %d nodes expanded\n",
//		   states.size(), ce->GetPathLength(states), astar.GetNodesExpanded());
}

void RoboticArm::PopObstacle()
{
	if (obstacles.size() > 0)
	{
		obstacles.pop_back();
		ce->PopObstacle();
	}
}

armAngles RoboticArm::GetRandomState()
{
	armAngles ar;
	ar.SetNumArms(DOF);
	for (int x = 0; x < DOF; x++)
		ar.SetAngle(x, 2*(random()%512));
	return ar;
}

double RoboticArm::HCost(const armAngles &node1, const armAngles &node2) const
{
	double h;
	if (!node1.IsGoalState() && !node2.IsGoalState())
	{
		double val = 0;
		for (int x = 0; x < node1.GetNumArms(); x++)
		{
			double tmp = abs(node1.GetAngle(x) - node2.GetAngle(x));
			val += tmp/2;
		}
		//printf("Default heur: %f\n", val);
		for (unsigned int x = 0; x < heuristics.size(); x++)
			val = max(val, heuristics[x]->HCost(node1, node2));
		return val;
	}
	
	if (node1.IsGoalState()) return HCost(node2, node1);
	assert(node2.IsGoalState());

	std::vector<line2d> armSegments1;
	GenerateLineSegments(node1, armSegments1);
	Graphics::point a = armSegments1.back().end;
	float x, y;
	node2.GetGoal(x, y);
	double actDistance = sqrt((x-a.x)*(x-a.x)+(y-a.y)*(y-a.y));
	double movementAmount = (node1.GetNumArms()*armLength*sin(TWOPI*4.0/1024.0));

	{
		Graphics::point g(x, y, 0);
		ce->StoreGoal(g);
		localAStar.GetPath(ce, a, g, states);
		actDistance = max(actDistance, ce->GetPathLength(states));
	}
	
	h = actDistance / movementAmount;

	return h;

#if 0
	Graphics::point a, b;
	if (node1.IsGoalState())
		node1.GetGoal(a.x, a.y);
	else {
		GenerateLineSegments(node1, armSegments);
		a = armSegments.back().end;
	}
	
	if (node2.IsGoalState())
		node2.GetGoal(b.x, b.y);
	else {
		GenerateLineSegments(node2, armSegments);
		b = armSegments.back().end;
	}
	return sqrt((a.x-b.x)*(a.x-b.x) + (a.y-b.y)*(a.y-b.y));
#endif
}

#if 0
double RoboticArm::GCost(const armAngles &node1, const armAngles &node2)
{
	Graphics::point a, b;
	if (node1.IsGoalState())
		node1.GetGoal(a.x, a.y);
	else {
		GenerateLineSegments(node1, armSegments);
		a = armSegments.back().end;
	}

	if (node2.IsGoalState())
		node2.GetGoal(b.x, b.y);
	else {
		GenerateLineSegments(node2, armSegments);
		b = armSegments.back().end;
	}
	return sqrt((a.x-b.x)*(a.x-b.x) + (a.y-b.y)*(a.y-b.y));
}
#endif

#if 0
double RoboticArm::GCost(const armAngles &node1, const armRotations &act)
{
	armAngles node2;
	GetNextState(node1, act, node2);
	return GCost(node1, node2);
}
#endif

bool RoboticArm::GoalTest(const armAngles &node, const armAngles &goal) const
{
	if (!goal.IsGoalState())
	{
		return (node == goal);
	}
	assert(goal.IsGoalState());
	GenerateLineSegments(node, armSegments);

	Graphics::point a;
	a = armSegments.back().end;
	float x, y;
	goal.GetGoal(x, y);
	return x-a.x <= tolerance && a.x-x < tolerance
	  && y-a.y <= tolerance && a.y-y < tolerance;
}

uint64_t RoboticArm::GetStateHash(const armAngles &node) const
{
	// want a perfect hash function
	//return node.angles;
	uint64_t res = 0;
	for (int x = 0; x < node.GetNumArms(); x++)
		res = (uint64_t)(res<<9)|((uint64_t)node.GetAngle(x)/2);
//	assert(res < 512*512*512);
	return res;
}

void RoboticArm::GetStateFromHash(uint64_t hash, armAngles &a) const
{
	for (int x = a.GetNumArms()-1; x >= 0; x--)
	{
		a.SetAngle(x, (hash&0x1FF)*2);
		hash >>= 9;
	}
}

uint64_t RoboticArm::GetActionHash(armRotations act) const
{
	return act.rotations;
}

void RoboticArm::Draw(Graphics::Display &display) const
{
	display.FillRect({-1, -1, 1, 1}, Colors::darkgray);
	
	for (unsigned int x = 0; x < obstacles.size(); x++)
	{
		display.DrawLine(point3d(obstacles[x].start.x, obstacles[x].start.y),
						 point3d(obstacles[x].end.x, obstacles[x].end.y),
						 0.01f, Colors::red);
//		DrawLine(obstacles[x]);
	}
}

void RoboticArm::Draw(Graphics::Display &display, const armAngles &a, int which, rgbColor c) const
{
	Graphics::point e;
	if (a.IsGoalState())
	{
		e.z = 0;
		a.GetGoal(e.x, e.y);
	}
	else {
		GenerateLineSegments(a, armSegments);
		
		int x = which;
		{
			display.DrawLine(point3d(armSegments[x].start.x, armSegments[x].start.y),
							 point3d(armSegments[x].end.x, armSegments[x].end.y),
							 0.01f, c);
		}
		e = armSegments.back().end;
	}

	display.FrameCircle(point3d(e.x, e.y), tolerance, Colors::green, 0.01f);
}

void RoboticArm::Draw(Graphics::Display &display, const armAngles &a) const
{
	Graphics::point e;
	if (a.IsGoalState())
	{
		e.z = 0;
		a.GetGoal(e.x, e.y);
	}
	else {
		GenerateLineSegments(a, armSegments);
		
		for (unsigned int x = 0; x < armSegments.size(); x++)
		{
			display.DrawLine(point3d(armSegments[x].start.x, armSegments[x].start.y),
							 point3d(armSegments[x].end.x, armSegments[x].end.y),
							 0.01f, Colors::white);
		}
		e = armSegments.back().end;
	}

	display.FrameCircle(point3d(e.x, e.y), tolerance, Colors::green, 0.01f);
}

void RoboticArm::Draw(Graphics::Display &display, const armAngles &, const armRotations &) const
{
	
}

//void RoboticArm::OpenGLDraw() const
//{
////	if (window == 1)
////	{
////		ce->OpenGLDraw(1);
////		return;
////	}
//	glBegin(GL_QUADS);
//	glColor3f(0, 0, 0.1);
//	glVertex3f(-1, -1, 0.1);
//	glVertex3f(1, -1, 0.1);
//	glVertex3f(1, 1, 0.1);
//	glVertex3f(-1, 1, 0.1);
//	glEnd();
//	
//	glColor3f(1, 0, 0);
//	for (unsigned int x = 0; x < obstacles.size(); x++)
//	{
//		DrawLine(obstacles[x]);
//	}
//}
//
//void RoboticArm::OpenGLDraw(const armAngles &a) const
//{
////	if (window == 1)
////	{
////		glColor3f(1, 1, 0);
////		for (unsigned int x = 1; x < states.size(); x++)
////		{
////			DrawLine(line2d(states[x-1], states[x]));
////		}
////		ce->OpenGLDraw(1);
////		return;
////	}
//
//	Graphics::point e;
//	if (a.IsGoalState())
//	{
//		e.z = 0;
//		a.GetGoal(e.x, e.y);
//	}
//	else {
//		GenerateLineSegments(a, armSegments);
//		
//		for (unsigned int x = 0; x < armSegments.size(); x++)
//		{
//			glColor3f(1, 1, 1);
//			//printf("Drawing line segment %d of %d\n", x, armSegments.size());
//			DrawLine(armSegments[x]);
//		}
//		e = armSegments.back().end;
//	}
//	
//	glBegin(GL_LINE_LOOP);
//	glColor3f(0, 1.0, 0);
//	glVertex3f(e.x+tolerance, e.y+tolerance, 0);
//	glVertex3f(e.x-tolerance, e.y+tolerance, 0);
//	glVertex3f(e.x-tolerance, e.y-tolerance, 0);
//	glVertex3f(e.x+tolerance, e.y-tolerance, 0);
//	glEnd();
//	
//}
//
//void RoboticArm::OpenGLDraw(const armAngles &, const armRotations &) const
//{
//}
//
////void RoboticArm::OpenGLDraw(const armAngles &, const armRotations &, GLfloat, GLfloat, GLfloat) const
////{
////}
////
////void RoboticArm::OpenGLDraw(const armAngles &, GLfloat, GLfloat, GLfloat) const
////{
////}
//
//void RoboticArm::DrawLine(line2d l) const
//{
//	glLineWidth(5);
//	glBegin(GL_LINES);
//	glVertex3f(l.start.x, l.start.y, 0);
//	glVertex3f(l.end.x, l.end.y, 0);
//	glEnd();
//	glLineWidth(1);
//}

void RoboticArm::GetNextState(const armAngles &currents, armRotations dir, armAngles &news) const
{
	news = currents;
	ApplyAction(news, dir);
}

bool RoboticArm::LegalAction(const armAngles &a, double tipx, double tipy) const
{
	GenerateLineSegments(a, armSegments);
	// build line from tip to tip after move
	line2d move = armSegments.back();
	move.start.x = tipx;
	move.start.y = tipy;

	// avoid letting the resolution of the move jump over line segments
	// Note that this only prevents the tip from going through, not the middle of the segment
	for (unsigned int y = 0; y < obstacles.size(); y++)
	{
		if (move.crosses(obstacles[y]))
			return false;
	}
	for (unsigned int y = 0; y < armSegments.size(); y++)
		if (move.crosses(armSegments[y]))
			return false;

	for (unsigned int x = 0; x < armSegments.size(); x++)
	{
		for (unsigned int y = 0; y < obstacles.size(); y++)
		{
			if (armSegments[x].crosses(obstacles[y]))
				return false;
		}
		for (unsigned int y = x+2; y < armSegments.size(); y++)
			if (armSegments[x].crosses(armSegments[y]))
				return false;

		if ((x > 0) && (a.GetAngle(x) == 0))
			return false;
	}
	return true;
}

bool RoboticArm::LegalState(armAngles &a) const
{
	GenerateLineSegments(a, armSegments);
	for (unsigned int x = 0; x < armSegments.size(); x++)
	{
		for (unsigned int y = 0; y < obstacles.size(); y++)
		{
			if (armSegments[x].crosses(obstacles[y]))
				return false;
		}
		for (unsigned int y = x+2; y < armSegments.size(); y++)
			if (armSegments[x].crosses(armSegments[y]))
				return false;

		if ((x > 0) && (a.GetAngle(x) == 0))
			return false;
	}
	return true;
}

bool RoboticArm::LegalArmConfig(armAngles &a) const
{
//	if (m_TableComplete)
//		return legals[a.GetAngle(1)][a.GetAngle(2)];
	GenerateLineSegments(a, armSegments);
	for (unsigned int x = 0; x < armSegments.size(); x++)
	{
		for (unsigned int y = x+2; y < armSegments.size(); y++)
			if (armSegments[x].crosses(armSegments[y]))
				return false;
		
		if ((x > 0) && (a.GetAngle(x) == 0))
			return false;
	}
	return true;
}

void RoboticArm::GenerateLineSegments(const armAngles &a, std::vector<line2d> &armSegments1) const
{
	armSegments1.resize(0);
	for (int x = 0; x < a.GetNumArms(); x++)
	{
		Graphics::point prev;
		Graphics::point start;
		Graphics::point end;

		//double angle = TWOPI*a.GetAngle(x)/1024.0;
		int angle = a.GetAngle(x);
		if (x == 0)
		{
			start.x = 0;
			start.y = 0;
			start.z = 0;
			prev = start;
			prev.y = -armLength;
		}
		else {
			start = armSegments1.back().end;
			prev = armSegments1.back().start;
		}

		// offset to origin
		prev.x -= start.x;
		prev.y -= start.y;
		
		end.x = prev.x*GetCos(angle) - prev.y*GetSin(angle) + start.x;
		end.y = prev.x*GetSin(angle) + prev.y*GetCos(angle) + start.y;
		end.z = 0;
		
		armSegments1.push_back(line2d(start, end));
	}
	assert(armSegments1.size() > 0);
}

double RoboticArm::GetSin(int angle) const
{
	return sinTable[angle];
}

double RoboticArm::GetCos(int angle) const
{
	return cosTable[angle];
}

void RoboticArm::BuildSinCosTables()
{
	sinTable.resize(1024);
	cosTable.resize(1024);
	for (int x = 0; x < 1024; x++)
	{
		sinTable[x] = sin(TWOPI*(double)x/1024.0);
		cosTable[x] = cos(TWOPI*(double)x/1024.0);
	}
}

ArmToTipHeuristic::ArmToTipHeuristic(RoboticArm *r)
{
	ra = r;
	m_TableComplete = false;
	legalStateTable = NULL;
	legalGoalTable = NULL;
	tipPositionTables = NULL;
	GenerateCPDB();
}

double ArmToTipHeuristic::HCost(const armAngles &node1, const armAngles &node2) const
{
	float x, y;
	node2.GetGoal(x, y);

	double h = 0;
	uint16_t tableH;
	for (unsigned i = 0; i < distancesTables.size(); ++i )
	{
		if (node1.GetNumArms() == tablesNumArms[ i ] )
		{
			tableH = UseHeuristic( node1, x, y,
								  distancesTables[ i ],
								  minTipDistancesTables[ i ],
								  maxTipDistancesTables[ i ] );
			if ((double)tableH > h )
			{
				h = (double)tableH;
			}
		}
	}
	return h;
}


void ArmToTipHeuristic::GenerateCPDB()
{
	int numArms = 3;
	//void GenerateLegalArmConfigs();
	armAngles a;
	a.SetNumArms(numArms);
	a.SetAngle(0, 0);
	// 0, 512, 512
	std::vector<std::vector<bool> > legals;
	legals.resize(512);
	for (unsigned int x = 0; x < legals.size(); x++)
	{
		legals[x].resize(512);
		printf("Building table for %d\n", x);
		for (unsigned int y = 0; y < legals[x].size(); y++)
		{
			a.SetAngle(1, 2*x);
			a.SetAngle(2, 2*y);
			legals[x][y] = ra->LegalArmConfig(a);
		}
	}
	m_TableComplete = true;
}


uint64_t ArmToTipHeuristic::ArmAnglesIndex( const armAngles &arm ) const
{
	uint64_t idx;
	int s;

	idx = 0;
	for ( s = 0; s < arm.GetNumArms(); ++s ) {
		idx <<= 9;
		idx |= arm.GetAngle( s ) >> 1;
	}

	return idx;
}

int ArmToTipHeuristic::TipPositionIndex( const double x, const double y,
										const double minX, const double minY,
										const double width ) const
{
	int idx;

	// if we had a guarantee that width was a multiple of
	// tolerance, we could do a bunch of simplification

	idx = (int)floor( ( y - minY ) / ra->GetTolerance() );
	idx *= (int)floor( width / ra->GetTolerance() );
	idx += (int)floor( ( x - minX ) / ra->GetTolerance() );

	return idx;
}

int ArmToTipHeuristic::WriteArmAngles(FILE *file, armAngles &a)
{
	int s;
	int16_t v;

	v = a.GetNumArms();
	if ( fwrite( &v, sizeof( v ), 1, file ) < 1 ) {
		return 0;
	}
	for ( s = 0; s < a.GetNumArms(); ++s ) {
		v = a.GetAngle( s );
		if ( fwrite( &v, sizeof( v ), 1, file ) < 1 ) {
			return 0;
		}
	}
	return 1;
}

int ArmToTipHeuristic::ReadArmAngles(FILE *file, armAngles &a)
{
	armAngles angles;
	int s;
	int16_t v;

	if ( fread( &v, sizeof( v ), 1, file ) < 1 ) {
		return 0;
	}
	angles.SetNumArms( v );
	for ( s = 0; s < angles.GetNumArms(); ++s ) {
		if ( fread( &v, sizeof( v ), 1, file ) < 1 ) {
			return 0;
		}
		angles.SetAngle( s, v );
	}

	a = angles;
	return 1;
}

void ArmToTipHeuristic::UpdateTipDistances( armAngles &arm, uint16_t distance,
				     uint16_t *minTipDistances,
				     uint16_t *maxTipDistances )
{
	int idx;
	double x, y;

	ra->GetTipPosition( arm, x, y );
	idx = TipPositionIndex( x, y, -1.0, -1.0, 2.0 );
	if ( distance < minTipDistances[ idx ] ) {
	  minTipDistances[ idx ] = distance;
	}
	if ( distance > maxTipDistances[ idx ] ) {
	  maxTipDistances[ idx ] = distance;
	}
}

// this works because of two things
// a) clockwise and counterclockwise are each others inverses
// and both actions are (potentially) legal moves for every arm segment
// b) action costs are uniform, so all children which have not
// been examined are in the next frontier
// returns the number of states in nextFile
uint64_t ArmToTipHeuristic::GenerateNextDepth( FILE *curFile, FILE *nextFile,
					uint16_t curDistance,
					uint16_t *distances,
					uint16_t *minTipDistances,
					uint16_t *maxTipDistances,
					armAngles &lastAdded )
{
	uint64_t count = 0, idx;
	unsigned i;
	armAngles arm;
	std::vector<armRotations> actions;

	while( ReadArmAngles( curFile, arm ) ) {
		ra->GetActions( arm, actions );
		for ( i = 0; i < actions.size(); ++i ) {
			armAngles child = arm;
			ra->ApplyAction( child, actions[ i ] );

			idx = ArmAnglesIndex( child );
			if ( distances[ idx ]
			    > curDistance + 1 ) {
				// new arm configuration

				distances[ idx ] = curDistance + 1;
				WriteArmAngles( nextFile, child );
				if ( minTipDistances ) {
				  UpdateTipDistances( child, curDistance + 1,
						      minTipDistances,
						      maxTipDistances );
				}
				++count;
			}
		}
	}

	lastAdded = arm;

	return count;
}


/* returns 1 if goal will generate a reasonable heuristic, 0 otherwise */
int ArmToTipHeuristic::GenerateHeuristicSub( const armAngles &sampleArm,
				      const bool quiet,
				      armAngles *goals, const int numGoals,
				      uint16_t *distances,
				      uint16_t *minTipDistances,
				      uint16_t *maxTipDistances,
				      armAngles &lastAdded )
{
	uint64_t count, i, total;
	int numArms = sampleArm.GetNumArms(), segment, g;
	uint16_t distance;
	armAngles arm;
	FILE *curFile, *nextFile;

	arm.SetNumArms( numArms );

	count = NumArmAnglesIndices( arm );
	for ( i = 0; i < count; ++i ) {
		distances[ i ] = 65535;
	}

	if ( minTipDistances ) {
		count = NumTipPositionIndices();
		for ( i = 0; i < count; ++i ) {
			minTipDistances[ i ] = 65535;
			maxTipDistances[ i ] = 0;
		}
	}

	nextFile = tmpfile();
	assert( nextFile != NULL );

	count = 0;
	total = 0;

	if ( !quiet ) {
		printf( "populating depth 0\n" );
	}
	for ( segment = 0; segment < numArms; ++segment ) {
		arm.SetAngle( segment, 0 );
	}
	while( 1 ) {
		if ( ra->LegalState( arm ) ) {
		  ++total;

		  for ( g = 0; g < numGoals; ++g ) {
		    if ( ra->GoalTest( arm, goals[ g ] ) ) {
		      // new distance 0 (goal) arm configuration

		      distances[ ArmAnglesIndex( arm ) ] = 0;
		      WriteArmAngles( nextFile, arm );
		      UpdateTipDistances( arm, 0, minTipDistances,
					  maxTipDistances );
		      ++count;
		      break;
		    }
		  }
		}

		// next configuration
		segment = 0;
		do {
			armRotations action;
			action.SetRotation( segment, kRotateCW );
			ra->ApplyAction( arm, action );
			if ( arm.GetAngle( segment ) != 0 ) {
				break;
			}
		} while( ++segment < numArms );
		if ( segment == numArms ) {
			// tried all configurations
			break;
		}
	}

	printf( "%" PRId64 " legal states\n", total );

	if ( !count ) {
	  fclose( nextFile );
	  return 0;
	}

	distance = 0;
	total = 0;
	do {
		total += count;
		if ( !quiet ) {
			printf( "%" PRId64 " states at distance %u\n",
				count, distance );
		}
		curFile = nextFile;
		rewind( curFile );
		nextFile = tmpfile();

		count = GenerateNextDepth( curFile, nextFile, distance,
					   distances, minTipDistances,
					   maxTipDistances, lastAdded );

		++distance;
		fclose( curFile );
	} while( count );
	fclose( nextFile );

	if ( !quiet ) {
		printf( "%" PRId64 " total states\n", total );
	}

	// there are a number of small disconnected subspaces,
	// so make sure goal doesn't correspond to this subspace!
	if ( distances[ ArmAnglesIndex( sampleArm ) ] == 65535 ) {
		// sample arm not reachable!
		return 0;
	}

	return 1;
}

/* this function generates a table-based heuristic function for
   the current environment */
void ArmToTipHeuristic::GenerateRandomHeuristic( const armAngles &sampleArm )
{
	uint16_t *distances, *minTipDistances, *maxTipDistances;
	armAngles goal, last;

	distances = new uint16_t[ NumArmAnglesIndices( sampleArm ) ];
	minTipDistances = new uint16_t[ NumTipPositionIndices() ];
	maxTipDistances = new uint16_t[ NumTipPositionIndices() ];

	do {
		goal.SetGoal( (double)random()/(double)RAND_MAX * 2.0 - 1.0,
			      (double)random()/(double)RAND_MAX * 2.0 - 1.0 );
	} while( !GenerateHeuristicSub( sampleArm, false, &goal, 1, distances,
					minTipDistances, maxTipDistances,
					last ) );

	distancesTables.push_back( distances );
	minTipDistancesTables.push_back( minTipDistances );
	maxTipDistancesTables.push_back( maxTipDistances );
	tablesNumArms.push_back( sampleArm.GetNumArms() );
}

/* this function generates a table-based heuristic function for
   the current environment */
int ArmToTipHeuristic::GenerateHeuristic( const armAngles &sampleArm,
				   armAngles &goal )
{
	uint16_t *distances, *minTipDistances, *maxTipDistances;
	armAngles last;

	distances = new uint16_t[ NumArmAnglesIndices( sampleArm ) ];
	minTipDistances = new uint16_t[ NumTipPositionIndices() ];
	maxTipDistances = new uint16_t[ NumTipPositionIndices() ];

	if ( !GenerateHeuristicSub( sampleArm, false, &goal, 1, distances,
				   minTipDistances, maxTipDistances, last ) ) {
		delete[] maxTipDistances;
		delete[] minTipDistances;
		delete[] distances;
		return 0;
	}

	distancesTables.push_back( distances );
	minTipDistancesTables.push_back( minTipDistances );
	maxTipDistancesTables.push_back( maxTipDistances );
	tablesNumArms.push_back( sampleArm.GetNumArms() );

	return 1;
}

/* this function generates a table-based heuristic function for
   the current environment */
int ArmToTipHeuristic::GenerateMaxDistHeuristics( const armAngles &sampleArm,
					   const int numHeuristics )
{
	int i, ret;
	uint16_t *distances, *minTipDistances, *maxTipDistances;
	armAngles last;
	armAngles *goals;
	goals = new armAngles[numHeuristics];
	double x, y;

	last = sampleArm;
	ra->GetTipPosition( last, x, y );
	last.SetGoal( x, y );
	for ( i = 0; i < numHeuristics; ++i ) {
		distances = new uint16_t[ NumArmAnglesIndices( sampleArm ) ];
		minTipDistances = new uint16_t[ NumTipPositionIndices() ];
		maxTipDistances = new uint16_t[ NumTipPositionIndices() ];

		printf( "generating next goal position\n" );
		if ( i ) {
		  ret = GenerateHeuristicSub( sampleArm, true, goals, i,
					      distances, minTipDistances,
					      maxTipDistances, goals[ i ] );
		} else {
		  ret = GenerateHeuristicSub( sampleArm, true, &last, 1,
					      distances, minTipDistances,
					      maxTipDistances, goals[ i ] );
		}
		if ( !ret ) {
			delete[] maxTipDistances;
			delete[] minTipDistances;
			delete[] distances;
			return i;
		}
		ra->GetTipPosition( goals[ i ], x, y );
		printf( "new goal position: (%lf,%lf)\n", x, y );
		goals[ i ].SetGoal( x, y );


		if ( !GenerateHeuristicSub( sampleArm, false, &goals[ i ], 1,
					   distances, minTipDistances,
					   maxTipDistances, last ) ) {
			delete[] maxTipDistances;
			delete[] minTipDistances;
			delete[] distances;
			return i;
		}

		distancesTables.push_back( distances );
		minTipDistancesTables.push_back( minTipDistances );
		maxTipDistancesTables.push_back( maxTipDistances );
		tablesNumArms.push_back( sampleArm.GetNumArms() );
	}
	delete []goals;
	return i;
}

uint16_t ArmToTipHeuristic::UseHeuristic(const armAngles &s, armAngles &g,
				   uint16_t *distances ) const
{
	uint16_t d_s, d_g;

	d_s = distances[ ArmAnglesIndex( s ) ];
	d_g = distances[ ArmAnglesIndex( g ) ];
	if ( d_s < d_g ) {
	  return d_g - d_s;
	}
	return d_s - d_g;
}

uint16_t ArmToTipHeuristic::UseHeuristic(const armAngles &arm,
				   double goalX, double goalY,
				   uint16_t *distances,
				   uint16_t *minTipDistances,
				   uint16_t *maxTipDistances ) const
{
	int32_t mind, maxd, t;	// 32 bits because of the subtraction below
	int i, j, index;
	double x, y;

	mind = 65535;
	maxd = 0;
	for ( y = goalY - ra->GetTolerance(), i = 0; i < 3; y += ra->GetTolerance(), ++i ) {
	  for ( x = goalX - ra->GetTolerance(), j = 0; j < 3; x += ra->GetTolerance(), ++j ) {
	    index =  TipPositionIndex( x, y, -1.0, -1.0, 2.0 );
	    if ( minTipDistances[ index ] < mind ) {
	      mind = minTipDistances[ index ];
	    }
	    if ( maxTipDistances[ index ] > maxd ) {
	      maxd = maxTipDistances[ index ];
	    }
	  }
	}

	t = distances[ ArmAnglesIndex( arm ) ];
//printf( "dist(s->g)=%u - max(dist(c->g))=%u\n", (unsigned)t, (unsigned)maxd );
//printf( "min(dist(c->g))=%u - dist(s->g)=%u\n", (unsigned)mind, (unsigned)t );
	maxd = t - maxd;
	mind = mind - t;
	if ( mind > maxd ) {
	  maxd = mind;
	}
	if ( maxd < 0 ) {
	  maxd = 0;
	}

//if ( maxd > 0 ) printf( "%u\n", (unsigned)maxd );

	return maxd;
}

bool ArmToTipHeuristic::ValidGoalPosition( double goalX, double goalY )
{
	int i, j, index;
	double x, y, tx, ty;

	if ( legalGoalTable == NULL ) {
		// no table to use, can't prove anything
		return false;
	}

	for ( y = goalY - ra->GetTolerance(), i = 0; i < 3; y += ra->GetTolerance(), ++i ) {
	  for ( x = goalX - ra->GetTolerance(), j = 0; j < 3; x += ra->GetTolerance(), ++j ) {
	    tx = x;
	    if ( tx < -1.0 ) {
	      tx = -1.0;
	    } else if ( tx >= 1.0 ) {
	      tx = 0.999999;
	    }
	    ty = y;
	    if ( ty < -1.0 ) {
	      ty = -1.0;
	    } else if ( ty >= 1.0 ) {
	      ty = 0.999999;
	    }

	    index =  TipPositionIndex( tx, ty, -1.0, -1.0, 2.0 );
	    if ( !( legalGoalTable[ index >> 3 ] & ( 1 << ( index & 7 ) ) ) ) {
	      // given one unreachable square, there's a possibility
	      // that even if the rest are reachable, they're
	      // only reachable because of states that are
	      // actually further than tolerance away from (x,y)
	      return false;
	    }
	  }
	}

	return true;
}

void ArmToTipHeuristic::GenerateLegalStateTable( armAngles &legalArm )
{
	uint64_t numStates, numTip, i;
	uint16_t *distances, *minTipDistances, *maxTipDistances, distance;
	uint8_t *lst, *lgt;
	armAngles arm, lastAdded;
	FILE *curFile, *nextFile;

	printf( "generating legal state table\n" );

	numStates = NumArmAnglesIndices( legalArm );
	numTip = NumTipPositionIndices();

	lst = new uint8_t[ ( numStates + 7 ) >> 3 ];
	memset( lst, 0, ( numStates + 7 ) >> 3 );
	lgt = new uint8_t[ ( numTip + 7 ) >> 3 ];
	memset( lgt, 0, ( numTip + 7 ) >> 3 );
	distances = new uint16_t[ numStates ];
	for ( i = 0; i < numStates; ++i ) {
		distances[ i ] = 65535;
	}
	minTipDistances = new uint16_t[ numTip ];
	maxTipDistances = new uint16_t[ numTip ];
	for ( i = 0; i < numTip; ++i ) {
		minTipDistances[ i ] = 65535;
		maxTipDistances[ i ] = 0;
	}

	nextFile = tmpfile();
	assert( nextFile != NULL );

	distances[ ArmAnglesIndex( legalArm ) ] = 0;
	UpdateTipDistances( legalArm, 0, minTipDistances, maxTipDistances );
	WriteArmAngles( nextFile, legalArm );

	distance = 0;
	do {
		curFile = nextFile;
		rewind( curFile );
		nextFile = tmpfile();

		i = GenerateNextDepth( curFile, nextFile, distance,
				       distances, minTipDistances,
				       maxTipDistances, lastAdded );

		++distance;
		fclose( curFile );
	} while( i );
	fclose( nextFile );
	printf( "done: maximum distance of %d from chosen state\n",
		(int)distance );

	for ( i = 0; i < numStates; ++i ) {
		if ( distances[ i ] < 65535 ) {
			lst[ i >> 3 ] |= 1 << ( i & 7 );
		}
	}

	for ( i = 0; i < numTip; ++i ) {
		if ( minTipDistances[ i ]
		    <= maxTipDistances[ i ] ) {
			lgt[ i >> 3 ] |= 1 << ( i & 7 );
		}
	}

	legalStateTable = lst;
	legalGoalTable = lgt;
	delete[] maxTipDistances;
	delete[] minTipDistances;
	delete[] distances;
}

void ArmToTipHeuristic::GenerateTipPositionTables( armAngles &sampleArm )
{
	int numArms = sampleArm.GetNumArms(), segment;
	double x, y;
	armAngles arm;

	printf( "generating tip position table\n" );

	if ( tipPositionTables != NULL ) {
		return;
	}

	tipPositionTables = new std::vector<armAngles>[ NumTipPositionIndices() ];

	arm.SetNumArms( numArms );

	for (segment = 0; segment < numArms; ++segment)
	{
		arm.SetAngle( segment, 0 );
	}

	while( 1 )
	{
		if ( ra->LegalState( arm ) )
		{
			ra->GetTipPosition( arm, x, y );
			tipPositionTables[TipPositionIndex( x, y, -1.0, -1.0, 2.0 )].push_back( arm );
		}

		// next configuration
		segment = 0;
		do {
			armRotations action;
			action.SetRotation( segment, kRotateCW );
			ra->ApplyAction( arm, action );
			if ( arm.GetAngle( segment ) != 0 ) {
				break;
			}
		} while( ++segment < numArms );
		if (segment == numArms )
		{
			// tried all configurations
			break;
		}
	}

}

ArmToArmHeuristic::ArmToArmHeuristic(RoboticArm *r, armAngles &initial, bool optimize)
{
	optimizeLocations = optimize;
	ra = r;
	GenerateLegalStates(initial);
}

double ArmToArmHeuristic::HCost(const armAngles &node1, const armAngles &node2) const
{
	if (node1.IsGoalState() || node2.IsGoalState())
	{
		//printf("Wrong problem type!\n");
		return 0.0;
	}
	double hval = 0.0;
	for (unsigned int x = 0; x < distances.size(); x++)
	{
		double nextval = abs(distances[x][ra->GetStateHash(node1)] - distances[x][ra->GetStateHash(node2)]);
//		printf("Heuristic %d: |%d-%d| = %f\n", x,
//			   distances[x][ra->GetStateHash(node1)],
//			   distances[x][ra->GetStateHash(node2)],
//			   nextval);
		hval = max(nextval, hval);
	}
	return hval;
}

int ArmToArmHeuristic::TipPositionIndex(const double x, const double y,
										const double minX, const double minY,
										const double width )
{
	int idx;
	
	// if we had a guarantee that width was a multiple of
	// tolerance, we could do a bunch of simplification
	idx = (int)floor( ( y - minY ) / ra->GetTolerance() );
	idx *= (int)floor( width / ra->GetTolerance() );
	idx += (int)floor( ( x - minX ) / ra->GetTolerance() );
	
	return idx;
}

void ArmToArmHeuristic::AddDiffTable()
{
	armAngles start = SelectStartNode();

	int which = distances.size();
	printf("Building new heuristic table [%d]\n", which);
	
	distances.resize(which+1);
	//distances[which].resize();
	std::deque<armAngles> q;
	q.push_back(start);
	canonicalStates.push_back(start);
	while (q.size() > 0)
	{
//		if ((cnt++%10000) == 0)
//			printf("(%d) Q size: %d\n", cnt, (int)q.size());
		armAngles a = q.front();
		q.pop_front();
		uint64_t hash = ra->GetStateHash(a);
		if (hash >= distances[which].size())
			distances[which].resize(hash+1);
		std::vector<armAngles> moves;
		ra->GetSuccessors(a, moves);
//		printf("Getting successors of node %lld at depth %d\n", hash, distances[which][hash]);
		for (unsigned int x = 0; x < moves.size(); x++)
		{
			uint64_t newHash = ra->GetStateHash(moves[x]);
			if (newHash >= distances[which].size())
				distances[which].resize(newHash+1);
			if (distances[which][newHash] != 0)
				continue;
			
			distances[which][newHash] = distances[which][hash]+1;
//			printf("Setting cost of node %lld to %d (1+%d from %lld)\n", newHash, distances[which][newHash],
//				   distances[which][hash], hash);
			q.push_back(moves[x]);
		}
	}
	printf("Done\n");
}

armAngles ArmToArmHeuristic::SelectStartNode()
{
	armAngles start;
	std::deque<armAngles> q;
	std::vector<bool> used;
	used.resize(512*512*512);
	if ((!optimizeLocations) || (distances.size() == 0))
	{
		do {
			start = ra->GetRandomState();
		} while (!IsLegalState(start));
	}
	if (!optimizeLocations)
		return start;
	if (distances.size() == 0)
	{
		used[ra->GetStateHash(start)] = true;
		q.push_back(start);
	}
	for (unsigned int x = 0; x < canonicalStates.size(); x++)
	{
		q.push_back(canonicalStates[x]);
		used[ra->GetStateHash(canonicalStates[x])] = true;
	}
	armAngles a;
	while (q.size() > 0)
	{
		a = q.front();
		q.pop_front();

		std::vector<armAngles> moves;
		ra->GetSuccessors(a, moves);
		for (unsigned int x = 0; x < moves.size(); x++)
		{
			if (used[ra->GetStateHash(moves[x])])
				continue;
			used[ra->GetStateHash(moves[x])] = true;
			q.push_back(moves[x]);
		}
	}
	return a;
}

bool ArmToArmHeuristic::IsLegalState(armAngles &arm)
{
	return legalStates[ra->GetStateHash(arm)];
}

void ArmToArmHeuristic::GenerateLegalStates(armAngles &init)
{
	int cnt = 0;
	//std::vector<bool> legalStates;
	printf("Getting legal states and tip positions\n");
	legalStates.resize(512*512*512);
	std::deque<armAngles> q;
	q.push_back(init);
	tipPositionTables.resize(200*200);
	while (q.size() > 0)
	{
		if ((cnt++%10000) == 0)
			printf("(%d) Q size: %d\n", cnt, (int)q.size());
		armAngles a = q.front();
		q.pop_front();
		std::vector<armAngles> moves;
		ra->GetSuccessors(a, moves);
		for (unsigned int x = 0; x < moves.size(); x++)
		{
			if (legalStates[ra->GetStateHash(moves[x])])
				continue;
			legalStates[ra->GetStateHash(moves[x])] = true;
			q.push_back(moves[x]);
			double x1, y1;
			ra->GetTipPosition(moves[x], x1, y1);
			int tipIndex = TipPositionIndex(x1, y1);
			tipPositionTables[tipIndex].push_back(moves[x]);
		}
	}
	printf("Done\n");
}

