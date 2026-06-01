//
//  Racetrack.cpp
//  hog2
//

#include "Racetrack.h"
#include "TemplateAStar.h"



std::ostream &operator<<(std::ostream &out, const RacetrackState &s)
{
	out << "{" << s.xLoc << ", " << s.yLoc << "}:(" << s.xVelocity << ", " << s.yVelocity << ")";
	return out;
}

bool operator==(const RacetrackState &l1, const RacetrackState &l2) {
	return
	((l1.xLoc == l2.xLoc) &&
	 (l1.yLoc == l2.yLoc) &&
	 (l1.xVelocity == l2.xVelocity) &&
	 (l1.yVelocity == l2.yVelocity));
}

bool operator!=(const RacetrackState &l1, const RacetrackState &l2) {
	return !(l1 == l2);
}

std::ostream &operator<<(std::ostream &out, const RacetrackMove &m)
{
	out << "(" << +m.xDelta << ", " << +m.yDelta << ")";
	return out;
}

bool operator==(const RacetrackMove &m1, const RacetrackMove &m2) {
	return (m1.xDelta == m2.xDelta) && (m1.yDelta == m2.yDelta);
}


Racetrack::Racetrack(Map *map)
:me(0)
{
	UpdateMap(map);
}


Racetrack::~Racetrack()
{
	delete me;
}

void Racetrack::UpdateMap(Map *map)
{
	delete me;
	me = new MapEnvironment(map);
	this->map = map;
	me->SetFourConnected();
	std::vector<xyLoc> path;
	TemplateAStar<xyLoc, tDirection, MapEnvironment> astar;
	astar.SetStopAfterGoal(false);
	int goals = 0;
	for (int y = 0; y < map->GetMapHeight(); y++)
	{
		for (int x = 0; x < map->GetMapWidth(); x++)
		{
			if (map->GetTerrainType(x, y) == kEndTerrain)
			{
				xyLoc f(x, y);
				if (goals == 0)
					astar.InitializeSearch(me, f, f, path);
				else
					astar.AddAdditionalStartState(f);
				goals++;
			}
		}
	}
	while (astar.GetNumOpenItems() > 0)
		astar.DoSingleSearchStep(path);
	heuristic.resize(map->GetMapWidth()*map->GetMapHeight());
	for (int x = 0; x < astar.GetNumItems(); x++)
	{
		auto i = astar.GetItem(x);
		heuristic[GetIndex(i.data.x, i.data.y)] = i.g;
	}
}

void Racetrack::Reset(RacetrackState &s) const
{
	s.xVelocity = 0;
	s.yVelocity = 0;
	for (int x = 0; x < map->GetMapWidth(); x++ )
	{
		for (int y = 0; y < map->GetMapHeight(); y++ )
		{
			if (map->GetTerrainType(x, y) == kStartTerrain)
			{
				s.xLoc = x;
				s.yLoc = y;
				return;
			}
		}
	}
	std::cout << "No start terrain found! \n";
	exit(1);
}


void Racetrack::GetSuccessors(const RacetrackState &nodeID, std::vector<RacetrackState> &neighbors) const
{
	static std::vector<RacetrackMove> actions;
	this->GetActions(nodeID, actions);
	neighbors.clear();

	for (int i=0; i < actions.size(); i++)
	{
		RacetrackState temp = nodeID; //resets the temporary state to its original
		ApplyAction(temp, actions[i]); // applies the action to get the state
		neighbors.push_back(temp); // adds the state to the vector
	}
}

void Racetrack::GetActions(const RacetrackState &nodeID, std::vector<RacetrackMove> &actions) const
{ 
	actions.clear();

//	RacetrackMove up(0, -1);
//	RacetrackMove down(0, 1);
//	RacetrackMove left(-1, 0);
//	RacetrackMove right(1, 0);
//	RacetrackMove same(0, 0);

	for (int x = -1; x <= 1; x++)
	{
		for (int y = -1; y <= 1; y++)
		{
			RacetrackMove m(x, y);
			if (Legal(nodeID, m))
				actions.push_back(m);
		}
	}
//	if (this->Legal(nodeID, up) == true)
//	{
//		actions.push_back(up);
//	}
//	if (this->Legal(nodeID, down) == true)
//	{
//		actions.push_back(down);
//	}
//	if (this->Legal(nodeID, left)==true)
//	{
//		actions.push_back(left);
//	}
//	if (this->Legal(nodeID, right)==true)
//	{
//		actions.push_back(right);
//	}
//	if (this->Legal(nodeID, same)==true)
//	{
//		actions.push_back(same);
//	}
}

void Racetrack::ApplyAction(RacetrackState &s, RacetrackMove a) const
{ // When x y velocity and action is applied -- location changes when velocity changes
	if (a.hitGoal)
	{
		s.xVelocity += a.xDelta;
		s.yVelocity += a.yDelta;
		s.xLoc = a.xGoal;
		s.yLoc = a.yGoal;
	}
	else {
		s.xVelocity += a.xDelta;
		s.yVelocity += a.yDelta;
		s.xLoc += s.xVelocity;
		s.yLoc += s.yVelocity;
	}
	
//	this->Boundaries(s, a);
}

// ------------ Boundaries ----------- //

void Racetrack::Boundaries(RacetrackState &s, RacetrackMove &v) const
{
	if (s.xLoc > 60000)
	{
		s.xLoc = 0;
		s.xVelocity = 0;
		//v.xDelta = 0;
	}
	else if (s.xLoc >= map->GetMapWidth() - 1)
	{
		s.xLoc = map->GetMapWidth()-1;
		s.xVelocity = 0;
		//v.xDelta = 0;
	}
	if (s.yLoc > 60000)
	{
		// makes the agent stop
		s.yLoc = 0;
		s.yVelocity = 0;
		//v.yDelta = 0;
	}
	
	else if (s.yLoc >= map->GetMapHeight() - 1)
	{
		s.yLoc = map->GetMapHeight()-1;
		s.yVelocity = 0;
		//v.yDelta = 0;
	}
	
	
	
}

bool Racetrack::InvertAction(RacetrackMove &a) const
{
	// Actions are not invertable
	return false;
}

RacetrackMove Racetrack::GetAction(const RacetrackState &s1, RacetrackState &s2) const
{
	RacetrackMove m;
	int newX = s1.xLoc + s1.xVelocity;
	int expectedX = s2.xLoc;
	m.xDelta = expectedX-newX;

	int newY = s1.yLoc + s1.yVelocity;
	int expectedY = s2.yLoc;
	m.xDelta = expectedY-newY;

	return m;
}


/*
 * The goal is implicit: We reach the goal if we have reached kEndTerrain.
 * So, we ignore the speicfic goal state
 */
bool Racetrack::GoalTest(const RacetrackState &node, const RacetrackState &) const
{
	// Use the node to see if the location matches the goal location

	if (map->GetTerrainType(node.xLoc, node.yLoc) == kEndTerrain)
	{
		//std::cout << "Touched the goal! \n";
		return true;
	}
	return false;
}

// --- The legal function, which checks whether an action is legal --- //
bool Racetrack::Legal(const RacetrackState &node1, RacetrackMove &act) const
{
	// Check if move goes out of bounds -- note -- you can pass the goal and then
	// go out of bounds, so this isn't sufficient
//	if ((node1.xLoc + node1.xVelocity + act.xDelta >= map->GetMapWidth()) ||
//		(node1.xLoc + node1.xVelocity + act.xDelta < 0))
//		return false;
//	if ((node1.yLoc + node1.yVelocity + act.yDelta >= map->GetMapHeight()) ||
//		(node1.yLoc + node1.yVelocity + act.yDelta < 0))
//		return false;

	
	// Check if moving too fast
	if ((node1.xVelocity + act.xDelta > maxVelocity) ||
		(node1.xVelocity + act.xDelta < -maxVelocity))
		return false;
	if ((node1.yVelocity + act.yDelta > maxVelocity) ||
		(node1.yVelocity + act.yDelta < -maxVelocity))
		return false;

	// Check if not moving - can't stay in place
	if (node1.xVelocity == 0 && act.xDelta == 0 && node1.yVelocity == 0 && act.yDelta == 0)
		return false;
	
	auto currTerrain = map->GetTerrainType(node1.xLoc, node1.yLoc);

	// Check locations - every x border and y border
	act.hitGoal = false;
	RacetrackState node2 = node1;
	ApplyAction(node2, act);
	Graphics::point s(node1.xLoc, node1.yLoc);
	Graphics::point g(node2.xLoc, node2.yLoc);

	// We only have to be consistent with ourselves, so we do a simple
	// line sweep and check 5 points on each sweep location
	int numSegments = static_cast<int>(2*ceilf((g-s).length()));
	for (int x = 1; x <= numSegments; x ++)
	{
		Graphics::point next;
		float ratio = static_cast<float>(x)/static_cast<float>(numSegments);
		next = s*(1-ratio)+g*(ratio);
		float offsets[5][2] = {{0, 0}, {0.4f, 0.4f}, {-0.4f, -0.4f}, {-0.4f, 0.4f}, {0.4f, -0.4f}};
		for (int y = 0; y < 5; y++)
		{
			// check if next is overlapping blocked cells or the goal
			int xNext = static_cast<int>(roundf(next.x+offsets[y][0]));
			int yNext = static_cast<int>(roundf(next.y+offsets[y][1]));
			if ((xNext >= map->GetMapWidth()) || (xNext < 0))
				return false;
			if ((yNext >= map->GetMapHeight()) || (yNext < 0))
				return false;

			auto terrain = map->GetTerrainType(xNext, yNext);
			if (!CanPass(currTerrain, terrain))
				return false;
//			if ((terrain&0x40) != 0x20)
//				return false;
			if (terrain == kEndTerrain)
			{
				act.hitGoal = true;
				act.xGoal = xNext;
				act.yGoal = yNext;
				return true;
			} // mark goal for efficiency
		}
	}
	return true;
}

double Racetrack::HCost(const RacetrackState &node1, const RacetrackState &node2) const
{
	return HCost(node1);
}

double Racetrack::HCost(const RacetrackState &node) const
{
	int len = heuristic[GetIndex(node.xLoc, node.yLoc)];
	int speed = abs(node.xVelocity)+abs(node.yVelocity);
	int h = 0;
	// Could do this much faster with rather simple math
	while (len > 0)
	{
		h++;
		speed = std::min((speed+2), 8); // max speed is 8
		len -= speed;
	}
	return h;
}


uint64_t Racetrack::GetStateHash(const RacetrackState &node) const
{
	return ((static_cast<uint64_t>(node.xLoc)<<32) | (static_cast<uint64_t>(node.yLoc)<<16) | (static_cast<uint64_t>(node.xVelocity+maxVelocity)<<8) | (static_cast<uint64_t>(node.yVelocity+maxVelocity)));
}

uint64_t Racetrack::GetActionHash(RacetrackMove act) const
{
	assert(false);
	return 0;
}

void Racetrack::Draw(Graphics::Display &display) const
{
	me->Draw(display);
}

void Racetrack::Draw(Graphics::Display &display, const RacetrackState &s) const
{
	// draws agent location and movement vector
	xyLoc temp(s.xLoc + (s.xVelocity), s.yLoc + (s.yVelocity)); // predicted next location
//	xyLoc temp2(s.xLoc-s.xVelocity, s.yLoc-s.yVelocity); // location from where agent was before to where it is now
	
	xyLoc agent(static_cast<uint16_t>(s.xLoc), static_cast<uint16_t>(s.yLoc));
	if (temp.x < map->GetMapWidth() && temp.y < map->GetMapHeight() && temp.x > 0 && temp.y > 0)
	{
		me->SetColor(Colors::black);
		me->DrawLine(display, agent, temp, 2.0);
	}
//	me->SetColor(Colors::red);
//	me->DrawLine(display, s.loc, temp2);

	me->SetColor(GetColor()); // sets agent color
//	me->Draw(display, agent); // draws agent location
//	GLdouble xx, yy, zz, rad;
//	map->GetCoord(s.xLoc, s.yLoc, xx, yy, zz, rad);
//
//	Graphics::point center(xx, yy);
//	Graphics::point p1(s.xVelocity, s.yVelocity);
//	p1.normalise();
//	if (s.xVelocity == 0 && s.yVelocity == 0)
//		p1.x = 1;
//	p1 = p1*(rad);
//	Graphics::point p2(-s.yVelocity, s.xVelocity);
//	p2.normalise();
//	if (s.xVelocity == 0 && s.yVelocity == 0)
//		p2.y = 1;
//	p2 = p2*(0.5f*rad);
//	display.FillTriangle(center+p1, center-p1+p2, center-p1-p2, this->color);
	Graphics::point c, p1, p2, p3;
	GetCarCoordinates(s, c, p1, p2, p3);

	display.FillTriangle(c+p1, c+p2, c+p3, GetColor());
}

void Racetrack::GetCarCoordinates(const RacetrackState &s, Graphics::point &center, Graphics::point &t1, Graphics::point &t2, Graphics::point &t3) const
{
	double xx, yy, zz, r;
	map->GetCoord(s.xLoc, s.yLoc, xx, yy, zz, r);
	float rad = (float)r;
	
	center = Graphics::point(xx, yy);
	Graphics::point p1(s.xVelocity, s.yVelocity);
	p1.normalise();
	if (s.xVelocity == 0 && s.yVelocity == 0)
		p1.y = 1;
	p1 = p1*(rad);
	Graphics::point p2(-s.yVelocity, s.xVelocity);
	p2.normalise();
	if (s.xVelocity == 0 && s.yVelocity == 0)
		p2.x = 1;
	Graphics::point zero;
	p2 = p2*(0.5f*rad);
	t1 = p1;
	t2 = zero-p1+p2;
	t3 = zero-p1-p2;
}

void Racetrack::Draw(Graphics::Display &display, const RacetrackState &s, RacetrackMove &a) const
{
	// draws agent location and movement vector
	
	xyLoc temp(s.xLoc + (s.xVelocity + a.xDelta), s.yLoc + (s.yVelocity + a.yDelta)); // predicted next location
	xyLoc temp2(s.xLoc-s.xVelocity, s.yLoc-s.yVelocity); // location from where agent was before to where it is now
	
	xyLoc agent(static_cast<uint16_t>(s.xLoc), static_cast<uint16_t>(s.yLoc));
	me->SetColor(Colors::blue);
	me->DrawLine(display, agent, temp);
	me->SetColor(Colors::red);
	me->DrawLine(display, agent, temp2);
	me->SetColor(Colors::black); // sets agent color to black
	me->Draw(display, agent); // draws agent location
}

void Racetrack::Draw(Graphics::Display &display, const RacetrackState &s, const RacetrackState &t, float v) const
{
	me->SetColor(Colors::blue);
//	xyLoc agent1(static_cast<uint16_t>(s.xLoc), static_cast<uint16_t>(s.yLoc));
//	xyLoc agent2(static_cast<uint16_t>(t.xLoc), static_cast<uint16_t>(t.yLoc));
//	me->Draw(display, agent1, agent2, v);

	Graphics::point c1, c2;
	Graphics::point p1, p2, p3;
	GetCarCoordinates(s, c1, p1, p2, p3);
	Graphics::point q1, q2, q3;
	GetCarCoordinates(t, c2, q1, q2, q3);
	Graphics::point middle = c1*(1-v)+c2*v;

	if (v < 0.1) // turn
	{
		display.FillTriangle((middle+p1)*(1-v)+(middle+q1)*v, (middle+p2)*(1-v)+(middle+q2)*v, (middle+p3)*(1-v)+(middle+q3)*v, GetColor());
	}
	else {
		display.FillTriangle(middle+q1, middle+q2, middle+q3, GetColor());
	}
	

}

void Racetrack::DrawLine(Graphics::Display &display, const RacetrackState &c, const RacetrackState &d, float width) const
{
	// TODO: draw line between two locations
	xyLoc agent1(static_cast<uint16_t>(c.xLoc), static_cast<uint16_t>(c.yLoc));
	xyLoc agent2(static_cast<uint16_t>(d.xLoc), static_cast<uint16_t>(d.yLoc));
	me->SetColor(GetColor());
	me->DrawLine(display, agent1, agent2, width);
}
