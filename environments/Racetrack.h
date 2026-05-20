//
//  Racetrack.h
//  hog2 glut
//
//  Created by Nathan Sturtevant on 7/5/21.
//

#ifndef RACETRACK_H
#define RACETRACK_H

#include <stdio.h>
#include <cstdint>
#include <math.h>
#include "SearchEnvironment.h"
#include "Map2DEnvironment.h"
#include "Map.h"

// TODO: Need to move code into its own namespace

struct RacetrackMove {
	RacetrackMove(int x=0, int y=0):xDelta(x), yDelta(y), hitGoal(false) {};
	int xDelta;
	int yDelta;
	bool hitGoal;
	int xGoal, yGoal;
};


struct RacetrackState {
	int xLoc, yLoc; // was using xyLoc from Map2DEnvironment, but can't use unsigned for racetracks
	int xVelocity, yVelocity;
};

const int maxVelocity = 4;

std::ostream &operator<<(std::ostream &out, const RacetrackState &s);
bool operator==(const RacetrackState &l1, const RacetrackState &l2);
bool operator!=(const RacetrackState &l1, const RacetrackState &l2);
std::ostream &operator<<(std::ostream &out, const RacetrackMove &m);
bool operator==(const RacetrackMove &m1, const RacetrackMove &m2); // comparing 



//Different types of domain
const tTerrain kStartTerrain = kSwamp; 
const tTerrain kEndTerrain = kGrass;
const tTerrain kObstacle = kTrees;

class Racetrack : public SearchEnvironment<RacetrackState, RacetrackMove> {
public:
	Racetrack(Map *map); 
	~Racetrack();
	void UpdateMap(Map *map);
	void GetSuccessors(const RacetrackState &nodeID, std::vector<RacetrackState> &neighbors) const; //current state --> pass in a vector (array lists) __> fill in -- no modification
	void GetActions(const RacetrackState &nodeID, std::vector<RacetrackMove> &actions) const; // no modification
	int GetNumSuccessors(const RacetrackState &stateID) const
	{ std::vector<RacetrackState> neighbors; GetSuccessors(stateID, neighbors); return (int)neighbors.size(); }
	
	void Reset(RacetrackState &s) const;
	
	
	void ApplyAction(RacetrackState &s, RacetrackMove a) const;// a = action s(state) gets changed
	bool InvertAction(RacetrackMove &a) const;
	RacetrackMove GetAction(const RacetrackState &s1, RacetrackState &s2) const;

	void Boundaries(RacetrackState &s, RacetrackMove &v) const;
	bool Legal(const RacetrackState &node1, RacetrackMove &act) const;
	
	/** Heuristic value between two arbitrary nodes. **/
	double HCost(const RacetrackState &node1, const RacetrackState &node2) const;
	/** Heuristic value between node and the stored goal. Asserts that the
	 goal is stored **/
	double HCost(const RacetrackState &node) const;

	// estimated distance ignoring costs. For this domain, no difference
	double DCost(const RacetrackState &node1, const RacetrackState &node2) const { return HCost(node1, node2); }
	double DCost(const RacetrackState &node) const { return HCost(node); }

	
	double GCost(const RacetrackState &node1, const RacetrackState &node2) const { return 1; };
	double GCost(const RacetrackState &node, const RacetrackMove &act) const { return 1; };
	bool GoalTest(const RacetrackState &node, const RacetrackState &goal) const; // Goal reached?
	uint64_t GetStateHash(const RacetrackState &node) const; // turn into a number
	uint64_t GetActionHash(RacetrackMove act) const;
	//
	static uint64_t Hash(const RacetrackState &node)
	{ 	return ((static_cast<uint64_t>(node.xLoc)<<32) | (static_cast<uint64_t>(node.yLoc)<<16) | (static_cast<uint64_t>(node.xVelocity+maxVelocity)<<8) | (static_cast<uint64_t>(node.yVelocity+maxVelocity)));
	}

	// Deprecated
//	void OpenGLDraw() const {};
//	void OpenGLDraw(const RacetrackState&) const {};
//	void OpenGLDraw(const RacetrackState&, const RacetrackMove&) const {};

	void Draw(Graphics::Display &display) const;
	void Draw(Graphics::Display &display, const RacetrackState &s) const;
	void Draw(Graphics::Display &display, const RacetrackState&, RacetrackMove&) const;
	void Draw(Graphics::Display &display, const RacetrackState &l1, const RacetrackState &l2, float v) const;

	void DrawLine(Graphics::Display &display, const RacetrackState &x, const RacetrackState &y, float width) const;
protected: // take two states and draw a line
private:
	MapEnvironment *me;
	Map *map;
	std::vector<int> heuristic;
	int GetIndex(int x, int y) const {return static_cast<int>(map->GetMapWidth()*y+x);}
	void GetCarCoordinates(const RacetrackState &s, Graphics::point &center, Graphics::point &p1, Graphics::point &p2, Graphics::point &p3) const;
};

template<>
struct std::hash<RacetrackState>
{
	std::size_t operator()(const RacetrackState& s) const noexcept
	{
		return Racetrack::Hash(s);
	}
};

#endif /* TOH_hpp */
