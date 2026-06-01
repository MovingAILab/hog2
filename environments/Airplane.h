//
//  Airplane.h
//  hog2 glut
//
//  Created by Nathan Sturtevant on 5/4/16.
//  Copyright © 2016 University of Denver. All rights reserved.
//

#ifndef Airplane_h
#define Airplane_h

#include <vector>
#include <cassert>
#include "SearchEnvironment.h"

// turn type:
// 45, 90, 0, shift
// speed:
// faster, slower
// height:
// up / down

const uint8_t k45 = 1;
const uint8_t k90 = 2;
const uint8_t kShift = 3;

struct airplaneAction {
public:
	airplaneAction(int8_t t=0, int8_t s=0, int8_t h=0)
	:turn(t), speed(s), height(h) {}
	int8_t turn;
	int8_t speed;
	int8_t height;
};

// state
struct airplaneState {
public:
	airplaneState() { x = y = heading = 0; speed = 1; height = 20; }
	uint8_t x;
	uint8_t y;
	uint8_t height;
	int speed : 2;
	int heading : 6;
};

bool operator==(const airplaneState &s1, const airplaneState &s2);
bool operator==(const airplaneAction &s1, const airplaneAction &s2);

//class GoalTester {
//public:
//	virtual ~GoalTester() {}
//	virtual bool goalTest(const airplaneState &i1) const = 0;
//};

class AirplaneEnvironment : public SearchEnvironment<airplaneState, airplaneAction>
{
public:
	AirplaneEnvironment();
	void GetSuccessors(const airplaneState &nodeID, std::vector<airplaneState> &neighbors) const;
	void GetActions(const airplaneState &nodeID, std::vector<airplaneAction> &actions) const;
	virtual void ApplyAction(airplaneState &s, airplaneAction dir) const;
	virtual void UndoAction(airplaneState &s, airplaneAction dir) const;
	virtual void GetNextState(const airplaneState &currents, airplaneAction dir, airplaneState &news) const;
	
	virtual OccupancyInterface<airplaneState,airplaneAction> *GetOccupancyInfo() { return 0; }
	virtual bool InvertAction(airplaneAction &a) const { return false; }
	
	virtual double HCost(const airplaneState &node1, const airplaneState &node2) const;
	virtual double HCost(const airplaneState &)  const { assert(false); return 0; }
	virtual double GCost(const airplaneState &node1, const airplaneState &node2) const;
	virtual double GCost(const airplaneState &node1, const airplaneAction &act) const;
	
//	void SetGoalTest(GoalTester *t) {test = t;}
	bool GoalTest(const airplaneState &node, const airplaneState &goal) const;
	bool GoalTest(const airplaneState &) const { assert(false); return false; }
	uint64_t GetStateHash(const airplaneState &node) const;
	uint64_t GetActionHash(airplaneAction act) const;
	Graphics::point GetCoordinate(int x, int y, int z) const;
private:
	void SetGround(int x, int y, uint8_t val);
	uint8_t GetGround(int x, int y) const;
	bool Valid(int x, int y);
	Graphics::point &GetNormal(int x, int y);
	Graphics::point GetNormal(int x, int y) const;
	void RecurseGround(int x1, int y1, int x2, int y2);
	const int width = 80;
	const int length = 80;
	const int height = 20;
	std::vector<uint8_t> ground;
	std::vector<Graphics::point> groundNormals;
	void DoNormal(Graphics::point pa, Graphics::point pb) const;
	mutable std::vector<airplaneAction> internalActions;
};

#endif /* Airplane_h */
