//
//  Map2DHeading.h
//  hog2 glut
//
//  Created by Nathan Sturtevant on 11/12/12.
//  Copyright (c) 2012 University of Denver. All rights reserved.
//

#ifndef __hog2_glut__Map2DHeading__
#define __hog2_glut__Map2DHeading__

#include "SearchEnvironment.h"
#include "Map.h"

#include <cassert>
#include <iostream>
#include <unordered_map>

//#include "BaseMapOccupancyInterface.h"

struct xyhLoc {
public:
	xyhLoc() { x = -1; y = -1; h = -1; }
	xyhLoc(uint16_t _x, uint16_t _y, uint8_t heading) :x(_x), y(_y), h(heading) {}
	uint16_t x;
	uint16_t y;
	uint8_t h;
};

static std::ostream& operator <<(std::ostream & out, const xyhLoc &loc)
{
	out << "(" << loc.x << ", " << loc.y << ", " << static_cast<int>(loc.h) << ")";
	return out;
}
	
static bool operator==(const xyhLoc &l1, const xyhLoc &l2)
{
	return (l1.x == l2.x) && (l1.y == l2.y) && (l1.h == l2.h);
}
	
struct xyhAct {
	uint8_t oldHeading;
	uint8_t newHeading;
};

static bool operator==(const xyhAct &l1, const xyhAct &l2)
{
	return (l1.oldHeading == l2.oldHeading) && (l1.newHeading == l2.newHeading);
}

class Map2DHeading : public SearchEnvironment<xyhLoc, xyhAct>
{
public:
	Map2DHeading(Map *m);
//	Map2DHeading(Map2DHeading *);
	virtual ~Map2DHeading();
	virtual void GetSuccessors(const xyhLoc &nodeID, std::vector<xyhLoc> &neighbors) const;
	void GetActions(const xyhLoc &nodeID, std::vector<xyhAct> &actions) const;
	xyhAct GetAction(const xyhLoc &s1, const xyhLoc &s2) const;
	virtual void ApplyAction(xyhLoc &s, xyhAct dir) const;
	
	virtual bool InvertAction(xyhAct &a) const;
	
	//	bool Contractable(const xyhLoc &where);
	
	virtual double HCost(const xyhLoc &) const {
		fprintf(stderr, "ERROR: Single State HCost not implemented\n");
		exit(1); return -1.0;}
	virtual double HCost(const xyhLoc &node1, const xyhLoc &node2) const;
	virtual double GCost(const xyhLoc &node1, const xyhLoc &node2) const;
	virtual double GCost(const xyhLoc &node1, const xyhAct &act) const;
	bool GoalTest(const xyhLoc &node, const xyhLoc &goal) const;
	
	bool GoalTest(const xyhLoc &) const {
		fprintf(stderr, "ERROR: Single State Goal Test not implemented \n");
		exit(1); return false;}
	
	uint64_t GetStateHash(const xyhLoc &node) const;
	void GetStateFromHash(uint64_t hash, xyhLoc &node) const;
	uint64_t GetActionHash(xyhAct act) const;
//	virtual void OpenGLDraw() const;
//	virtual void OpenGLDraw(const xyhLoc &l) const;
//	virtual void OpenGLDraw(const xyhLoc &l1, const xyhLoc &l2, float v) const;
//	virtual void OpenGLDraw(const xyhLoc &, const xyhAct &) const;
//	virtual void GLLabelState(const xyhLoc &, const char *) const;
//	virtual void GLDrawLine(const xyhLoc &x, const xyhLoc &y) const;
	//virtual void OpenGLDraw(const xyhLoc &, const xyhAct &, GLfloat r, GLfloat g, GLfloat b) const;
	//virtual void OpenGLDraw(const xyhLoc &l, GLfloat r, GLfloat g, GLfloat b) const;
	Map* GetMap() const { return map; }
	
	virtual void GetNextState(const xyhLoc &currents, xyhAct dir, xyhLoc &news) const;
	
	void StoreGoal(xyhLoc &) {} // stores the locations for the given goal state
	void ClearGoal() {}
	bool IsGoalStored() const {return false;}
	void SetDiagonalCost(double val) { DIAGONAL_COST = val; }
	double GetDiagonalCost() { return DIAGONAL_COST; }
	bool FourConnected() { return fourConnected; }
	bool EightConnected() { return !fourConnected; }
	void SetFourConnected() { fourConnected = true; }
	void SetEightConnected() { fourConnected = false; }
	
	void SetCost(const xyhLoc &, double seen, double dist);
	void ClearCost(const xyhLoc &);
	void ClearAllCosts();
	bool drawWeights;
protected:
	Map *map;
	double DIAGONAL_COST;
	bool fourConnected;
	std::vector<float> cosTable;
	std::vector<float> sinTable;

	// P is visibility setting
	double GetCost(const xyhLoc &a, const xyhLoc &b, double P, double d) const;

	struct hdData { double seen; double dist; };
	typedef std::unordered_map<uint64_t, hdData, Hash64> CostTable;
	CostTable costs;
	bool LegalState(const xyhLoc &s);
	void BuildAngleTables();
	float mySin(int dir) const;
	float myCos(int dir) const;
};

#endif /* defined(__hog2_glut__Map2DHeading__) */
