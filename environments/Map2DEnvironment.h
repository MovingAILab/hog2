/*
 *  Map2DEnvironment.h
 *  hog2
 *
 *  Created by Nathan Sturtevant on 4/20/07.
 *  Copyright 2007 Nathan Sturtevant, University of Alberta. All rights reserved.
 *
 */

#ifndef MAP2DENVIRONMENT_H
#define MAP2DENVIRONMENT_H

#include <stdint.h>
#include <stdlib.h>
#include <string>
#include <iostream>
#include "Map.h"
//#include "MapAbstraction.h"
#include "SearchEnvironment.h"
#include "BitVector.h"
#include "GraphEnvironment.h"
#include "Graphics.h"

#include <cassert>

//#include "BaseMapOccupancyInterface.h"

enum drawOptions {
	kNoOptions = 0x0,
	kEfficientCells = 0x1, // This is expensive because the regions are optimized, but results in far fewer drawing commands
	kTerrainBorderLines = 0x2, // This is expensive because the lines are optimized, but results in far fewer drawing commands
	kCellBorderLines = 0x4,
	kLightMode = 0x8
};

struct xyLoc {
public:
	xyLoc() { x = -1; y = -1; }
	xyLoc(uint16_t _x, uint16_t _y) :x(_x), y(_y) {}
	uint16_t x;
	uint16_t y;
	bool operator<(const xyLoc &b)
	{
		if (y == b.y)
			return (x < b.x);
		if (y < b.y)
			return true;
		return false;
	}
};

struct xyLocHash
{
	std::size_t operator()(const xyLoc & x) const
	{
		return (x.x<<16)|(x.y);
	}
};


static std::ostream& operator <<(std::ostream & out, const xyLoc &loc)
{
	out << "(" << loc.x << ", " << loc.y << ")";
	return out;
}

static bool operator==(const xyLoc &l1, const xyLoc &l2) {
	return (l1.x == l2.x) && (l1.y == l2.y);
}

static bool operator!=(const xyLoc &l1, const xyLoc &l2) {
	return (l1.x != l2.x) || (l1.y != l2.y);
}


enum tDirection {
	kN=0x8, kS=0x4, kE=0x2, kW=0x1, kNW=kN|kW, kNE=kN|kE,
	kSE=kS|kE, kSW=kS|kW, kStay=0, kTeleport=kSW|kNE, kAll = kSW|kNE
};

/*
class BaseMapOccupancyInterface : public OccupancyInterface<xyLoc,tDirection>
{
public:
	BaseMapOccupancyInterface(Map* m);
	virtual ~BaseMapOccupancyInterface();
	virtual void SetStateOccupied(const xyLoc&, bool);
	virtual bool GetStateOccupied(const xyLoc&);
	virtual bool CanMove(const xyLoc&, const xyLoc&);
	virtual void MoveUnitOccupancy(const xyLoc &, const xyLoc&);

private:
	//BitVector *bitvec; /// For each map position, set if occupied
	std::vector<bool> bitvec;
	long mapWidth; /// Used to compute index into bitvector
	long mapHeight; /// used to compute index into bitvector

	long CalculateIndex(uint16_t x, uint16_t y);
};
*/

const int numPrimitiveActions = 8;
const int numActions = 10;
const tDirection possibleDir[numActions] = { kN, kNE, kE, kSE, kS, kSW, kW, kNW, kStay, kTeleport };
const int kStayIndex = 8; // index of kStay


namespace std {
	
	template <>
	struct hash<xyLoc>
	{
		std::size_t operator()(const xyLoc& k) const
		{
			return (((std::size_t)k.x)<<16)|k.y;
		}
	};
	
}


class EuclideanDistance : public Heuristic<xyLoc> {
public:
	double HCost(const xyLoc &a, const xyLoc &b) const
	{
		return sqrt((a.x-b.x)*(a.x-b.x) + (a.y-b.y)*(a.y-b.y));
	}
};

//typedef OccupancyInterface<xyLoc, tDirection> BaseMapOccupancyInterface;


class MapEnvironment : public SearchEnvironment<xyLoc, tDirection>
{
public:
void SetTerrainCost(double costs[])
    {
        //[0]=kSwamp, [1]=kWater,[2]=kGrass, [3]=kTrees
        for(int i=0; i<4; i++)
            TerrainCosts[i] = costs[i];
    }
    /*
    sets the input weight of the search.
    */
    void SetInputWeight(double w)
    {
        inputWeight=w;
    }
    double GetInputWeight()
    {
        return inputWeight;
    }
	MapEnvironment(Map *m, bool useOccupancy = false);
	MapEnvironment(MapEnvironment *);
	virtual ~MapEnvironment();
	void SetGraphHeuristic(GraphHeuristic *h);
	GraphHeuristic *GetGraphHeuristic();
	virtual void GetSuccessors(const xyLoc &nodeID, std::vector<xyLoc> &neighbors) const;
	bool GetNextSuccessor(const xyLoc &currOpenNode, const xyLoc &goal, xyLoc &next, double &currHCost, uint64_t &special, bool &validMove);
	bool GetNext4Successor(const xyLoc &currOpenNode, const xyLoc &goal, xyLoc &next, double &currHCost, uint64_t &special, bool &validMove);
	bool GetNext8Successor(const xyLoc &currOpenNode, const xyLoc &goal, xyLoc &next, double &currHCost, uint64_t &special, bool &validMove);
	void GetActions(const xyLoc &nodeID, std::vector<tDirection> &actions) const;
	tDirection GetAction(const xyLoc &s1, const xyLoc &s2) const;
	virtual void ApplyAction(xyLoc &s, tDirection dir) const;
  //virtual BaseMapOccupancyInterface *GetOccupancyInfo() { return oi; }

	virtual bool InvertAction(tDirection &a) const;
	std::string GetName() { return std::string(map->GetMapName()); }
//	bool Contractable(const xyLoc &where);
	
	virtual double HCost(const xyLoc &) const {
		fprintf(stderr, "ERROR: Single State HCost not implemented for MapEnvironment\n");
		exit(1); return -1.0;}
	virtual double HCost(const xyLoc &node1, const xyLoc &node2) const;

	virtual double GCost(const xyLoc &node1, const xyLoc &node2) const;
	virtual double GCost(const xyLoc &node1, const tDirection &act) const;
	bool GoalTest(const xyLoc &node, const xyLoc &goal) const;

	bool GoalTest(const xyLoc &){
		fprintf(stderr, "ERROR: Single State Goal Test not implemented for MapEnvironment\n");
		exit(1); return false;}

	uint64_t GetMaxHash() const;
	uint64_t GetStateHash(const xyLoc &node) const;
	void GetStateFromHash(uint64_t parent, xyLoc &s) const;
	uint64_t GetActionHash(tDirection act) const;
//	virtual void OpenGLDraw() const;
//	virtual void OpenGLDraw(const xyLoc &l) const;
//	virtual void OpenGLDraw(const xyLoc &l1, const xyLoc &l2, float v) const;
//	virtual void OpenGLDraw(const xyLoc &, const tDirection &) const;
//	virtual void GLLabelState(const xyLoc &, const char *) const;
//	virtual void GLLabelState(const xyLoc &s, const char *str, double scale) const;
//	virtual void GLDrawLine(const xyLoc &x, const xyLoc &y) const;
	
	std::string SVGHeader();
	std::string SVGDraw();
	std::string SVGDraw(const xyLoc &);
	std::string SVGLabelState(const xyLoc &, const char *, double scale) const;
	std::string SVGLabelState(const xyLoc &, const char *, double scale, double xoff, double yoff) const;
	std::string SVGDrawLine(const xyLoc &x, const xyLoc &y, int width=1) const;
	std::string SVGFrameRect(int left, int top, int right, int bottom, int width = 1);
	
	void Draw(Graphics::Display &disp) const;
	void Draw(Graphics::Display &disp, const xyLoc &l) const;
	void DrawAlternate(Graphics::Display &disp, const xyLoc &l) const;
	void Draw(Graphics::Display &disp, const xyLoc &l1, const xyLoc &l2, float v) const;
	void DrawStateLabel(Graphics::Display &disp, const xyLoc &l1, const char *txt) const;
	void DrawStateLabel(Graphics::Display &disp, const xyLoc &l1, const xyLoc &l2, float v, const char *txt) const;
	void DrawLine(Graphics::Display &disp, const xyLoc &x, const xyLoc &y, double width = 1.0) const;
	void DrawArrow(Graphics::Display &disp, const xyLoc &x, const xyLoc &y, double width = 1.0) const;
	Graphics::point GetStateLoc(const xyLoc &l1);
	void SetDrawOptions(drawOptions o) {drawParams = o; }
	
	//virtual void OpenGLDraw(const xyLoc &, const tDirection &, GLfloat r, GLfloat g, GLfloat b) const;
	//virtual void OpenGLDraw(const xyLoc &l, GLfloat r, GLfloat g, GLfloat b) const;
	Map* GetMap() const { return map; }

	virtual void GetNextState(const xyLoc &currents, tDirection dir, xyLoc &news) const;

	void StoreGoal(xyLoc &) {} // stores the locations for the given goal state
	void ClearGoal() {}
	bool IsGoalStored() const {return false;}
	void SetDiagonalCost(double val) { DIAGONAL_COST = val; }
	double GetDiagonalCost() { return DIAGONAL_COST; }
	bool FourConnected() { return fourConnected; }
	bool EightConnected() { return !fourConnected; }
	void SetFourConnected() { fourConnected = true; }
	void SetEightConnected() { fourConnected = false; }
	//virtual BaseMapOccupancyInterface* GetOccupancyInterface(){std::cout<<"Mapenv\n";return oi;}
	//virtual xyLoc GetNextState(xyLoc &s, tDirection dir);
//	double GetPathLength(std::vector<xyLoc> &neighbors);
private:
	double inputWeight;
    double TerrainCosts[4]; //[0]=kWater,[1]kSwamp,[2]kGrass,[3]kTrees
	void GetMaxRect(long terrain, int x, int y, int endx, int endy, std::vector<bool> &drawn, Graphics::rect &r) const;
	void DrawSingleTerrain(long terrain, Graphics::Display &disp, std::vector<bool> &drawn) const;
protected:
	GraphHeuristic *h;
	Map *map;
  //BaseMapOccupancyInterface *oi;
	double DIAGONAL_COST;
	bool fourConnected;
	drawOptions drawParams;
};
/*
class AbsMapEnvironment : public MapEnvironment
{
public:
	AbsMapEnvironment(MapAbstraction *ma);
	virtual ~AbsMapEnvironment();
	MapAbstraction *GetMapAbstraction() { return ma; }
	void OpenGLDraw() const { map->OpenGLDraw(); ma->OpenGLDraw(); }
	void OpenGLDraw(const xyLoc &l) const { MapEnvironment::OpenGLDraw(l); }
	void OpenGLDraw(const xyLoc& s, const tDirection &dir) const {MapEnvironment::OpenGLDraw(s,dir);}
	void OpenGLDraw(const xyLoc &l1, const xyLoc &l2, float v) const { MapEnvironment::OpenGLDraw(l1, l2, v); }

	//virtual BaseMapOccupancyInterface* GetOccupancyInterface(){std::cout<<"AbsMap\n";return oi;}
protected:
	MapAbstraction *ma;
};
*/
//typedef UnitSimulation<xyLoc, tDirection, AbsMapEnvironment> UnitAbsMapSimulation;


//template<>
//void UnitSimulation<xyLoc, tDirection, MapEnvironment>::OpenGLDraw()
//{
//	env->OpenGLDraw();
//	for (unsigned int x = 0; x < units.size(); x++)
//	{
//		units[x]->agent->OpenGLDraw(env);
//	}
//}
//
//template<>
//void UnitSimulation<xyLoc, tDirection, AbsMapEnvironment>::OpenGLDraw()
//{
//	env->OpenGLDraw();
//	for (unsigned int x = 0; x < units.size(); x++)
//	{
//		units[x]->agent->OpenGLDraw(env);
//	}
//}

#endif
