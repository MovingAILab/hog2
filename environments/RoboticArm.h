/*
 *  RoboticArm.h
 *  hog2
 *
 *  Created by Nathan Sturtevant on 11/15/08.
 *  Copyright 2008 __MyCompanyName__. All rights reserved.
 *
 */

#ifndef ROBOTICARM_H
#define ROBOTICARM_H

#include <stdint.h>
#include <iostream>
#include <string.h>
#include <cinttypes>
#include "Map.h"
#include "SearchEnvironment.h"
#include "UnitSimulation.h"
#include "ReservationProvider.h"
#include "ConfigEnvironment.h"
#include "FrontierBFS.h"
#include "TemplateAStar.h"
#include <cassert>

//#include "BaseMapOccupancyInterface.h"



class armAngles {
public:
	armAngles() { angles = 0; }
	int GetAngle(int which) const;
	void SetAngle(int which, int value);
	int GetNumArms() const;
	void SetNumArms(int count);
	void SetGoal(float x, float y);
	void GetGoal(float &x, float &y) const;
	bool IsGoalState() const;
	uint64_t angles;
};

static std::ostream& operator <<(std::ostream & out, const armAngles &loc)
{
	if (loc.IsGoalState())
	{
		float x, y;
		loc.GetGoal(x, y);
		out << "(" << x << ", " << y << ")" << std::endl;
	}
	else {
		out << "[";
		for (int x = 0; x < loc.GetNumArms()-1; x++)
			out << loc.GetAngle(x) << ", ";
		out << loc.GetAngle(loc.GetNumArms()-1) << "]";
	}
	return out;
}

static bool operator==(const armAngles &l1, const armAngles &l2) {
	return (l1.angles == l2.angles);
}

enum tRotation {
	kRotateCCW = -1,
	kNoRotation = 0,
	kRotateCW = 1
};

class armRotations {
public:
	armRotations()
	:rotations() {}
	uint16_t rotations;
	tRotation GetRotation(int which) const;
	void SetRotation(int which, tRotation dir);
};

static bool operator==(const armRotations &l1, const armRotations &l2) {
	return (l1.rotations == l2.rotations);
}

class RoboticArmHeuristic {
public:
	virtual ~RoboticArmHeuristic() {}
	virtual double HCost(const armAngles &node1, const armAngles &node2) const = 0;
};

class RoboticArm : public SearchEnvironment<armAngles, armRotations>
{
public:
	RoboticArm(int DOF, double armLength, double tolerance = 0.01);
	virtual ~RoboticArm();

	double GetTolerance() const { return tolerance; }
	void GetTipPosition(const armAngles &s, double &x, double &y ) const;
	int TipPositionIndex(armAngles &s,
						 const double minX=-1, const double minY=-1,
						 const double width=2 );
	
	void AddObstacle(line2d obs);
	void PopObstacle();
	void GetSuccessors(const armAngles &nodeID, std::vector<armAngles> &neighbors) const;
	void GetActions(const armAngles &nodeID, std::vector<armRotations> &actions) const;
	armRotations GetAction(const armAngles &s1, const armAngles &s2) const;
	virtual void ApplyAction(armAngles &s, armRotations dir) const;
	armAngles GetRandomState();

	virtual bool InvertAction(armRotations &a) const;

	void AddHeuristic(RoboticArmHeuristic *h) { heuristics.push_back(h); }

	virtual double HCost(const armAngles &) const {
		printf("Single State HCost Failure: method not implemented for RoboticArm\n");
		exit(0); return -1.0;}

	virtual double HCost(const armAngles &node1, const armAngles &node2) const;

	virtual double GCost(const armAngles &, const armAngles &) const { return 1; }
	virtual double GCost(const armAngles &, const armRotations &) const { return 1; }
	bool GoalTest(const armAngles &node, const armAngles &goal) const;
	void GetStateFromHash(uint64_t hash, armAngles &) const;
	uint64_t GetStateHash(const armAngles &node) const;
	uint64_t GetActionHash(armRotations act) const;

private:
//	virtual void OpenGLDraw() const;
//	virtual void OpenGLDraw(const armAngles &l) const;
//	virtual void OpenGLDraw(const armAngles &, const armRotations &) const;
//	virtual void OpenGLDraw(const armAngles&, const armAngles&, float) const {}
public:
//	virtual void OpenGLDraw(const armAngles &, const armRotations &, GLfloat r, GLfloat g, GLfloat b) const;
//	virtual void OpenGLDraw(const armAngles &l, GLfloat r, GLfloat g, GLfloat b) const;
	virtual void Draw(Graphics::Display &display) const;
	virtual void Draw(Graphics::Display &display, const armAngles&) const;
	virtual void Draw(Graphics::Display &display, const armAngles&, int which, rgbColor c) const;
	virtual void Draw(Graphics::Display &display, const armAngles &, const armRotations &) const;
	
	virtual void GetNextState(const armAngles &currents, armRotations dir, armAngles &news) const;

	bool LegalState(armAngles &a) const;
	bool LegalArmConfig(armAngles &a) const;

	void StoreGoal(armAngles &) {}
	void ClearGoal(){}
	bool IsGoalStored()const {return false;}
	virtual bool GoalTest(const armAngles &) const {
		printf("Single State Goal Test Failure: method not implemented for RoboticArm\n");
		exit(0); return false;}

private:
	bool LegalAction(const armAngles &a, double tipx, double tipy) const;
	void DrawLine(line2d l) const;
	void GenerateLineSegments(const armAngles &a, std::vector<line2d> &armSegments) const;

	int DOF;
	double armLength, tolerance;
	std::vector<std::vector<bool> > legals;

	double GetSin(int angle) const;
	double GetCos(int angle) const;
	void BuildSinCosTables();
	std::vector<double> sinTable;
	std::vector<double> cosTable;
	std::vector<line2d> obstacles;
	mutable std::vector<line2d> armSegments;

	mutable std::vector<Graphics::point> states;

	std::vector<RoboticArmHeuristic *> heuristics;
	ConfigEnvironment *ce;
	mutable TemplateAStar<Graphics::point, line2d, ConfigEnvironment> localAStar;
};

class ArmToArmHeuristic : public RoboticArmHeuristic {
public:
	ArmToArmHeuristic(RoboticArm *r, armAngles &initial, bool optimize = false);
	virtual ~ArmToArmHeuristic() {}
	double HCost(const armAngles &node1, const armAngles &node2) const;
	void AddDiffTable();
	bool IsLegalState(armAngles &arm);
	const std::vector<armAngles> &GetTipPositions(double x, double y)
	{ return tipPositionTables[TipPositionIndex(x, y)]; }
private:
	armAngles SelectStartNode();
	int TipPositionIndex(const double x, const double y,
						 const double minX = -1.0, const double minY = -1.0,
						 const double width = 2.0);
	void GenerateLegalStates(armAngles &init);
	bool optimizeLocations;
	RoboticArm *ra;
	std::vector<std::vector<uint16_t> > distances;
	std::vector<armAngles> canonicalStates;
	std::vector<std::vector<armAngles> > tipPositionTables;
	std::vector<bool> legalStates;
};

class ArmToArmCompressedHeuristic : public RoboticArmHeuristic {
public:
	ArmToArmCompressedHeuristic(RoboticArm *ra, const char *file)
	{
		r = ra;
		Load(file);
	}
	ArmToArmCompressedHeuristic(RoboticArm *ra, std::vector<int> reductionPower, std::vector<int> offset)
	{
		r = ra;
		reduction = reductionPower;
		// we are actually only using 9 of the bits anyway
		// strip 1, 2, 3, 4 or 5 additional bits
		uint64_t mask[6] = {0x1, 0x3, 0x7, 0xF, 0x1F, 0x3F }; //{ 0x3FC, 0x3F8, 0x3F0, 0x3E0, 0x3C0 };
		theMask = theResult = 0;
		theSize = 1;
		for (int x = reduction.size()-1; x >= 0 ; x--)
		{
			theMask = (theMask<<(10))|(mask[reduction[x]-1]);
			theResult = (theResult<<(10))|(offset[x]<<1);
			theSize *= 512/pow(2,reduction[x]-1);
		}
		//theSize = (uint32_t)pow(512/reductionPower, numArms);
		printf("Using mask 0x%llX\n", theMask);
		printf("Using result 0x%llX\n", theResult);
		printf("%" PRId64 " entries in table\n", theSize);
		distances = new uint16_t[theSize];
		memset ( distances, 0xFFFF, theSize*sizeof(uint16_t) );
	}
	ArmToArmCompressedHeuristic(RoboticArm *ra, int numArms, int reductionPower, int offset = 0)
	{
		r = ra;
//		reduction = reductionPower;
		// we are actually only using 9 of the bits anyway
		// strip 1, 2, 3, 4 or 5 additional bits
		uint64_t mask[5] = {0x3, 0x7, 0xF, 0x1F, 0x3F }; //{ 0x3FC, 0x3F8, 0x3F0, 0x3E0, 0x3C0 };
		theMask = theResult = 0;
		for (int x = 0; x < numArms; x++)
		{
			reduction.push_back(reductionPower);
			theMask = (theMask<<(10))|(mask[reductionPower-2]);
			theResult = (theResult<<(10))|(offset<<1);
		}
		theSize = (uint32_t)pow(512/reductionPower, numArms);
		printf("Using mask 0x%llX\n", theMask);
		printf("Using result 0x%llX\n", theResult);
		printf("%" PRId64 " entries in table\n", theSize);
		distances = new uint16_t[theSize];
	}
	~ArmToArmCompressedHeuristic()
	{
		delete [] distances;
	}
	// takes start state and returns farthest state from it
	armAngles BuildHeuristic(armAngles &config)
	{
		memset ( distances, 0xFFFF, theSize*sizeof(uint16_t) );

		FrontierBFS<armAngles, armRotations> fbfs;
		printf("Performing frontier BFS!\n");

		std::cout << "Using " << config << " as basis for heuristic." << std::endl;
		int depth = 0;
		armAngles tmp = config;
		fbfs.InitializeSearch(r, tmp);
		while (!fbfs.DoOneIteration(r))
		{
			const FrontierBFSClosedList closed = fbfs.GetCurrentClosedList();
			for (FrontierBFSClosedList::const_iterator it = closed.begin(); it != closed.end(); it++)
			{
				r->GetStateFromHash((*it).first, tmp);
				AddState(tmp, depth);
				//assert(r->HCost(tmp, config) <= depth);
			}
			depth++;
		}
		return tmp;
	}

	// takes set of start states, finds farthest to build heuristic from, and returns source of heuristic
	armAngles BuildHeuristic(std::vector<armAngles> &config)
	{
		memset ( distances, 0xFFFF, theSize*sizeof(uint16_t) );
		
		FrontierBFS<armAngles, armRotations> fbfs;
		printf("Performing frontier BFS!\n");
		//std::cout << "Starting from " << config[0] << std::endl;
		
		armAngles tmp = config[0];
		armAngles source = tmp;

		std::cout << "Finding farthest states from: ";
		for (unsigned int x = 0; x < config.size(); x++)
			std::cout << config[x] << " ";
		std::cout << std::endl;
		fbfs.InitializeSearch(r, config);
		while (!fbfs.DoOneIteration(r))
		{
			const FrontierBFSClosedList closed = fbfs.GetCurrentClosedList();
			for (FrontierBFSClosedList::const_iterator it = closed.begin(); it != closed.end(); it++)
			{
				r->GetStateFromHash((*it).first, tmp);
			}
		}

		std::cout << "Using " << tmp << " as basis for heuristic." << std::endl;
		int depth = 0;
		source = tmp;
		fbfs.InitializeSearch(r, tmp);
		while (!fbfs.DoOneIteration(r))
		{
			const FrontierBFSClosedList closed = fbfs.GetCurrentClosedList();
			for (FrontierBFSClosedList::const_iterator it = closed.begin(); it != closed.end(); it++)
			{
				r->GetStateFromHash((*it).first, tmp);
				AddState(tmp, depth);
				//assert(r->HCost(tmp, config) <= depth);
			}
			depth++;
		}
		return source;
	}
	void AddState(armAngles &a, int dist)
	{
		//printf("Using mask 0x%llX on 0x%llX (0x%llX) (0x%llX)?\n", theMask, a.angles, a.angles&theMask, theResult);
		if ((a.angles&theMask) != theResult)
			return;
		uint64_t index = GetIndex(a);
		assert(distances[index] == 0xFFFF);
		//std::cout << a << " goes in the table with index " << index << " : " << (a.angles&0xFFFFFFFF) << std::endl;
		distances[index] = dist;
	}
	uint64_t GetIndex(const armAngles &a) const
	{
		//std::cout << "Index of " << a << " : " << (a.angles&0xFFFFFFFF) << " is ";
		uint64_t index = 0;
		for (int x = 0; x < a.GetNumArms(); x++)
		{
			index = (index<<(10-reduction[x]))|(a.GetAngle(x)>>reduction[x]);
			//std::cout << index << ", ";
		}
		//std::cout << std::endl;
		assert(index < theSize);
		return index;
	}
	
	double HCost(const armAngles &from, const armAngles &to) const
	{
		if ((from.angles&theMask) != theResult)
			return 0;
		if (!(to == goal))
			SetupGoal(to);
		double heuristic = 0;
		assert(distances[GetIndex(from)]<10000);
		double baseDist = distances[GetIndex(from)];
		for (unsigned int x = 0; x < values.size(); x++)
		{
			heuristic = max(heuristic, std::abs(baseDist-values[x])-errors[x]);
		}
		return heuristic;
	}
	
	void SetupGoal(const armAngles &referenceState) const
	{
		values.resize(0);
		errors.resize(0);
		FrontierBFS<armAngles, armRotations> fbfs;
		armAngles tmp = referenceState;
		goal = referenceState;
		int depth = 0;
		fbfs.InitializeSearch(r, tmp);
		while (!fbfs.DoOneIteration(r))
		{
			const FrontierBFSClosedList closed = fbfs.GetCurrentClosedList();
			for (FrontierBFSClosedList::const_iterator it = closed.begin(); it != closed.end(); it++)
			{
				r->GetStateFromHash((*it).first, tmp);
				if ((tmp.angles&theMask) == theResult)
				{
					std::cout << "Found state " << tmp << " distance " << distances[GetIndex(tmp)] << " with error " << depth << std::endl;
					values.push_back(distances[GetIndex(tmp)]);
					errors.push_back(depth);
				}
			}
			if (values.size() > 0)
				return;
			depth++;
		}
	}
	
	void Save(const char *file)
	{
		FILE *f = fopen(file, "w+");
		if (f == 0) assert(!"file could not be opened");
		fprintf(f, "%" PRId64 " %" PRId64 " %" PRId64 " %d\n", theSize, theMask, theResult, (int)reduction.size());
		fwrite(&reduction[0], sizeof(int), reduction.size(), f);
		fwrite(distances, sizeof(uint16_t), theSize, f);
		fclose(f);
	}
	void Load(const char *file)
	{
		int numArms;
		FILE *f = fopen(file, "r");
		if (f == 0) assert(!"file could not be opened");
		fscanf(f, "%" PRId64 " %" PRId64 " %" PRId64 " %d\n", &theSize, &theMask, &theResult, &numArms);
		reduction.resize(numArms);
		fread(&reduction[0], sizeof(int), reduction.size(), f);
		distances = new uint16_t[theSize];
		fread(distances, sizeof(uint16_t), theSize, f);
		fclose(f);
	}
private:
	uint64_t theSize;
	std::vector<int> reduction;

	// caching for each goal state
	mutable std::vector<int> values;
	mutable std::vector<int> errors;
	mutable armAngles goal;
	RoboticArm *r;
	uint16_t *distances;
	uint64_t theMask;
	uint64_t theResult;
};

class ArmToTipHeuristic : public RoboticArmHeuristic {
public:
	ArmToTipHeuristic(RoboticArm *r);
	virtual ~ArmToTipHeuristic() {}
	double HCost(const armAngles &node1, const armAngles &node2) const;

	void GenerateLegalStateTable( armAngles &legalArm );
	void GenerateTipPositionTables( armAngles &sampleArm );

	void GenerateRandomHeuristic( const armAngles &sampleArm );
	int GenerateHeuristic( const armAngles &sampleArm, armAngles &goal );
	int GenerateMaxDistHeuristics( const armAngles &sampleArm,
								  const int numHeuristics );
	// Returns true if the position is guaranteed to be valid.
	// Returns false if the position is either unvalid,
	// or possibly unvalid
	bool ValidGoalPosition( double goalX, double goalY );
private:
	void GenerateCPDB();


	RoboticArm *ra;
	bool m_TableComplete;

	uint8_t *legalStateTable;
	uint8_t *legalGoalTable;

	std::vector<uint16_t *> distancesTables;
	std::vector<uint16_t *> minTipDistancesTables;
	std::vector<uint16_t *> maxTipDistancesTables;
	std::vector<uint16_t> tablesNumArms;

	std::vector<armAngles> *tipPositionTables;


	// convert an arm configuration into an index
	uint64_t ArmAnglesIndex( const armAngles &arm ) const;
	uint64_t NumArmAnglesIndices( const armAngles &arm ) const {
		return 1 << ( 9 * arm.GetNumArms() );
	}

	// convert a tip position into an index
	int TipPositionIndex( const double x, const double y,
						 const double minX, const double minY,
						 const double width ) const;
	int NumTipPositionIndices() const {
		int count = (int)ceil( 2.0 / ra->GetTolerance() );
		return count * count;
	}

	// write/read a binary representation of a configuration
	// DOES NOT WRITE/READ THE ARM LENGTHS!
	int WriteArmAngles(FILE *file, armAngles &a);
	int ReadArmAngles(FILE *file, armAngles &a);

	void UpdateTipDistances( armAngles &arm, uint16_t distance,
							uint16_t *minTipDistances,
							uint16_t *maxTipDistances );

	// common subroutine for all the different heuristic
	// generation functions
	int GenerateHeuristicSub( const armAngles &sampleArm, const bool quiet,
							 armAngles *goals, const int numGoals,
							 uint16_t *distances,
							 uint16_t *minTipDistances,
							 uint16_t *maxTipDistances,
							 armAngles &lastAdded );
	// function needed for GenerateHeuristic
	// given a file curFile of the frontier of positions at depth
	// curDistance, generates a file of the positions at curDistance+1
	// uses and updates distances[] to avoid re-visiting positions
	// if XXXTipDistances are not NULL, it will update XXXTipDistances
	// to be the min/maximum distance for all positions with that
	// tip position
	uint64_t GenerateNextDepth( FILE *curFile, FILE *nextFile,
							   uint16_t curDistance, uint16_t *distances,
							   uint16_t *minTipDistances,
							   uint16_t *maxTipDistances,
							   armAngles &lastAdded );

	// use a heuristic table
	uint16_t UseHeuristic(const armAngles &s, armAngles &g,
						  uint16_t *distances ) const;
	uint16_t UseHeuristic(const armAngles &arm, double goalX, double goalY,
						  uint16_t *distances, uint16_t *minTipDistances,
						  uint16_t *maxTipDistances ) const;
};

//typedef UnitSimulation<armAngles, armRotations, MapEnvironment> UnitMapSimulation;

#endif
