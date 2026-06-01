//
//  RAJPS.hpp
//  hog2 glut
//
//  Created by Nathan Sturtevant on 2/2/16.
//  Copyright © 2016 University of Denver. All rights reserved.
//

#ifndef RAJPS_hpp
#define RAJPS_hpp

#include <stdio.h>
#include "RoboticArm.h"
#include "SearchAlgorithm.h"
#include "TemplateAStar.h"

struct xyArmParent
{
	//xyLoc loc;
	armAngles loc;
	uint8_t parent;
};

struct jpsArmSuccessor
{
	jpsArmSuccessor() {}
	jpsArmSuccessor(armAngles a, uint8_t parent, double cost)
	{ s.loc = a; s.parent = parent; this->cost = cost; }
	xyArmParent s;
	double cost;
};

class RAJPS : public GenericSearchAlgorithm<armAngles, armRotations, RoboticArm>
{
public:
	RAJPS();
	void GetPath(RoboticArm *env, const armAngles &from, const armAngles &to, std::vector<armAngles> &path);
	void GetPath(RoboticArm *env, const armAngles &from, const armAngles &to, std::vector<armRotations> &path);
	
	bool InitializeSearch(RoboticArm *env, const armAngles& from, const armAngles& to, std::vector<armAngles> &thePath);
	bool DoSingleSearchStep(std::vector<armAngles> &thePath);
	
	const char *GetName() { return "JPS"; }
	uint64_t GetNodesExpanded() const;
	uint64_t GetNodesTouched() const;
	uint64_t GetNumOpenItems() const { return openClosedList.OpenSize(); }
	void SetWeight(double val) { weight = val; }
	void SetJumpLimit(uint32_t val) { jumpLimit = val; }
	void LogFinalStats(StatCollection *stats);
	void OpenGLDraw() const;
	void OpenGLDraw(const RoboticArm *env) const;
private:
	void GetJPSSuccessors(const xyArmParent &s, const armAngles &goal);
	void GetJPSSuccessors(xyArmParent s, uint8_t parent, const armAngles &goal, double cost);
	bool Passable(armAngles);
	void ExtractPathToStartFromID(uint64_t node, std::vector<armAngles> &thePath);
	AStarOpenClosed<xyArmParent, AStarCompare<xyArmParent> > openClosedList;
	std::vector<jpsArmSuccessor> successors;
	RoboticArm *env;
	armAngles to;
	uint64_t nodesExpanded, nodesTouched;
	double weight;
	uint32_t jumpLimit;
};

#endif /* RAJPS_hpp */
