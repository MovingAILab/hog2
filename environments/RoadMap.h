//
//  RoadMap.h
//  hog2 glut
//
//  Created by Nathan Sturtevant on 8/25/17.
//  Copyright © 2017 University of Denver. All rights reserved.
//

#ifndef RoadMap_h
#define RoadMap_h

#include <stdio.h>
#include "SearchEnvironment.h"
#include "GraphEnvironment.h"

typedef graphState intersection;
typedef graphMove neighbor;

class RoadMap : public SearchEnvironment<intersection, neighbor> {
public:
	RoadMap(const char *graph, const char *coordinates, bool time);
	virtual ~RoadMap();
	virtual void GetSuccessors(const intersection &nodeID, std::vector<intersection> &neighbors) const;
	virtual void GetActions(const intersection &nodeID, std::vector<neighbor> &actions) const;
	Graph *GetGraph() { return g; }
	virtual neighbor GetAction(const intersection &s1, const intersection &s2) const;
	virtual void ApplyAction(intersection &s, neighbor a) const;
	
	virtual bool InvertAction(neighbor &a) const;
	
	/** Heuristic value between two arbitrary nodes. **/
	virtual double HCost(const intersection &node1, const intersection &node2) const;
	virtual double GCost(const intersection &node1, const intersection &node2) const;
	virtual double GCost(const intersection &node, const neighbor &act) const;
	virtual bool GoalTest(const intersection &node, const intersection &goal) const;
	
	virtual uint64_t GetMaxHash() const;
	virtual uint64_t GetStateHash(const intersection &node) const;
	virtual void GetStateFromHash(uint64_t parent, intersection &s) const;
	
	virtual uint64_t GetActionHash(neighbor act) const;
	std::string GetName() { return std::string("RoadMap"); }
	virtual void SetColor(float rr, float g, float b, float t = 1.0) const;
	virtual void GetColor(float& rr, float& g, float& b, float &t) const;
//	virtual void OpenGLDraw() const;
//	virtual void OpenGLDraw(const intersection&) const;
//	virtual void OpenGLDraw(const intersection&, const neighbor&) const;
//	virtual void GLDrawPath(const std::vector<intersection> &x) const;
protected:
	GraphEnvironment *ge;
	Graph *g;
	double maxSpeed;
	double scale;
};


#endif /* RoadMap_h */
