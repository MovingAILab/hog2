/**
* A 2D map environment with edge costs weighted according to the 
* number of times a unit has passed over each edge. 
* 
* @file WeightedMap2DEnvironment.h
* @package hog2
* @brief A 2D map environment with edge costs weighted according to the number of times a unit has passed over each edge. 
* @author Renee Jansen
* @date 06/20/2007
*
* This file is part of HOG2.
* HOG : http://www.cs.ualberta.ca/~nathanst/hog.html
* HOG2: http://code.google.com/p/hog2/
*
* HOG2 is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation; either version 2 of the License, or
* (at your option) any later version.
* 
* HOG2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
* 
* You should have received a copy of the GNU General Public License
* along with HOG2; if not, write to the Free Software
* Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#ifndef WEIGHTEDMAP2DENVIRONMENT_H
#define WEIGHTEDMAP2DENVIRONMENT_H

#include "Map2DEnvironment.h" 
#include "BitVector.h"
#include "OccupancyInterface.h"
#include "Graph.h"
#include <cmath>
#include <unordered_map>

class Vector2D {
	public:
		Vector2D(float _x,float _y):x(_x),y(_y),updateTime(0),accessTime(0) {/*Normalize();*/}
		Vector2D():x(0),y(0),updateTime(0),accessTime(0) {}
		void Set(float _x, float _y) { x=_x; y=_y; /*Normalize();*/}
		
		void SetUpdateTime(double t) {updateTime=t;}
		double GetUpdateTime() {return updateTime;}
		
		void SetAccessTime(double t) {accessTime=t;}
		double GetAccessTime() {return accessTime;}
	
		bool operator==(const Vector2D &rhs) {return ((x == rhs.x)&&(y==rhs.y));}
		std::ostream& operator <<(std::ostream & out)
		{
			out << "(" << x << ", " << y << ")";
			return out;	
		}
		
		friend Vector2D operator *(const Vector2D& vec, const double num)
		{
			Vector2D returnme(vec.x * num, vec.y * num);
			return returnme;
		}
		
		friend Vector2D operator *(const double num, const Vector2D& vec)
		{
			Vector2D returnme(vec.x * num, vec.y * num);
			return returnme;
		}
		
		friend Vector2D operator +(const Vector2D& v1, const Vector2D& v2)
		{
			Vector2D returnme(v1.x + v2.x, v1.y + v2.y);
			//returnme.Normalize();
			return returnme;
		}
	//private:
		float x, y;
		double updateTime, accessTime;
		
		void Normalize()
		{
			if ((x==0)&&(y==0))
				return;
			float magnitude = sqrt(x*x + y*y);
			x /= magnitude;
			y /= magnitude;
		}
};


namespace AngleUtil {

	class AngleSearchNode{
		public:
			AngleSearchNode(const xyLoc &s,uint64_t key)
				:node(s), hashKey(key) {}
			xyLoc node;
			uint64_t hashKey;
	};

	struct SearchNodeEqual {
		bool operator()(const AngleSearchNode &i1, const AngleSearchNode &i2) const
		{ return (i1.node == i2.node); } };
		
	struct SearchNodeHash {
		size_t operator()(const AngleSearchNode &x) const
		{ return (size_t)(x.hashKey); } };
	
	 typedef std::unordered_map<AngleUtil::AngleSearchNode,Vector2D, AngleUtil::SearchNodeHash, AngleUtil::SearchNodeEqual> AngleLookupTable;
};

/** Edge labels */
enum {
 	kForwardCount = 2, // The number of units passing from From to To
 	kBackwardCount = 3 // The number of units passing from To to From
};


class WeightedMap2DEnvironment : public AbsMapEnvironment
{
public:
	WeightedMap2DEnvironment(MapAbstraction *ma);
	WeightedMap2DEnvironment(AbsMapEnvironment *ame);
	virtual ~WeightedMap2DEnvironment();
	void ApplyAction(xyLoc &s, tDirection dir) const;
	virtual double GCost(const xyLoc &node1, const xyLoc &node2) const;
	virtual double GCost(const xyLoc &node1, const tDirection &act) const { return AbsMapEnvironment::GCost(node1, act); }
	//virtual BaseMapOccupancyInterface* GetOccupancyInterface(){std::cout<<"Returning "<<oi<<std::endl;return oi;}
	virtual BaseMapOccupancyInterface* GetOccupancyInfo(){return oi;}
//	void OpenGLDraw() const;
//	void OpenGLDraw(const xyLoc &l)  const{ AbsMapEnvironment::OpenGLDraw(l); }
//	void OpenGLDraw(const xyLoc& s, const tDirection &dir) const;
//	void OpenGLDraw(const xyLoc &l1, const xyLoc &l2, float v) const { MapEnvironment::OpenGLDraw(l1, l2, v); }
//	void DrawEdge(const edge* e) const;
	
	void UpdateAngle(const xyLoc &old, const xyLoc &s, double t);
	
	/** Set the cost of moving in the "wrong" direction */ 
	void SetWeight(double wt) { diffWeight = wt; }
	/** Set the weight (proportion) of the old angle */ 
	void SetProportionOld(double prop) { assert((prop>=0) && (prop <=1)); oldProportion = prop; }
	
	void SetUpdateOnQuery(bool b) { updateOnQuery = b;}
	/** Set the weight (proportion) of the old angle for queried edge updates */
	void SetQueryProportionOld(double prop) { assert((prop>=0) && (prop <=1)); queryOldProportion = prop; }
	
	void SetUpdateSurrounding(bool b) { updateSurrounding = b; }
	void SetSurroundingProportion(double prop) { assert((prop>=0) && (prop <=1)); surroundingProportion = prop;}
	
	// Use weights within window only
	void UseWindow(bool b) {useWindow=b;}
	void SetWindowCenter(xyLoc l) {windowCenter=l;}
	void SetWindowSize(double d) {windowSize=d;}
	
	// Needed for local copy updating of weights
	void SetAngle(xyLoc &l, Vector2D angle);
	Vector2D GetAngle(xyLoc &l);
	void SetNoWeighting(bool b) {noWeighting = b; }
	
	// For perceptron update rule
	void UsePerceptron(double lr) { usePerceptron = true; learningRate = lr; }
	
	double ComputeArrowMetric(bool timed=false,double time=0, bool DoNormalize=false, double maxtime=0);
	Vector2D GetAngleFromDirection(tDirection dir) const;

private:
	BaseMapOccupancyInterface* oi;
	
	//typedef std::unordered_map<AngleUtil::AngleSearchNode,Vector2D, AngleUtil::SearchNodeHash, AngleUtil::SearchNodeEqual>
	typedef std::unordered_map<AngleUtil::AngleSearchNode,Vector2D, AngleUtil::SearchNodeHash, AngleUtil::SearchNodeEqual> AngleLookupTable;
	AngleLookupTable angleLookup;
	
	void UpdateAngle(const xyLoc &old, const xyLoc &s, double prop, double t);
	
	double diffWeight;
	double oldProportion;
	
	// Only use weights within some window
	xyLoc windowCenter;
	bool useWindow;
	double windowSize;
	
	bool noWeighting;
	
	bool updateOnQuery;
	double queryOldProportion;
	
	bool updateSurrounding;
	double surroundingProportion;
	
	bool usePerceptron;
	double learningRate;
};

typedef UnitSimulation<xyLoc, tDirection, WeightedMap2DEnvironment> UnitWeightedMapSimulation;

// Weighting with node angles




#endif
