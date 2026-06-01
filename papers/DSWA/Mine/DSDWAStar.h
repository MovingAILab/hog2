//
//  DSDWAStar.h
//  HOG2 ObjC
//
//  Created by Nathan Sturtevant on 12/26/22.
//  Copyright © 2022 MovingAI. All rights reserved.
//

#ifndef DSDWAStar_h
#define DSDWAStar_h

#include "TemplateAStar.h" // to get state definitions

enum tExpansionPriority {
    kWA=0,
    kpwXD=1,
    kpwXU=2,
    kXDP=3,
    kXUP=4,
    DWP=5,
    MAP=6,
    kHalfEdgeDrop=7,
    kGreedy=8,
    kDSDPolicyCount=9,
};

const char *policyName[] = {"Weighted A*", "pwXD", "pwXU", "XDP", "XUP", "DWP", "MAP", "Half Edge", "Greedy", "[error]"};

double table_step=2.0;
template <class state, class action, class environment, class openList = AStarOpenClosed<state, AStarCompareWithF<state>, AStarOpenClosedDataWithF<state>> >
class DSDWAStar : public GenericSearchAlgorithm<state,action,environment> {
public:
	DSDWAStar() {
		ResetNodeCount(); env = 0; stopAfterGoal = true; weight=1; reopenNodes = false; theHeuristic = 0;
		theConstraint = 0;
	}
	virtual ~DSDWAStar() {}
	void GetPath(environment *env, const state& from, const state& to, std::vector<state> &thePath);
	void GetPath_v2(environment *env, const state& from, const state& to, std::vector<state> &thePath);
	void GetPath(environment *, const state&, const state&, std::vector<action> & );
	
	openList openClosedList;
	state goal, start;
	
	bool InitializeSearch(environment *env, const state& from, const state& to, std::vector<state> &thePath);
	bool InitializeSearch_v2(environment *env, const state& from, const state& to, std::vector<state> &thePath);
	bool DoSingleSearchStep(std::vector<state> &thePath);
	bool DoSingleSearchStep_v2(std::vector<state> &thePath);
	void AddAdditionalStartState(state& newState);
	void AddAdditionalStartState(state& newState, double cost);
	
	state CheckNextNode();
	void ExtractPathToStart(state &node, std::vector<state> &thePath)
	{
		thePath.clear();
		uint64_t theID;
		if (openClosedList.Lookup(env->GetStateHash(node), theID) != kNotFound)
			ExtractPathToStartFromID(theID, thePath);
	}
	void ExtractPathToStartFromID(uint64_t node, std::vector<state> &thePath);
	const state &GetParent(const state &s);
	virtual const char *GetName();
	
	void PrintStats();
	uint64_t GetUniqueNodesExpanded() { return uniqueNodesExpanded; }
	void ResetNodeCount() { nodesExpanded = nodesTouched = 0; uniqueNodesExpanded = 0; firstLast=0; secondLast=0; thirdLast=0;}
	int GetMemoryUsage();
	
	bool GetClosedListGCost(const state &val, double &gCost) const;
	bool GetOpenListGCost(const state &val, double &gCost) const;
	bool GetHCost(const state &val, double &hCost) const;
	bool GetClosedItem(const state &s, AStarOpenClosedDataWithF<state> &);
	unsigned int GetNumOpenItems() { return openClosedList.OpenSize(); }
	inline const AStarOpenClosedDataWithF<state> &GetOpenItem(unsigned int which) { return openClosedList.Lookat(openClosedList.GetOpenItem(which)); }
	inline const int GetNumItems() { return openClosedList.size(); }
	inline const AStarOpenClosedDataWithF<state> &GetItem(unsigned int which) { return openClosedList.Lookat(which); }
	bool HaveExpandedState(const state &val)
	{ uint64_t key; return openClosedList.Lookup(env->GetStateHash(val), key) != kNotFound; }
	dataLocation GetStateLocation(const state &val)
	{ uint64_t key; return openClosedList.Lookup(env->GetStateHash(val), key); }
	
	void SetReopenNodes(bool re) { reopenNodes = re; }
	bool GetReopenNodes() { return reopenNodes; }

	void SetHeuristic(Heuristic<state> *h) { theHeuristic = h; }
	void SetConstraint(Constraint<state> *c) { theConstraint = c; }

	uint64_t GetNodesExpanded() const { return nodesExpanded; }
	uint64_t GetNodesTouched() const { return nodesTouched; }
	uint64_t GetNecessaryExpansions() const;
	void LogFinalStats(StatCollection *) {}
	
	void SetStopAfterGoal(bool val) { stopAfterGoal = val; }
	bool GetStopAfterGoal() { return stopAfterGoal; }
		
	void OpenGLDraw() const;
	void Draw(Graphics::Display &disp) const;
	void DrawPriorityGraph(Graphics::Display &disp) const;
	void DrawPriorityGraph_v2(Graphics::Display &disp) const;
	
	double Phi(double h, double g)
	{
		if (policy == kXDP)
        {
                float nextF = (g+(2*weight-1)*h+sqrt((g-h)*(g-h)+4*weight*g*h))/(2*weight);
                return nextF;
        }
        if (policy == kXUP)
        {
                float nextF = (g+h+sqrt((g+h)*(g+h)+4*weight*(weight-1)*h*h))/(2*weight);
                return nextF;
        }
        else return GetPriority(h, g);
	}

	// This is a new version that uses a vector lookup table
    double Phi_v2(double h, double g)
    {
        if (policy == kXDP)
        {
                float nextF = (g+(2*weight-1)*h+sqrt((g-h)*(g-h)+4*weight*g*h))/(2*weight);
                return nextF;
        }
        if (policy == kXUP)
        {
                float nextF = (g+h+sqrt((g+h)*(g+h)+4*weight*(weight-1)*h*h))/(2*weight);
                return nextF;
        }
        return GetPriority_v2(h, g);
    }

	void SetWeight(double w)
	{
		weight = w;
	}
	double GetWeight() { return weight; }
	float GetPriority(float h, float g)
	{
		if (data.size() == 0)
		{
			if (fequal(g, 0))
				return h;
			printf("WARNING: Invalid priority lookups %s line %d\n", __FILE__, __LINE__);
			return INFINITY;
		}
		float slope = g/h;
		if (fgreater(slope, data.back().slope))
		{
			printf("WARNING: Invalid priority lookups %s line %d\n", __FILE__, __LINE__);
			return INFINITY;
		}
		// dumb/slow but correct
		for (int x = 0; x < data.size(); x++)
			if (flesseq(slope, data[x].slope))
				return data[x].K*(g + data[x].weight * h);
		return INFINITY;
	}

	/**
    * Returns the current the priority of a state given its h anf g values.
    * Iterates through the data, to find the region that covers the slope (g/h) of the given state.
    * @param h and @param g The h and g values of the state
    * This is a new version that uses a vector lookup table
    **/
    float GetPriority_v2(float h, float g)
    {
        if (LookUpVector.size() == 1)
        {
            if (fequal(g, 0))
                return h;
            printf("WARNING: Invalid priority lookups %s line %d - code 0\n", __FILE__, __LINE__);
            return INFINITY;
        }

        if(fequal(h, 0)) return LookUpVector[90*(1/table_step)].K*(g + LookUpVector[90*(1/table_step)].weight * h);

        float slope = g/h;
        float angle = atan2f(g, h)/PID180;
        int rounded_angle = int(angle*(1/table_step));
        long long int next_angle = LookUpVector.size();

        if (fgreater(rounded_angle, next_angle-1))
        {
            printf("WARNING: Invalid priority lookups %s line %d - code 1\n", __FILE__, __LINE__);
            // std::cout<<"Code 1: last_in_lookup: "<<last_in_lookup<<" | ceil(angle): "<<ceil(angle)<<"\n";
            std::cout<<"Code 1: last_stored_angle: "<<next_angle-1<<" | rounded_angle: "<<rounded_angle<<"\n";
            return INFINITY;
        }
        
        return LookUpVector[rounded_angle].K*(g + LookUpVector[rounded_angle].weight * h);
    }

	/**
    * Returns the current weight for this state--slope.
    * Returns the last weight if it is a new slope (not in data)
    * @param h and @param g The h and g values of the state
    **/
    float GetCurrentWeight(float h, float g)
    {
        if(data.size()>0)
        {
            float slope = g/h;
            for (int x = 0; x < data.size(); x++)
                if (flesseq(slope, data[x].slope))
                    {
                        return data[x].weight;
                    }
            return data.back().weight;
        }
        else
        {
            return weight;
        }
    }
	
	// directly target next suboptimality
	void SetNextWeight(float h, float g, float targetWeight) // const Graphics::point &loc
	{
		if (fequal(h, 0))
		{
			float w;
			point3d last;
			if (data.size() > 0)
				 last = data.back().crossPoint;
			else
				last = {1, 0};
			w = (weight-last.y)/last.x;
			// connect to y axis at bound*(1)
			float K = 1/(last.y+w*last.x);
			data.push_back({INFINITY, w, K, {static_cast<float>(weight), 0.0f}});
		}
		if (fgreater(h, 0) && fgreater(g, 0))
		{
			float slope = g/h;
			if (data.size() == 0 || data.back().slope < slope)
			{
				//std::cout << "Virtual hit of " << loc << " slope " << slope << "\n";
				float minWeight, maxWeight;
				if (data.size() > 0)
					GetNextWeightRange(minWeight, maxWeight, data.back().crossPoint, slope);
				else
					GetNextWeightRange(minWeight, maxWeight, {1, 0}, slope);

				float K;
				float nextWeight = std::min(maxWeight, std::max(minWeight, targetWeight));
				point3d last;
				if (data.size() == 0)
				{
					last = point3d(1, 0);
				}
				else {
					last = data.back().crossPoint;
				}
				K = 1/(last.y+nextWeight*last.x);

				// our cross point of next slope
				point3d crossPoint1;
				crossPoint1.x = 1.0f/(K*(slope+nextWeight));
				crossPoint1.y = crossPoint1.x*slope;
				
				data.push_back({slope, nextWeight, K, crossPoint1});
			}
		}
	}

	// directly target next suboptimality
    // sets the targetWeight to the region that these new h ang g values create (if they do)
    // This is a new version that uses a vector lookup table
    void SetNextWeight_v2(float h, float g, float targetWeight) // const Graphics::point &loc
    {
        float slope = g/h;
        float angle = atan2f(g, h)/PID180;
        int rounded_angle = int(angle*(1/table_step));
        int next_angle = LookUpVector.size();

        if (fequal(h, 0))
        {
            float w;
            point3d last;
            if (next_angle > 1)
                 last = LookUpVector.back().crossPoint;
            else
                last = {1, 0};
            w = (weight-last.y)/last.x;
            // connect to y axis at bound*(1)
            float K = 1/(last.y+w*last.x);

            //Method 1: use for loop and push_back
            // for(int i=next_angle; i<= 90*(1/table_step); i++){
            //     LookUpVector.push_back({w, K, {static_cast<float>(weight), 0.0f}});
            // }
            //Method 2: use insert instead of for loop and push_back
            LookUpVector.insert(LookUpVector.end(), 90*(1/table_step)-next_angle+1, {w, K, {static_cast<float>(weight), 0.0f}});
        }
        if (fgreater(h, 0) && fgreater(g, 0))
        {
            
            if (next_angle == 1 || fless(next_angle-1, rounded_angle)){
                float minWeight, maxWeight;
                if (next_angle > 1)
                    GetNextWeightRange(minWeight, maxWeight, LookUpVector.back().crossPoint, slope);
                else
                    GetNextWeightRange(minWeight, maxWeight, {1, 0}, slope);

                float K;
                float nextWeight = std::min(maxWeight, std::max(minWeight, targetWeight));
                point3d last;
                if (next_angle == 1)
                {
                    last = point3d(1, 0);
                }
                else {
                    last = LookUpVector.back().crossPoint;
                }
                K = 1/(last.y+nextWeight*last.x);

                // our cross point of next slope
                point3d crossPoint1;
                crossPoint1.x = 1.0f/(K*(slope+nextWeight));
                crossPoint1.y = crossPoint1.x*slope;
                
                // loop through all angles between last in lookup and angle and define all of them with nextWeight

                //Method 1: use for loop and push_back
                // for(int i=next_angle; i<= rounded_angle; i++){
                //     LookUpVector.push_back({nextWeight, K, crossPoint1});
                // }
                //Method 2: use insert instead of for loop and push_back
                LookUpVector.insert(LookUpVector.end(), rounded_angle-next_angle+1, {nextWeight, K, crossPoint1});

                // std::cout<<"prev angle "<<next_angle-1<<" | rounded_angle: "<<rounded_angle<<"| nextWeight: "<<nextWeight<<"\n";

            }
            else{
                std::cout<<"Error code 3!\n";
            }
        }
    }
	
	void SetNextPriority(float h, float g, float target) // const Graphics::point &loc
	{
		if (fequal(h, 0))
		{
			float w;
			point3d last;
			if (data.size() > 0)
				 last = data.back().crossPoint;
			else
				last = {1, 0};
			w = (weight-last.y)/last.x;
			// connect to y axis at bound*(1)
			float K = 1/(last.y+w*last.x);
			data.push_back({INFINITY, w, K, {static_cast<float>(weight), 0.0f}});
		}
		if (fgreater(h, 0) && fgreater(g, 0))
		{
			float slope = g/h;
			if (data.size() == 0 || data.back().slope < slope)
			{
				//std::cout << "Virtual hit of " << loc << " slope " << slope << "\n";
				float minWeight, maxWeight;
				if (data.size() > 0)
					GetNextWeightRange(minWeight, maxWeight, data.back().crossPoint, slope);
				else
					GetNextWeightRange(minWeight, maxWeight, {1, 0}, slope);

				float K;
				float nextWeight;
				point3d last;
				if (data.size() == 0)
				{
					last = point3d(1, 0);
				}
				else {
					last = data.back().crossPoint;
				}

				
				switch (policy)
				{
					case kWA: // Weighted A*
					{
						K = 1/weight;
						nextWeight = weight;
						break;
					}
					case kXUP: // PWXUP
					{
						nextWeight = maxWeight;
						K = 1/(last.y+nextWeight*last.x);
						break;
					}
					case kXDP: // PWXDP
					{
						nextWeight = minWeight;
						K = 1/(last.y+nextWeight*last.x);
						break;
					}
					default:
					case kGreedy:
					{
						// K (g + [w] * h) = 1 at previous point
						// returns nextWeight and K
						nextWeight = ChooseWeightForTargetPriority({h, g}, target, minWeight, maxWeight, last, K);
					}
				}
				

				// our cross point of next slope
				point3d crossPoint1;
				crossPoint1.x = 1.0f/(K*(slope+nextWeight));
				crossPoint1.y = crossPoint1.x*slope;
				
				data.push_back({slope, nextWeight, K, crossPoint1});
				
	//			std::cout << "Cross Priorities: ";
	//			for (const auto &i : data)
	//			{
	//				std::cout << i.crossPoint << ": ";
	//				std::cout << GetPriority(i.crossPoint.x, i.crossPoint.y) << " ";
	//			}
	//			std::cout << "\n";
			}
		}
	}
	
	//using a target priority value, sets the w_i for the new region.
    // This is a new version that uses a vector lookup table
    void SetNextPriority_v2(float h, float g, float target) // const Graphics::point &loc
    {
        float slope = g/h;
        float angle = atan2f(g, h)/PID180;
        int rounded_angle = int(angle*(1/table_step));
        int next_angle = LookUpVector.size();

        if (fequal(h, 0))
        {
            float w;
            point3d last;
            if (next_angle > 1)
                 last = LookUpVector.back().crossPoint;
            else
                last = {1, 0};
            w = (weight-last.y)/last.x;
            // connect to y axis at bound*(1)
            float K = 1/(last.y+w*last.x);
            
            //Method 1: use for loop and push_back
            // for(int i=next_angle; i<= 90*(1/table_step); i++){
            //     LookUpVector.push_back({w, K, {static_cast<float>(weight), 0.0f}});
            // }
            //Method 2: use insert instead of for loop and push_back
            LookUpVector.insert(LookUpVector.end(), 90*(1/table_step)-next_angle+1, {w, K, {static_cast<float>(weight), 0.0f}});

        }
        if (fgreater(h, 0) && fgreater(g, 0))
        {
            if (next_angle == 1 || fless(next_angle-1, rounded_angle))
            {
                //std::cout << "Virtual hit of " << loc << " slope " << slope << "\n";
                float minWeight, maxWeight;
                if (next_angle > 1)
                    GetNextWeightRange(minWeight, maxWeight, LookUpVector.back().crossPoint, slope);
                else
                    GetNextWeightRange(minWeight, maxWeight, {1, 0}, slope);
                
                float K;
                float nextWeight;
                point3d last;
                if (next_angle == 1)
                {
                    last = point3d(1, 0);
                }
                else {
                    last = LookUpVector.back().crossPoint;
                }
    
                switch (policy)
                {
                    case kWA: // Weighted A*
                    {
                        K = 1/weight;
                        nextWeight = weight;
                        break;
                    }
                    case kpwXU: // PWXUP
                    {
                        nextWeight = maxWeight;
                        K = 1/(last.y+nextWeight*last.x);
                        break;
                    }
                    case kpwXD: // PWXDP
                    {
                        nextWeight = minWeight;
                        K = 1/(last.y+nextWeight*last.x);
                        break;
                    }
                    default:
                    // case kGreedy:
                    {
                        // K (g + [w] * h) = 1 at previous point
                        // returns nextWeight and K
                        nextWeight = ChooseWeightForTargetPriority({h, g}, target, minWeight, maxWeight, last, K);
                    }
                }
                
                // our cross point of next slope
                point3d crossPoint1;
                crossPoint1.x = 1.0f/(K*(slope+nextWeight));
                crossPoint1.y = crossPoint1.x*slope;
                
                // loop through all slopes between last in lookup and slope and define all of them with nextWeight

                //Method 1: use for loop and push_back
                // for(int i=next_angle; i<= rounded_angle; i++){
                //     LookUpVector.push_back({nextWeight, K, crossPoint1});
                // }
                //Method 2: use insert instead of for loop and push_back
                LookUpVector.insert(LookUpVector.end(), rounded_angle-next_angle+1, {nextWeight, K, crossPoint1});

                // std::cout<<"prev angle "<<next_angle-1<<" | rounded_angle: "<<rounded_angle<<"| nextWeight: "<<nextWeight<<"\n";
                
                // std::cout << "Cross Priorities: ";
                // for (const auto &i : data)
                // {
                //     std::cout << i.crossPoint << ": ";
                //     std::cout << GetPriority(i.crossPoint.x, i.crossPoint.y) << " ";
                // }
                // std::cout << "\n";
            }
        }
    }

	/**
	 * Given the slope of the next bounding line, give the possbile range of weights that can be used in the priority function
	 *
	 * \param minWeight (returned) The minimum weight that can be used without going under the lower limit
	 * \param maxWeight (returned) The maximum weight that can be used without going over the upper limit
	 **/
	void GetNextWeightRange(float &minWeight, float &maxWeight, float nextSlope)
	{
		if (data.size() > 0)
			GetNextWeightRange(minWeight, maxWeight, data.back().crossPoint, nextSlope);
		else
			GetNextWeightRange(minWeight, maxWeight, {1, 0}, nextSlope);
	}

	/**
     * Given the slope of the next bounding line, give the possbile range of weights that can be used in the priority function
     *
     * \param minWeight (returned) The minimum weight that can be used without going under the lower limit
     * \param maxWeight (returned) The maximum weight that can be used without going over the upper limit
     * // This is a new version that uses a vector lookup table
     **/
    void GetNextWeightRange_v2(float &minWeight, float &maxWeight, float nextSlope)
    {
        if (LookUpVector.size() > 1)
            GetNextWeightRange(minWeight, maxWeight, LookUpVector.back().crossPoint, nextSlope);
        else
            GetNextWeightRange(minWeight, maxWeight, {1, 0}, nextSlope);
        
    }

	/**
	 * Given the slope of the next bounding line, give the possbile range of weights that can be used in the priority function
	 *
	 * \param minWeight (returned) The minimum weight that can be used without going under the lower limit
	 * \param maxWeight (returned) The maximum weight that can be used without going over the upper limit
	 * \param currPoint The point on the previous bounding line with priorirty 1.0
	 * \param nextSlope The slope of the next bounding line
	 **/
	void GetNextWeightRange(float &minWeight, float &maxWeight, point3d currPoint, float nextSlope)
	{
		// defaults
		minWeight = 1;
		maxWeight = 2*weight-1;

		// 0. next slope line is y = slope*x
		// 1. cannot go over the [y = -x + w] line
		// slope*x = -x + w; slope*x + x = w; x = w/(slope+1)
		// y/slope = w-y; y = slope * w - slope *y; y*(1+slope) = slope*w; y = slope*w/(1+slope)
		point3d upperPoint(weight/(nextSlope+1),
						   nextSlope*weight/(1+nextSlope));
	//	std::cout << "Upper point: " << upperPoint << "\n";
		// 2. cannot go under the [y = -(2w-1)x + w] line
		// slope*x = -(2w-1)x + w; x*(slope + (2w-1)) = w
		// y/slope = (w-y)/(2w-1)
		// y (2w-1) = slope * w - slope*y
		// y (slope + 2w-1) = slope*w
		point3d lowerPoint(weight/(nextSlope+2*weight-1),
						   nextSlope*weight / (nextSlope + 2*weight-1));
		// get (negative) slopes to upper and lower points
		minWeight = std::max(minWeight, (lowerPoint.y-currPoint.y)/(currPoint.x-lowerPoint.x));
		if (upperPoint.x < currPoint.x)
			maxWeight = std::min(maxWeight, (upperPoint.y-currPoint.y)/(currPoint.x-upperPoint.x));
	//	printf("Weight needs to be [%f, %f]\n", minWeight, maxWeight);
	}
	
	/*
	 * Given location, and a range of priorities, try to give the point the exact desired priority
	 * Returned priority will always be in the minWeight/maxWeight range
	 */
	float ChooseWeightForTargetPriority(point3d loc, float priority, float minWeight, float maxWeight, point3d last, float &K)
	{
		float weight;
		// New: Last point is the priority 1 crossing point.
		// 1. Find the point on the previous line with priority {priority}
		point3d projectedPoint = last*priority;
		// 2. FYI: loc is the point on the new line that we want to have the desired priority
		// 3. Find the slope between the last point and our new point.
		//    Which is delta y / delta x
		if (flesseq(loc.y, projectedPoint.y))
		{
			//printf("Ill defined case (new y < old y); defaulting to min\n");

			// Explainer: State can be immediately expanded, just use min
			K = 1/(last.y+minWeight*last.x);
			return minWeight;
		}
		if (fgreatereq(loc.x, projectedPoint.x))
		{
//			printf("Ill defined case (new x > old x); defaulting to max\n");
			
			K = 1/(last.y+maxWeight*last.x);
			return maxWeight;
		}
		
		// Then try to extend that point to the new point giving it the desired priority
		weight = (loc.y-projectedPoint.y)/(projectedPoint.x-loc.x);
		// and bound by min/max weight
		weight = std::max(std::min(weight, maxWeight), minWeight);
		K = 1/(last.y+weight*last.x);
		return weight;
	}
	tExpansionPriority policy = kHalfEdgeDrop;
private:
	point3d HOGToLocal(point3d p) const
	{
		return point3d((p.x+1)*weight/2.0f , (p.y-1)*weight/-2.0);
	}

	point3d LocalToHOG(point3d p) const
	{
		return point3d(2.0f*(p.x/weight-0.5), -2.0*(p.y/weight-0.5));
	}

	uint64_t nodesTouched, nodesExpanded;

	uint64_t firstLast, secondLast, thirdLast;

	std::vector<state> neighbors;
	std::vector<uint64_t> neighborID;
	std::vector<double> edgeCosts;
	std::vector<double> heuristicCosts;
	std::vector<dataLocation> neighborLoc;
	environment *env;
	bool stopAfterGoal;
	
	double goalFCost;
	double weight;
	bool reopenNodes;
	uint64_t uniqueNodesExpanded;
	environment *radEnv;
	Heuristic<state> *theHeuristic;
	Constraint<state> *theConstraint;

	struct DSDdata {
		float slope;
		float weight;
		float K;
		point3d crossPoint; // cached for simplicity
	};
	std::vector<DSDdata> data;

	struct DSDdata_v2 {
        float weight;
        float K;
        point3d crossPoint; // cached for simplicity
    };

    std::vector<DSDdata_v2> LookUpVector;
};

/**
 * Return the name of the algorithm.
 * @author Nathan Sturtevant
 * @date 03/22/06
 *
 * @return The name of the algorithm
 */

template <class state, class action, class environment, class openList>
const char *DSDWAStar<state,action,environment,openList>::GetName()
{
	static char name[32];
	sprintf(name, "DSDWAStar[]");
	return name;
}

/**
 * Perform an A* search between two states.
 * @author Nathan Sturtevant
 * @date 03/22/06
 *
 * @param _env The search environment
 * @param from The start state
 * @param to The goal state
 * @param thePath A vector of states which will contain an optimal path
 * between from and to when the function returns, if one exists.
 */
template <class state, class action, class environment, class openList>
void DSDWAStar<state,action,environment,openList>::GetPath(environment *_env, const state& from, const state& to, std::vector<state> &thePath)
{
	if (!InitializeSearch(_env, from, to, thePath))
	{
		return;
	}
	while (!DoSingleSearchStep(thePath))
	{
		if (10000000 == nodesExpanded){
            //Terminate the search after 10 million node expansions.
            printf("%" PRId64 " nodes expanded, %" PRId64 " generated. ", nodesExpanded, nodesTouched);
            std::cout<<"Policy "<<policy<<" => Terminated.\n";
            break;
        }
//		if (0 == nodesExpanded%100000)
//			printf("%" PRId64 " nodes expanded, %" PRId64 " generated\n", nodesExpanded, nodesTouched);
	}
}

/**
 * Perform an A* search between two states.
 * @param _env The search environment
 * @param from The start state
 * @param to The goal state
 * @param thePath A vector of states which will contain an optimal path
 * between from and to when the function returns, if one exists.
 * This is a new version: -- it uses the lookup table
 */
template <class state, class action, class environment, class openList>
void DSDWAStar<state,action,environment,openList>::GetPath_v2(environment *_env, const state& from, const state& to, std::vector<state> &thePath)
{
    if (!InitializeSearch_v2(_env, from, to, thePath))
    {
        return;
    }
    // keeps doing single search step, which is expansion of one node, and adding its successors to open
    // until the solution is found or open is empty.
    while (!DoSingleSearchStep_v2(thePath))
    {
        if (10000000 == nodesExpanded){
            //Terminate the search after 10 million node expansions.
            printf("%" PRId64 " nodes expanded, %" PRId64 " generated. ", nodesExpanded, nodesTouched);
            std::cout<<"Policy "<<policy<<" => Terminated.\n";
            break;
        }
    }

}


template <class state, class action, class environment, class openList>
void DSDWAStar<state,action,environment,openList>::GetPath(environment *_env, const state& from, const state& to, std::vector<action> &path)
{
	std::vector<state> thePath;
	if (!InitializeSearch(_env, from, to, thePath))
	{
		return;
	}
	path.resize(0);
	while (!DoSingleSearchStep(thePath))
	{
	}
	for (size_t x = 0; x < thePath.size()-1; x++)
	{
		path.push_back(_env->GetAction(thePath[x], thePath[x+1]));
	}
}


/**
 * Initialize the A* search
 * @author Nathan Sturtevant
 * @date 03/22/06
 *
 * @param _env The search environment
 * @param from The start state
 * @param to The goal state
 * @return TRUE if initialization was successful, FALSE otherwise
 */
template <class state, class action, class environment, class openList>
bool DSDWAStar<state,action,environment,openList>::InitializeSearch(environment *_env, const state& from, const state& to, std::vector<state> &thePath)
{
	if (theHeuristic == 0)
		theHeuristic = _env;
	thePath.resize(0);
	env = _env;
	openClosedList.Reset(env->GetMaxHash());
	ResetNodeCount();
	start = from;
	goal = to;
	if (env->GoalTest(from, to) && (stopAfterGoal)) //assumes that from and to are valid states
	{
		return false;
	}
	data.resize(0);

	double h = theHeuristic->HCost(start, goal);
	openClosedList.AddOpenNode(start, env->GetStateHash(start), Phi(h, 0), 0, h);
	
	return true;
}

/**
 * Initialize the A* search
 * @param _env The search environment
 * @param from The start state
 * @param to The goal state
 * @return TRUE if initialization was successful, FALSE otherwise
 * This is a new version that uses a vector lookup table
 */
template <class state, class action, class environment, class openList>
bool DSDWAStar<state,action,environment,openList>::InitializeSearch_v2(environment *_env, const state& from, const state& to, std::vector<state> &thePath)
{
    if (theHeuristic == 0)
        theHeuristic = _env;
    thePath.resize(0);
    env = _env;
    openClosedList.Reset(env->GetMaxHash());
    ResetNodeCount();
    start = from;
    goal = to;
    if (env->GoalTest(from, to) && (stopAfterGoal)) //assumes that from and to are valid states
    {
        return false;
    }

    LookUpVector.resize(0);
    //Insert a dummy element, to start the LookUpVector from index 1.
    point3d crossPoint1;
    crossPoint1.x = 1.0f;
    crossPoint1.y = 1.0f;
    LookUpVector.push_back({0, 0, crossPoint1});


    double h = theHeuristic->HCost(start, goal);
    openClosedList.AddOpenNode(start, env->GetStateHash(start), Phi_v2(h, 0), 0, h);
    
    return true;
}


/**
 * Add additional start state to the search. This should only be called after Initialize Search and before DoSingleSearchStep.
 * @author Nathan Sturtevant
 * @date 01/06/08
 */
template <class state, class action, class environment, class openList>
void DSDWAStar<state,action,environment,openList>::AddAdditionalStartState(state& newState)
{
	double h = theHeuristic->HCost(newState, goal);
	openClosedList.AddOpenNode(newState, env->GetStateHash(newState), Phi(h, 0), 0, h);
}

/**
 * Add additional start state to the search. This should only be called after Initialize Search
 * @author Nathan Sturtevant
 * @date 09/25/10
 */
template <class state, class action, class environment, class openList>
void DSDWAStar<state,action,environment,openList>::AddAdditionalStartState(state& newState, double cost)
{
	double h = theHeuristic->HCost(newState, goal);
	openClosedList.AddOpenNode(newState, env->GetStateHash(newState), Phi(h, cost), cost, h);
}

/**
 * Expand a single node.
 * @author Nathan Sturtevant
 * @date 03/22/06
 *
 * @param thePath will contain an optimal path from start to goal if the
 * function returns TRUE
 * @return TRUE if there is no path or if we have found the goal, FALSE
 * otherwise
 */
template <class state, class action, class environment, class openList>
bool DSDWAStar<state,action,environment,openList>::DoSingleSearchStep(std::vector<state> &thePath)
{
    if (openClosedList.OpenSize() == 0)
    {
        thePath.resize(0); // no path found!
        //closedList.clear();
        return true;
    }
    uint64_t nodeid = openClosedList.Close();
//    if (openClosedList.Lookup(nodeid).g+openClosedList.Lookup(nodeid).h > lastF)
//    { lastF = openClosedList.Lookup(nodeid).g+openClosedList.Lookup(nodeid).h;
//        //printf("Updated limit to %f\n", lastF);
//    }
    if (!openClosedList.Lookup(nodeid).reopened)
        uniqueNodesExpanded++;
    nodesExpanded++;

    if ((stopAfterGoal) && (env->GoalTest(openClosedList.Lookup(nodeid).data, goal)))
    {
        ExtractPathToStartFromID(nodeid, thePath);
        // Path is backwards - reverse
        reverse(thePath.begin(), thePath.end());
        goalFCost = openClosedList.Lookup(nodeid).f;// + openClosedList.Lookup(nodeid).h;

        return true;
    }
    
    neighbors.resize(0);
    edgeCosts.resize(0);
    neighborID.resize(0);
    neighborLoc.resize(0);
    heuristicCosts.resize(0);
    
    //std::cout << "Expanding: " << env->GetStateHash(openClosedList.Lookup(nodeid).data) << " with f:";
    //std::cout << openClosedList.Lookup(nodeid).g+openClosedList.Lookup(nodeid).h << std::endl;
    
    env->GetSuccessors(openClosedList.Lookup(nodeid).data, neighbors);
    float maxSlope = 0;
    float maxSlopeG=-1, maxSlopeH=-1;
    int which = -1;
    // 1. load all the children
    for (unsigned int x = 0; x < neighbors.size(); x++)
    {
        uint64_t theID;
        neighborLoc.push_back(openClosedList.Lookup(env->GetStateHash(neighbors[x]), theID));
        neighborID.push_back(theID);
        edgeCosts.push_back(env->GCost(openClosedList.Lookup(nodeid).data, neighbors[x]));
        heuristicCosts.push_back(theHeuristic->HCost(neighbors[x], goal));

        // open list can only be reduced in cost, thus not needing to extend the priority function
        if (neighborLoc[x] == kOpenList)
            continue;
        float slope = (openClosedList.Lookup(nodeid).g+edgeCosts[x])/heuristicCosts[x];
        //among all children, finds the find with the maximum slope, to check if we must define a new region.
        //as we've considered the child with maximum slope, there won't be any node being placed in an undefined region.
        if (fgreater(slope, maxSlope))
        {
            maxSlope = slope;
            maxSlopeG = openClosedList.Lookup(nodeid).g+edgeCosts[x];
            maxSlopeH = heuristicCosts[x];
            which = x;
        }
    }
    //if there is any child with a slope greater than zero
    if (fgreater(maxSlope, 0))
    {
        //each policy, sets a weight for the new region (if we must define a new one).
        if (openClosedList.OpenSize() != 0)
        {
            if (policy == kGreedy)
            {
                //new nodes will have the same priority as the node on top of open.
                //to place the new node on top of open, and move forward as fast as possible.
                SetNextPriority(maxSlopeH, maxSlopeG, openClosedList.Lookup(openClosedList.Peek()).f);
            }
            else if (policy == kHalfEdgeDrop) {
                //reduced a small amount from the priority value of the node on top of open, to handle local minimas better.
                SetNextPriority(maxSlopeH, maxSlopeG, openClosedList.Lookup(openClosedList.Peek()).f-edgeCosts[which]*(weight-1)/(2.0*weight));
            }
            else if (policy == kXDP)
            {
                //one of the baselines.
                float minWeight, maxWeight;
                GetNextWeightRange(minWeight, maxWeight, maxSlope);

                float nextF = (maxSlopeG+(2*weight-1)*maxSlopeH+sqrt((maxSlopeG-maxSlopeH)*(maxSlopeG-maxSlopeH)+4*weight*maxSlopeG*maxSlopeH))/(2*weight);
                SetNextPriority(maxSlopeH, maxSlopeG, nextF);

            }
            else if (policy == kXUP)
            {
                //one of the baselines.
                float minWeight, maxWeight;
                GetNextWeightRange(minWeight, maxWeight, maxSlope);

                float nextF = (maxSlopeG+maxSlopeH+sqrt((maxSlopeG+maxSlopeH)*(maxSlopeG+maxSlopeH)+4*weight*(weight-1)*maxSlopeH*maxSlopeH))/(2*weight);
                SetNextPriority(maxSlopeH, maxSlopeG, nextF);

            }
            else if (policy == MAP)
            {
                //assigns new weight, based on the number of node expansions of the previous regions.
                //assigns smaller weights, if the search is progressive and
                // most of the recent node expansions were in the most recent created region.

                //CHECK THE SLOPE OF THE EXPANDED NODE TO SEE WHICH REGION IT WAS FROM.
                //ADD ONE TO THE NUMBER OF THAT REGION.
                float nodeSlope = openClosedList.Lookup(nodeid).g/openClosedList.Lookup(nodeid).h;
                float wma_N;
                if(data.size()>=3)
                {
                    if(nodeSlope <= data[data.size()-3].slope)
                        thirdLast += 1;
                    else if(nodeSlope <= data[data.size()-2].slope)
                        secondLast += 1;
                    else if(nodeSlope <= data[data.size()-1].slope)
                        firstLast += 1;
                }
                if(fgreater(maxSlope, data.back().slope) || data.size() == 0){
                    float minWeight, maxWeight, midWeight, lowMidWeight, highMidWeight;
                    GetNextWeightRange(minWeight, maxWeight, maxSlope);

                    midWeight = (maxWeight + minWeight)/2;
                    lowMidWeight = (midWeight + minWeight)/2;
                    highMidWeight = (maxWeight + midWeight)/2;

                    std::vector<uint64_t> tempRegionsVec;
                    tempRegionsVec.push_back(firstLast);
                    tempRegionsVec.push_back(secondLast);
                    tempRegionsVec.push_back(thirdLast);
                    std::sort(tempRegionsVec.begin(), tempRegionsVec.end());

                    float WMA = (3*firstLast + 2*secondLast + 1*thirdLast)/6;

                    float rangeTop = (3*tempRegionsVec[2] + 2*tempRegionsVec[1] + 1*tempRegionsVec[0])/6;
                    float rangeButtom = (1*tempRegionsVec[2] + 2*tempRegionsVec[1] + 3*tempRegionsVec[0])/6;

                    //// 0<=wma_N<=1
                    if(rangeTop - rangeButtom !=0)
                        wma_N = 1-(WMA - rangeButtom)/(rangeTop - rangeButtom);
                    else
                        wma_N = 0;

                    float TheNextWeight = lowMidWeight + (highMidWeight-lowMidWeight)*wma_N;
               
                    SetNextWeight(maxSlopeH, maxSlopeG, TheNextWeight);

                    thirdLast = secondLast;
                    secondLast = firstLast;
                    firstLast = 0;
                }
            }
            else if (policy == DWP)
            {
                //changes the next weight to maxweight, if the search has reached a weighted region.
                //otherwise slowly increases the weight as the search proceeds.
                float minWeight, maxWeight;
                GetNextWeightRange(minWeight, maxWeight, maxSlope);
                float angle = atan2f(maxSlopeG,maxSlopeH)/PID180;
                if(fgreater(maxSlope, data.back().slope) || data.size() == 0){
                    double edgeHvalue = theHeuristic->HCost(openClosedList.Lookup(nodeid).data, neighbors[which]);
                    double edgeGvalue = edgeCosts[which];
                    if(fless(edgeHvalue/edgeGvalue, 1)){
                        SetNextWeight(maxSlopeH, maxSlopeG, edgeCosts[which]);
                    }
                    else{
                        SetNextWeight(maxSlopeH, maxSlopeG, minWeight+(maxWeight-minWeight)*pow((angle/90), 3));
                    }
                }
            }
            else {// To Run BaseLines using DSDWA* for graphic
                // -> CL code, uses template Astar
                // last argument will be ignored
                SetNextPriority(maxSlopeH, maxSlopeG, 0.01);
            }
        }
        else {
            // Expansion of the start state. open is empty.

            float minWeight, maxWeight;
            GetNextWeightRange(minWeight, maxWeight, maxSlope);

            if (policy == MAP || policy == DWP)
                SetNextWeight(maxSlopeH, maxSlopeG, weight);
            else
                SetNextPriority(maxSlopeH, maxSlopeG, 0.01);
        }
    }
    
    // iterate again updating costs and writing out to memory
    for (size_t x = 0; x < neighbors.size(); x++)
    {
        nodesTouched++;
        //std::cout << "looking at child with hash : " << env->GetStateHash(neighbors[x]) << "and g-cost"<<openClosedList.Lookup(nodeid).g+edgeCosts[x]<<std::endl;
        if (theConstraint &&
            theConstraint->ShouldNotGenerate(start, openClosedList.Lookup(nodeid).data, neighbors[x],
                                             openClosedList.Lookup(nodeid).g+edgeCosts[x], goal))
            continue;

        switch (neighborLoc[x])
        {
            case kClosedList:
                // TODO: Can update parent pointers when shorter paths are found to improve solution quality
                if (reopenNodes)
                {
                    if (fless(openClosedList.Lookup(nodeid).g+edgeCosts[x], openClosedList.Lookup(neighborID[x]).g))
                    {
                        auto &i = openClosedList.Lookup(neighborID[x]);
                        i.parentID = nodeid;
                        i.g = openClosedList.Lookup(nodeid).g+edgeCosts[x];
                        i.f = Phi(i.h, i.g);
                        openClosedList.Reopen(neighborID[x]);
                        // This line isn't normally needed, but in some state spaces we might have
                        // equality but different meta information, so we need to make sure that the
                        // meta information is also copied, since this is the most generic A* implementation
                        i.data = neighbors[x];
                    }
                }
                break;
            case kOpenList:
                if (fless(openClosedList.Lookup(nodeid).g+edgeCosts[x], openClosedList.Lookup(neighborID[x]).g))
                {
                    auto &i = openClosedList.Lookup(neighborID[x]);
                    i.parentID = nodeid;
                    i.g = openClosedList.Lookup(nodeid).g+edgeCosts[x];
                    i.f = Phi(i.h, i.g);
                    // This line isn't normally needed, but in some state spaces we might have
                    // equality but different meta information, so we need to make sure that the
                    // meta information is also copied, since this is the most generic A* implementation
                    i.data = neighbors[x];
                    openClosedList.KeyChanged(neighborID[x]);
//                    std::cout << " Reducing cost to " << openClosedList.Lookup(nodeid).g+edgeCosts[x] << "\n";
                }
                else {
//                    std::cout << " no cheaper \n";
                }
                break;
            case kNotFound:
                {
                    double h = heuristicCosts[x];
                    openClosedList.AddOpenNode(neighbors[x],
                                               env->GetStateHash(neighbors[x]),
                                               Phi(h, openClosedList.Lookup(nodeid).g+edgeCosts[x]),
                                               openClosedList.Lookup(nodeid).g+edgeCosts[x],
                                               h,
                                               nodeid);
                }
        }
    }

    return false;
}

/**
 * Expand a single node, add successors to open
 * @param thePath will contain an optimal path from start to goal if the
 * function returns TRUE
 * @return TRUE if there is no path or if we have found the goal, FALSE
 * otherwise
 * This is a new version that uses a vector lookup table
 */
template <class state, class action, class environment, class openList>
bool DSDWAStar<state,action,environment,openList>::DoSingleSearchStep_v2(std::vector<state> &thePath)
{
    if (openClosedList.OpenSize() == 0)
    {
        thePath.resize(0); // no path found!
        //closedList.clear();
        return true;
    }
    uint64_t nodeid = openClosedList.Close();
//    if (openClosedList.Lookup(nodeid).g+openClosedList.Lookup(nodeid).h > lastF)
//    { lastF = openClosedList.Lookup(nodeid).g+openClosedList.Lookup(nodeid).h;
//        //printf("Updated limit to %f\n", lastF);
//    }
    if (!openClosedList.Lookup(nodeid).reopened)
        uniqueNodesExpanded++;
    nodesExpanded++;

    if ((stopAfterGoal) && (env->GoalTest(openClosedList.Lookup(nodeid).data, goal)))
    {
        ExtractPathToStartFromID(nodeid, thePath);
        // Path is backwards - reverse
        reverse(thePath.begin(), thePath.end());
        goalFCost = openClosedList.Lookup(nodeid).f;// + openClosedList.Lookup(nodeid).h;

        return true;
    }
    
    neighbors.resize(0);
    edgeCosts.resize(0);
    neighborID.resize(0);
    neighborLoc.resize(0);
    heuristicCosts.resize(0);
    
    //std::cout << "Expanding: " << env->GetStateHash(openClosedList.Lookup(nodeid).data) << " with f:";
    //std::cout << openClosedList.Lookup(nodeid).g+openClosedList.Lookup(nodeid).h << std::endl;
    
    env->GetSuccessors(openClosedList.Lookup(nodeid).data, neighbors);
    float maxSlope = 0;
    float maxSlopeG=-1, maxSlopeH=-1;
    int which = -1;
    // 1. load all the children
    for (unsigned int x = 0; x < neighbors.size(); x++)
    {
        uint64_t theID;
        neighborLoc.push_back(openClosedList.Lookup(env->GetStateHash(neighbors[x]), theID));
        neighborID.push_back(theID);
        edgeCosts.push_back(env->GCost(openClosedList.Lookup(nodeid).data, neighbors[x]));
        heuristicCosts.push_back(theHeuristic->HCost(neighbors[x], goal));

        // open list can only be reduced in cost, thus not needing to extend the priority function
        if (neighborLoc[x] == kOpenList)
            continue;
        float slope = (openClosedList.Lookup(nodeid).g+edgeCosts[x])/heuristicCosts[x];
        //among all children, finds the find with the maximum slope, to check if we must define a new region.
        //as we've considered the child with maximum slope, there won't be any node being placed in an undefined region.
        if (fgreater(slope, maxSlope))
        {
            maxSlope = slope;
            maxSlopeG = openClosedList.Lookup(nodeid).g+edgeCosts[x];
            maxSlopeH = heuristicCosts[x];
            which = x;
        }
    }
    //if there is any child with a slope greater than zero
    if (fgreater(maxSlope, 0))
    {
        //each policy, sets a weight for the new region (if we must define a new one).
        if (openClosedList.OpenSize() != 0)
        {
            if (policy == kGreedy)
            {
                //new nodes will have the same priority as the node on top of open.
                //to place the new node on top of open, and move forward as fast as possible.
                SetNextPriority_v2(maxSlopeH, maxSlopeG, openClosedList.Lookup(openClosedList.Peek()).f);
            }
            else if (policy == kHalfEdgeDrop) {
                //reduced a small amount from the priority value of the node on top of open, to handle local minimas better.
                SetNextPriority_v2(maxSlopeH, maxSlopeG, openClosedList.Lookup(openClosedList.Peek()).f-edgeCosts[which]*(weight-1)/(2.0*weight));
            }
            else if (policy == kXDP)
            {
                //one of the baselines.
                float minWeight, maxWeight;
                GetNextWeightRange_v2(minWeight, maxWeight, maxSlope);

                float nextF = (maxSlopeG+(2*weight-1)*maxSlopeH+sqrt((maxSlopeG-maxSlopeH)*(maxSlopeG-maxSlopeH)+4*weight*maxSlopeG*maxSlopeH))/(2*weight);
                SetNextPriority_v2(maxSlopeH, maxSlopeG, nextF);

            }
            else if (policy == kXUP)
            {
                //one of the baselines.
                float minWeight, maxWeight;
                GetNextWeightRange_v2(minWeight, maxWeight, maxSlope);

                float nextF = (maxSlopeG+maxSlopeH+sqrt((maxSlopeG+maxSlopeH)*(maxSlopeG+maxSlopeH)+4*weight*(weight-1)*maxSlopeH*maxSlopeH))/(2*weight);
                SetNextPriority_v2(maxSlopeH, maxSlopeG, nextF);

            }
            else if (policy == MAP)
            {
                //assigns new weight, based on the number of node expansions of the previous regions.
                //assigns smaller weights, if the search is progressive and
                // most of the recent node expansions were in the most recent created region.
                
                //CHECK THE SLOPE OF THE EXPANDED NODE TO SEE WHICH REGION IT WAS FROM.
                //ADD ONE TO THE NUMBER OF THAT REGION.
                float minWeight, maxWeight;
                GetNextWeightRange_v2(minWeight, maxWeight, maxSlope);
                float angle = atan2f(maxSlopeG,maxSlopeH)/PID180;
                int rounded_angle = int(angle*(1/table_step));
                int next_angle = LookUpVector.size();
                
                if(next_angle>=3)
                {
                    double prev_angle = (next_angle-1)/(1/table_step);
                    double prev_slope = tan(prev_angle* PID180);

                    double second_prev_angle = (next_angle-2)/(1/table_step);
                    double second_prev_slope = tan(second_prev_angle* PID180);

                    double third_prev_angle = (next_angle-3)/(1/table_step);
                    double third_prev_slope = tan(third_prev_angle* PID180);

                    if(maxSlope <= third_prev_slope)
                        thirdLast += 1;
                    else if(maxSlope <= second_prev_slope)
                        secondLast += 1;
                    else if(maxSlope <= prev_slope)
                        firstLast += 1;
                }

                if(next_angle == 1 || fgreater(rounded_angle, next_angle-1)){
                    float wma_N;

                    float minWeight, maxWeight, midWeight, lowMidWeight, highMidWeight;
                    GetNextWeightRange_v2(minWeight, maxWeight, maxSlope);

                    midWeight = (maxWeight + minWeight)/2;
                    lowMidWeight = (midWeight + minWeight)/2;
                    highMidWeight = (maxWeight + midWeight)/2;

                    std::vector<uint64_t> tempRegionsVec;
                    tempRegionsVec.push_back(firstLast);
                    tempRegionsVec.push_back(secondLast);
                    tempRegionsVec.push_back(thirdLast);
                    std::sort(tempRegionsVec.begin(), tempRegionsVec.end());

                    float WMA = (3*firstLast + 2*secondLast + 1*thirdLast)/6;

                    float rangeTop = (3*tempRegionsVec[2] + 2*tempRegionsVec[1] + 1*tempRegionsVec[0])/6;
                    float rangeButtom = (1*tempRegionsVec[2] + 2*tempRegionsVec[1] + 3*tempRegionsVec[0])/6;

                    //// 0<=wma_N<=1
                    if(rangeTop - rangeButtom !=0)
                        wma_N = 1-(WMA - rangeButtom)/(rangeTop - rangeButtom);
                    else
                        wma_N = 0;

                    float TheNextWeight = lowMidWeight + (highMidWeight-lowMidWeight)*wma_N;
                    
                    SetNextWeight_v2(maxSlopeH, maxSlopeG, TheNextWeight);

                    thirdLast = secondLast;
                    secondLast = firstLast;
                    firstLast = 0;
                }
            }
            else if (policy == DWP)
            {
                //changes the next weight to maxweight, if the search has reached a weighted region.
                //otherwise slowly increases the weight as the search proceeds.
                float minWeight, maxWeight;
                GetNextWeightRange_v2(minWeight, maxWeight, maxSlope);
                float angle = atan2f(maxSlopeG,maxSlopeH)/PID180;
                int rounded_angle = int(angle*(1/table_step));
                int next_angle = LookUpVector.size();
                
                if(next_angle == 1 || fgreater(rounded_angle, next_angle-1)){
                    // std::cout<<"angle is greater: "<<angle<<"\n";
                    double edgeHvalue = theHeuristic->HCost(openClosedList.Lookup(nodeid).data, neighbors[which]);
                    double edgeGvalue = edgeCosts[which];
                    if(fless(edgeHvalue/edgeGvalue, 1)){
                        // std::cout<<"weight type 1\n";
                        SetNextWeight_v2(maxSlopeH, maxSlopeG, edgeCosts[which]);
                    }
                    else{
                        // std::cout<<"weight type 2\n";
                        SetNextWeight_v2(maxSlopeH, maxSlopeG, minWeight+(maxWeight-minWeight)*pow((angle/90), 3));
                    }
                }
            }
            else {// To Run BaseLines using DSDWA* for graphic
                // -> CL code, uses template Astar
                // last argument will be ignored
                SetNextPriority_v2(maxSlopeH, maxSlopeG, 0.01);
            }
        }
        else {
            // Expansion of the start state. open is empty.

            float minWeight, maxWeight;
            GetNextWeightRange_v2(minWeight, maxWeight, maxSlope);

            if (policy == MAP || policy == DWP)
                SetNextWeight_v2(maxSlopeH, maxSlopeG, weight);
            else
                SetNextPriority_v2(maxSlopeH, maxSlopeG, 0.01);
        }
    }
    
    // iterate again updating costs and writing out to memory
    for (size_t x = 0; x < neighbors.size(); x++)
    {
        nodesTouched++;
        //std::cout << "looking at child with hash : " << env->GetStateHash(neighbors[x]) << "and g-cost"<<openClosedList.Lookup(nodeid).g+edgeCosts[x]<<std::endl;
        if (theConstraint &&
            theConstraint->ShouldNotGenerate(start, openClosedList.Lookup(nodeid).data, neighbors[x],
                                             openClosedList.Lookup(nodeid).g+edgeCosts[x], goal))
            continue;

        switch (neighborLoc[x])
        {
            case kClosedList:
                // TODO: Can update parent pointers when shorter paths are found to improve solution quality
                if (reopenNodes)
                {
                    if (fless(openClosedList.Lookup(nodeid).g+edgeCosts[x], openClosedList.Lookup(neighborID[x]).g))
                    {
                        auto &i = openClosedList.Lookup(neighborID[x]);
                        i.parentID = nodeid;
                        i.g = openClosedList.Lookup(nodeid).g+edgeCosts[x];
                        i.f = Phi_v2(i.h, i.g);
                        openClosedList.Reopen(neighborID[x]);
                        // This line isn't normally needed, but in some state spaces we might have
                        // equality but different meta information, so we need to make sure that the
                        // meta information is also copied, since this is the most generic A* implementation
                        i.data = neighbors[x];
                    }
                }
                break;
            case kOpenList:
                if (fless(openClosedList.Lookup(nodeid).g+edgeCosts[x], openClosedList.Lookup(neighborID[x]).g))
                {
                    auto &i = openClosedList.Lookup(neighborID[x]);
                    i.parentID = nodeid;
                    i.g = openClosedList.Lookup(nodeid).g+edgeCosts[x];
                    i.f = Phi_v2(i.h, i.g);
                    // This line isn't normally needed, but in some state spaces we might have
                    // equality but different meta information, so we need to make sure that the
                    // meta information is also copied, since this is the most generic A* implementation
                    i.data = neighbors[x];
                    openClosedList.KeyChanged(neighborID[x]);
//                    std::cout << " Reducing cost to " << openClosedList.Lookup(nodeid).g+edgeCosts[x] << "\n";
                }
                else {
//                    std::cout << " no cheaper \n";
                }
                break;
            case kNotFound:
                {
                    double h = heuristicCosts[x];
                    openClosedList.AddOpenNode(neighbors[x],
                                               env->GetStateHash(neighbors[x]),
                                               Phi_v2(h, openClosedList.Lookup(nodeid).g+edgeCosts[x]),
                                               openClosedList.Lookup(nodeid).g+edgeCosts[x],
                                               h,
                                               nodeid);
                }
        }
    }

    return false;
}


/**
 * Returns the next state on the open list (but doesn't pop it off the queue).
 * @author Nathan Sturtevant
 * @date 03/22/06
 *
 * @return The first state in the open list.
 */
template <class state, class action, class environment, class openList>
state DSDWAStar<state, action,environment,openList>::CheckNextNode()
{
	uint64_t key = openClosedList.Peek();
	return openClosedList.Lookup(key).data;
	//assert(false);
	//return openQueue.top().currNode;
}

/**
 * Get the path from a goal state to the start state
 * @author Nathan Sturtevant
 * @date 03/22/06
 *
 * @param goalNode the goal state
 * @param thePath will contain the path from goalNode to the start state
 */
template <class state, class action,class environment,class openList>
void DSDWAStar<state, action,environment,openList>::ExtractPathToStartFromID(uint64_t node,
																	 std::vector<state> &thePath)
{
	do {
		thePath.push_back(openClosedList.Lookup(node).data);
		node = openClosedList.Lookup(node).parentID;
	} while (openClosedList.Lookup(node).parentID != node);
	thePath.push_back(openClosedList.Lookup(node).data);
}

template <class state, class action,class environment,class openList>
const state &DSDWAStar<state, action,environment,openList>::GetParent(const state &s)
{
	uint64_t theID;
	openClosedList.Lookup(env->GetStateHash(s), theID);
	theID = openClosedList.Lookup(theID).parentID;
	return openClosedList.Lookup(theID).data;
}

template <class state, class action, class environment, class openList>
uint64_t DSDWAStar<state, action,environment,openList>::GetNecessaryExpansions() const
{
	uint64_t n = 0;
	for (unsigned int x = 0; x < openClosedList.size(); x++)
	{
		const auto &data = openClosedList.Lookat(x);
		if (fless(data.g + data.h, goalFCost))
			n++;
	}
	return n;
}

/**
 * A function that prints the number of states in the closed list and open
 * queue.
 * @author Nathan Sturtevant
 * @date 03/22/06
 */
template <class state, class action, class environment, class openList>
void DSDWAStar<state, action,environment,openList>::PrintStats()
{
	printf("%u items in closed list\n", (unsigned int)openClosedList.ClosedSize());
	printf("%u items in open queue\n", (unsigned int)openClosedList.OpenSize());
}

/**
 * Return the amount of memory used by DSDWAStar
 * @author Nathan Sturtevant
 * @date 03/22/06
 *
 * @return The combined number of elements in the closed list and open queue
 */
template <class state, class action, class environment, class openList>
int DSDWAStar<state, action,environment,openList>::GetMemoryUsage()
{
	return openClosedList.size();
}

/**
 * Get state from the closed list
 * @author Nathan Sturtevant
 * @date 10/09/07
 *
 * @param val The state to lookup in the closed list
 * @gCost The g-cost of the node in the closed list
 * @return success Whether we found the value or not
 * the states
 */
template <class state, class action, class environment, class openList>
bool DSDWAStar<state, action,environment,openList>::GetClosedListGCost(const state &val, double &gCost) const
{
	uint64_t theID;
	dataLocation loc = openClosedList.Lookup(env->GetStateHash(val), theID);
	if (loc == kClosedList)
	{
		gCost = openClosedList.Lookat(theID).g;
		return true;
	}
	return false;
}

template <class state, class action, class environment, class openList>
bool DSDWAStar<state, action,environment,openList>::GetOpenListGCost(const state &val, double &gCost) const
{
	uint64_t theID;
	dataLocation loc = openClosedList.Lookup(env->GetStateHash(val), theID);
	if (loc == kOpenList)
	{
		gCost = openClosedList.Lookat(theID).g;
		return true;
	}
	return false;
}

template <class state, class action, class environment, class openList>
bool DSDWAStar<state, action,environment,openList>::GetHCost(const state &val, double &hCost) const
{
	uint64_t theID;
	dataLocation loc = openClosedList.Lookup(env->GetStateHash(val), theID);
	if (loc != kNotFound)
	{
		hCost = openClosedList.Lookat(theID).h;
		return true;
	}
	return false;
}

template <class state, class action, class environment, class openList>
bool DSDWAStar<state, action,environment,openList>::GetClosedItem(const state &s, AStarOpenClosedDataWithF<state> &result)
{
	uint64_t theID;
	dataLocation loc = openClosedList.Lookup(env->GetStateHash(s), theID);
	if (loc == kClosedList)
	{
		result = openClosedList.Lookat(theID);
		return true;
	}
	return false;

}


/**
 * Draw the open/closed list
 * @author Nathan Sturtevant
 * @date 03/12/09
 * Deprecated
 */
template <class state, class action, class environment, class openList>
void DSDWAStar<state, action,environment,openList>::OpenGLDraw() const
{} // deprecated

/**
 * Draw the open/closed list
 * @author Nathan Sturtevant
 * @date 7/12/16
 *
 */
template <class state, class action, class environment, class openList>
void DSDWAStar<state, action,environment,openList>::Draw(Graphics::Display &disp) const
{
	double transparency = 1.0;
	if (openClosedList.size() == 0)
		return;
	uint64_t top = -1;
	//	double minf = 1e9, maxf = 0;
	if (openClosedList.OpenSize() > 0)
	{
		top = openClosedList.Peek();
	}
	for (unsigned int x = 0; x < openClosedList.size(); x++)
	{
		const auto &data = openClosedList.Lookat(x);
		if (x == top)
		{
			env->SetColor(1.0, 1.0, 0.0, transparency);
			env->Draw(disp, data.data);
		}
		else if ((data.where == kOpenList) && (data.reopened))
		{
			env->SetColor(0.0, 0.5, 0.5, transparency);
			env->Draw(disp, data.data);
		}
		else if (data.where == kOpenList)
		{
			env->SetColor(0.0, 1.0, 0.0, transparency);
			env->Draw(disp, data.data);
		}
		else if ((data.where == kClosedList) && (data.reopened))
		{
			env->SetColor(0.5, 0.0, 0.5, transparency);
			env->Draw(disp, data.data);
		}
		else if (data.where == kClosedList)
		{
			//			if (top != -1)
			//			{
			//				env->SetColor((data.g+data.h-minf)/(maxf-minf), 0.0, 0.0, transparency);
			//			}
			//			else {
			if (data.parentID == x)
				env->SetColor(1.0, 0.5, 0.5, transparency);
			else
				env->SetColor(1.0, 0.0, 0.0, transparency);
			//			}
			env->Draw(disp, data.data);
		}
	}
	env->SetColor(1.0, 0.5, 1.0, 0.5);
	env->Draw(disp, goal);
}


template <class state, class action, class environment, class openList>
void DSDWAStar<state, action,environment,openList>::DrawPriorityGraph(Graphics::Display &display) const
{
	point3d origin(-1, 1);
	float priority = 1;
	float bound = weight;
	
	// draw bounding line
	point3d bl1(priority, 0), bl2(0, priority*bound); // main suboptimality line
	point3d bl3(priority*bound, 0);//(0+2, priority*bound-2); //
	point3d bl4(priority*bound/(2*bound-1), 0);//priority*bound-(2*bound-1));
	point3d bl2a(0, priority);
	point3d bl2c(priority-priority*bound/(2*bound-1), priority*bound);
	// WA* priority line
	display.DrawLine(LocalToHOG(bl1), LocalToHOG(bl2), 1/100.0f, Colors::yellow);
	// 45° upper bound line
	display.DrawLine(LocalToHOG(bl2), LocalToHOG(bl3), 1/100.0f, Colors::darkgray);
	// 2w-1 upper bound line
	display.DrawLine(LocalToHOG(bl1), LocalToHOG(bl2c), 1/100.0f, Colors::darkgray);

	// 45° lower bound line
	display.DrawLine(LocalToHOG(bl2a), LocalToHOG(bl1), 1/100.0f, Colors::lightgray);
	// 2w-1 lower bound line
	display.DrawLine(LocalToHOG(bl2), LocalToHOG(bl4), 1/100.0f, Colors::lightgray);
	
	
	// Draw actual priority line across
	for (int x = 0; x < data.size(); x++)
	{
		point3d value = origin;
		
		// y = slope * x // x=1 -> y = slope; y=1 -> x = 1/slope;
		if (isinf(data[x].slope))
		{
			value.x = -1;
			value.y = -1;
		}
		else if (data[x].slope < 1)
		{
			value.x += 2;
			value.y -= 2*data[x].slope;
		}
		else {
			value.x += 2/data[x].slope;
			value.y -= 2;
		}
		display.DrawLine(origin, value, 1/200.0f, Colors::blue);
		
		if (isinf(data[x].slope))
		{
			point3d crossPoint2;
			float lastSlope = ((x==0)?(0):(data[x-1].slope));
			crossPoint2.x = priority/(data[x].K*(lastSlope+data[x].weight));
			crossPoint2.y = crossPoint2.x*lastSlope;
			display.DrawLine(LocalToHOG({0, static_cast<float>(weight)}), LocalToHOG(crossPoint2), 1/100.0f, Colors::red);
		}
		else {
			point3d crossPoint1, crossPoint2;
			crossPoint1.x = priority/(data[x].K*(data[x].slope+data[x].weight));
			crossPoint1.y = crossPoint1.x*data[x].slope;
			float lastSlope = ((x==0)?(0):(data[x-1].slope));
			crossPoint2.x = priority/(data[x].K*(lastSlope+data[x].weight));
			crossPoint2.y = crossPoint2.x*lastSlope;
			display.DrawLine(LocalToHOG(crossPoint1), LocalToHOG(crossPoint2), 1/100.0f, Colors::red);
		}
	}
	for (int x = 0; x < data.size(); x++)
		display.FillCircle(LocalToHOG(data[x].crossPoint), 0.01, Colors::darkgreen);

	display.DrawLine(origin, {1, 1}, 1./100.0f, Colors::white);
	display.DrawLine(origin, {-1, -1}, 1./100.0f, Colors::white);
}

template <class state, class action, class environment, class openList>
void DSDWAStar<state, action,environment,openList>::DrawPriorityGraph_v2(Graphics::Display &display) const
{
    point3d origin(-1, 1);
    float priority = 1;
    float bound = weight;
    
    // draw bounding line
    point3d bl1(priority, 0), bl2(0, priority*bound); // main suboptimality line
    point3d bl3(priority*bound, 0);//(0+2, priority*bound-2); //
    point3d bl4(priority*bound/(2*bound-1), 0);//priority*bound-(2*bound-1));
    point3d bl2a(0, priority);
    point3d bl2c(priority-priority*bound/(2*bound-1), priority*bound);
    // WA* priority line
    display.DrawLine(LocalToHOG(bl1), LocalToHOG(bl2), 1/100.0f, Colors::yellow);
    // 45° upper bound line
    display.DrawLine(LocalToHOG(bl2), LocalToHOG(bl3), 1/100.0f, Colors::darkgray);
    // 2w-1 upper bound line
    display.DrawLine(LocalToHOG(bl1), LocalToHOG(bl2c), 1/100.0f, Colors::darkgray);

    // 45° lower bound line
    display.DrawLine(LocalToHOG(bl2a), LocalToHOG(bl1), 1/100.0f, Colors::lightgray);
    // 2w-1 lower bound line
    display.DrawLine(LocalToHOG(bl2), LocalToHOG(bl4), 1/100.0f, Colors::lightgray);
    
    
    // Draw actual priority line across
    for (int x = 1; x < LookUpVector.size(); x++)
    {
        point3d value = origin;
        double angle = x/(1/table_step);
        double prev_angle = (x-1)/(1/table_step);
        double slope = std::tan(angle* PID180);
        const DSDdata_v2& entry = LookUpVector[x];
        
        // y = slope * x // x=1 -> y = slope; y=1 -> x = 1/slope;
        if (std::isinf(slope))
        {
            value.x = -1;
            value.y = -1;
        }
        else if (slope < 1)
        {
            value.x += 2;
            value.y -= 2 * slope;
        }
        else {
            value.x += 2 / slope;
            value.y -= 2;
        }
        display.DrawLine(origin, value, 1 / 200.0f, Colors::blue);

        if (std::isinf(slope))
        {
            point3d crossPoint2;
            float lastSlope = (x == 1) ? 0 : std::tan(prev_angle* PID180);
            crossPoint2.x = priority / (entry.K * (lastSlope + entry.weight));
            crossPoint2.y = crossPoint2.x * lastSlope;
            display.DrawLine(LocalToHOG({0, static_cast<float>(weight)}), LocalToHOG(crossPoint2), 1 / 100.0f, Colors::red);
        }
        else {
            point3d crossPoint1, crossPoint2;
            crossPoint1.x = priority / (entry.K * (slope + entry.weight));
            crossPoint1.y = crossPoint1.x * slope;
            float lastSlope = (x == 1) ? 0 : std::tan(prev_angle* PID180);
            crossPoint2.x = priority / (entry.K * (lastSlope + entry.weight));
            crossPoint2.y = crossPoint2.x * lastSlope;
            display.DrawLine(LocalToHOG(crossPoint1), LocalToHOG(crossPoint2), 1 / 100.0f, Colors::red);
        }
    }
    
    for (int x = 1; x < LookUpVector.size(); x++)
        display.FillCircle(LocalToHOG(LookUpVector[x].crossPoint), 0.01, Colors::darkgreen);

    display.DrawLine(origin, {1, 1}, 1. / 100.0f, Colors::white);
    display.DrawLine(origin, {-1, -1}, 1. / 100.0f, Colors::white);
}

#endif /* DSDWAStar_h */
