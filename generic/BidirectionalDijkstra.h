//
//  BidirectionalDijkstra.h
//  hog2 glut
//
//  Created by Sneha Sawlani on 2/27/16.
//  Copyright (c) 2016 University of Denver. All rights reserved.
//

#ifndef hog2_glut_BidirectionalDijkstra_h
#define hog2_glut_BidirectionalDijkstra_h

#include "AStarOpenClosed.h"
#include "FPUtil.h"

template <class state>
struct BDCompare{
    bool operator() (const AStarOpenClosedData<state> &i1,const AStarOpenClosedData<state> &i2) const
    {
        return fgreater(i1.g,i2.g);    //low priority over high
    }
    
};

template <class state, class action, class environment, class priorityQueue = AStarOpenClosed<state,BDCompare<state>> >
class BIdijkstra{
    public:
    BIdijkstra() {  env = 0; ResetNodeCount(); countRegions = false; }
    virtual ~BIdijkstra() {}
    void GetPath(environment *env, const state& from, const state& to,std::vector<state> &thePath);
    bool InitializeSearch(environment *env, const state& from, const state& to,std::vector<state> &thePath);
    bool DoSingleSearchStep(std::vector<state> &thePath);
    void ExtractPath(uint64_t node1,uint64_t node2,std::vector<state> &thePath);
    void DoRegionAnalysis(environment* env, const state& from, const state& to, double optimalPathCost, uint64_t optimalPathLength);
    inline void SetVersion(int v) { version = v;}
    inline int GetVersion(){ return version; }
    inline void SetEpsilon(double minedge){
        epsilon=minedge;
    }
    inline double GetEpsilon(){
       return epsilon;
    }
    
    priorityQueue forwardQueue, backwardQueue;
    state goal, start;
    
    
    virtual const char *GetName() { return "Bidirectional Dijkstra"; }
    
    void ResetNodeCount() { nodesExpanded = nodesTouched = 0; }
    
    inline const int GetNumForwardItems() { return forwardQueue.size(); }
    
    inline const int GetNumBackwardItems() { return backwardQueue.size(); }
    
    inline const AStarOpenClosedData<state> &GetForwardItem(unsigned int which) { return forwardQueue.Lookat(which); }
    
    inline const AStarOpenClosedData<state> &GetBackwardItem(unsigned int which) { return backwardQueue.Lookat(which); }
    
    uint64_t GetNodesExpanded() const { return nodesExpanded; }
    uint64_t GetNodesTouched() const { return nodesTouched; }
    
    inline uint64_t GetNodesExpanded_NN_PathCost() const { return nodesExpanded_NN_PathCost; }
    inline uint64_t GetNodesExpanded_NF_PathCost() const { return nodesExpanded_NF_PathCost; }
    inline uint64_t GetNodesExpanded_FN_PathCost() const { return nodesExpanded_FN_PathCost; }
    inline uint64_t GetNodesExpanded_FF_PathCost() const { return nodesExpanded_FF_PathCost; }
    inline uint64_t GetNodesExpanded_RN_PathCost() const { return nodesExpanded_RN_PathCost; }
    inline uint64_t GetNodesExpanded_RF_PathCost() const { return nodesExpanded_RF_PathCost; }
    
    inline uint64_t GetNodesExpanded_NN_PathLength() const { return nodesExpanded_NN_PathLength; }
    inline uint64_t GetNodesExpanded_NF_PathLength() const { return nodesExpanded_NF_PathLength; }
    inline uint64_t GetNodesExpanded_FN_PathLength() const { return nodesExpanded_FN_PathLength; }
    inline uint64_t GetNodesExpanded_FF_PathLength() const { return nodesExpanded_FF_PathLength; }
    inline uint64_t GetNodesExpanded_RN_PathLength() const { return nodesExpanded_RN_PathLength; }
    inline uint64_t GetNodesExpanded_RF_PathLength() const { return nodesExpanded_RF_PathLength; }
    
    double GetPathCost() const { return optimalPathCost; } // path cost is the sum of all edges; path length is the number of edges
    uint64_t GetPathLength() const { return optimalPathLength; }
	state GetMeetingPoint() const { return forwardQueue.Lookat(middleNode).data; }
//    void OpenGLDraw() const;

private:
//    void OpenGLDraw(const priorityQueue &queue) const;
    
    void Expand(priorityQueue &current,priorityQueue &opposite,const state &target);
    
    uint64_t nodesTouched, nodesExpanded;
    
    uint64_t middleNode;
    uint64_t middleNode2;
    double currentCost;
    std::vector<state> neighbors;
    environment *env;
    double epsilon;
    int turn;
    int version;
    bool countRegions;
    
    uint64_t nodesExpanded_NF_PathCost, nodesExpanded_NN_PathCost, nodesExpanded_RF_PathCost, nodesExpanded_FN_PathCost, nodesExpanded_FF_PathCost, nodesExpanded_RN_PathCost;
    
    uint64_t nodesExpanded_NF_PathLength, nodesExpanded_NN_PathLength, nodesExpanded_RF_PathLength, nodesExpanded_FN_PathLength, nodesExpanded_FF_PathLength, nodesExpanded_RN_PathLength;
    
    double optimalPathCost;
    double optimalPathLength;
    bool forwardDirection;
};

template <class state,class action,class environment,class priorityQueue>
void BIdijkstra<state,action,environment,priorityQueue>::GetPath(environment* env,const state& from,const state& to, std::vector<state> &thePath)
{
    
            if(InitializeSearch(env,from,to,thePath)==false)
                return;
                              
            while(!DoSingleSearchStep(thePath))
            {}
}
    
template<class state,class action,class environment,class priorityQueue>
bool BIdijkstra<state,action,environment,priorityQueue>::InitializeSearch(environment *env,const state& from,const state& to,std::vector<state> &thePath)
{
    this->env=env;
    currentCost=DBL_MAX;
    forwardQueue.Reset();
    backwardQueue.Reset();
    ResetNodeCount();
    thePath.resize(0);
    start=from;
    goal=to;
    
    turn = 0;
                              
    if(start==goal)
        return false;
                              
    forwardQueue.AddOpenNode(start,env->GetStateHash(start),0,0);
    backwardQueue.AddOpenNode(goal,env->GetStateHash(goal),0,0);
                              
    return true;
}
    
template<class state,class action,class environment,class priorityQueue>
bool BIdijkstra<state,action,environment,priorityQueue>::DoSingleSearchStep(std::vector<state> &thePath)
{
        if(forwardQueue.OpenSize()==0 && backwardQueue.OpenSize()==0)
        {
            thePath.resize(0);
            return true;
        }
        if(forwardQueue.OpenSize()==0)
        {
            Expand(backwardQueue,forwardQueue,start);
        }
        else if(backwardQueue.OpenSize()==0)
        {
            Expand(forwardQueue,backwardQueue,goal);
        }
        else {
            uint64_t forward = forwardQueue.Peek();
            uint64_t backward = backwardQueue.Peek();
            
            const AStarOpenClosedData<state> &nextForward = forwardQueue.Lookat(forward);
            const AStarOpenClosedData<state> &nextBackward = backwardQueue.Lookat(backward);
            
            double p1=nextForward.g;
            double p2=nextBackward.g;
            
            switch(version)
            {
                case 1:
                    if(fless(p1,p2))
                        forwardDirection = true;
                    else
                        forwardDirection = false;
                    break;
                
                case 2:
                    if(turn == 0)
                    {
                        turn = 1;
                        forwardDirection = true;
                    }
                    else
                    {
                        turn = 0;
                        forwardDirection = false;
                    }
                    break;
                case 3:
                    if(forwardQueue.size() <= backwardQueue.size())
                    {
                        forwardDirection = true;
                    }
                    else
                    {
                        forwardDirection = false;
                    }
            }
            
            if(forwardDirection)
            {
                Expand(forwardQueue,backwardQueue,goal);
            }
            else
            {
                Expand(backwardQueue, forwardQueue, start);
            }
            
        }
    
        //check if we can terminate
        
        if(fless(currentCost,DBL_MAX))
        {
            uint64_t fminID = forwardQueue.Peek();
            uint64_t bminID = backwardQueue.Peek();
            
            const AStarOpenClosedData<state> &fmin = forwardQueue.Lookat(fminID);
            const AStarOpenClosedData<state> &bmin = backwardQueue.Lookat(bminID);
            
            if (!fless((fmin.g+bmin.g+epsilon), currentCost))
            {
                optimalPathCost = currentCost;
                ExtractPath(middleNode,middleNode2,thePath);
                return true;
            }
        }
    
        return false;
            
        
}

template<class state,class action,class environment,class priorityQueue>
void BIdijkstra<state,action,environment,priorityQueue>::ExtractPath(uint64_t node1,uint64_t node2,std::vector<state>& thePath)
{
    while(forwardQueue.Lookup(node1).parentID != node1)
    {
        thePath.insert(thePath.begin(), forwardQueue.Lookup(node1).data);
        node1 = forwardQueue.Lookup(node1).parentID;
    }

    thePath.insert(thePath.begin(), forwardQueue.Lookup(node1).data);
    
    while(backwardQueue.Lookup(node2).parentID != node2)
    {
        thePath.push_back(backwardQueue.Lookup(node2).data);
        node2 = backwardQueue.Lookup(node2).parentID;
    }
    
    thePath.push_back(backwardQueue.Lookup(node2).data);
}

template<class state,class action,class environment,class priorityQueue>
void BIdijkstra<state,action,environment,priorityQueue>::Expand(priorityQueue &current,priorityQueue &opposite,const state &target)
{
        uint64_t nextID=current.Close();
    
//        if(countRegions)
//        {
//            double costFromStart;
//            double costFromGoal;
//            
//            uint64_t lengthFromStart;
//            uint64_t lengthFromGoal;
//            
//            BIdijkstra<state, action, environment> bidijkstra;
//            
//            bidijkstra.SetEpsilon(epsilon);
//            bidijkstra.SetVersion(3);
//            
//            std::vector<state> path;
//            
//            if(forwardDirection)
//            {
//                costFromStart = current.Lookup(nextID).g;
//                lengthFromStart = current.Lookup(nextID).gLength;
//                
//                bidijkstra.GetPath(env, current.Lookup(nextID).data, goal, path);
//                
//                costFromGoal = bidijkstra.GetPathCost();
//                lengthFromGoal = path.size();
//            }
//            else
//            {
//                costFromGoal = current.Lookup(nextID).g;
//                lengthFromGoal = current.Lookup(nextID).gLength;
//                
//                bidijkstra.GetPath(env, start, current.Lookup(nextID).data,path);
//                
//                costFromStart = bidijkstra.GetPathCost();
//                lengthFromStart = path.size();
//                
//            }
//            
//            if(flessOrEqual(costFromStart, optimalPathCost/2) && flessOrEqual(costFromGoal, optimalPathCost/2))
//                nodesExpanded_NN_PathCost++;
//            
//            else if(flessOrEqual(costFromStart, optimalPathCost/2) && fgreater(costFromGoal, optimalPathCost/2))
//                nodesExpanded_NF_PathCost++;
//            
//            else if(flessOrEqual(costFromStart, optimalPathCost) && flessOrEqual(costFromGoal, optimalPathCost/2))
//                nodesExpanded_FN_PathCost++;
//            
//            else if(flessOrEqual(costFromStart, optimalPathCost) && fgreater(costFromGoal, optimalPathCost/2))
//                nodesExpanded_FF_PathCost++;
//            
//            else if(flessOrEqual(costFromGoal, optimalPathCost/2))
//                nodesExpanded_RN_PathCost++;
//            
//            else
//                nodesExpanded_RF_PathCost++;
//            
//            
//            if(flessOrEqual(lengthFromStart, optimalPathLength/2) && flessOrEqual(lengthFromGoal, optimalPathLength/2))
//                nodesExpanded_NN_PathLength++;
//            
//            else if(flessOrEqual(lengthFromStart, optimalPathLength/2) && fgreater(lengthFromGoal, optimalPathLength/2))
//                nodesExpanded_NF_PathLength++;
//            
//            else if(flessOrEqual(lengthFromStart, optimalPathLength) && flessOrEqual(lengthFromGoal, optimalPathLength/2))
//                nodesExpanded_FN_PathLength++;
//            
//            else if(flessOrEqual(lengthFromStart, optimalPathLength) && fgreater(lengthFromGoal, optimalPathLength/2))
//                nodesExpanded_FF_PathLength++;
//            
//            else if(flessOrEqual(lengthFromGoal, optimalPathLength/2))
//                nodesExpanded_RN_PathLength++;
//            
//            else
//                nodesExpanded_RF_PathLength++;
//            
//            
//        }
			
        nodesExpanded++;
        
        env->GetSuccessors(current.Lookup(nextID).data,neighbors);
        
        for(auto &succ : neighbors)
        {
            nodesTouched++;
            uint64_t childID;
            
            
            auto loc = current.Lookup(env->GetStateHash(succ), childID);
            
            switch(loc)
            {
                case kClosedList: //ignore
                    break;
                case kOpenList:  //update cost if needed
                {
                    double edgeCost=env->GCost(current.Lookup(nextID).data,succ);
                    
                    if(fless(current.Lookup(nextID).g+edgeCost,current.Lookup(childID).g))
                    {
                        current.Lookup(childID).parentID=nextID;
                        current.Lookup(childID).g=current.Lookup(nextID).g+edgeCost;
                        current.KeyChanged(childID);
//                        current.Lookup(childID).gLength = current.Lookup(nextID).gLength + 1;
                        
                        //TODO: check if we improved the current solution?
                        
                        uint64_t reverseLoc;
                        
                        auto loc=opposite.Lookup(env->GetStateHash(succ),reverseLoc);
                        
                        if(loc==kOpenList)
                        {
                            if(fless(current.Lookup(nextID).g+edgeCost+opposite.Lookup(reverseLoc).g,currentCost))
                            {
                                //TODO: store current solution
                                //printf("Potential updated solution found,cost: %1.2f + %1.2f = %1.2f\n",current.Lookup(nextID).g+edgeCost,opposite.Lookup(reverseLoc).g,current.Lookup(nextID).g+edgeCost+opposite.Lookup(reverseLoc).g);
                                
                                currentCost = current.Lookup(nextID).g+edgeCost + opposite.Lookup(reverseLoc).g;
                                
                                if(forwardDirection)
                                {
                                    middleNode = nextID;
                                    middleNode2 = reverseLoc;
                                }
                                else
                                {
                                    middleNode = reverseLoc;
                                    middleNode2 = nextID;
                                }
                                

                            }
                        }
                    }
                }
                    break;
                case kNotFound:
                {
                    double edgeCost=env->GCost(current.Lookup(nextID).data,succ);
                    
                    current.AddOpenNode(succ,env->GetStateHash(succ),current.Lookup(nextID).g+edgeCost,0,nextID);
                    
                    //check for solution
                    
                    uint64_t reverseLoc;
                    auto loc = opposite.Lookup(env->GetStateHash(succ), reverseLoc);
                    
                    if (loc == kOpenList)
                    {
                        if (fless(current.Lookup(nextID).g+edgeCost + opposite.Lookup(reverseLoc).g, currentCost))
                        {
                            // TODO: store current solution
                            /*printf("Potential solution found, cost: %1.2f + %1.2f = %1.2f\n",
                                   current.Lookup(nextID).g+edgeCost,
                                   opposite.Lookup(reverseLoc).g,
                                   current.Lookup(nextID).g+edgeCost+opposite.Lookup(reverseLoc).g);*/
                            currentCost = current.Lookup(nextID).g+edgeCost + opposite.Lookup(reverseLoc).g;
                            
                            if(forwardDirection)
                            {
                                middleNode = nextID;
                                middleNode2 = reverseLoc;
                            }
                            else
                            {
                                middleNode = reverseLoc;
                                middleNode2 = nextID;
                            }}
                    }

                }
            }
        }
}
    

//template<class state,class action,class environment,class priorityQueue>
//void BIdijkstra<state,action,environment,priorityQueue>::OpenGLDraw() const
//{
//    OpenGLDraw(forwardQueue);
//    OpenGLDraw(backwardQueue);
//}
//
//    template<class state,class action,class environment,class priorityQueue>
//    void BIdijkstra<state,action,environment,priorityQueue>::OpenGLDraw(const priorityQueue &queue) const
//    {
//        double transparency=0.9;
//        if(queue.size()==0)
//            return;
//        uint64_t top=-1;
//        
//        if (queue.OpenSize() > 0)
//        {
//            top = queue.Peek();
//        }
//        for (unsigned int x = 0; x < queue.size(); x++)
//        {
//            const AStarOpenClosedData<state> &data = queue.Lookat(x);
//            if (x == top)
//            {
//                env->SetColor(1.0, 1.0, 0.0, transparency);
//                env->OpenGLDraw(data.data);
//            }
//            if ((data.where == kOpenList) && (data.reopened))
//            {
//                env->SetColor(0.0, 0.5, 0.5, transparency);
//                env->OpenGLDraw(data.data);
//            }
//            else if (data.where == kOpenList)
//            {
//                env->SetColor(0.0, 1.0, 0.0, transparency);
//                env->OpenGLDraw(data.data);
//            }
//            else if ((data.where == kClosedList) && (data.reopened))
//            {
//                env->SetColor(0.5, 0.0, 0.5, transparency);
//                env->OpenGLDraw(data.data);
//            }
//            else if (data.where == kClosedList)
//            {
//                env->SetColor(1.0, 0.0, 0.0, transparency);
//                env->OpenGLDraw(data.data);
//            }
//        }
//
//    }

template<class state, class action, class environment, class priorityQueue>
void BIdijkstra<state, action, environment, priorityQueue>::DoRegionAnalysis(environment* env, const state& from, const state& to, double optimalPathCost, uint64_t optimalPathLength)
{
    countRegions = true;
    
    nodesExpanded_NN_PathCost = nodesExpanded_NF_PathCost = nodesExpanded_FN_PathCost = nodesExpanded_FF_PathCost = nodesExpanded_RN_PathCost = nodesExpanded_RF_PathCost = 0;
    
    nodesExpanded_NN_PathLength = nodesExpanded_NF_PathLength = nodesExpanded_FN_PathLength = nodesExpanded_FF_PathLength = nodesExpanded_RN_PathLength = nodesExpanded_RF_PathLength = 0;
    
    this->optimalPathCost = optimalPathCost;
    this->optimalPathLength = optimalPathLength;
    
    std::vector<state> thePath;
    
    GetPath(env, from, to, thePath);
    
    countRegions = false;
}



    
    


#endif
