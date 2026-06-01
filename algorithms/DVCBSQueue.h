//
//  DVCBSQueue.h
//  hog2 mac native demos
//
//  Created by Shahaf S. Shperberg
//

#ifndef DVCBSQueue_h
#define DVCBSQueue_h

#include "DVCBSOpenClosed.h"
#include "DVCBSQueue.h"
#include <vector>
#include <climits>
#include <unordered_map>
#include <utility>
#include <algorithm>


template <typename state, int epsilon = 0, bool isAllSolutions = false>
class DVCBSQueue {
public:
	bool getVertexCover(std::vector<uint64_t> &nextForward, std::vector<uint64_t> &nextBackward,int TieBreakingPolicy)
	{
		
		while (true)
		{
			if (forwardQueue.OpenSize() == 0)
				return false;
			if (backwardQueue.OpenSize() == 0)
				return false;
			
			// move items with f<CLowerBound to ready
			
			while (forwardQueue.OpenWaitingSize() != 0 && ((!isAllSolutions && fless(forwardQueue.getFirstKey(kOpenWaiting),CLowerBound)) || (isAllSolutions && !fgreater(forwardQueue.getFirstKey(kOpenWaiting),CLowerBound))))
			{
				forwardQueue.PutToReady();
			}
			while (backwardQueue.OpenWaitingSize() != 0 && ((!isAllSolutions && fless(backwardQueue.getFirstKey(kOpenWaiting), CLowerBound)) || (isAllSolutions && !fgreater(backwardQueue.getFirstKey(kOpenWaiting), CLowerBound))))
			{
				backwardQueue.PutToReady();
			}
			
			if ((forwardQueue.OpenReadySize() != 0) && (backwardQueue.OpenReadySize() != 0) &&
				(!fgreater(forwardQueue.getFirstKey(kOpenReady) + backwardQueue.getFirstKey(kOpenReady) + epsilon, CLowerBound)))
			{
				std::vector<std::pair<double,uint64_t> > forwardCluster;
				std::vector<std::pair<double,uint64_t> > backwardCluster;
				for (auto it = forwardQueue.getNodesMapBegin(kOpenReady); it != forwardQueue.getNodesMapEnd(kOpenReady) && it->first <= CLowerBound-backwardQueue.getFirstKey(kOpenReady) - epsilon+TOLERANCE; it++){
					forwardCluster.push_back(std::make_pair(it->first,it->second.size()));
				}
				for (auto it = backwardQueue.getNodesMapBegin(kOpenReady); it != backwardQueue.getNodesMapEnd(kOpenReady) && it->first <= CLowerBound-forwardQueue.getFirstKey(kOpenReady) - epsilon+TOLERANCE; it++){
					backwardCluster.push_back(std::make_pair(it->first,it->second.size()));
				}
				int minJ = INT_MAX;
				int minI = INT_MAX;
				int minValue = INT_MAX;
				uint64_t NumForwardInVC = 0;
				uint64_t NumBackwardInVC = 0;
				std::vector<std::pair<int,int> > minimalVertexCovers;
				for (int i = -1; i < ((int)forwardCluster.size()); i++){
					if (i >= 0){
						NumForwardInVC += forwardCluster[i].second;
					}
					else{
						NumForwardInVC = 0;
					}
					bool skip = false;
					for (int j = -1; j < ((int)backwardCluster.size()) && !skip; j++){
						if (j >= 0){
							NumBackwardInVC += backwardCluster[j].second;
						}
						else{
							NumBackwardInVC = 0;
						}
						if (i == ((int)forwardCluster.size())-1){
							if (NumForwardInVC < minValue){
								minimalVertexCovers.clear();
							}
							if(NumForwardInVC <= minValue){
								minimalVertexCovers.push_back(std::make_pair(i,j));
								minValue = NumForwardInVC;
							}
							skip = true;
						}
						else if(j == ((int)backwardCluster.size())-1) {
							if (NumBackwardInVC < minValue){
								minimalVertexCovers.clear();
							}
							if(NumBackwardInVC <= minValue){
								minimalVertexCovers.push_back(std::make_pair(i,j));
								minValue = NumBackwardInVC;
							}
							skip = true;
						}
						else if(fgreater(backwardCluster[j+1].first+forwardCluster[i+1].first + epsilon, CLowerBound)){
							if (NumBackwardInVC+NumForwardInVC < minValue){
								minimalVertexCovers.clear();
							}
							if(NumBackwardInVC+NumForwardInVC <= minValue){
								minimalVertexCovers.push_back(std::make_pair(i,j));
								minValue = NumBackwardInVC+NumForwardInVC;
							}
							skip = true;
						}
					}
				}
				
				
				
				std::pair<int,int> chosenVC = computeTieBreaking(minimalVertexCovers,forwardCluster,backwardCluster,TieBreakingPolicy);
				
				for (int i = 0; i <= chosenVC.first;i++){
					auto v = forwardQueue.getNodesMapElements(kOpenReady,forwardCluster[i].first);
					nextForward.insert( nextForward.end(), v.begin(), v.end() );
				}
				for (int j = 0; j <= chosenVC.second;j++){
					auto v = backwardQueue.getNodesMapElements(kOpenReady,backwardCluster[j].first);
					nextBackward.insert( nextBackward.end(), v.begin(), v.end() );
				}
				
				
				return true;
			}
			else
			{
				bool changed = false;
				if (/*backwardQueue.OpenReadySize() == 0 && */backwardQueue.OpenWaitingSize() != 0)
				{
					const auto i4 = backwardQueue.getFirstKey(kOpenWaiting);
					if (!fgreater(i4, CLowerBound))
					{
						changed = true;
						backwardQueue.PutToReady();
					}
				}
				if (/*forwardQueue.OpenReadySize() == 0 && */forwardQueue.OpenWaitingSize() != 0)
				{
					const auto i3 = forwardQueue.getFirstKey(kOpenWaiting);
					if (!fgreater(i3, CLowerBound))
					{
						changed = true;
						forwardQueue.PutToReady();
					}
				}
				if (!changed){
					CLowerBound = DBL_MAX;
					if (forwardQueue.OpenWaitingSize() != 0)
					{
						const auto i5 = forwardQueue.getFirstKey(kOpenWaiting);
						CLowerBound = std::min(CLowerBound, i5);
					}
					if (backwardQueue.OpenWaitingSize() != 0)
					{
						const auto i6 = backwardQueue.getFirstKey(kOpenWaiting);
						CLowerBound = std::min(CLowerBound, i6);
					}
					if ((forwardQueue.OpenReadySize() != 0) && (backwardQueue.OpenReadySize() != 0))
						CLowerBound = std::min(CLowerBound, forwardQueue.getFirstKey(kOpenReady) + backwardQueue.getFirstKey(kOpenReady) + epsilon);
				}
				
				
			}
			
			
		}
		return false;
	}
	
	void Reset()
	{
		CLowerBound = 0;
		forwardQueue.Reset();
		backwardQueue.Reset();
	}
	double GetLowerBound() { return CLowerBound; }
	DVCBSOpenClosed<state> forwardQueue;
	DVCBSOpenClosed<state> backwardQueue;
private:
	double CLowerBound;
	bool tieBreakCriteria(int i,int j,int minI,int minJ,std::vector<std::pair<uint64_t,uint64_t> > forwardCluster, std::vector<std::pair<uint64_t,uint64_t> > backwardCluster){
		int iValue = 0;
		int jValue = 0;
		int minIValue = 0;
		int minJValue = 0;
		if (i > 0){
			iValue += forwardQueue.Lookup(forwardCluster[i].first).g;
		}
		if (minI > 0){
			minIValue += forwardQueue.Lookup(forwardCluster[minI].first).g;
		}
		if (j > 0){
			jValue += backwardQueue.Lookup(backwardCluster[j].first).g;
		}
		if (minJ > 0){
			minJValue += backwardQueue.Lookup(backwardCluster[minJ].first).g;
		}
		return (std::max(iValue,jValue) > std::max(minIValue,minJValue));
	}
	bool tieBreakCriteria(double currentSum,double minSum){
		return (currentSum > minSum);
	}
	
	std::pair<int,int> computeTieBreaking(std::vector<std::pair<int,int> >& minimalVertexCovers,std::vector<std::pair<double,uint64_t> >& forwardCluster, std::vector<std::pair<double,uint64_t> >& backwardCluster, int TieBreakingPolicy){
		switch(TieBreakingPolicy) {
			case 1 : return computeFullMaxGTieBreaking(minimalVertexCovers,forwardCluster,backwardCluster);
			case 2 : return computeSingleClusterMaxGTieBreaking(minimalVertexCovers,forwardCluster,backwardCluster);
			case 3 : return computeSingleClusterMinGTieBreaking(minimalVertexCovers,forwardCluster,backwardCluster);
			case 4 : return computeSingleClusterMinNodesTieBreaking(minimalVertexCovers,forwardCluster,backwardCluster);
			case 5 : return computeSingleClusterMaxNodesTieBreaking(minimalVertexCovers,forwardCluster,backwardCluster);
			case 6 : return computeFullMinGTieBreaking(minimalVertexCovers,forwardCluster,backwardCluster);
			case 7 : return computeMajorityMaxTieBreaking(minimalVertexCovers,forwardCluster,backwardCluster);
			case 8 : return computeMajorityMinTieBreaking(minimalVertexCovers,forwardCluster,backwardCluster);
			case 9 : return computeMajorityMinNodesTieBreaking(minimalVertexCovers,forwardCluster,backwardCluster);
			case 10: return computeSingleClusterMinNodesMaxGFTieBreaking(minimalVertexCovers,forwardCluster,backwardCluster);
			case 11 : return computeSingleClusterCardNoMVC(minimalVertexCovers,forwardCluster,backwardCluster);
			case 12: return computeSingleClusterMinGNoMVC(minimalVertexCovers,forwardCluster,backwardCluster);
			default: assert(false);
				return std::make_pair(-1,-1);
		}
		
	}
	
	std::pair<int,int> computeFullMaxGTieBreakingOld(std::vector<std::pair<int,int> >& minimalVertexCovers,std::vector<std::pair<double,uint64_t> >& forwardCluster, std::vector<std::pair<double,uint64_t> >& backwardCluster){
		
		int maxValue = -1;
		std::pair<int,int> maxPair;
		for(std::vector<std::pair<int,int> >::iterator it = minimalVertexCovers.begin(); it != minimalVertexCovers.end(); ++it) {
			int maxF = 0;
			int maxB = 0;
			if (it->first > 0){
				maxF = forwardCluster[it->first].first;
			}
			if (it->second > 0){
				maxB = backwardCluster[it->second].first;
			}
			if ((maxF > maxValue) || (maxB > maxValue)){
				maxPair = *it;
				maxValue = std::max(maxF,maxB);
			}
		}
		return maxPair;
	}
	
	std::pair<int,int> computeFullMaxGTieBreaking(std::vector<std::pair<int,int> >& minimalVertexCovers,std::vector<std::pair<double,uint64_t> >& forwardCluster, std::vector<std::pair<double,uint64_t> >& backwardCluster){
		
		int maxValue = -1;
		std::pair<int,int> maxPair;
		for(std::vector<std::pair<int,int> >::iterator it = minimalVertexCovers.begin(); it != minimalVertexCovers.end(); ++it) {
			int maxF = -1;
			int maxB = -1;
			if (it->first >= 0){
				maxF = forwardCluster[it->first].first;
			}
			if (it->second >= 0){
				maxB = backwardCluster[it->second].first;
			}
			if ((maxF > maxValue) || (maxB > maxValue)){
				maxPair = *it;
				maxValue = std::max(maxF,maxB);
			}
		}
		return maxPair;
	}
	
	std::pair<int,int> computeSingleClusterMaxGTieBreaking(std::vector<std::pair<int,int> >& minimalVertexCovers,std::vector<std::pair<double,uint64_t> >& forwardCluster, std::vector<std::pair<double,uint64_t> >& backwardCluster){
		int maxF = -1;
		int maxB = -1;
		std::pair<int,int> maxPair;
		for(std::vector<std::pair<int,int> >::iterator it = minimalVertexCovers.begin(); it != minimalVertexCovers.end(); ++it) {
			if (maxF > 0 && maxB > 0){
				break;
			}
			if (it->first >= 0){
				maxF = forwardCluster[0].first;
			}
			if (it->second >= 0){
				maxB = backwardCluster[0].first;
			}
		}
		if (maxF >= maxB){
			return std::make_pair(0,-1);
		}
		else{
			return std::make_pair(-1,0);
		}
		return maxPair;
	}
	
	std::pair<int,int> computeSingleClusterMinGTieBreaking(std::vector<std::pair<int,int> >& minimalVertexCovers,std::vector<std::pair<double,uint64_t> >& forwardCluster, std::vector<std::pair<double,uint64_t> >& backwardCluster){
		int maxF = INT_MAX;
		int maxB = INT_MAX;
		std::pair<int,int> maxPair;
		for(std::vector<std::pair<int,int> >::iterator it = minimalVertexCovers.begin(); it != minimalVertexCovers.end(); ++it) {
			if (maxF < INT_MAX && maxB < INT_MAX){
				break;
			}
			if (it->first >= 0){
				maxF = forwardCluster[0].first;
			}
			if (it->second >= 0){
				maxB = backwardCluster[0].first;
			}
		}
		if (maxF < maxB){
			return std::make_pair(0,-1);
		}
		else{
			return std::make_pair(-1,0);
		}
		return maxPair;
	}
	
	std::pair<int,int> computeSingleClusterMinNodesTieBreaking(std::vector<std::pair<int,int> >& minimalVertexCovers,std::vector<std::pair<double,uint64_t> >& forwardCluster, std::vector<std::pair<double,uint64_t> >& backwardCluster){
		int maxF = INT_MAX;
		int maxB = INT_MAX;
		std::pair<int,int> maxPair;
		for(std::vector<std::pair<int,int> >::iterator it = minimalVertexCovers.begin(); it != minimalVertexCovers.end(); ++it) {
			if (maxF < INT_MAX && maxB < INT_MAX){
				break;
			}
			if (it->first >= 0){
				maxF = forwardCluster[0].second;
			}
			if (it->second >= 0){
				maxB = backwardCluster[0].second;
			}
		}
		if (maxF < maxB){
			return std::make_pair(0,-1);
		}
		else{
			return std::make_pair(-1,0);
		}
		return maxPair;
	}
	
	std::pair<int,int> computeSingleClusterCardNoMVC(std::vector<std::pair<int,int> >& minimalVertexCovers,std::vector<std::pair<double,uint64_t> >& forwardCluster, std::vector<std::pair<double,uint64_t> >& backwardCluster){
		int maxF = forwardCluster[0].second;
		int maxB = backwardCluster[0].second;
		if (maxF < maxB){
			return std::make_pair(0,-1);
		}
		else{
			return std::make_pair(-1,0);
		}
	}
	
	std::pair<int,int> computeSingleClusterMinGNoMVC(std::vector<std::pair<int,int> >& minimalVertexCovers,std::vector<std::pair<double,uint64_t> >& forwardCluster, std::vector<std::pair<double,uint64_t> >& backwardCluster){
		double maxF = forwardCluster[0].first;
		double maxB = backwardCluster[0].first;
		if (maxF < maxB){
			return std::make_pair(0,-1);
		}
		else{
			return std::make_pair(-1,0);
		}
	}
	
	std::pair<int,int> computeSingleClusterMinNodesMaxGFTieBreaking(std::vector<std::pair<int,int> >& minimalVertexCovers,std::vector<std::pair<double,uint64_t> >& forwardCluster, std::vector<std::pair<double,uint64_t> >& backwardCluster){
		int maxF = INT_MAX;
		int maxB = INT_MAX;
		std::pair<int,int> maxPair;
		for(std::vector<std::pair<int,int> >::iterator it = minimalVertexCovers.begin(); it != minimalVertexCovers.end(); ++it) {
			if (maxF < INT_MAX && maxB < INT_MAX){
				break;
			}
			if (it->first >= 0){
				maxF = forwardCluster[0].second;
			}
			if (it->second >= 0){
				maxB = backwardCluster[0].second;
			}
		}
		if (maxF < maxB){
			return std::make_pair(0,-1);
		}
		else if (maxF > maxB){
			return std::make_pair(-1,0);
		}
		else{
			if (forwardCluster[0].first >= backwardCluster[0].first){
				return std::make_pair(0,-1);
			}
			else{
				return std::make_pair(-1,0);
			}
		}
		return maxPair;
	}
	
	std::pair<int,int> computeMajorityMinNodesTieBreaking(std::vector<std::pair<int,int> >& minimalVertexCovers,std::vector<std::pair<double,uint64_t> >& forwardCluster, std::vector<std::pair<double,uint64_t> >& backwardCluster){
		
		int forwardCount = 0;
		int backwardCount = 0;
		for(std::vector<std::pair<int,int> >::iterator it = minimalVertexCovers.begin(); it != minimalVertexCovers.end(); ++it) {
			if (it->first >= 0){
				forwardCount++;
			}
			if (it->second >= 0){
				backwardCount++;
			}
		}
		if (forwardCount > backwardCount){
			return std::make_pair(0,-1);
		}
		else if (forwardCount < backwardCount){
			return std::make_pair(-1,0);
		}
		else{
			int maxF = forwardCluster[0].second;
			int maxB = backwardCluster[0].second;
			if (maxF < maxB){
				return std::make_pair(0,-1);
			}
			else{
				return std::make_pair(-1,0);
			}
		}
	}
	
	std::pair<int,int> computeSingleClusterMaxNodesTieBreaking(std::vector<std::pair<int,int> >& minimalVertexCovers,std::vector<std::pair<double,uint64_t> >& forwardCluster, std::vector<std::pair<double,uint64_t> >& backwardCluster){
		int maxF = -1;
		int maxB = -1;
		std::pair<int,int> maxPair;
		for(std::vector<std::pair<int,int> >::iterator it = minimalVertexCovers.begin(); it != minimalVertexCovers.end(); ++it) {
			if (maxF > 0 && maxB > 0){
				break;
			}
			if (it->first >= 0){
				maxF = forwardCluster[0].second;
			}
			if (it->second >= 0){
				maxB = backwardCluster[0].second;
			}
		}
		if (maxF >= maxB){
			return std::make_pair(0,-1);
		}
		else{
			return std::make_pair(-1,0);
		}
		return maxPair;
	}
	
	std::pair<int,int> computeSingleClusterMinGTieBreakingWithSub(std::vector<std::pair<int,int> >& minimalVertexCovers,std::vector<std::pair<double,uint64_t> >& forwardCluster, std::vector<std::pair<double,uint64_t> >& backwardCluster){
		int maxF = INT_MAX;
		int maxB = INT_MAX;
		int minOpenForward = 0;
		int minOpenBackward = 0;
		if (forwardCluster.size() > 0){
			minOpenForward = forwardCluster[0].first;
		}
		if (backwardCluster.size() > 0){
			minOpenBackward = backwardCluster[0].first;
		}
		std::pair<int,int> maxPair;
		for(std::vector<std::pair<int,int> >::iterator it = minimalVertexCovers.begin(); it != minimalVertexCovers.end(); ++it) {
			if (maxF < INT_MAX && maxB < INT_MAX){
				break;
			}
			if (it->first >= 0){
				maxF = forwardCluster[0].first;
			}
			if (it->second >= 0){
				maxB = backwardCluster[0].first;
			}
		}
		if (maxF - minOpenBackward < maxB - minOpenForward){
			return std::make_pair(0,-1);
		}
		else{
			return std::make_pair(-1,0);
		}
		return maxPair;
	}
	
	std::pair<int,int> computeSingleClusterMaxGTieBreakingWithSub(std::vector<std::pair<int,int> >& minimalVertexCovers,std::vector<std::pair<double,uint64_t> >& forwardCluster, std::vector<std::pair<double,uint64_t> >& backwardCluster){
		int maxF = -1;
		int maxB = -1;
		int minOpenForward = 0;
		int minOpenBackward = 0;
		if (forwardCluster.size() > 0){
			minOpenForward = forwardCluster[0].first;
		}
		if (backwardCluster.size() > 0){
			minOpenBackward = backwardCluster[0].first;
		}
		std::pair<int,int> maxPair;
		for(std::vector<std::pair<int,int> >::iterator it = minimalVertexCovers.begin(); it != minimalVertexCovers.end(); ++it) {
			if (maxF > 0 && maxB > 0){
				break;
			}
			if (it->first >= 0){
				maxF = forwardCluster[0].first;
			}
			if (it->second >= 0){
				maxB = backwardCluster[0].first;
			}
		}
		if (maxF - minOpenBackward >= maxB - minOpenForward){
			return std::make_pair(0,-1);
		}
		else{
			return std::make_pair(-1,0);
		}
		return maxPair;
	}
	
	std::pair<int,int> computeFullMinGTieBreaking(std::vector<std::pair<int,int> >& minimalVertexCovers,std::vector<std::pair<double,uint64_t> >& forwardCluster, std::vector<std::pair<double,uint64_t> >& backwardCluster){
		
		int minValue = INT_MAX;
		std::pair<int,int> minPair;
		for(std::vector<std::pair<int,int> >::iterator it = minimalVertexCovers.begin(); it != minimalVertexCovers.end(); ++it) {
			int maxF = INT_MAX;
			int maxB = INT_MAX;
			if (it->first >= 0){
				maxF = forwardCluster[it->first].first;
			}
			if (it->second >= 0){
				maxB = backwardCluster[it->second].first;
			}
			if ((maxF < minValue) || (maxB < minValue)){
				minPair = *it;
				minValue = std::min(maxF,maxB);
			}
		}
		return minPair;
	}
	
	
	std::pair<int,int> computeMinGTieBreakingWithSub(std::vector<std::pair<int,int> >& minimalVertexCovers,std::vector<std::pair<double,uint64_t> >& forwardCluster, std::vector<std::pair<double,uint64_t> >& backwardCluster){
		int minValue = INT_MAX;
		int minOpenForward = 0;
		int minOpenBackward = 0;
		if (forwardCluster.size() > 0){
			minOpenForward = forwardCluster[0].first;
		}
		if (backwardCluster.size() > 0){
			minOpenBackward = backwardCluster[0].first;
		}
		std::pair<int,int> minPair;
		for(std::vector<std::pair<int,int> >::iterator it = minimalVertexCovers.begin(); it != minimalVertexCovers.end(); ++it) {
			int maxF = INT_MAX;
			int maxB = INT_MAX;
			if (it->first >= 0){
				maxF = forwardCluster[it->first].first;
			}
			if (it->second >= 0){
				maxB = backwardCluster[it->second].first;
			}
			if ((maxF - minOpenBackward < minValue) || (maxB - minOpenForward < minValue)){
				minPair = *it;
				minValue = std::min(maxF- minOpenBackward,maxB - minOpenForward);
			}
		}
		return minPair;
	}
	
	std::pair<int,int> computeMajorityMaxTieBreaking(std::vector<std::pair<int,int> >& minimalVertexCovers,std::vector<std::pair<double,uint64_t> >& forwardCluster, std::vector<std::pair<double,uint64_t> >& backwardCluster){
		
		int forwardCount = 0;
		int backwardCount = 0;
		for(std::vector<std::pair<int,int> >::iterator it = minimalVertexCovers.begin(); it != minimalVertexCovers.end(); ++it) {
			if (it->first >= 0){
				forwardCount++;
			}
			if (it->second >= 0){
				backwardCount++;
			}
		}
		if (forwardCount > backwardCount){
			return std::make_pair(0,-1);
		}
		else if (forwardCount < backwardCount){
			return std::make_pair(-1,0);
		}
		else{
			
			int fVal = forwardCluster[0].first;
			int bVal = backwardCluster[0].first;
			if (fVal >= bVal){
				return std::make_pair(0,-1);
			}
			else{
				return std::make_pair(-1,0);
			}
		}
	}
	
	
	std::pair<int,int> computeMajorityMinTieBreaking(std::vector<std::pair<int,int> >& minimalVertexCovers,std::vector<std::pair<double,uint64_t> >& forwardCluster, std::vector<std::pair<double,uint64_t> >& backwardCluster){
		
		int forwardCount = 0;
		int backwardCount = 0;
		for(std::vector<std::pair<int,int> >::iterator it = minimalVertexCovers.begin(); it != minimalVertexCovers.end(); ++it) {
			if (it->first >= 0){
				forwardCount++;
			}
			if (it->second >= 0){
				backwardCount++;
			}
		}
		if (forwardCount > backwardCount){
			return std::make_pair(0,-1);
		}
		else if (forwardCount < backwardCount){
			return std::make_pair(-1,0);
		}
		else{
			
			int fVal = forwardCluster[0].first;
			int bVal = backwardCluster[0].first;
			if (fVal < bVal){
				return std::make_pair(0,-1);
			}
			else{
				return std::make_pair(-1,0);
			}
		}
	}
	
	std::pair<int,int> computeMajorityMaxWithSubTieBreaking(std::vector<std::pair<int,int> >& minimalVertexCovers,std::vector<std::pair<double,uint64_t> >& forwardCluster, std::vector<std::pair<double,uint64_t> >& backwardCluster){
		
		int forwardCount = 0;
		int backwardCount = 0;
		int minOpenForward = 0;
		int minOpenBackward = 0;
		if (forwardCluster.size() > 0){
			minOpenForward = forwardCluster[0].first;
		}
		if (backwardCluster.size() > 0){
			minOpenBackward = backwardCluster[0].first;
		}
		for(std::vector<std::pair<int,int> >::iterator it = minimalVertexCovers.begin(); it != minimalVertexCovers.end(); ++it) {
			if (it->first >= 0){
				forwardCount++;
			}
			if (it->second >= 0){
				backwardCount++;
			}
		}
		if (forwardCount > backwardCount){
			return std::make_pair(0,-1);
		}
		else if (forwardCount < backwardCount){
			return std::make_pair(-1,0);
		}
		else{
			
			int fVal = forwardCluster[0].first - minOpenBackward;
			int bVal = backwardCluster[0].first - minOpenForward;
			if (fVal >= bVal){
				return std::make_pair(0,-1);
			}
			else{
				return std::make_pair(-1,0);
			}
		}
	}
	
	std::pair<int,int> computeMajorityMinWithSubTieBreaking(std::vector<std::pair<int,int> >& minimalVertexCovers,std::vector<std::pair<double,uint64_t> >& forwardCluster, std::vector<std::pair<double,uint64_t> >& backwardCluster){
		
		int forwardCount = 0;
		int backwardCount = 0;
		int minOpenForward = 0;
		int minOpenBackward = 0;
		if (forwardCluster.size() > 0){
			minOpenForward = forwardCluster[0].first;
		}
		if (backwardCluster.size() > 0){
			minOpenBackward = backwardCluster[0].first;
		}
		for(std::vector<std::pair<int,int> >::iterator it = minimalVertexCovers.begin(); it != minimalVertexCovers.end(); ++it) {
			if (it->first >= 0){
				forwardCount++;
			}
			if (it->second >= 0){
				backwardCount++;
			}
		}
		if (forwardCount > backwardCount){
			return std::make_pair(0,-1);
		}
		else if (forwardCount < backwardCount){
			return std::make_pair(-1,0);
		}
		else{
			
			int fVal = forwardCluster[0].first - minOpenBackward;
			int bVal = backwardCluster[0].first - minOpenForward;
			if (fVal < bVal){
				return std::make_pair(0,-1);
			}
			else{
				return std::make_pair(-1,0);
			}
		}
	}
	
};

#endif /* DVCBSQueue_h */
