/*
 *  RIBS.h
 *  hog2
 *
 *  Created by Nathan Sturtevant on 8/31/09.
 *  Copyright 2009 NS Software. All rights reserved.
 *
 */


#include "Unit.h"
#include <unordered_map>
#include <math.h>

namespace LocalSensing {

	static const bool verbose = false;
	
	template <class state, class action>
	struct stateInfo {
		stateInfo() { bestGCost = -1; lastFCost = -1; deadCell = false; superfluousCell = false; /*noParent = false;*/ }
		
		double bestGCost;
		double lastFCost; // could be reduced to 1 bit -- parity of round
		bool deadCell;
		bool superfluousCell;
//		bool noParent;
//		double fCost;
//		double exploreCost;
		action toParent;

		double hCost;
		
		// if we want full duplicate ancestor pruning
		std::vector<action> optimalParents;
		
		// just for drawing purposes, remove if memory constrained
		state theState;
	};
	
	template <class state, class action, class environment>
	class RIBS : public Unit<state, action, environment> {
	public:
		RIBS(state &loc, state &goal);
		
		const char *GetName() { sprintf(name, "RIBS(w=%1.2f)", searchWeight); return name;}
		bool MakeMove(environment *, OccupancyInterface<state,action> *, SimulationInfo<state,action,environment> *, action& a);
		void OpenGLDraw(const environment *e, const SimulationInfo<state,action,environment> *) const;
		
		void UpdateLocation(environment *, state &s, bool success, SimulationInfo<state,action,environment> *)
		{
			if (success) 
			{
				if (verbose) std::cout << "Updating location from " << currentLoc << " to " << s << std::endl;
				currentLoc = s;
			}
			else
				std::cout << "Move failed, not updating location" << std::endl;
		}
		void GetGoal(state &s) { s = goalLoc; }
		void GetLocation(state &s) { s = currentLoc; }
		bool Done() { return (currentLoc == goalLoc);} 
		
		void SetWeight(double w) { searchWeight = w; }
		/** log an stats that may have been computed during the last run */
		void LogStats(StatCollection *) {}
		/** log any final one-time stats before a simulation is ended */
		void LogFinalStats(StatCollection *s)
		{
			s->AddStat("nodesExpanded", GetName(), (long)nodesExpanded);
			s->AddStat("nodesTouched", GetName(), (long)nodesTouched);
		}
		void StartNewTrial(StatCollection *s)
		{
			s->AddStat("nodesExpanded", GetName(), (long)nodesExpanded);
			s->AddStat("nodesTouched", GetName(), (long)nodesTouched);
			nodesExpanded = 0;
			nodesTouched = 0;
//			stateInfo<state, action> &si = hashTable[env->GetStateHash(currentLoc)];
//			stateInfo<state, action> &start = hashTable[env->GetStateHash(startLoc)];
//			
//			searchWeight = si.bestGCost/start.hCost;
//			//searchWeight -= 0.1;
//			actionCount = 0;
//			printf("New trial; weight %f\n", searchWeight);
//			currentIterationCost = 0;
//			nextIterationCost = DBL_MAX;
		}

	private:
		void RemoveChildrenPointers(environment *e, state &currentLoc);
		bool AddOptimalParent(stateInfo<state, action> &info, action a);

		typedef std::unordered_map<uint64_t, stateInfo<state, action>, Hash64 > LSStateStorage;
		LSStateStorage hashTable;
		
		char name[255];
		unsigned long nodesExpanded, nodesTouched;
		
		bool init;
		bool returnToStart;
		state currentLoc, startLoc, goalLoc; // start loc is just for drawing purposes
		double currentIterationCost, currentPathCost;
		double nextIterationCost;
		double searchWeight;
		int actionCount;
		environment *env;
	};
	
	
	template <class state, class action, class environment>
	RIBS<state, action, environment>::RIBS(state &loc, state &goal)
	:currentLoc(loc), startLoc(loc), goalLoc(goal)
	{
		nodesExpanded = 0;
		nodesTouched = 0;
		init = false;
		returnToStart = false;
		searchWeight = 1.0f;
		actionCount = 0;
		env = 0;
	}
	
	template <class state, class action, class environment>
	bool RIBS<state, action, environment>::MakeMove(environment *e, OccupancyInterface<state,action> *,
																SimulationInfo<state,action,environment> *si, action& a)
	{
		env = e;
		if (currentLoc == goalLoc)
		{
			static bool once = false;
			if (!once) { /*printf("Finished after %d actions\n", actionCount);*/ once = true; }
			//searchWeight -= 0.5; // this will happen too much...
			return false;
		}
		actionCount++;
		if (0 == actionCount%1000) {
			//printf("%d actions ", actionCount);
			//std::cout << currentLoc << goalLoc << std::endl;
		}
		/////////////////
		//// run with weighted heuristic -- doesn't follow heuristic...
		/////////////////
		if (!init)
		{
			currentPathCost = 0.0;
			nextIterationCost = DBL_MAX;
			stateInfo<state, action> &si = hashTable[e->GetStateHash(currentLoc)];
			
			si.bestGCost = 0;
			si.lastFCost = searchWeight*e->HCost(currentLoc, goalLoc);
			si.hCost = e->HCost(currentLoc, goalLoc);
			si.theState = currentLoc;
			currentIterationCost = si.lastFCost;//ceil(si.lastFCost);
			init = true;
		}		
		
		if (verbose) std::cout << "\nInside RIBS::MakeMove; " << currentPathCost << " limit " << currentIterationCost << std::endl;
		stateInfo<state, action> &current = hashTable[e->GetStateHash(currentLoc)];
		if (verbose) std::cout << "MakingMove at node " << currentLoc << " with g-cost " << current.bestGCost <<
			" on iteration " << currentIterationCost << std::endl;

		// heuristic "learning"
//		current.hCost = std::max(current.hCost,
//								 (currentIterationCost-current.bestGCost)/searchWeight);
		
		nodesExpanded++;
		nodesTouched++;
		std::vector<state> neighbors;
		e->GetSuccessors(currentLoc, neighbors);		
		int bestNeighbor = -1;
		double bestFCost = DBL_MAX;
		double bestGCost = 0;
		
		// mark dead by default; clear if outoing node found
		current.deadCell = true;
		int eligibleChildren = 0;
		bool requiredNode = false;

		for (unsigned int x = 0; x < neighbors.size(); x++)
		{
			nodesTouched++;
			if (verbose) std::cout << "Successor " << x << ": " << neighbors[x] << std::endl;
			stateInfo<state, action> &next = hashTable[e->GetStateHash(neighbors[x])];

			if (next.deadCell) // completely skip dead cells
				continue;
			if (next.superfluousCell)
				continue;
			
			// haven't explored here before
			if (fequal(next.bestGCost, -1))
			{
				eligibleChildren++; // from before (working)
				current.deadCell = false;
				next.bestGCost = current.bestGCost+e->GCost(neighbors[x], currentLoc);
				//next.lastFCost = 0;//currentIterationCost;
				next.theState = neighbors[x];
				next.hCost = e->HCost(neighbors[x], goalLoc);
				if (verbose) std::cout << "Touching " << neighbors[x] << " for the first time with g-cost " << next.bestGCost << std::endl;
				next.toParent = e->GetAction(neighbors[x], currentLoc);
				requiredNode = AddOptimalParent(next, next.toParent)||requiredNode;
				assert(requiredNode == true);
			}
			// have been here with lower g-cost
			else if (fless(next.bestGCost, current.bestGCost+e->GCost(neighbors[x], currentLoc)))
			{
				if (verbose) printf("Been here before cheaper\n");
				// DualProp rule -- make this our new parent!
				if (fless(next.bestGCost+e->GCost(neighbors[x], currentLoc), current.bestGCost))
				{
					//printf("::::: Invoking DualProp rule\n");
					current.bestGCost = next.bestGCost+e->GCost(neighbors[x], currentLoc);
					current.optimalParents.resize(0);
					a = current.toParent;
					current.toParent = e->GetAction(currentLoc, neighbors[x]);
					requiredNode = AddOptimalParent(current, current.toParent);
					current.deadCell = false;
					return false;
					// should we go back to our old parent to maintain search tree?
					// or stay here with a new parent and re-explore?
				}
				continue;
			}
			// have been here with higher g-cost
			else if (fgreater(next.bestGCost, current.bestGCost+e->GCost(neighbors[x], currentLoc)))
			{
				eligibleChildren++; // from before (working)
				if (verbose) printf("Found faster route here!\n");
				next.bestGCost = current.bestGCost+e->GCost(neighbors[x], currentLoc);
				next.toParent = e->GetAction(neighbors[x], currentLoc);
				next.optimalParents.resize(0);
				requiredNode = AddOptimalParent(next, next.toParent)||requiredNode;
				current.deadCell = false;
				assert(requiredNode == true);
			}
			// have been here already this iteration (equal g-cost)
			else if (fequal(currentIterationCost, next.lastFCost))
			{
				// this was working before -- uncommenting to try to get a more complicated system working
//				if (next.toParent == e->GetAction(neighbors[x], currentLoc)) // another route to this node
				eligibleChildren++;

				assert(fequal(current.bestGCost+e->GCost(neighbors[x], currentLoc), next.bestGCost));
				
				requiredNode = AddOptimalParent(next, e->GetAction(neighbors[x], currentLoc))||requiredNode;

				current.deadCell = false;
				if (verbose) printf("Been here on this iteration!\n");
				continue;
			}
			// haven't visited this iteration(equal g-cost)
			else {
				current.deadCell = false;
				eligibleChildren++;
				bool onlyPath = AddOptimalParent(next, e->GetAction(neighbors[x], currentLoc));
				requiredNode = requiredNode||onlyPath;
				if (verbose) printf("Haven't visited on this iteration! (%s)\n", onlyPath?"only path":"multiple paths");
			}
			
//			if (!fgreater(current.bestGCost+e->GCost(neighbors[x], currentLoc), next.bestGCost))
//			{
//				current.deadCell = false;
//			}
			
			// check if we want to explore
			//double fCost = next.bestGCost+searchWeight*e->HCost(neighbors[x], goalLoc);
			double fCost = next.bestGCost+searchWeight*next.hCost;
			if (!fgreater(fCost, currentIterationCost))
			{
				// test this tie-breaking; unclear how much it helps
				if ((bestNeighbor == -1) || fless(fCost, bestFCost) ||
					(fequal(fCost, bestFCost) && (next.bestGCost >= bestGCost)))
				{
					bestNeighbor = x;
					bestFCost = fCost;
					bestGCost = next.bestGCost;
				}
			}
			else {
				nextIterationCost = min(nextIterationCost,
										next.bestGCost+searchWeight*next.hCost);//e->HCost(neighbors[x], goalLoc));
			}
		}
		if ((current.deadCell) && verbose)
			std::cout << current.theState << " has been shown to be dead" << std::endl;
		
//		if (eligibleChildren == 0)
//			current.superfluousCell = true;

		if ((!requiredNode)&&1)
		{
			//if ((eligibleChildren == 1)||(random()%2))
			{
				current.superfluousCell = true;
				RemoveChildrenPointers(e, currentLoc);
				a = current.toParent;
				return true;
			}
		}
		
		if (bestNeighbor != -1)
		{
			stateInfo<state, action> &next = hashTable[e->GetStateHash(neighbors[bestNeighbor])];

			next.lastFCost = currentIterationCost;
			
			// set up way to return to our parent
			a = e->GetAction(currentLoc, neighbors[bestNeighbor]);
			next.toParent = a;
			//next.noParent = false;
			e->InvertAction(next.toParent);
			
			return true;
		}
		
		// start next iteration
		if (fequal(current.bestGCost, 0))
		{
			if (verbose) std::cout << "Increasing iteration cost to " << nextIterationCost << std::endl;
			//currentIterationCost = nextIterationCost;
			currentIterationCost = nextIterationCost;//ceil(nextIterationCost);
			nextIterationCost = DBL_MAX;
			return false;
		}
		if (verbose) std::cout << " Returning to parent\n ";
		a = current.toParent;
		return true;
	}
	
	template <class state, class action, class environment>
	bool RIBS<state, action, environment>::AddOptimalParent(stateInfo<state, action> &info, action a)
	{
		for (unsigned int x = 0; x < info.optimalParents.size(); x++)
			if (info.optimalParents[x] == a)
				return (info.optimalParents.size()==1);
		info.optimalParents.push_back(a);
		return (info.optimalParents.size()==1);
	}

	template <class state, class action, class environment>
	void RIBS<state, action, environment>::RemoveChildrenPointers(environment *e, state &where)
	{
		std::vector<state> neighbors;
		e->GetSuccessors(where, neighbors);		
		for (unsigned int x = 0; x < neighbors.size(); x++)
		{
			stateInfo<state, action> &next = hashTable[e->GetStateHash(neighbors[x])];
			action a = e->GetAction(neighbors[x], where);
			for (unsigned int y = 0; y < next.optimalParents.size(); y++)
			{
				if (next.optimalParents[y] == a)
				{
					next.optimalParents[y] = next.optimalParents.back();
					next.optimalParents.pop_back();
					next.toParent = next.optimalParents[0];
					break;
				}
			}
		}
	}
	
	template <class state, class action, class environment>
	void RIBS<state, action, environment>::OpenGLDraw(const environment *e, const SimulationInfo<state,action,environment> *si) const
	{
		PublicUnitInfo<state, action, environment> i;
		si->GetPublicUnitInfo(si->GetCurrentUnit(), i);
		e->SetColor(1.0, 0.0, 0.0, 1.0);
		if (fgreater(si->GetSimulationTime(), i.nextTime))
			e->OpenGLDraw(i.currentState);
		else
			e->OpenGLDraw(i.lastState, i.currentState,
						  (si->GetSimulationTime()-i.lastTime)/(i.nextTime-i.lastTime));
		//e->OpenGLDraw(currentLoc);
		
		e->SetColor(0.0, 1.0, 0.0, 1.0);
		e->OpenGLDraw(goalLoc);
		e->SetColor(1.0, 1.0, 0.0, 1.0);
		e->OpenGLDraw(startLoc);
		
		e->SetColor(0.0, 0.0, 0.5, 0.25);
		for (typename LSStateStorage::const_iterator it = hashTable.begin(); it != hashTable.end(); it++)
		{
			if ((*it).second.deadCell)
				e->SetColor(0.5, 0.0, 0.0, 1.0);
			else if ((*it).second.superfluousCell)
				e->SetColor(1.0, 0.0, 1.0, 1.0);
			else
				e->SetColor(0.0, 0.0, 1.0, 0.25);
				
			if ((*it).second.theState == currentLoc)
			{
			}
			else {
				e->OpenGLDraw((*it).second.theState);
			}
		}
	}
	
}
