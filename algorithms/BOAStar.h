//
//  BOAStar.h
//  HOG2 ObjC
//
//  Created by Nathan Sturtevant on 2/16/23.
//  Copyright © 2023 MovingAI. All rights reserved.
//
// Bi-objective A* By Ulloa et, 2020

#include "TemplateAStar.h"

#ifndef BOAStar_h
#define BOAStar_h

template <class state, class action, class environment>
class BOAStar //: public GenericSearchAlgorithm<state,action,environment>
{
public:
	BOAStar() { ResetNodeCount(); };
	virtual ~BOAStar() {}
	void GetPath(environment *env1, environment *env2, Heuristic<state> *h1, Heuristic<state> *h2,
				 const state& from, const state& to, std::vector<state> &thePath);
//	void GetPath(environment *, const state& , const state& , std::vector<action> & );
	
	state goal, start;
	
	bool InitializeSearch(environment *env1, environment *env2, Heuristic<state> *h1, Heuristic<state> *h2,
						  const state& from, const state& to, std::vector<state> &thePath);
	bool DoSingleSearchStep(std::vector<state> &thePath);
	
	void ExtractPathToStart(state &node, std::vector<state> &thePath);
	void ExtractPathToStartFromID(size_t node, std::vector<state> &thePath);
	virtual const char *GetName() { return "BOAStar"; }
	
	uint64_t GetUniqueNodesExpanded() { return uniqueNodesExpanded; }
	void ResetNodeCount() { nodesExpanded = nodesTouched = 0; uniqueNodesExpanded = 0; }
//	int GetMemoryUsage();
//
	size_t GetNumItems() const { return allStates.size(); }
	bool IsOpen(size_t item) const { return allStates[item].open; }
	state GetItem(size_t item) const { return allStates[item].s; }
	float GetItemGCost(size_t item) const { return allStates[item].gCost; }
	float GetItemHCost(size_t item) const { return allStates[item].hCost; }
	
//	void SetHeuristic(Heuristic<state> *h) { theHeuristic = h; }
	
	uint64_t GetNodesExpanded() const { return nodesExpanded; }
	uint64_t GetNodesTouched() const { return nodesTouched; }
	
	void LogFinalStats(StatCollection *) {}
//
	void Draw(Graphics::Display &d) const;
	void DrawFrontier(Graphics::Display &d, int selected = -1) const;
	void DrawAllPaths(Graphics::Display &d) const;
	void DrawGoal(Graphics::Display &d, int which, rgbColor c = Colors::blue, float width = 1.0) const;
	int GetClosestGoal(Graphics::point p, float limit);
	void SetPhi(std::function<double(double, double)> p)
	{
		phi = p;
	}
	double Phi(double h, double g) const
	{
		return phi(h, g);
	}
	void SetOptimalityBound(double w)
	{
		phi = [w](double h, double g){ return g/w+h; };
		bound = w;
	}
	double GetOptimalityBound() { return bound; }
private:
	void GetMinFOnOpen() const;
	size_t GetBestStateOnOpen() const;

	void DrawOpen(Graphics::Display &d) const;
	void Expand(size_t which);
	void UpdateCost(size_t next, size_t parent);
	void AddState(const state &s, size_t parent);
	size_t Lookup(const state &s);
	uint64_t nodesTouched, nodesExpanded;
	
	struct stateInfo {
		stateInfo() {gmin = std::numeric_limits<float>::max();}
		bool Update(float v) {if (fless(v, gmin)) {gmin = std::min(gmin, v); return true; } return false; }
		float gmin;
	};
	struct item {
		state s;
		float gCost1, gCost2;
		float hCost1, hCost2;
		float fCost1() const { return gCost1+hCost1; }
		float fCost2() const { return gCost2+hCost2; }
		bool open;
		size_t parent;
	};
	// stores gmin for all states seen thus far
	std::unordered_map<uint64_t, stateInfo> hashTable;
	std::vector<item> allStates;
	std::vector<state> neighbors;
//	std::vector<item> closed;
//	std::vector<uint64_t> neighborID;
//	std::vector<double> edgeCosts;
//	std::vector<dataLocation> neighborLoc;

	std::vector<item> goals;
	std::vector<int> openDrawing;
	mutable std::vector<Graphics::point> goalDrawLocs;
	std::vector<state> solution;
	environment *e1, *e2;
	Heuristic<state> *h1, *h2;
	//	double bestSolution;
	double bound;
	uint64_t uniqueNodesExpanded;
	//Heuristic<state> *theHeuristic;

	std::function<double(double, double)> phi;
};

template <class state, class action, class environment>
void BOAStar<state, action, environment>::GetPath(environment *env1, environment *env2, Heuristic<state> *h1, Heuristic<state> *h2,
												 const state& from, const state& to, std::vector<state> &thePath)
{
	if (!InitializeSearch(env1, env2, h1, h2, from, to, thePath))
	{
		return;
	}
	while (!DoSingleSearchStep(thePath))
	{
	}
}

template <class state, class action, class environment>
bool BOAStar<state, action, environment>::InitializeSearch(environment *env1, environment *env2,
														  Heuristic<state> *h1, Heuristic<state> *h2,
														  const state& from, const state& to, std::vector<state> &thePath)
{
	hashTable.clear();
	thePath.resize(0);
	e1 = env1;
	e2 = env2;
	allStates.resize(0);
	solution.clear();
	ResetNodeCount();
	start = from;
	goal = to;
	goals.resize(0);
	this->h1 = h1;
	this->h2 = h2;
	allStates.push_back({start, 0, 0,
		(float)h1->HCost(start, goal),
		(float)h2->HCost(start, goal),
		true, 0});
	// so we only draw each location once, even if it is in open many times
	// keeping track of how many are in open
	// unseen states are -1
	openDrawing.resize(e1->GetMaxHash());
	std::fill(openDrawing.begin(), openDrawing.end(), -1);
	openDrawing[e1->GetStateHash(start)]+=2;
	
	return true; // success
}

template <class state, class action, class environment>
bool BOAStar<state, action, environment>::DoSingleSearchStep(std::vector<state> &thePath)
{
	// 1. Find min lexicographical state on open
	size_t which = GetBestStateOnOpen();

	if (which == allStates.size()) // search is done
	{
		std::cout << "Search done; " << GetNodesExpanded() << " expanded;" << goals.size() << " paths found\n";
		return true;
	}
	
	// 2. Prune this state if it is dominated
	allStates[which].open = false;
	openDrawing[e1->GetStateHash(allStates[which].s)]--;
	
//	printf("Goal gmin: %f\n", hashTable[e1->GetStateHash(goal)].gmin);
	
	// Didn't decrease g2; since f1 is increasing, node is dominated
	//if (i.gCost2 >= hashTable[e1->GetStateHash(neighbors[x])].gmin)
	if (allStates[which].gCost2 >= hashTable[e1->GetStateHash(allStates[which].s)].gmin)
	{
//		std::cout << "dominated1\n";
		return false;
	}

//	printf("Goal gmin: %f\n", hashTable[e1->GetStateHash(goal)].gmin);
	// Can't reach the goal on a non-dominated path
	if (allStates[which].fCost2() >= hashTable[e1->GetStateHash(goal)].gmin)
	{
//		std::cout << "dominated2\n";
		return false;
	}

	hashTable[e1->GetStateHash(allStates[which].s)].gmin = allStates[which].gCost2;

	// 3. Check for goal
	if (e1->GoalTest(allStates[which].s, goal))
	{
		std::cout << "Adding goal " << allStates[which].s << " with costs {" << to_string_with_precision(allStates[which].gCost1, 10) << ", " << allStates[which].gCost2 << "}\n";
		goals.push_back(allStates[which]);
		assert(allStates[which].open == false);
		//openDrawing[e1->GetStateHash(allStates[which].s)]--;
	}
	
	// 4. Expand state
	Expand(which);

	return false;
}

template <class state, class action, class environment>
size_t BOAStar<state, action, environment>::GetBestStateOnOpen() const
{
	bool found = false;
	size_t which;
	for (size_t x = 0; x < allStates.size(); x++)
	{
		if (allStates[x].open)
		{
			found = true;
			which = x;
			break;
		}
	}
	if (!found)
		return allStates.size();
	
	for (size_t x = which+1; x < allStates.size(); x++)
	{
		if (allStates[x].open &&
			(fless(allStates[x].fCost1(), allStates[which].fCost1()) ||
			 (fequal(allStates[x].fCost1(), allStates[which].fCost1()) &&
			  fless(allStates[x].fCost2(), allStates[which].fCost2()))
			  )
			)
		{
			which = x;
		}
	}
//	std::cout << "x" << which << "-" << allStates[which].s << " [" << goal << "]";
//	printf(" Next expansions: f1: %1.2f, f2: %1.2f ", allStates[which].fCost1(), allStates[which].fCost2());
//	std::cout << "{parent: " << allStates[allStates[which].parent].s << "|\n";
	return which;
}

template <class state, class action, class environment>
void BOAStar<state, action, environment>::Expand(size_t which)
{
	nodesExpanded++;
	e1->GetSuccessors(allStates[which].s, neighbors);
	for (int x = 0; x < neighbors.size(); x++)
	{
//		std::cout << "g" << neighbors[x];
		item i =
		{neighbors[x],
			static_cast<float>(allStates[which].gCost1+e1->GCost(allStates[which].s, neighbors[x])),
			static_cast<float>(allStates[which].gCost2+e2->GCost(allStates[which].s, neighbors[x])),
			static_cast<float>(h1->HCost(neighbors[x], goal)),
			static_cast<float>(h2->HCost(neighbors[x], goal)),
			true,
			which
		};
		// Check if state is dominated
		// Already have shorter gmin for neighbor
		if (i.gCost2 >= hashTable[e1->GetStateHash(neighbors[x])].gmin)
		{
//			std::cout << "prune\n";
			continue;
		}

		// Can't reach the goal on a non-dominated path
		if (i.fCost2() >= hashTable[e1->GetStateHash(goal)].gmin)
		{
//			std::cout << "prune\n";
			continue;
		}

		allStates.push_back(i);
		auto hash = e1->GetStateHash(i.s);
		openDrawing[hash]++;
		if (openDrawing[hash] == 0)
			openDrawing[hash]++;
//		std::cout << "add " << allStates.size()-1 << "\n";
	}
}

template <class state, class action, class environment>
void BOAStar<state, action, environment>::Draw(Graphics::Display &d) const
{
	int maxVal = 0;
	for (int x = 0; x < e1->GetMaxHash(); x++)
		maxVal = std::max(openDrawing[x], maxVal);
	for (int x = 0; x < e1->GetMaxHash(); x++)
	{
		if (openDrawing[x] == -1)
			continue;
		state s;
		e1->GetStateFromHash(x, s);
		if (openDrawing[x] > 0) // open
		{
			rgbColor c = Colors::green;
			c *= openDrawing[x]/(float)maxVal;
			e1->SetColor(c);
			e1->Draw(d, s);
		}
		else {
			e1->SetColor(Colors::red);
			e1->Draw(d, s);
		}
	}
//	for (size_t x = 0; x < allStates.size(); x++)
//	{
//		if (allStates[x].open)
//		{
//			e1->SetColor(Colors::green);
//			e1->Draw(d, allStates[x].s);
//		}
//		else {
//			e1->SetColor(Colors::red);
//			e1->Draw(d, allStates[x].s);
//		}
//	}
	e1->SetColor(Colors::blue);
	e1->Draw(d, goal);
}

template <class state, class action, class environment>
void BOAStar<state, action, environment>::DrawFrontier(Graphics::Display &d, int selected) const
{
	static std::string s;
	s = std::to_string(goals.size())+" goals";
	d.DrawText(s.c_str(), {1, -1}, Colors::white, 0.1f, Graphics::textAlignRight, Graphics::textBaselineTop);

	if (selected != -1)
	{
		static std::string s;
		s = "Path cost {"+to_string_with_precision(goals[selected].gCost1, 2)+", "+to_string_with_precision(goals[selected].gCost2, 2)+"}";
		d.DrawText(s.c_str(), {1, -1+0.11}, Colors::white, 0.1f, Graphics::textAlignRight, Graphics::textBaselineTop);
	}

	goalDrawLocs.resize(0);
	float minf1 = (float)h1->HCost(start, goal);
	float maxf1 = (float)h1->HCost(start, goal);
	float minf2 = (float)h2->HCost(start, goal);
	float maxf2 = (float)h2->HCost(start, goal);
	for (int x = 0; x < goals.size(); x++)
	{
		minf1 = std::min(minf1, goals[x].fCost1());
		maxf1 = std::max(maxf1, goals[x].fCost1());
		minf2 = std::min(minf2, goals[x].fCost2());
		maxf2 = std::max(maxf2, goals[x].fCost2());
	}
	// axes
	d.DrawLine({-1, 1}, {-1, -1}, 0.0125, Colors::white);
	d.DrawText("Objective 1", {0, 1+0.02f}, Colors::white, 0.05, Graphics::textAlignCenter, Graphics::textBaselineTop);
	d.DrawLine({-1, 1}, {1, 1}, 0.0125, Colors::white);
	const char o2[] = "O\0b\0j\0e\0c\0t\0i\0v\0e\0 \0002\0";
	for (int y = 0; y < 11; y++)
	{
		Graphics::point p(-1-0.025, 0-5.5*0.05+0.05f*y);
		d.DrawText(&o2[2*y], p, Colors::white, 0.05, Graphics::textAlignCenter, Graphics::textBaselineMiddle);
	}
	// x-labels
	d.DrawText(to_string_with_precision(minf1, 1).c_str(), {-1, 1+.01}, Colors::white, 0.05, Graphics::textAlignLeft, Graphics::textBaselineTop);
	d.DrawText(to_string_with_precision(maxf1, 1).c_str(), {1, 1+.01}, Colors::white, 0.05, Graphics::textAlignRight, Graphics::textBaselineTop);
	// y-labels
	d.DrawText(to_string_with_precision(minf2, 1).c_str(), {-1-.01, 1}, Colors::white, 0.05, Graphics::textAlignRight, Graphics::textBaselineBottom);
	d.DrawText(to_string_with_precision(maxf2, 1).c_str(), {-1-.01, -1}, Colors::white, 0.05, Graphics::textAlignRight, Graphics::textBaselineTop);

	for (int x = 0; x < goals.size(); x++)
	{
		float denom1 = 1;
		if (!fequal(maxf1, minf1))
			denom1 = maxf1-minf1;
		float denom2 = 1;
		if (!fequal(maxf2, minf2))
			denom2 = maxf2-minf2;
		Graphics::point p(-1+2*(goals[x].fCost1()-minf1)/(denom1), 1-2*((goals[x].fCost2()-minf2)/(denom2)));
		if (x != selected)
			d.FillCircle(p, 0.025, Colors::lightblue);
		else
			d.FillCircle(p, 0.025*1.5, Colors::lightblue);
		goalDrawLocs.emplace_back(p);
//		maxf1 = std::max(maxf1, goals[x].fCost1());
//		maxf2 = std::max(maxf2, goals[x].fCost2());
	}

	//	for (size_t x = 0; x < allStates.size(); x++)
//	{
//		if (allStates[x].open)
//		{
//			e1->SetColor(Colors::green);
//			e1->Draw(d, allStates[x].s);
//		}
//		else {
//			e1->SetColor(Colors::red);
//			e1->Draw(d, allStates[x].s);
//		}
//	}
//	e1->SetColor(Colors::blue);
//	e1->Draw(d, goal);
}

template <class state, class action, class environment>
void BOAStar<state, action, environment>::DrawAllPaths(Graphics::Display &d) const
{
	e1->SetColor(Colors::blue);

	for (int x = 0; x < goals.size(); x++)
	{
		int current = goals[x].parent;
		e1->DrawLine(d, goals[x].s, allStates[current].s, 4);
		while (current != 0)
		{
			e1->DrawLine(d, allStates[current].s, allStates[allStates[current].parent].s, 4);
			current = allStates[current].parent;
		}
	}
}

template <class state, class action, class environment>
void BOAStar<state, action, environment>::DrawGoal(Graphics::Display &d, int which, rgbColor c, float width) const
{
	int x = which;
	e1->SetColor(c);

	{
		int current = goals[x].parent;
		e1->DrawLine(d, goals[x].s, allStates[current].s);
		while (current != 0)
		{
			e1->DrawLine(d, allStates[current].s, allStates[allStates[current].parent].s, width);
			current = allStates[current].parent;
		}
	}
}

template <class state, class action, class environment>
int BOAStar<state, action, environment>::GetClosestGoal(Graphics::point p, float limit)
{
	if (goals.size() < 1)
		return -1;
	int which = 0;
	float dist = (goalDrawLocs[0]-p).squaredLength();
	for (int x = 1; x < goals.size(); x++)
	{
		float d = (goalDrawLocs[x]-p).squaredLength();
		if (fless(d, dist))
		{
			dist = d;
			which = x;
		}
	}
	if (fless(dist, limit*limit))
		return which;
	return -1;
}

#endif /* BOAStar_h */
