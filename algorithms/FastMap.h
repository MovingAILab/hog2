//
//  GridHeuristics.hpp
//  HOG2 ObjC
//
//  Created by Nathan Sturtevant on 2/6/23.
//  Copyright © 2023 MovingAI. All rights reserved.
//

#include <stdio.h>
#include "Heuristic.h"
#include "Map2DEnvironment.h"
#include "TemplateAStar.h"
#include <cassert>

enum tEmbeddingFunction {
	kDifferential,
	kFastMap,
	kFMUniform,
	kFMShrink80
};

enum tPlacementScheme {
	kFurthest, // Baseline for both DH and FastMap
	kHeuristicError,
	kCustom,
};

enum tMetric {
	kL1, // additive
	kLINF, // max
};


template<class baseEnv, class state>
class EmbeddingEnvironment : public baseEnv {
public:
	EmbeddingEnvironment(Heuristic<state> *h)
	:h(h) {}
	double GCost(const state &node1, const state &node2) const
	{
		//assert(fequal(h->HCost(node1, node2), h->HCost(node2, node1)));
		double g = baseEnv::GCost(node1, node2) - h->HCost(node1, node2);
		// Rounding issues can lead g to be very close to 0, but outside tolerances
		return std::max(g, 0.0);
	}
	double HCost(const state &node1, const state &node2) const { return 0; }
private:
	Heuristic<state> *h;
};





template<class state, class environment, class action>
class GridEmbedding : public Heuristic<state>
{
public:
	GridEmbedding(environment *e, int numDimensions, tMetric m);
	bool AddDimension(tEmbeddingFunction e, tPlacementScheme p, Heuristic<state> *h = 0);
	bool AddDimension(tEmbeddingFunction e, state p1, state p2, Heuristic<state> *h = 0);
//	bool AddDimension(state l);
	double HCost(const state &a, const state &b) const;
	point3d Lookup(const state &l) const;
	std::vector<state> GetPivots()
	{
		std::vector<state> res;
		res.clear();
		for (auto v : pivots)
		{
			for (auto item : v)
			{
				res.push_back(item);
			}
		}
		return res;
	}
	//	void DrawAlternate(Graphics::Display &disp, const state &l) const;
private:
	void GetConnectedComponents();
	void SelectPivots(tPlacementScheme p, int component, Heuristic<state> *h);
	void SelectCustomPivots(int component, state p1, state p2);
	void Embed(tEmbeddingFunction e, int component);
	void GetDimensionLimits();
	state GetRandomState(int component);
	state GetFurthest(state l, TemplateAStar<state, action, environment> &astar, environment *env);
	state GetFurthest(std::vector<state> &l, TemplateAStar<state, action, environment> &astar, environment *env);
	std::vector<double> embedding;
	environment *env;
	tMetric metric;
	int currDim, maxDim;
	std::vector<std::vector<state>> pivots;
	std::vector<std::pair<float, float>> scale;
	EmbeddingEnvironment<environment, state> residual;
	TemplateAStar<state, action, environment> s1, s2;
	std::vector<state> path;
	std::vector<uint8_t> connectedComponents;
	int numConnectedComponents;
};

template<class state, class environment, class action>
GridEmbedding<state, environment, action>::GridEmbedding(environment *e, int numDimensions, tMetric m)
:env(e), residual(this), metric(m)
{
	currDim = 0;
	maxDim = numDimensions;
	embedding.resize(env->GetMaxHash()*maxDim);
	std::fill(embedding.begin(), embedding.end(), -1);
	GetConnectedComponents();
	pivots.resize(numConnectedComponents);
}

template<class state, class environment, class action>
bool GridEmbedding<state, environment, action>::AddDimension(tEmbeddingFunction e, tPlacementScheme p, Heuristic<state> *h)
{
	if (currDim == maxDim)
		return false;
	srandom(20230208);

	for (int component = 0; component < numConnectedComponents; component++)
	{
		SelectPivots(p, component, h==0?env:h);
		Embed(e, component);
	}
	GetDimensionLimits();
	currDim++;

	return true;
}

template<class state, class environment, class action>
bool GridEmbedding<state, environment, action>::AddDimension(tEmbeddingFunction e, state p1, state p2, Heuristic<state> *h)
{
	if (currDim == maxDim)
		return false;
	srandom(20230208);

	for (int component = 0; component < numConnectedComponents; component++)
	{
		SelectCustomPivots(component, p1, p2);
		Embed(e, component);
	}
	GetDimensionLimits();
	currDim++;

	return true;
}

///**
// * Add a dimension of the embedding using \input l as the pivot
// * Assumes the kDifferential embedding function
// * Also assumes a single connected component
// * (Note: Add a 2-parameter embedding if other methods need to be supported)
// */
//bool GridEmbedding::AddDimension(state l)
//{
//	if (currDim == maxDim)
//		return false;
//	pivots.push_back(l);
//	// Now get distances from the pivot
//	GetFurthest(l, s1, env);
//	Embed(kDifferential);
//	return true;
//}
template<class state, class environment, class action>
void GridEmbedding<state, environment, action>::SelectPivots(tPlacementScheme p, int component, Heuristic<state> *h)
{
	if (metric == kL1)
	{
		switch (p)
		{
			case kFurthest:
			{
				// Get random state
				state rand = GetRandomState(component);
				// Get furthest point (p1)
				state p1 = GetFurthest(rand, s2, &residual);
				
				// Get furthest from that point (p1) storing distances in s1
				state p2 = GetFurthest(p1, s1, &residual);

				// Get distances for second pivot in s2
				GetFurthest(p2, s2, &residual);
				pivots[component].push_back(p1);
				pivots[component].push_back(p2);
				break;
			}
			case kHeuristicError:
			{
				// Get random state
				state rand = GetRandomState(component);
				// Get furthest point (p1)
				state p1 = GetFurthest(rand, s2, &residual);

				double maxValue = 0;
				// Now update to furthest by error
				// (Could be done more efficiently during the search)
				for (int x = 0; x < env->GetMaxHash(); x++)
				{
					if (connectedComponents[x] != component)
						continue;
					state candidate;
					env->GetStateFromHash(x, candidate);
					double g;
					s2.GetClosedListGCost(candidate, g);

					// Reza's code -- bHE = 2
					//(gc + bHE*(gc-hCost)>max)
					if (fgreater(3*g - 2*h->HCost(rand, candidate),  maxValue))
					{
						maxValue = 3*g - 2*h->HCost(rand, candidate);
						// new p1 pivot
						p1 = candidate;
					}
				}
				
				// Get furthest from that point (p1) storing distances in s1
				state p2 = GetFurthest(p1, s1, &residual);

				maxValue = 0;
				// Now update to furthest by error
				for (int x = 0; x < env->GetMaxHash(); x++)
				{
					if (connectedComponents[x] != component)
						continue;
					state candidate;
					env->GetStateFromHash(x, candidate);
					double g;
					s1.GetClosedListGCost(candidate, g);

					// Reza's code -- bHE = 2
					//(gc + bHE*(gc-hCost)>max)
					if (fgreater(3*g - 2*h->HCost(p1, candidate),  maxValue))
					{
						maxValue = 3*g - 2*h->HCost(p1, candidate);
						// new p1 pivot
						p2 = candidate;
					}
				}
				
				// Get distances for second pivot in s2
				GetFurthest(p2, s2, &residual);
				pivots[component].push_back(p1);
				pivots[component].push_back(p2);
				break;
			}
			case kCustom:
			{
				break;
			}
		}
	}
	else if (metric == kLINF)
	{
		switch (p)
		{
			case kFurthest:
			{
				assert(currDim == pivots[component].size());
				state pivot;
				if (pivots[component].size() == 0)
					pivot = GetFurthest(GetRandomState(component), s1, env);
				else
					pivot = GetFurthest(pivots[component], s1, env);
				pivots[component].push_back(pivot);
				// Now get distances from the pivot
				GetFurthest(pivot, s1, env);
			}
				break;
			case kHeuristicError:
				assert(!"Not defined (although it could easily be defined and tested");
				break;
		}
	}
}

template<class state, class environment, class action>
void GridEmbedding<state, environment, action>::SelectCustomPivots(int component, state p1, state p2)
{
	GetFurthest(p2, s2, &residual);
	GetFurthest(p1, s1, &residual);
	pivots[component].push_back(p1);
	pivots[component].push_back(p2);
}

/**
 * Embed the next dimension
 * Assumes the distances are already calculated in s1 and s2 so these can be used
 */
 template<class state, class environment, class action>
void GridEmbedding<state, environment, action>::Embed(tEmbeddingFunction e, int component)
{
	for (int s = 0; s < env->GetMaxHash(); s++)
	{
		state next;
		double dp1, dp2, dp1p2;
		if (connectedComponents[s] != component)
			continue;
		
		env->GetStateFromHash(s, next);
		
		// This isn't a valid state in the current component - ignore
		if (!s1.GetClosedListGCost(next, dp1))
		{
			assert(!"In the connected component; should be in the search");
			continue;
		}
		
		switch (e)
		{
			case kDifferential:
				embedding[s*maxDim+currDim] = dp1;
				break;
			case kFastMap:
				s1.GetClosedListGCost(pivots[component].back(), dp1p2);
				if (!s2.GetClosedListGCost(next, dp2))
					assert(!"FM: In the connected component [s2]; should be in the search");
				embedding[s*maxDim+currDim] = (dp1+dp1p2-dp2)/2;
				break;
			case kFMUniform:
				s1.GetClosedListGCost(pivots[component].back(), dp1p2);
				if (!s2.GetClosedListGCost(next, dp2))
					assert(!"In the connected component [s2]; should be in the search");
				embedding[s*maxDim+currDim] = (dp1/(dp1+dp2))*dp1p2;
				break;
			case kFMShrink80:
				s1.GetClosedListGCost(pivots[component].back(), dp1p2);
				s2.GetClosedListGCost(next, dp2);
				embedding[s*maxDim+currDim] = 0.8*(dp1+dp1p2-dp2)/2;
				break;
		}
	}
}

// In maps with more than one connected components we need to get the
// limits after all dimensions are built
template<class state, class environment, class action>
void GridEmbedding<state, environment, action>::GetDimensionLimits()
{
	assert(scale.size() == currDim);

	double maxd = std::numeric_limits<double>::lowest();
	double mind = std::numeric_limits<double>::infinity();
	for (int s = 0; s < env->GetMaxHash(); s++)
	{
		if (!fequal(embedding[s*maxDim+currDim], -1))
		{
			maxd = std::max(maxd, embedding[s*maxDim+currDim]);
			mind = std::min(mind, embedding[s*maxDim+currDim]);
		}
	}
	scale.push_back({mind, maxd});
}


// TODO: Need to handle maps with different connected components
template<class state, class environment, class action>
state GridEmbedding<state, environment, action>::GetRandomState(int component)
{
	// TODO: If connected component is small, it would be better to directly go through embedding
	state l;
	while (true)
	{
		int hash = (int)random()%env->GetMaxHash();
		if (connectedComponents[hash] == component)
		{
			env->GetStateFromHash(hash, l);
			return l;
		}
	}
}


template<class state, class environment, class action>
state GridEmbedding<state, environment, action>::GetFurthest(state l, TemplateAStar<state, action, environment> &astar, environment *env)
{
	state furthest = l;
	astar.SetHeuristic(0);
	astar.SetStopAfterGoal(false);
	astar.InitializeSearch(env, l, l, path);
	while (!astar.DoSingleSearchStep(path))
	{
		if (astar.GetNumOpenItems() > 0)
			furthest = astar.GetOpenItem(0).data;
	}
	return furthest;
}


template<class state, class environment, class action>
state GridEmbedding<state, environment, action>::GetFurthest(std::vector<state> &l, TemplateAStar<state, action, environment> &astar, environment *env)
{
	assert(l.size() > 0);
	state furthest = l[0];
	astar.SetStopAfterGoal(false);
	astar.InitializeSearch(env, l[0], l[0], path);
	for (int i = 1; i < l.size(); i++)
		astar.AddAdditionalStartState(l[i]);

	while (!astar.DoSingleSearchStep(path))
	{
		if (astar.GetNumOpenItems() > 0)
			furthest = astar.GetOpenItem(0).data;
	}
	return furthest;
}


template<class state, class environment, class action>
void GridEmbedding<state, environment, action>::GetConnectedComponents()
{
	numConnectedComponents = 1;
	s1.SetStopAfterGoal(false);
	connectedComponents.resize(env->GetMaxHash());
	std::fill(connectedComponents.begin(), connectedComponents.end(), 0xFF);
	for (int s = 0; s < env->GetMaxHash(); s++)
	{
		connectedComponents[s] = 0;
		/*
		state next;
		env->GetStateFromHash(s, next);
		if (env->GetMap()->GetTerrainType(next.x, next.y) != kGround)
			continue;
		if (connectedComponents[s] != 0xFF)
			continue;
		// Do search to find all connected components
		s1.GetPath(env, next, next, path);
		// Mark them all
		for (int x = 0; x < s1.GetNumItems(); x++)
		{
			auto i = s1.GetItem(x);
			auto hash = env->GetStateHash(i.data);
			assert(connectedComponents[hash] == 0xFF);
			connectedComponents[hash] = numConnectedComponents;
		}
		// Increment Region
		numConnectedComponents++;
		*/
	}
	printf("Map has %d connected components\n", numConnectedComponents);
}

template<class state, class environment, class action>
double GridEmbedding<state, environment, action>::HCost(const state &a, const state &b) const
{
	double h = 0;
	uint64_t hash1 = env->GetStateHash(a);
	if (hash1 < 0 || hash1 >= env->GetMaxHash())
		return std::numeric_limits<double>::max();
	uint64_t hash2 = env->GetStateHash(b);
	switch (metric)
	{
		case kL1:
			for (int x = 0; x < currDim; x++)
			{
				h += fabs(embedding[hash1*maxDim+x]-embedding[hash2*maxDim+x]);
			}
			break;
		case kLINF:
			for (int x = 0; x < currDim; x++)
			{
				h = std::max(h, fabs(embedding[hash1*maxDim+x]-embedding[hash2*maxDim+x]));
			}
			break;
	}
	return h;
}

template<class state, class environment, class action>
point3d GridEmbedding<state, environment, action>::Lookup(const state &l) const
{
	if (currDim <= 1)
		return {-1,-1,0};

	float rad;
	if (env->GetMap()->GetMapHeight() > env->GetMap()->GetMapWidth())
	{
		rad = 1.0/(float)(env->GetMap()->GetMapHeight());
	}
	else {
		rad = 1.0/(float)(env->GetMap()->GetMapWidth());
	}
	
	int i = (int)env->GetStateHash(l);
	if (i >= env->GetMaxHash())
		return {-1,-1,0};
	float l1 = embedding[i*maxDim+0];
	float l2 = embedding[i*maxDim+1];
	if (l1 == -1)
		return {-1,-1,0};
	float xDim = 0.9*(2*(l1-scale[0].first)/(scale[0].second-scale[0].first)-1);
	float yDim = 0.9*(2*(l2-scale[1].first)/(scale[1].second-scale[1].first)-1);
	return {xDim, yDim};
}


template <int numOfDisks, int pattern, int offset = 0>
class TOHFM : public Heuristic<TOHState<numOfDisks>>
{
public:
	TOHFM()
	{
		
	}
	TOHFM(GridEmbedding<TOHState<pattern>, TOH<pattern>, TOHMove> *_embedding, TOHPDB<pattern, numOfDisks, offset> *_pdb)
	{
		embedding = _embedding;
		pdb = _pdb;
	}

	double HCost(const TOHState<numOfDisks> &a, const TOHState<numOfDisks> &b) const
	{
		TOHState<pattern> _a, _b;
		auto hs = pdb->GetAbstractHash(a);
		pdb->GetStateFromPDBHash(hs, _a);
		auto hg = pdb->GetAbstractHash(b);
		pdb->GetStateFromPDBHash(hg, _b);
		return embedding->HCost(_a, _b);
	}
	TOHPDB<pattern, numOfDisks, offset> *pdb;
	GridEmbedding<TOHState<pattern>, TOH<pattern>, TOHMove> *embedding;
};


template <int numOfDisks, int pattern, int offset>
class DoubleTOHFM : public Heuristic<TOHState<numOfDisks>>
{
public:
	DoubleTOHFM(GridEmbedding<TOHState<pattern>, TOH<pattern>, TOHMove> *_embedding, TOHPDB<pattern, numOfDisks> *_pdb, TOHPDB<pattern, numOfDisks, offset> *_pdb1)
	{
		embedding = _embedding;
		pdb = _pdb;
		pdb1 = _pdb1;
	}
	double HCost(const TOHState<numOfDisks> &a, const TOHState<numOfDisks> &b) const
	{
		TOHState<pattern> _a, _b;
		auto hs = pdb->GetAbstractHash(a);
		pdb->GetStateFromPDBHash(hs, _a);
		auto hg = pdb->GetAbstractHash(b);
		pdb->GetStateFromPDBHash(hg, _b);
		auto res = embedding->HCost(_a, _b);

		hs = pdb1->GetAbstractHash(a);
		pdb1->GetStateFromPDBHash(hs, _a);
		hg = pdb1->GetAbstractHash(b);
		pdb1->GetStateFromPDBHash(hg, _b);
		res += embedding->HCost(_a, _b);
		return res;
	}
	TOHPDB<pattern, numOfDisks> *pdb;
	TOHPDB<pattern, numOfDisks, offset> *pdb1;
	GridEmbedding<TOHState<pattern>, TOH<pattern>, TOHMove> *embedding;
};

//private:
//std::vector<uint32_t> embedding;
//const environment *env;
//tMetric m;
