//
//  GridHeuristics.cpp
//  HOG2 ObjC
//
//  Created by Nathan Sturtevant on 2/6/23.
//  Copyright © 2023 MovingAI. All rights reserved.
//

#include "GridHeuristics.h"
#include <cassert>

GridEmbedding::GridEmbedding(MapEnvironment *e, int numDimensions, tMetric m)
:env(e), residual(e->GetMap(), this), metric(m)
{
	currDim = 0;
	maxDim = numDimensions;
	embedding.resize(env->GetMaxHash()*maxDim);
	std::fill(embedding.begin(), embedding.end(), -1);
	GetConnectedComponents();
	pivots.resize(numConnectedComponents);
}

bool GridEmbedding::AddDimension(tEmbeddingFunction e, tPlacementScheme p, Heuristic<xyLoc> *h)
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

///**
// * Add a dimension of the embedding using \input l as the pivot
// * Assumes the kDifferential embedding function
// * Also assumes a single connected component
// * (Note: Add a 2-parameter embedding if other methods need to be supported)
// */
//bool GridEmbedding::AddDimension(xyLoc l)
//{
//	if (currDim == maxDim)
//		return false;
//	pivots.push_back(l);
//	// Now get distances from the pivot
//	GetFurthest(l, s1, env);
//	Embed(kDifferential);
//	return true;
//}

void GridEmbedding::SelectPivots(tPlacementScheme p, int component, Heuristic<xyLoc> *h)
{
	if (metric == kL1)
	{
		switch (p)
		{
			case kFurthest:
			{
				// Get random state
				xyLoc rand = GetRandomState(component);
				// Get furthest point (p1)
				xyLoc p1 = GetFurthest(rand, s2, &residual);
				
				// Get furthest from that point (p1) storing distances in s1
				xyLoc p2 = GetFurthest(p1, s1, &residual);

				// Get distances for second pivot in s2
				GetFurthest(p2, s2, &residual);
				pivots[component].push_back(p1);
				pivots[component].push_back(p2);
				break;
			}
			case kHeuristicError:
			{
				// Get random state
				xyLoc rand = GetRandomState(component);
				// Get furthest point (p1)
				xyLoc p1 = GetFurthest(rand, s2, &residual);

				double maxValue = 0;
				// Now update to furthest by error
				// (Could be done more efficiently during the search)
				for (int x = 0; x < env->GetMaxHash(); x++)
				{
					if (connectedComponents[x] != component)
						continue;
					xyLoc candidate;
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
				xyLoc p2 = GetFurthest(p1, s1, &residual);

				maxValue = 0;
				// Now update to furthest by error
				for (int x = 0; x < env->GetMaxHash(); x++)
				{
					if (connectedComponents[x] != component)
						continue;
					xyLoc candidate;
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
		}
	}
	else if (metric == kLINF)
	{
		switch (p)
		{
			case kFurthest:
			{
				assert(currDim == pivots[component].size());
				xyLoc pivot;
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

/**
 * Embed the next dimension
 * Assumes the distances are already calculated in s1 and s2 so these can be used
 */
void GridEmbedding::Embed(tEmbeddingFunction e, int component)
{
	for (int s = 0; s < env->GetMaxHash(); s++)
	{
		xyLoc next;
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
void GridEmbedding::GetDimensionLimits()
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
xyLoc GridEmbedding::GetRandomState(int component)
{
	// TODO: If connected component is small, it would be better to directly go through embedding
	xyLoc l;
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

xyLoc GridEmbedding::GetFurthest(xyLoc l, TemplateAStar<xyLoc, tDirection, MapEnvironment> &astar, MapEnvironment *environment)
{
	xyLoc furthest = l;
	astar.SetHeuristic(0);
	astar.SetStopAfterGoal(false);
	astar.InitializeSearch(environment, l, l, path);
	while (!astar.DoSingleSearchStep(path))
	{
		if (astar.GetNumOpenItems() > 0)
			furthest = astar.GetOpenItem(0).data;
	}
	return furthest;
}

xyLoc GridEmbedding::GetFurthest(const std::vector<xyLoc> &l, TemplateAStar<xyLoc, tDirection, MapEnvironment> &astar, MapEnvironment *environment)
{
	assert(l.size() > 0);
	xyLoc furthest = l[0];
	astar.SetStopAfterGoal(false);
	astar.InitializeSearch(environment, l[0], l[0], path);
	for (int i = 1; i < l.size(); i++)
		astar.AddAdditionalStartState(l[i]);

	while (!astar.DoSingleSearchStep(path))
	{
		if (astar.GetNumOpenItems() > 0)
			furthest = astar.GetOpenItem(0).data;
	}
	return furthest;
}

void GridEmbedding::GetConnectedComponents()
{
	numConnectedComponents = 0;
	s1.SetStopAfterGoal(false);
	connectedComponents.resize(env->GetMaxHash());
	std::fill(connectedComponents.begin(), connectedComponents.end(), 0xFF);
	for (int s = 0; s < env->GetMaxHash(); s++)
	{
		xyLoc next;
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
	}
	//printf("Map has %d connected components\n", numConnectedComponents);
}


double GridEmbedding::HCost(const xyLoc &a, const xyLoc &b) const
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

void GridEmbedding::Draw(Graphics::Display &disp) const
{
	if (currDim <= 1)
		return;
	
	float rad;
	if (env->GetMap()->GetMapHeight() > env->GetMap()->GetMapWidth())
	{
		rad = 1.0/(float)(env->GetMap()->GetMapHeight());
	}
	else {
		rad = 1.0/(float)(env->GetMap()->GetMapWidth());
	}
	
	for (int i = 0; i < env->GetMaxHash(); i++)
	{
		float l1 = embedding[i*maxDim+0];
		float l2 = embedding[i*maxDim+1];
		if (l1 == -1)
			continue;
		float xDim = 0.9*(2*(l1-scale[0].first)/(scale[0].second-scale[0].first)-1);
		float yDim = 0.9*(2*(l2-scale[1].first)/(scale[1].second-scale[1].first)-1);
		disp.FillRect(Graphics::rect({xDim, yDim}, rad), Colors::white);
	}
}

void GridEmbedding::DrawPivots(Graphics::Display &disp) const
{
	for (int components = 0; components < numConnectedComponents; components++)
	{
		for (int x = 0; x < pivots.size(); x++)
		{
			env->SetColor(Colors::blue);
			env->Draw(disp, pivots[components][x]);
		}
	}
}

/**
 * Draws an overlay showing the relationship of the heuristic based on the distances from the pivots
 * for a single particular state.
 */
void GridEmbedding::DrawPivots(Graphics::Display &disp, const xyLoc &l) const
{
	if (metric == kLINF)
	{
		for (int component = 0; component < numConnectedComponents; component++)
		{
			for (int x = 0; x < pivots.size(); x++)
			{
				env->SetColor(Colors::darkgreen);
				env->DrawLine(disp, pivots[component][x], l);
				auto p1 = env->GetStateLoc(pivots[component][x]);
				auto p2 = env->GetStateLoc(l);
				auto h1 = env->GetStateHash(pivots[component][x]);
				auto h2 = env->GetStateHash(l);
				if (h2 < 0 || h2 >= env->GetMaxHash())
					continue;
				double h = embedding[h1*maxDim+x];
				h-=embedding[h2*maxDim+x];
				h = fabs(h);
				disp.DrawText(std::to_string(h).c_str(), (p1+p2)*0.5, Colors::darkgreen, 0.05);
			}
		}
	}
}


void GridEmbedding::Draw(Graphics::Display &disp, const xyLoc &l) const
{
	if (currDim <= 1)
		return;

	float rad;
	if (env->GetMap()->GetMapHeight() > env->GetMap()->GetMapWidth())
	{
		rad = 1.0/(float)(env->GetMap()->GetMapHeight());
	}
	else {
		rad = 1.0/(float)(env->GetMap()->GetMapWidth());
	}
	
	int i = env->GetStateHash(l);
	if (i < 0 || i > env->GetMaxHash())
		return;
	
	float l1 = embedding[i*maxDim+0];
	float l2 = embedding[i*maxDim+1];
	if (l1 == -1)
		return;
	float xDim = 0.9*(2*(l1-scale[0].first)/(scale[0].second-scale[0].first)-1);
	float yDim = 0.9*(2*(l2-scale[1].first)/(scale[1].second-scale[1].first)-1);
	disp.FillCircle({xDim, yDim}, 2*rad, Colors::red);
}


void GridEmbedding::Draw(Graphics::Display &disp, float tween) const
{
	if (currDim <= 1)
		return;

	float rad;
	if (env->GetMap()->GetMapHeight() > env->GetMap()->GetMapWidth())
	{
		rad = 1.0/(float)(env->GetMap()->GetMapHeight());
	}
	else {
		rad = 1.0/(float)(env->GetMap()->GetMapWidth());
	}
	
	for (int i = 0; i < env->GetMaxHash(); i++)
	{
		float l1 = embedding[i*maxDim+0];
		float l2 = embedding[i*maxDim+1];
		if (l1 == -1)
			continue;
		float xDim = 0.9*(2*(l1-scale[0].first)/(scale[0].second-scale[0].first)-1);
		float yDim = 0.9*(2*(l2-scale[1].first)/(scale[1].second-scale[1].first)-1);

		auto lerp = [](float a, float b, float mix)
		{ float tmp1 = mix*mix*mix; float tmp2 = (1-mix)*(1-mix)*(1-mix);
				mix = (1-mix)*tmp1+mix*(1-tmp2); return (1-mix)*a+mix*b;
		};

		xyLoc l;
		env->GetStateFromHash(i, l);
		double px, py, tmp, rad2;
		env->GetMap()->GetCoord(l.x, l.y, px, py, tmp, rad2);
		disp.FillRect(Graphics::rect({lerp(px, xDim, tween), lerp(py, yDim, tween)}, rad), Colors::white);
	}
}

point3d GridEmbedding::Lookup(const xyLoc &l) const
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
//private:
//std::vector<uint32_t> embedding;
//const MapEnvironment *env;
//tMetric m;
