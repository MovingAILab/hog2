//
//  WeightedVertexGraph.h
//  hog2 glut
//
//  Created by Nathan Sturtevant on 7/1/17.
//  Copyright © 2017 University of Denver. All rights reserved.
//

#ifndef WeightedVertexGraph_h
#define WeightedVertexGraph_h

#include "TemplateAStar.h"
#include "SVGUtil.h"
#include <map>
#include <sstream>
#include "StringUtils.h"

template <class state, class action, class environment>
class BidirectionalProblemAnalyzer {
public:
	BidirectionalProblemAnalyzer(const state &s, const state &g, environment *e,
								 Heuristic<state> *f, Heuristic<state> *b)
	:start(s), goal(g), e(e), f(f), b(b)
	{
		drawFullGraph = false;
		drawProblemInstance = true;
		drawMinimumVC = true;
		drawAllG = true;
		drawStatistics = true;
		flipBackwardsGCost = false;
		drawSumOnEdge = false;
		drawShortenedEdges = true;
		BuildDataStructures();
	}
	
	void BuildDataStructures()
	{
		std::vector<state> path;
		astarf.SetHeuristic(f);
		astarf.GetPath(e, start, goal, path);
		astarb.SetHeuristic(b);
		astarb.GetPath(e, goal, start, path);
		optCost = e->GetPathLength(path);
		forwardOptG = -1;
		backwardOptG = -1;
		forwardSum = 0;
		backwardSum = 0;
		optCostStr = "Optimal cost: "+std::to_string(optCost);
		if (astarf.GetNecessaryExpansions() == 0 || astarb.GetNecessaryExpansions() == 0)
		{
			printf("No necessary expansions\n");
			return;
		}
		for (int x = 0; x < astarf.GetNumItems(); x++)
		{
			const auto &i = astarf.GetItem(x);
			if (i.where != kClosedList)
				continue;
			auto i2 = m_f.find(i.g);
			if (fgreatereq(i.g+i.h, optCost))
			{
				if (fless(i.g, optCost) && (i2 == m_f.end()) && drawAllG)
				{
					m_f.insert({i.g, 0});
				}
			}
			else {
				if (i2 == m_f.end())
				{
					m_f.insert({i.g, 1});
				}
				else {
					i2->second++;
				}
			}
		}
		if (m_f.size() == 0)
		{
			printf("No forward item\n");
			return;
		}
		for (int x = 0; x < astarb.GetNumItems(); x++)
		{
			const auto &i = astarb.GetItem(x);
			if (i.where != kClosedList)
				continue;
			auto i2 = m_b.find(i.g);
			if (fgreatereq(i.g+i.h, optCost))
			{
				if (fless(i.g, optCost) && (i2 == m_b.end()) && drawAllG)
				{
					m_b.insert({i.g, 0});
				}
			}
			else {
				if (i2 == m_b.end())
				{
					m_b.insert({i.g, 1});
				}
				else {
					i2->second++;
				}
			}
		}
		if (m_b.size() == 0)
		{
			printf("No backward item\n");
			return;
		}
		//	std::cout << "Forward\n";
		for (auto i = m_f.begin(); i != m_f.end(); i++)
		{
			//		std::cout << i->first << " : " << i->second << "\n";
			forwardSum += i->second;
		}
		//	std::cout << "Backward\n";
		for (auto i = m_b.begin(); i != m_b.end(); i++)
		{
			//		std::cout << i->first << " : " << i->second << "\n";
			backwardSum += i->second;
		}
		height = 75*std::max(m_f.size(), m_b.size())+50;
		width = height*0.5;
		
		totalWork = backwardSum;
		const int forwardCopy = forwardSum;
		const int backwardCopy = backwardSum;
		//	printf("--At f: %1f b:%1f work is %d\n", -1.0f, m_b.rbegin()->first, backwardSum);
		{
			// compute optimal partition
			auto fi = m_f.begin();
			auto bi = m_b.rbegin();
			backwardOptG = bi->first;
			forwardOptG = -1;
			forwardSum = 0;
			
			// fi points to the first unused entry
			// bi points to the last used entry
			while (true)
			{
				do {
					// Remove the next backwards entry
					backwardSum -= bi->second;
					//				printf("Subtract %d from back\n", bi->second);
					bi++;
					if (bi == m_b.rend())
						break;
				} while (fi->first+bi->first > optCost);
				if (bi == m_b.rend())
					break;
				// Find all the necessary forward entries to maintain the vertex cover
				while (true)
				{
					forwardSum += fi->second;
					//				printf("--Add %d to front\n", fi->second);
					fi++;
					// If we reach one end, we can remove everything in the other direction
					if (fi == m_f.end())
					{
						while (bi != m_b.rend())
						{
							backwardSum -= bi->second;
							bi++;
						}
						break;
					}
					//				printf("%1.1f+%1.1f vs %1.1f\n", fi->first, bi->first, optCost);
					auto tmp = bi;
					tmp--;
					if (fi->first+(tmp)->first >= optCost)
						break;
					
				}
				//			printf("Considering: %d + %d = %d\n", forwardSum, backwardSum, forwardSum+backwardSum);
				if (fi == m_f.end())
					break;
				//			printf("--At f: %1f b:%1f work is %d\n", fi->first, (bi == m_b.rend())?-1:bi->first, forwardSum+backwardSum);
				
				if (forwardSum+backwardSum < totalWork)
				{
					totalWork = forwardSum+backwardSum;
					forwardOptG = fi->first;
					if (bi == m_b.rend())
						backwardOptG = -1;
					else
						backwardOptG = bi->first;
				}
			}
			if (fi != m_f.end())
			{
				forwardSum += fi->second;
				fi++;
			}
			fi--;
			//		printf("--At f: %1f b:%1f work is %d\n", fi->first, -1.0f, forwardSum+backwardSum);
			if (forwardSum+backwardSum < totalWork)
			{
				totalWork = forwardSum+backwardSum;
				forwardOptG = fi->first;
				backwardOptG = -1;
			}
		}
		
		{
			printf("Forward/Backward/Min: %d %d %d %s\n", forwardCopy, backwardCopy, totalWork,
				   (std::min(forwardCopy, backwardCopy)!=totalWork)?"bidirectional wins!":"" );
			forwardSum = forwardCopy;
			backwardSum = backwardCopy;
		}
	}

	int GetMinWork()
	{
		return totalWork;
	}
	
	int GetForwardWork()
	{
		return forwardSum;
	}

	int GetBackwardWork()
	{
		return backwardSum;
	}

	double GetForwardMaxG()
	{
		return m_f.rbegin()->first;
		
	}

	double GetMaxG()
	{
		return std::max(m_f.rbegin()->first, m_b.rbegin()->first);
	}
	
	double GetBackwardMaxG()
	{
		return m_b.rbegin()->first;
	}
	
	size_t GetNumGCosts()
	{
		return std::max(m_f.size(), m_b.size());
	}
	
	void SaveSVG(const char *filename)
	{
		bool eshed = false;
		const int kLRMargin = 150;
		const int kTopMargin = 75;
		const int kBottomMargin = 50;
		
		const int kCountTextSize = 25;
		const int kInstanceTextSize = 20;
		const int kStatsTextSize = 20;
		const int kNodeGSize = 30;
		
		const int kNodeRadius = 25;
		const int kNodeGap = 75;
		const int epsilon = 0;
		
		height = kNodeGap*std::max(m_f.size(), m_b.size())+kBottomMargin;
		if (eshed) height += kNodeGap;
		width = height*0.5;

		
//		printf("Opt: %1.2f, For: %1.2f, Back: %1.2f\n", optCost, forwardOptG, backwardOptG);
		std::string s;
		// 10% margin on all sides of image
		s = "<svg xmlns=\"http://www.w3.org/2000/svg\" version=\"1.1\" width = \""+std::to_string(width)+"\" height = \""+std::to_string(height)+"\" ";
		s += "preserveAspectRatio = \"none\" ";
		s += ">\n";

		int fcnt = 0;
		int64_t fsum = 0;
		
		if (flipBackwardsGCost && drawSumOnEdge)
			s += SVGDrawText(width/2.0, (kTopMargin+kNodeGap*fcnt+height-kNodeGap*(m_b.size()-1)-kBottomMargin)/2.0-kCountTextSize,
							 to_string_separator(backwardSum).c_str(), Colors::black, kCountTextSize);

		// Draw Edges between sides
		for (auto i = m_f.begin(); i != m_f.end(); i++)
		{
			fsum += i->second;
			int64_t bsum = backwardSum;
			if (drawAllG || i->second != 0)
			{
				int bcnt = m_b.size()-1;
				bool drewBest = false;
				for (auto j = m_b.rbegin(); j != m_b.rend(); j++)
				{
					bsum -= j->second;
					auto tmp = i;
					tmp++;
					
					if (drawFullGraph)
					{
						if (fless(i->first + j->first+epsilon, optCost) && j->second != 0)
						{
							if (flipBackwardsGCost)
							{
								s += SVGDrawLine(kLRMargin, kTopMargin+kNodeGap*fcnt, width-kLRMargin, height-kNodeGap*bcnt-kBottomMargin, 1, Colors::black);
							}
							else {
								s += SVGDrawLine(kLRMargin, kTopMargin+kNodeGap*fcnt, width-kLRMargin, kTopMargin+kNodeGap*bcnt, 1, Colors::black);
							}
						}
					}
					else {
						if (!drewBest &&
							(fless(i->first + j->first+epsilon, optCost) &&
							((tmp) == m_f.end() || fgreatereq((tmp)->first + j->first + epsilon, optCost))
							&& (j->second != 0 || drawSumOnEdge)))
						{
							drewBest = true;
							if (flipBackwardsGCost) {
								s += SVGDrawLine(kLRMargin, kTopMargin+kNodeGap*fcnt, width-kLRMargin, height-kNodeGap*bcnt-kBottomMargin, 1, Colors::black);
								if (drawSumOnEdge)
								{
									if (bsum == 0)
										s += SVGDrawText(width/2.0, (kTopMargin+kNodeGap*fcnt+height-kNodeGap*bcnt-kBottomMargin)/2.0+kCountTextSize, to_string_separator(fsum+bsum).c_str(), Colors::black, kCountTextSize);
									else
										s += SVGDrawText(width/2.0, (kTopMargin+kNodeGap*fcnt+height-kNodeGap*bcnt-kBottomMargin)/2.0+kNodeGap/2.0, to_string_separator(fsum+bsum).c_str(), Colors::black, kCountTextSize);
								}
							}
							else {
								s += SVGDrawLine(kLRMargin, kTopMargin+kNodeGap*fcnt, width-kLRMargin, kTopMargin+kNodeGap*bcnt, 1, Colors::black);
							}
							//break;
						}
						else {
							if (drawShortenedEdges)
							{
								if (fless(i->first + j->first + epsilon, optCost))
								{
									double p1x, p1y, p2x, p2y;
									p1x = kLRMargin;
									p1y = kTopMargin+kNodeGap*fcnt;
									p2x = width-kLRMargin;
									
									if (flipBackwardsGCost)
									{
										p2y = height-kNodeGap*bcnt-kBottomMargin;
									}
									else {
										p2y = kTopMargin+kNodeGap*bcnt;
									}
									double length = sqrt((p1x-p2x)*(p1x-p2x)+(p1y-p2y)*(p1y-p2y));
									double scale = (1.5*kNodeRadius)/length;
									s += SVGDrawLine(p1x, p1y, p1x+(p2x-p1x)*scale, p1y+(p2y-p1y)*scale, 1, Colors::black);
									s += SVGDrawLine(p2x-(p2x-p1x)*scale, p2y-(p2y-p1y)*scale, p2x, p2y, 1, Colors::black);
								}
							}
						}
					}
					bcnt--;
				}
			}
			fcnt++;
		}
		
		// Draw forward nodes
		int cnt = 0;
//		printf("Min forward: %1.2f; Min backward: %1.2f\n", forwardOptG, backwardOptG);
		for (auto i = m_f.begin(); i != m_f.end(); i++)
		{
			if (!drawAllG && i->second == 0)
			{
				cnt++;
				continue;
			}
			std::string tmp = std::to_string((int)i->first);
//			std::cout << i->first << " : " << i->second << "\n";
			if ((fless(i->first, forwardOptG) || backwardOptG == -1) && (i->second != 0 || drawSumOnEdge) && drawMinimumVC)
			{
				s += SVGDrawCircle(kLRMargin, kTopMargin+kNodeGap*cnt, kNodeRadius, Colors::lightblue);
			}
			else {
				s += SVGDrawCircle(kLRMargin, kTopMargin+kNodeGap*cnt, kNodeRadius, Colors::lightgray);
			}
			s += SVGFrameCircle(kLRMargin, kTopMargin+kNodeGap*cnt, kNodeRadius, 1, Colors::black);
			s += SVGDrawText(kLRMargin, kTopMargin+kNodeGap*cnt, tmp.c_str(), Colors::black, kNodeGSize);
			//if (i->second != 0)
			{
				tmp = to_string_separator(i->second);
				//s += SVGDrawText((kLRMargin-kNodeRadius)/2, kTopMargin+kNodeGap*cnt, tmp.c_str(), Colors::black, kCountTextSize);
				s += SVGDrawText(kLRMargin-3*kNodeRadius/2, kTopMargin+kNodeGap*cnt, tmp.c_str(), Colors::black, kCountTextSize, 0, SVG::kRight);
			}
			cnt++;
		}

		// Draw backwards nodes
		cnt = 0;
		for (auto i = m_b.begin(); i != m_b.end(); i++)
		{
			if (!drawAllG && i->second == 0)
			{
				cnt++;
				continue;
			}
			std::string tmp = std::to_string((int)i->first);
			//std::cout << i->first << " : " << i->second << "\n";
			int loc = kTopMargin+kNodeGap*cnt;
			if (flipBackwardsGCost)
			{
				loc = height-kBottomMargin-kNodeGap*cnt;
			}
			if (flesseq(i->first, backwardOptG) && (i->second != 0 || drawSumOnEdge) && drawMinimumVC)
				s += SVGDrawCircle(width-kLRMargin, loc, kNodeRadius, Colors::lightblue);
			else
				s += SVGDrawCircle(width-kLRMargin, loc, kNodeRadius, Colors::lightgray);
			s += SVGFrameCircle(width-kLRMargin, loc, kNodeRadius, 1, Colors::black);
			s += SVGDrawText(width-kLRMargin, loc, tmp.c_str(), Colors::black, kNodeGSize);
			//tmp = std::to_string((int)i->second);
			//if (i->second != 0)
			{
				tmp = to_string_separator(i->second);
				//s += SVGDrawText(width-(kLRMargin-kNodeRadius)/2, loc, tmp.c_str(), Colors::black, kCountTextSize, 0, SVG::kRight);
				s += SVGDrawText(width-kLRMargin+3*kNodeRadius/2, loc, tmp.c_str(), Colors::black, kCountTextSize, 0, SVG::kLeft);
			}
			cnt++;
		}
		
		if (drawStatistics)
		{
			s += SVGDrawText(width/2, height-4*kStatsTextSize, optCostStr.c_str(), Colors::black, kStatsTextSize);
			optCostStr = "Forward A*: "+std::to_string(astarf.GetNecessaryExpansions());
			s += SVGDrawText(width/2, height-3*kStatsTextSize, optCostStr.c_str(), Colors::black, kStatsTextSize);
			optCostStr = "Backward A*: "+std::to_string(astarb.GetNecessaryExpansions());
			s += SVGDrawText(width/2, height-2*kStatsTextSize, optCostStr.c_str(), Colors::black, kStatsTextSize);
			optCostStr = "Optimal: "+std::to_string(totalWork);
			s += SVGDrawText(width/2, height-kStatsTextSize, optCostStr.c_str(), Colors::black, kStatsTextSize);
		}
		if (drawProblemInstance)
		{
			optCostStr = "Domain: "+e->GetName();
			s += SVGDrawText(width/2, kInstanceTextSize, optCostStr.c_str(), Colors::black, kInstanceTextSize);
			std::stringstream tmp;
			tmp << "Start: " << start;
			s += SVGDrawText(width/2, 2*kInstanceTextSize, tmp.str().c_str(), Colors::black, kInstanceTextSize);
			tmp.str("");
			tmp << "Goal: " << goal;
			s += SVGDrawText(width/2, 3*kInstanceTextSize, tmp.str().c_str(), Colors::black, kInstanceTextSize);
		}
		s += SVGDrawText(kLRMargin, kTopMargin-2*kNodeRadius, "g<tspan dy =\"15\">F</tspan>", Colors::black, kCountTextSize);
		s += SVGDrawText(width-kLRMargin, kTopMargin-2*kNodeRadius, "g<tspan dy =\"15\">B</tspan>", Colors::black, kCountTextSize);

		if (filename != 0)
		{
			std::fstream svgFile;
			svgFile.open(filename, std::fstream::out | std::fstream::trunc);
			svgFile << s;
			svgFile << "</svg>";
			svgFile.close();
			printf("Generated SVG '%s'\n", filename);
		}
	}

	void SaveSVG(const char *filename, int groupSize)
	{
		if (groupSize <= 0)
			groupSize = 1;
		int localHeight = 75*(std::max(m_f.size(), m_b.size())+groupSize-1)/groupSize+50;
		int localWidth = localHeight*0.5;
		std::string s;
		// 10% margin on all sides of image
		s = "<svg xmlns=\"http://www.w3.org/2000/svg\" version=\"1.1\" width = \""+std::to_string(localWidth)+"\" height = \""+std::to_string(localHeight)+"\" ";
		s += "preserveAspectRatio = \"none\" ";
		s += ">\n";
		
		int fcnt = 0;
		int lastF = -1, lastB = -1;
		for (auto i = m_f.begin(); i != m_f.end(); i++)
		{
			//if (i->second != 0)
			{
				int bcnt = m_b.size()-1;
				for (auto j = m_b.rbegin(); j != m_b.rend(); j++)
				{
					auto tmp = i;
					tmp++;
					if (drawFullGraph)
					{
						if (fless(i->first + j->first, optCost) && j->second != 0)
						{
							if (fcnt/groupSize != lastF && bcnt/groupSize != lastB)
							{
								s += SVGDrawLine(100, 75+75*(fcnt/groupSize), localWidth-100, 75+75*(bcnt/groupSize), 1, Colors::black);
								lastF = fcnt/groupSize;
								lastB = bcnt/groupSize;
							}
						}
					}
					else {
						if (fless(i->first + j->first, optCost) &&
							((tmp) == m_f.end() || fgreatereq((tmp)->first + j->first, optCost))
							&& j->second != 0)
						{
							if (fcnt/groupSize != lastF && bcnt/groupSize != lastB)
							{
								s += SVGDrawLine(100, 75+75*(fcnt/groupSize), localWidth-100, 75+75*(bcnt/groupSize), 1, Colors::black);
								lastF = fcnt/groupSize;
								lastB = bcnt/groupSize;
							}
							break;
						}
					}
					bcnt--;
				}
			}
			fcnt++;
		}
		
		int cnt = 0;
		uint64_t nodes = 0;
		for (auto i = m_f.begin(); i != m_f.end(); i++)
		{
			if (cnt%groupSize == 0)
			{
				//std::cout << i->first << " : " << i->second << "\n";
				if (flesseq(i->first, forwardOptG) && i->second != 0 && drawMinimumVC)
					s += SVGDrawCircle(100, 75+75*cnt/groupSize, 25, Colors::lightblue);
				else
					s += SVGDrawCircle(100, 75+75*cnt/groupSize, 25, Colors::lightgray);
				s += SVGFrameCircle(100, 75+75*cnt/groupSize, 25, 1, Colors::black);

				std::string tmp = std::to_string((int)i->first);
				s += SVGDrawText(100, 75+75*(cnt/groupSize), tmp.c_str(), Colors::black, 30);
				nodes = 0;
			}
			nodes += i->second;
			if (cnt%groupSize == groupSize-1)
			{
				std::string tmp = std::to_string((int)nodes);
				s += SVGDrawText(37.5, 75+75*(cnt/groupSize), tmp.c_str(), Colors::black, 20);
			}
			cnt++;
		}
		if (cnt%groupSize != 0)
		{
			std::string tmp = std::to_string((int)nodes);
			s += SVGDrawText(37.5, 75+75*(cnt/groupSize), tmp.c_str(), Colors::black, 20);
		}
		cnt = 0;
		nodes = 0;
		for (auto i = m_b.begin(); i != m_b.end(); i++)
		{
			if (cnt%groupSize == 0)
			{
				//std::cout << i->first << " : " << i->second << "\n";
				if (flesseq(i->first, backwardOptG) && i->second != 0 && drawMinimumVC)
					s += SVGDrawCircle(localWidth-100, 75+75*(cnt/groupSize), 25, Colors::lightblue);
				else
					s += SVGDrawCircle(localWidth-100, 75+75*(cnt/groupSize), 25, Colors::lightgray);
				s += SVGFrameCircle(localWidth-100, 75+75*(cnt/groupSize), 25, 1, Colors::black);

				std::string tmp = std::to_string((int)i->first);
				s += SVGDrawText(localWidth-100, 75+75*(cnt/groupSize), tmp.c_str(), Colors::black, 30);
				nodes = 0;
			}
			nodes += i->second;
			if (cnt%groupSize == groupSize-1)
			{
				std::string tmp = std::to_string((int)nodes);
				s += SVGDrawText(localWidth-37.5, 75+75*(cnt/groupSize), tmp.c_str(), Colors::black, 20);
			}
			cnt++;
		}
		if (cnt%groupSize != 0)
		{
			std::string tmp = std::to_string((int)nodes);
			s += SVGDrawText(localWidth-37.5, 75+75*(cnt/groupSize), tmp.c_str(), Colors::black, 20);
		}

		if (drawStatistics)
		{
			s += SVGDrawText(localWidth/2, localHeight-100, optCostStr.c_str(), Colors::black, 30);
			optCostStr = "Forward A*: "+std::to_string(astarf.GetNecessaryExpansions());
			s += SVGDrawText(localWidth/2, localHeight-70, optCostStr.c_str(), Colors::black, 20);
			optCostStr = "Backward A*: "+std::to_string(astarb.GetNecessaryExpansions());
			s += SVGDrawText(localWidth/2, localHeight-45, optCostStr.c_str(), Colors::black, 20);
			optCostStr = "Optimal: "+std::to_string(totalWork);
			s += SVGDrawText(localWidth/2, localHeight-20, optCostStr.c_str(), Colors::black, 20);
		}
		if (drawProblemInstance)
		{
			optCostStr = "Domain: "+e->GetName();
			s += SVGDrawText(localWidth/2, 20, optCostStr.c_str(), Colors::black, 20);
			std::stringstream tmp;
			tmp << "Start: " << start;
			s += SVGDrawText(localWidth/2, 45, tmp.str().c_str(), Colors::black, 20);
			tmp.str("");
			tmp << "Goal: " << goal;
			s += SVGDrawText(localWidth/2, 70, tmp.str().c_str(), Colors::black, 20);
		}
		s += SVGDrawText(100, 20, "g<tspan dy =\"15\">F</tspan>", Colors::black, 20);
		s += SVGDrawText(localWidth-100, 20, "g<tspan dy =\"15\">B</tspan>", Colors::black, 20);
		
		if (filename != 0)
		{
			std::fstream svgFile;
			svgFile.open(filename, std::fstream::out | std::fstream::trunc);
			svgFile << s;
			svgFile << "</svg>";
			svgFile.close();
			printf("Generated SVG '%s'\n", filename);
		}
	}

	
	static uint64_t GetWeightedVertexGraph(const state &start, const state &goal, environment *e, Heuristic<state> *f, Heuristic<state> *b, const char *filename = 0)
	{
		BidirectionalProblemAnalyzer analyze(start, goal, e, f, b);
		if (filename != 0)
			analyze.SaveSVG(filename);
		return analyze.totalWork;
	}
private:
	state start, goal;
	environment *e;
	Heuristic<state> *f, *b;
	std::map<double, int> m_f, m_b;

	TemplateAStar<state, action, environment> astarf;
	TemplateAStar<state, action, environment> astarb;
	int forwardSum;
	int backwardSum;
	int totalWork;
	double optCost;
	int height, width;
	std::string optCostStr;
	double forwardOptG;
	double backwardOptG;

public:
	bool drawFullGraph;
	bool drawProblemInstance;
	bool drawMinimumVC;
	bool drawAllG;
	bool drawStatistics;
	bool flipBackwardsGCost;
	bool drawSumOnEdge; // Note, this only applies if we aren't drawing the full graph and we are flipping the backwards g-costs
	bool drawShortenedEdges; // only applies if we aren't drawing the full graph
};

#endif /* WeightedVertexGraph_h */
