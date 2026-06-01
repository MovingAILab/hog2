/*
 *  $Id: sample.cpp
 *  hog2
 *
 *  Created by Nathan Sturtevant on 5/31/05.
 *  Modified by Nathan Sturtevant on 02/29/20.
 *
 * This file is part of HOG2. See https://github.com/nathansttt/hog2 for licensing information.
 *
 */

#include "Common.h"
#include "Driver.h"
#include "Map2DEnvironment.h"
#include "CanonicalGrid.h"
#include "TemplateAStar.h"
#include "TextOverlay.h"
#include "MapOverlay.h"
#include <string>
#include "NBS.h"
#include "fMM.h"
#include "SVGUtil.h"

struct GMXNode {
	std::string name;
	float gCost;
	int oppositeLine;
	Graphics::rect r;
	bool checkedWrong;
	bool MVC;
	graphState node;
};

std::vector<GMXNode> forward;
std::vector<GMXNode> backward;
float optimalCost;

int downNode=-1, upNode;


GraphEnvironment *ge;
Graph *g;
graphState from=-1, to=-1;

double distance(unsigned long n1, unsigned long n2);
node *FindClosestNode(Graph *gr, point3d loc);
void BuildGMX();
void CheckGMX();
void BuildGraph();
void GetMVC();

class GraphDistHeuristic : public Heuristic<graphState> {
public:
	double HCost(const graphState &a, const graphState &b) const
	{
		return 0.8f*distance(a, b);
	}
};

GraphDistHeuristic searchHeuristic;

int main(int argc, char* argv[])
{
	InstallHandlers();
	RunHOGGUI(argc, argv, 1200, 1200);
	return 0;
}

/**
 * Allows you to install any keyboard handlers needed for program interaction.
 */
void InstallHandlers()
{
	InstallKeyboardHandler(MyDisplayHandler, "Check", "Check Result", kAnyModifier, 'c');
	InstallKeyboardHandler(MyDisplayHandler, "MVC", "Show minimum vertex cover", kAnyModifier, 'm');
	InstallKeyboardHandler(MyDisplayHandler, "Rebuild", "Build another random graph", kAnyModifier, 'r');

	InstallWindowHandler(MyWindowHandler);

	InstallMouseClickHandler(MyClickHandler);
}

void MyWindowHandler(unsigned long windowID, tWindowEventType eType)
{
	if (eType == kWindowDestroyed)
	{
		printf("Window %ld destroyed\n", windowID);
		RemoveFrameHandler(MyFrameHandler, windowID, 0);
	}
	else if (eType == kWindowCreated)
	{
		InstallFrameHandler(MyFrameHandler, windowID, 0);
		SetNumPorts(windowID, 1);
		
		printf("Done creating window\n");

		srandom(time(0));
		BuildGraph();

		
		ReinitViewports(windowID, {-1, -1, 0, 1}, kScaleToSquare);
		AddViewport(windowID, {0, -1, 1, 1}, kScaleToSquare);
	}
}

int frameCnt = 0;

Graphics::point GetCenter(Graphics::rect &r)
{
	return {(r.left+r.right)/2.0f, (r.top+r.bottom)/2.0f};
}

void MyFrameHandler(unsigned long windowID, unsigned int viewport, void *)
{
	Graphics::Display &display = getCurrentContext()->display;
	float boxSize = 1.0f/(std::max(std::max(forward.size(), backward.size()), (size_t)8)+2.0f);
	display.FillRect({-1, -1, 1, 1}, Colors::white);

	if (viewport == 0)
	{
		ge->SetColor(Colors::black);
		// draw all edges
		ge->Draw(display);
		
		// draw all nodes
		Graph *g = ge->GetGraph();
		ge->SetNodeScale(1.5);
		ge->SetColor(Colors::lightbluegray);
		for (int x = 0; x < g->GetNumNodes(); x++)
		{
			ge->Draw(display, x);
		}
		
		if (from != -1)
		{
			ge->SetColor(Colors::green);
			ge->DrawLine(display, from, to, 1.5);//1.0/60.0);
		}

		for (auto i : forward)
		{
			if (i.MVC)
			{
				ge->SetColor(Colors::red);
				ge->Draw(display, i.node);
			}
		}
		for (auto i : backward)
		{
			if (i.MVC)
			{
				ge->SetColor(Colors::purple);
				ge->Draw(display, i.node);
			}
		}
		return;
	}
	
	std::string str = "Optimal cost: "+to_string_with_precision(optimalCost, 3);
	display.DrawText(str.c_str(), {0, 0.9}, Colors::darkred, 0.05, Graphics::textAlignCenter);

	// Draw lines in Gmx Graph
	for (auto i : forward)
	{
		for (int x = 0; x < i.oppositeLine; x++)
			display.DrawLine(GetCenter(i.r), GetCenter(backward[x].r), 1.0/90.0f, Colors::darkgray);
		if (i.oppositeLine != -1)
			display.DrawLine(GetCenter(i.r), GetCenter(backward[i.oppositeLine].r), 1.0/90.0f, i.checkedWrong?Colors::red:Colors::darkgreen);
	}
	
	if (downNode != -1 && upNode != -1)
	{
		display.DrawLine(GetCenter(forward[downNode].r), GetCenter(backward[upNode].r), 1.0/60.0, Colors::green);
	}

	float borderSize = 1./90.0;
	for (auto i : forward)
	{
		display.FillCircle(i.r, Colors::lightblue);
		if (i.checkedWrong)
			display.FrameCircle(i.r, Colors::orange, 3*borderSize);
		else if (i.MVC)
			display.FrameCircle(i.r, Colors::red, 3*borderSize);
		else
			display.FrameCircle(i.r, Colors::darkblue, borderSize);
		std::string name = std::to_string((int)i.gCost);//..to_string_with_precision(i.gCost, 3); //		std::to_string(i.gCost);
		Graphics::point p = GetCenter(i.r);
		display.DrawText(i.name.c_str(), p, Colors::white, boxSize/2.0f, Graphics::textAlignCenter, Graphics::textBaselineMiddle);
		p.x -= 2*boxSize;
		display.DrawText(name.c_str(), p, Colors::black, boxSize/2.0f, Graphics::textAlignCenter,
						 Graphics::textBaselineMiddle);
	}

	for (auto i : backward)
	{
		display.FillCircle(i.r, Colors::lightblue);
		if (i.MVC)
			display.FrameCircle(i.r, Colors::purple, 3*borderSize);
		else
			display.FrameCircle(i.r, Colors::darkblue, borderSize);
		std::string name = std::to_string((int)i.gCost);//to_string_with_precision(i.gCost, 3); //		std::to_string(i.gCost);
		Graphics::point p = GetCenter(i.r);
		display.DrawText(i.name.c_str(), p, Colors::white, boxSize/2.0f, Graphics::textAlignCenter, Graphics::textBaselineMiddle);
		p.x += 2*boxSize;
		display.DrawText(name.c_str(), p, Colors::black, boxSize/2.0f, Graphics::textAlignCenter, Graphics::textBaselineMiddle);
	}

}

int MyCLHandler(char *argument[], int maxNumArgs)
{
	if (maxNumArgs <= 1)
		return 0;
	strncpy(gDefaultMap, argument[1], 1024);
	return 2;
}


void MyDisplayHandler(unsigned long windowID, tKeyboardModifier mod, char key)
{
	switch (key)
	{
		case 'm':
			GetMVC();
			break;
		case 'r':
			BuildGraph();
			break;
		case 'c':
			CheckGMX();
			break;
			
	}
	
}


bool MyClickHandler(unsigned long , int vp, int windowX, int windowY, point3d loc, tButtonType button, tMouseEventType mType)
{
	if (button != kLeftButton)
		return false;
	if (vp == 1)
	{
		if (mType == kMouseDown)
		{
			downNode = -1;
			upNode = -1;
			for (int x = 0; x < forward.size(); x++)
			{
				if (Graphics::PointInRect(loc, forward[x].r))
				{
					downNode = x;
					break;
				}
			}
		}
		if (mType == kMouseDrag && downNode != -1)
		{
			upNode = -1;
			for (int x = 0; x < backward.size(); x++)
			{
				if (Graphics::PointInRect(loc, backward[x].r))
				{
					upNode = x;
					break;
				}
			}
		}
		if (mType == kMouseUp && downNode != -1)
		{
			forward[downNode].oppositeLine = upNode;
			backward[upNode].oppositeLine = downNode;
			downNode = -1;
			upNode = -1;
		}
	}
	if (vp == 0)
	{
		switch (mType)
		{
			case kMouseDown:
				from = to = FindClosestNode(g, loc)->GetNum(); break;
			case kMouseDrag:
				to = FindClosestNode(g, loc)->GetNum(); break;
			case kMouseUp:
				to = FindClosestNode(g, loc)->GetNum();
				BuildGMX();
				break;
		}
	}
	return true;
}
	
	
double distance(unsigned long node, point3d loc)
{
	double dx = g->GetNode(node)->GetLabelF(GraphSearchConstants::kXCoordinate);
	double dy = g->GetNode(node)->GetLabelF(GraphSearchConstants::kYCoordinate);
	
	double result = sqrt((dx-loc.x)*(dx-loc.x) + (dy-loc.y)*(dy-loc.y));
	result = (int)(result*100);
	return result;
}

double distance(unsigned long n1, unsigned long n2)
{
	double dx1 = g->GetNode(n1)->GetLabelF(GraphSearchConstants::kXCoordinate);
	double dy1 = g->GetNode(n1)->GetLabelF(GraphSearchConstants::kYCoordinate);
	
	double dx2 = g->GetNode(n2)->GetLabelF(GraphSearchConstants::kXCoordinate);
	double dy2 = g->GetNode(n2)->GetLabelF(GraphSearchConstants::kYCoordinate);
	
	double val = sqrt((dx1-dx2)*(dx1-dx2)+(dy1-dy2)*(dy1-dy2));
//	return val;
	val = (int)(val*100);
//	val *= 10;
//	val = (int)val;
//	val /= 10.0;
	return val;
}

node *FindClosestNode(Graph *gr, point3d loc)
{
	if (gr->GetNumNodes() == 0)
		return 0;
	unsigned long best = 0;
	double dist = distance(0, loc);
	for (unsigned long x = 1; x < gr->GetNumNodes(); x++)
	{
		if (fless(distance(x, loc), dist))
		{
			dist = distance(x, loc);
			best = x;
		}
	}
	return gr->GetNode(best);
}

float smallNoise()
{
	float f = random()%11;
	f -= 5;
	f/=5.0; // -1 to +1
	f/=10;
	return f;
}

void BuildGraph()
{
	if (g)
		delete g;
	if (ge)
		delete ge;
	forward.resize(0);
	backward.resize(0);
	from = to = -1;
	
	g = new Graph();
	ge = new GraphEnvironment(g);
	ge->SetDrawEdgeCosts(true);
	ge->SetDrawNodeLabels(true);
	ge->SetIntegerEdgeCosts(true);
	std::string s = "a";
	node *n;
	int gridSize = 3;
	float space = 2.0f/(gridSize+1.0f);
	for (int x = 0; x < gridSize; x++)
	{
		for (int y = 0; y < gridSize; y++)
		{
			g->AddNode(n = new node(s.c_str())); s[0]++;
			n->SetLabelF(GraphSearchConstants::kXCoordinate, -1+space*(x+1)+smallNoise());
			n->SetLabelF(GraphSearchConstants::kYCoordinate, -1+space*(y+1)+smallNoise());
		}
	}
	
	for (int x = 0; x < gridSize-1; x++)
	{
		for (int y = 0; y < gridSize-1; y++)
		{
			int f = x*gridSize+y;
			int t = (x+1)*gridSize+(y);
			g->AddEdge(new edge(f, t, distance(f, t)));
			t = (x)*gridSize+(y+1);
			g->AddEdge(new edge(f, t, distance(f, t)));
		}
	}
	g->AddEdge(new edge(gridSize*gridSize-2, gridSize*gridSize-1, distance(gridSize*gridSize-2, gridSize*gridSize-1)));
}

void BuildGMX()
{
	forward.resize(0);
	backward.resize(0);
	std::vector<graphState> path;

	TemplateAStar<graphState, graphMove, GraphEnvironment> astar;
	astar.SetHeuristic(&searchHeuristic);
	astar.GetPath(ge, from, to, path);
	optimalCost = ge->GetPathLength(path);

	for (int x = 0; x < astar.GetNumItems(); x++)
	{
		auto i = astar.GetItem(x);
		if (i.where == kClosedList && fless(i.g+i.h, optimalCost))
		{
			forward.push_back({g->GetNode(i.data)->GetName(), (float)i.g, -1, {-0.5, -1, 0.5, 1}, false, false, i.data});
		}
	}

	astar.GetPath(ge, to, from, path);
	for (int x = 0; x < astar.GetNumItems(); x++)
	{
		auto i = astar.GetItem(x);
//		printf("%s has g-cost %1.2f (h:%1.2f/%1.2f) f-cost %1.2f\n", g->GetNode(i.data)->GetName(), i.g, i.h, ge->HCost(i.data, from), i.g+i.h);
		if (i.where == kClosedList && fless(i.g+i.h, optimalCost))
		{
			backward.push_back({g->GetNode(i.data)->GetName(), (float)i.g, -1, {-0.5, -1, 0.5, 1}, false, false, i.data});
		}
	}

	auto func = [](const GMXNode &x, const GMXNode &y) {
		return x.gCost < y.gCost;
	};
	std::sort(forward.begin(), forward.end(), func);
	std::sort(backward.begin(), backward.end(), func);

	float boxSize = 1.0f/(std::max(std::max(forward.size(), backward.size()), (size_t)8)+2.0f);
//	float offset = 0;
	for (int x = 0; x < forward.size(); x++)
	{
		forward[x].r = {-0.5f, -1.f+boxSize*2*x+boxSize, -0.5f+boxSize, -1.f+boxSize*(2*x+1)+boxSize};
//		offset += boxSize;
	}
//	offset = std::max(offset, )
	for (int x = 0; x < backward.size(); x++)
	{
		backward[x].r = {0.5f, 1.f-(boxSize*(2*x+1)+boxSize), 0.5f+boxSize, 1.f-(boxSize*(2*x)+boxSize)};
	}

}

void CheckGMX()
{
	bool foundWrong = false;
	for (int x = 0; x < forward.size(); x++)
	{
		forward[x].checkedWrong = false;
		// case one - no line here needed
		if (forward[x].oppositeLine == -1)
		{
			if (forward[x].gCost < optimalCost)
			{
				forward[x].checkedWrong = true;
				foundWrong = true;
			}
		}
		// case two, line is too high
		else if (forward[x].gCost + backward[forward[x].oppositeLine].gCost >= optimalCost)
		{
			printf("Line from f:%d to b:%d has too high cost %1.2f\n", x, forward[x].oppositeLine,
				   forward[x].gCost + backward[forward[x].oppositeLine].gCost);
			forward[x].checkedWrong = true;
			foundWrong = true;
		}
		// case three, line too low
		else if (forward[x].oppositeLine+1 < backward.size() &&
				 forward[x].gCost + backward[forward[x].oppositeLine+1].gCost < optimalCost)
		{
			printf("Missing a line from f:%d b:%d with cost %1.2f\n", x, forward[x].oppositeLine+1,
				   forward[x].gCost + backward[forward[x].oppositeLine+1].gCost);
			forward[x].checkedWrong = true;
			foundWrong = true;
		}
	}
	if (foundWrong)
		submitTextToBuffer("Sorry; GMX is not correct; try again");
	else
		submitTextToBuffer("GMX is correct!");
}

void GetMVC()
{
	CheckGMX();
	for (auto &i : forward)
		if (i.checkedWrong)
		{
			submitTextToBuffer("Cannot compute MVC on incorrect GMX graph");
			return;
		}
	for (int x = 0; x < forward.size(); x++)
		forward[x].MVC = false;
	for (int x = 0; x < backward.size(); x++)
		backward[x].MVC = false;
	
	// really inefficient, but easy to get correct
	int bestSize = backward.size();
	int bestSplit = -1;
	for (int x = 0; x < forward.size(); x++)
	{
		int totalSize = x+1;
		if (x+1 < forward.size())
		{
			for (int y = 0; y < backward.size(); y++)
			{
				if (fless(forward[x+1].gCost+backward[y].gCost, optimalCost))
					totalSize++;
			}
		}
		if (totalSize < bestSize)
		{
			bestSize = totalSize;
			bestSplit = x;
		}
	}
	if (bestSplit == -1)
	{
		for (auto &i : backward)
			i.MVC = true;
		return;
	}

	for (int x = 0; x <= bestSplit; x++)
			forward[x].MVC = true;
	if (bestSplit+1 >= forward.size()) // purely forward
		return;
	for (int x = 0; x < backward.size(); x++)
	{
		if (fless(forward[bestSplit+1].gCost+backward[x].gCost, optimalCost))
			backward[x].MVC = true;
	}
}

