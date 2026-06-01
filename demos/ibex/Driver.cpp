/*
 * $Id: sample.cpp,v 1.23 2006/11/01 23:33:56 nathanst Exp $
 *
 *  sample.cpp
 *  hog
 *
 *  Created by Nathan Sturtevant on 5/31/05.
 *  Copyright 2005 Nathan Sturtevant, University of Alberta. All rights reserved.
 *
 * This file is part of HOG.
 *
 */

#include "Common.h"
#include "Driver.h"
#include "GraphEnvironment.h"
#include "TextOverlay.h"
#include <string>
#include "MNPuzzle.h"
#include "IncrementalIDA.h"
#include "IncrementalBTS.h"
#include "TemplateAStar.h"
#include "PermutationPDB.h"
#include "LexPermutationPDB.h"
#include "SVGUtil.h"

IncrementalIDA<graphState, graphMove> ida;
IncrementalBTS<graphState, graphMove> ibex;
bool useIDA = true;
bool layoutFullTree = false;
TemplateAStar<graphState, graphMove, GraphEnvironment> astar;

enum mode {
	kAddNodes,
	kAddEdges,
	kMoveNodes,
	kFindPath
};

enum view {
	kTreeView = 0,
	kTextView = 1,
	kStateView = 2
};

bool recording = false;
bool running = false;
bool stopOnStageChange = false;
double rate = 1.0/32.0;
int displayPrecision = 0;

MNPuzzle<3, 2> mnp;
MNPuzzleState<3, 2> start;
MNPuzzleState<3, 2> goal;
//MNPuzzleState<3, 2> t1;
//MNPuzzleState<3, 2> t2;
//MNPuzzleState<3, 2> t3;
//MNPuzzleState<3, 2> t4;
std::vector<slideDir> acts;

Graph *g = 0;
GraphEnvironment *ge;
graphState from=mnp.GetMaxStateHash()-1;
graphState hit = -1;
std::vector<graphState> path;
std::vector<graphState> currentPath;
std::vector<graphState> lastPath;

int colorScheme = 4;// 10, 7 & 4 are reasonable

TextOverlay te(15);
double timer = 0;

double distance(unsigned long n1, unsigned long n2);

int whichHeuristic = 1;
bool compressedPDBs = false;

class GraphDistHeuristic : public Heuristic<graphState> {
public:
	GraphDistHeuristic(double w = 1.0)
	:weight(w) {}
	double HCost(const graphState &a, const graphState &b) const
	{
		MNPuzzleState<3, 2> t1, t2;
		mnp.GetStateFromHash(t1, g->GetNode(a)->GetNum());
		mnp.GetStateFromHash(t2, g->GetNode(b)->GetNum());
		//return int(mnp.HCost(t1, t2));
		int md = int(mnp.HCost(t1, t2));
		
		if (((whichHeuristic>>0)&1) == 0) // MD
			md = 0;
		return weight*md;
	}
private:
	MNPuzzle<3, 2> mnp;
	double weight;
};

GraphDistHeuristic searchHeuristic;
//GraphDistHeuristic wh(5);

int main(int argc, char* argv[])
{
	InstallHandlers();
	RunHOGGUI(argc, argv, 1600, 800);
	return 0;
}

/**
 * Allows you to install any keyboard handlers needed for program interaction.
 */
void InstallHandlers()
{
	InstallKeyboardHandler(MyDisplayHandler, "Record", "Record a movie", kAnyModifier, 'r');
//	InstallKeyboardHandler(MyDisplayHandler, "Choose Heuristic", "Choose which heuristic is used", kAnyModifier, '0', '4');
	//	InstallKeyboardHandler(MyDisplayHandler, "Cycle Abs. Display", "Cycle which group abstraction is drawn", kAnyModifier, '\t');
	InstallKeyboardHandler(MyDisplayHandler, "Pause Simulation", "Pause simulation execution.", kNoModifier, 'p');
	InstallKeyboardHandler(MyDisplayHandler, "Go", "Go until the stage changes.", kNoModifier, 'g');
	InstallKeyboardHandler(MyDisplayHandler, "Step Simulation", "If the simulation is paused, step forward .1 sec.", kAnyModifier, 'o');
//	InstallKeyboardHandler(MyDisplayHandler, "Color", "Increase color schema", kAnyModifier, '}');
//	InstallKeyboardHandler(MyDisplayHandler, "Color", "Decrease color schema", kAnyModifier, '{');
	
	InstallKeyboardHandler(MyDisplayHandler, "Faster", "Speed up simulation", kAnyModifier, '+');
	InstallKeyboardHandler(MyDisplayHandler, "Slower", "Slow down simulation", kAnyModifier, '-');
	
//	InstallKeyboardHandler(MyDisplayHandler, "Compress", "Compress PDBs", kAnyModifier, 'c');
	
	
	InstallKeyboardHandler(MyDisplayHandler, "Problem+", "Increase start rank and reset graph", kAnyModifier, ']');
	InstallKeyboardHandler(MyDisplayHandler, "Problem-", "Decrease start rank and reset graph", kAnyModifier, '[');
	InstallKeyboardHandler(MyDisplayHandler, "Clear", "Clear graph", kAnyModifier, '|');
	//	InstallKeyboardHandler(MyDisplayHandler, "Help", "Draw help", kAnyModifier, '?');
	InstallKeyboardHandler(MyDisplayHandler, "Heuristic", "Toggle Heuristic", kAnyModifier, 'h');
	InstallKeyboardHandler(MyDisplayHandler, "Unit Cost", "Toggle Unit Costs", kAnyModifier, 'u');
	InstallKeyboardHandler(MyDisplayHandler, "IBEX/IDA", "Toggle Unit IDA*/IBEX", kAnyModifier, 'i');
	InstallKeyboardHandler(MyDisplayHandler, "Tree", "Layout full tree", kAnyModifier, 'f');
	//InstallKeyboardHandler(MyDisplayHandler, "Weight", "Toggle Dijkstra & A*", kAnyModifier, 'w');
	//InstallKeyboardHandler(DefaultGraph, "Default", "Build Deafult Graph", kAnyModifier, 'a', 'd');
	
	//InstallCommandLineHandler(MyCLHandler, "-map", "-map filename", "Selects the default map to be loaded.");
	
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
		printf("Window %ld created\n", windowID);
		
		//glClearColor(0.99, 0.99, 0.99, 1.0);
		InstallFrameHandler(MyFrameHandler, windowID, 0);
		
		//	kTreeView = 0,
		//	kTextView = 1,
		//	kStateView = 2,
		
		ReinitViewports(windowID, {-1, -1, 0, 1}, kScaleToSquare); // kTreeView
		//AddViewport(windowID, {0, -1, 1, 0}, kScaleToSquare); // kTextView
		AddViewport(windowID, {0, -1, 1, 1}, kScaleToSquare); // kTextView
		AddViewport(windowID, {0, 0, 1, 1}, kScaleToSquare); // kStateView
		
		g = new Graph();
		ge = new GraphEnvironment(g);
		ge->SetDrawNodeLabels(true);
		
		//mnp.SetWeighted(kUnitPlusFrac);
		BuildGraphFromPuzzle();
		ida.InitializeSearch(ge, from, 0, &searchHeuristic, path);
		ibex.InitializeSearch(ge, from, 0, &searchHeuristic, path);
		RedrawTextDisplay();
		ge->SetNodeScale(20);
	}
}

int frameCnt = 0;

void DrawEdgesInSearch(Graphics::Display &display)
{
	double limit = g->GetNode(0)->GetLabelF(GraphSearchConstants::kTemporaryLabel);
	for (int x = 0; x < g->GetNumEdges(); x++)
	{
		edge *e = g->GetEdge(x);
		double fromf = g->GetNode(e->getFrom())->GetLabelF(GraphSearchConstants::kTemporaryLabel);
		fromf += searchHeuristic.HCost(e->getFrom(), 0);
		double tof = g->GetNode(e->getTo())->GetLabelF(GraphSearchConstants::kTemporaryLabel);
		tof += searchHeuristic.HCost(e->getTo(), 0);
		double m = std::min(fromf, tof);
		if (flesseq(m, limit))
		{
			// regular edge between parent/child
			if (g->GetNode(e->getTo())->GetLabelL(GraphSearchConstants::kFirstData+1) == e->getFrom() ||
				g->GetNode(e->getFrom())->GetLabelL(GraphSearchConstants::kFirstData+1) == e->getTo())
			{
				ge->SetColor(Colors::black);
				ge->DrawLine(display, e->getFrom(), e->getTo(), 1);
			}
			else {
				ge->SetColor(Colors::gray);
				ge->DrawLine(display, e->getFrom(), e->getTo(), 0.75);
			}
		}
	}
}

void DrawEdgesInIteration(Graphics::Display &display)
{
	for (int x = 0; x < g->GetNumNodes(); x++)
	{
		node *n = g->GetNode(x);
		edge_iterator ei = n->getEdgeIter();
		for (edge *e = n->edgeIterNext(ei); e; e = n->edgeIterNext(ei))
		{
			double fromf = g->GetNode(e->getFrom())->GetLabelF(GraphSearchConstants::kTemporaryLabel);
			fromf += searchHeuristic.HCost(e->getFrom(), 0);
			double tof = g->GetNode(e->getTo())->GetLabelF(GraphSearchConstants::kTemporaryLabel);
			tof += searchHeuristic.HCost(e->getTo(), 0);
			double m = std::max(fromf, tof);
			if (flesseq(m, useIDA?ida.GetCurrentFLimit():ibex.GetCurrentFLimit()))
			{
				ge->SetColor(Colors::lighterblue);
				ge->DrawLine(display, e->getFrom(), e->getTo(), 40);
				
			}
		}
	}
	
}

void MyFrameHandler(unsigned long windowID, unsigned int viewport, void *)
{
	Graphics::Display &display = getCurrentContext()->display;
	//	display.FrameRect({-1, -1, 1, 1}, Colors::white, 1.0);
	
	
	if (viewport == kTreeView)
	{
		if (timer >= 1 && running)
		{
			MyDisplayHandler(windowID, kNoModifier, 'o');
		}
		
		display.FillRect({-1, -1, 1, 1}, Colors::white);
		if (ge == 0 || g == 0)
			return;
		DrawEdgesInIteration(display);
		DrawEdgesInSearch(display);
		// draws edges but not states
		//		ge->SetColor(0.5, 0.5, 1.0);
		//		ge->SetColor(0.95, 0.95, 0.95);
		//		ge->Draw(display);
		
		// draw in bold the search path
		if (currentPath.size() > 0)
		{
			ge->SetColor(Colors::black);
			for (int x = 1; x < currentPath.size(); x++)
			{
				ge->DrawLine(display, currentPath[x-1], currentPath[x], 5);
				ge->Draw(display, currentPath[x-1]);
				//ge->GLDrawLine(path[x-1], path[x]);
			}
		}
		
		
		ge->SetColor(0.75, 0.75, 1.0);
		double startf = g->GetNode(from)->GetLabelF(GraphSearchConstants::kTemporaryLabel)+searchHeuristic.HCost(from, 0);
		double goalf = g->GetNode(0)->GetLabelF(GraphSearchConstants::kTemporaryLabel);
		for (int x = 0; x < g->GetNumNodes(); x++)
		{
			double cost = g->GetNode(x)->GetLabelF(GraphSearchConstants::kTemporaryLabel)+searchHeuristic.HCost(x, 0);
			rgbColor c = Colors::GetColor(cost, startf, goalf, colorScheme);
			if (hit == x)
			{
				ge->SetColor(c.r, c.g, c.b);
				//ge->SetColor(1, 1, 1);
				ge->SetNodeScale(30);
				ge->Draw(display, x);
				ge->SetNodeScale(20);
			}
			else if (!fgreater(cost, goalf))
			{
				ge->SetColor(c.r, c.g, c.b);
				ge->Draw(display, x);
			}
		}
		
		// draw search states in bold too
		if (currentPath.size() > 0)
		{
			ge->SetColor(Colors::black);
			for (int x = 0; x < currentPath.size(); x++)
			{
				ge->Draw(display, currentPath[x]);
			}
		}
		
		// draw solution
		if (path.size() > 0)
		{
			//printf("Drawing edge on solution path\n");
			ge->SetColor(0, 1, 0);
			for (int x = 1; x < path.size(); x++)
			{
				ge->DrawLine(display, path[x-1], path[x], 10);
				//ge->GLDrawLine(path[x-1], path[x]);
			}
		}
		
		ge->SetColor(1.0, 0, 0);
		if (useIDA)
			ida.Draw(display);
		else {
			ibex.Draw(display);
			std::string tmp = "f-limit: "+ibex.fEquation;
			//tmp += ibex.GetCurrentFLimit()!=DBL_MAX?to_string_with_precision(ibex.GetCurrentFLimit(), 2):"∞";
			display.DrawText(tmp.c_str(), {-1+0.05, -0.9}, Colors::black, 0.05, Graphics::textAlignLeft);

			uint64_t small, large;
			ibex.GetNodeInterval(small, large);
			tmp = "nodes: ["+std::to_string(small) + ","+ (large==ibex.infiniteWorkBound?"∞":std::to_string(large)) +"]";
			display.DrawText(tmp.c_str(), {-1+0.05, -0.84}, Colors::black, 0.05, Graphics::textAlignLeft);


			tmp = "expand: "+std::to_string(ibex.GetIterationNodesExpanded());
			display.DrawText(tmp.c_str(), {-1+0.05, -0.78}, Colors::black, 0.05, Graphics::textAlignLeft);

		}
	}
	
	if (viewport == kTextView)
	{
		display.FillRect({-1, -1, 1, 1}, Colors::black);
		te.Draw(display);
	}
	if (viewport == kStateView)
	{
		MNPuzzleState<3, 2> t1, t2;
		
		display.FillRect({-1, -1, 1, 1}, Colors::black);
		if (hit != -1) // node selected
		{
			mnp.GetStateFromHash(t1, hit);
			mnp.Draw(display, t1);
		}
		else { // no node selected
			if (timer < 1)
				timer += rate;
			if (timer < 1 && currentPath.size() > 0 && lastPath.size() > 0)
			{
				mnp.GetStateFromHash(t1, currentPath.back());
				mnp.GetStateFromHash(t2, lastPath.back());
				mnp.Draw(display, t1, t2, timer);
			}
			else // no animation in progres
			{
				auto n = useIDA?ida.GetCurrentState():ibex.GetCurrentState();
				mnp.GetStateFromHash(t1, n);
				mnp.Draw(display, t1);
			}
		}
	}
	
	if (recording && viewport == GetNumPorts(windowID)-1)
	{
		char fname[255];
		sprintf(fname, "/Users/nathanst/Movies/tmp/1/ibex-%d%d%d%d%d.svg",
				(frameCnt/10000)%10, (frameCnt/1000)%10, (frameCnt/100)%10, (frameCnt/10)%10, frameCnt%10);
		MakeSVG(display, fname, 1024, 1024, 0);
		sprintf(fname, "/Users/nathanst/Movies/tmp/2/ibex-%d%d%d%d%d.svg",
				(frameCnt/1000)%10, (frameCnt/100)%10, (frameCnt/10)%10, frameCnt%10);
		MakeSVG(display, fname, 500, 500, kStateView);
		//		SaveScreenshot(windowID, fname);
		//printf("Saved %s\n", fname);
		frameCnt++;
		if (path.size() == 0)
		{
		}
		else {
			recording = false;
		}
	}
	
	return;
}

int MyCLHandler(char *argument[], int maxNumArgs)
{
	if (maxNumArgs <= 1)
		return 0;
	strncpy(gDefaultMap, argument[1], 1024);
	return 2;
}

void RedrawTextDisplay()
{
	te.Clear();
	std::string s;
	
	if (useIDA)
	{
		s = "IDA* Curr f-bound: ";
		s += to_string_with_precision(ida.GetCurrentFLimit(), displayPrecision);
		//te.AddLine(s.c_str());
		s += " next f: ";
		s += to_string_with_precision(ida.GetNextFLimit(), displayPrecision);
		te.AddLine(s.c_str());
		s = "Curr state f-cost: ";
		if (currentPath.size() > 0)
		{
			s += to_string_with_precision((searchHeuristic.HCost(currentPath.back(), 0)+ge->GetPathLength(currentPath)), displayPrecision);
			s += " (g: "+to_string_with_precision(ge->GetPathLength(currentPath), displayPrecision);
			s += " h: "+to_string_with_precision(searchHeuristic.HCost(currentPath.back(), 0), displayPrecision)+")";
		}
		else if (hit != -1)
		{
			s += to_string_with_precision((searchHeuristic.HCost(hit, 0))+g->GetNode(hit)->GetLabelF(GraphSearchConstants::kTemporaryLabel), displayPrecision);
			s += " (g: ";
			s += to_string_with_precision(g->GetNode(hit)->GetLabelF(GraphSearchConstants::kTemporaryLabel), displayPrecision);
			s += " h: "+to_string_with_precision(searchHeuristic.HCost(hit, 0), displayPrecision)+")";
		}
		else {
			s += "-";
		}
		te.AddLine(s.c_str());
		te.AddLine("");
		
		s = "Nodes exp: ";
		s += std::to_string(ida.GetNodesExpanded());
		s += " New last iter: ";
		s += std::to_string(ida.GetNewNodesLastIteration());
		te.AddLine(s.c_str());
		{
			static double bnd = ida.GetCurrentFLimit();
			if (!fequal(ida.GetCurrentFLimit(), bnd))
			{
				bnd = ida.GetCurrentFLimit();
				printf("Last iteration: %lld\n", ida.GetNewNodesLastIteration());
			}
		}
	}
	else {
		double l, u;
		uint64_t small, large;
		s = "IBEX/BTS - "+ibex.stage;
		te.AddLine(s.c_str());
		ibex.GetGlobalCostInterval(l, u);
		s = "f: ["+((l!=DBL_MAX)?to_string_with_precision(l, 2):"∞");
		u = ibex.GetCurrentFLimit();
		s += ","+((u!=DBL_MAX)?to_string_with_precision(u, 2):"∞");//+"]";
		ibex.GetGlobalCostInterval(l, u);
		s += ","+((u!=DBL_MAX)?to_string_with_precision(u, 2):"∞")+")";
		te.AddLine(s.c_str());

//		te.AddLine(s.c_str());

		ibex.GetNodeInterval(small, large);
		s = "node window ["+(std::to_string(small))+", "+((large!=ibex.infiniteWorkBound)?std::to_string(large):"∞")+"]";
		te.AddLine(s.c_str());
		s = "Nodes exp: ";
		s += std::to_string(ibex.GetIterationNodesExpanded());
		s += " (iter) ";
		s += std::to_string(ibex.GetNodesExpanded());
		s += " (tot)";
		te.AddLine(s.c_str());

		s = "Curr f: ";
		if (currentPath.size() > 0)
		{
			s += to_string_with_precision((searchHeuristic.HCost(currentPath.back(), 0)+ge->GetPathLength(currentPath)), displayPrecision);
			s += " (g: "+to_string_with_precision(ge->GetPathLength(currentPath), displayPrecision);
			s += " h: "+to_string_with_precision(searchHeuristic.HCost(currentPath.back(), 0), displayPrecision)+")";
		}
		else if (hit != -1)
		{
			s += to_string_with_precision((searchHeuristic.HCost(hit, 0))+g->GetNode(hit)->GetLabelF(GraphSearchConstants::kTemporaryLabel), displayPrecision);
			s += " (g: ";
			s += to_string_with_precision(g->GetNode(hit)->GetLabelF(GraphSearchConstants::kTemporaryLabel), displayPrecision);
			s += " h: "+to_string_with_precision(searchHeuristic.HCost(hit, 0), displayPrecision)+")";
		}
		else {
			s += "-";
		}
		te.AddLine(s.c_str());
	}

	s = "Heuristic: ";
	if (whichHeuristic&1)
		s += "MD ";
	if (whichHeuristic&2)
		s += "P1 ";
	if (whichHeuristic&4)
		s += "P2 ";
	if (whichHeuristic&8)
		s += "P3 ";
	te.AddLine(s.c_str());
}


void MyDisplayHandler(unsigned long windowID, tKeyboardModifier mod, char key)
{
	switch (key)
	{
		case 'f':
			layoutFullTree = !layoutFullTree;
			MyDisplayHandler(windowID, kNoModifier, '|');
			break;
		case 'i':
			useIDA = !useIDA;
			MyDisplayHandler(windowID, kNoModifier, '|');
			break;
		case 'u':
			if (mnp.GetWeighted() == kUnitWeight)
			{
				mnp.SetWeighted(kUnitPlusFrac);
				displayPrecision = 2;
				submitTextToBuffer("Using weighted puzzle");
			}
			else {
				mnp.SetWeighted(kUnitWeight);
				displayPrecision = 0;
				submitTextToBuffer("Using unweighted puzzle");
			}
			MyDisplayHandler(windowID, kNoModifier, '|');
			break;
		case 'h': whichHeuristic = (whichHeuristic+1)%16; break;
		case '+': rate *= 2; if (rate > 1) rate = 1; break;
		case '-': rate /= 2; break;
		case '{':
		{
			colorScheme = (colorScheme+20)%21;
			printf("Color scheme %d\n", colorScheme);
			break;
		}
		case '}':
		{
			colorScheme = (colorScheme+1)%21;
			printf("Color scheme %d\n", colorScheme);
			break;
		}
		case ']':
		{
			from = (from+1)%mnp.GetMaxStateHash();
			printf("Root: %lu\n", from);
			MyDisplayHandler(windowID, kNoModifier, '|');
//			BuildGraphFromPuzzle();
//			ida.InitializeSearch(ge, from, 0, &h, path);
//			ibex.InitializeSearch(ge, from, 0, &h, path);
//			RedrawTextDisplay();
//
//			currentPath.clear();
//			path.clear();
//			lastPath.clear();
			//MyDisplayHandler(windowID, kNoModifier, 'o');
		}
			break;
		case '[':
		{
			from = (from+mnp.GetMaxStateHash()-1)%mnp.GetMaxStateHash();
			printf("Root: %lu\n", from);
			MyDisplayHandler(windowID, kNoModifier, '|');
//			BuildGraphFromPuzzle();
//			ida.InitializeSearch(ge, from, 0, &h, path);
//			ibex.InitializeSearch(ge, from, 0, &h, path);
//			RedrawTextDisplay();
//
//			currentPath.clear();
//			path.clear();
//			lastPath.clear();
			//MyDisplayHandler(windowID, kNoModifier, 'o');
		}
			break;
		case '|':
		{
			ida.InitializeSearch(ge, from, 0, &searchHeuristic, path);
			ibex.InitializeSearch(ge, from, 0, &searchHeuristic, path);
			RedrawTextDisplay();
			
			BuildGraphFromPuzzle();
			path.resize(0);
			currentPath.clear();
			path.clear();
			lastPath.clear();
			running = false;
			submitTextToBuffer("");
		}
			break;
		case 'r':
			recording = !recording;
			if (running == false)
			{
				running = true;
				timer = 0;
			}
			//			running = true;
			//			submitTextToBuffer("Running automatically");
			break;
		case '0':
		{
			whichHeuristic = 0;
			MyDisplayHandler(windowID, kNoModifier, '|');
//			ida.InitializeSearch(ge, from, 0, &h, path);
//			ibex.InitializeSearch(ge, from, 0, &h, path);
//			RedrawTextDisplay();
//
//			currentPath.clear();
//			path.clear();
//			lastPath.clear();
//			RedrawTextDisplay();
		}
			break;
		case '1':
		{
			whichHeuristic ^= 0x1;
			MyDisplayHandler(windowID, kNoModifier, '|');
//			ida.InitializeSearch(ge, from, 0, &h, path);
//			ibex.InitializeSearch(ge, from, 0, &h, path);
//			RedrawTextDisplay();
//
//			currentPath.clear();
//			path.clear();
//			lastPath.clear();
//			RedrawTextDisplay();
		}
			break;
		case '2':
		{
			whichHeuristic ^= 0x2;
			MyDisplayHandler(windowID, kNoModifier, '|');
//			ida.InitializeSearch(ge, from, 0, &h, path);
//			ibex.InitializeSearch(ge, from, 0, &h, path);
//			RedrawTextDisplay();
//
//			currentPath.clear();
//			path.clear();
//			lastPath.clear();
//			RedrawTextDisplay();
		}
			break;
			
		case '3':
		{
			whichHeuristic ^= 0x4;
			MyDisplayHandler(windowID, kNoModifier, '|');
//			ida.InitializeSearch(ge, from, 0, &h, path);
//			ibex.InitializeSearch(ge, from, 0, &h, path);
//			RedrawTextDisplay();
//
//			currentPath.clear();
//			path.clear();
//			lastPath.clear();
//			RedrawTextDisplay();
		}
			break;
			
		case '4':
		{
			whichHeuristic ^= 0x8;
			MyDisplayHandler(windowID, kNoModifier, '|');
//			ida.InitializeSearch(ge, from, 0, &h, path);
//			ibex.InitializeSearch(ge, from, 0, &h, path);
//			RedrawTextDisplay();
//
//			currentPath.clear();
//			path.clear();
//			lastPath.clear();
//			RedrawTextDisplay();
		}
			break;
			
			//		case '\t':
			//			if (mod != kShiftDown)
			//				SetActivePort(windowID, (GetActivePort(windowID)+1)%GetNumPorts(windowID));
			//			else
			//			{
			//				SetNumPorts(windowID, 1+(GetNumPorts(windowID)%MAXPORTS));
			//			}
			//			break;
		case 'p':
			running = !running;
			if (running) {
				submitTextToBuffer("Running automatically");
				stopOnStageChange = false;
			}
			else
				submitTextToBuffer("");
			break;
		case 'g':
			stopOnStageChange = true;
			running = true;
			break;
		case 'o':
		{
			if (useIDA)
				ida.GetCurrentPath(lastPath);
			else
				ibex.GetCurrentPath(lastPath);
			auto i = ibex.currStage;
			bool done = useIDA?ida.DoSingleSearchStep(path):ibex.DoSingleSearchStep(path);
			if (i != ibex.currStage && stopOnStageChange)
			{
				///stopOnStageChange = false;
				running = false;
			}
			if (done && running)
				running = false;
			if (useIDA)
				ida.GetCurrentPath(currentPath);
			else
				ibex.GetCurrentPath(currentPath);
			timer = 0;
			RedrawTextDisplay();
		}
			break;
		default:
			break;
	}
	
}

#pragma mark - build graph -

void BuildGraphFromPuzzle()
{
	uint64_t root = from;
	
	g->Reset();
	// Get all nodes
	for (uint64_t x = 0; x < mnp.GetMaxStateHash(); x++)
	{
		node *n;
		g->AddNode(n = new node(""));
		// not seen
		n->SetLabelL(GraphSearchConstants::kTemporaryLabel, -1);
		
		// far off screen
		n->SetLabelF(GraphSearchConstants::kXCoordinate, 2.0*(x+1)/(mnp.GetMaxStateHash()+1)-1);
		n->SetLabelF(GraphSearchConstants::kYCoordinate, 1);
		n->SetLabelF(GraphSearchConstants::kZCoordinate, 0);

		// not a leaf node
		n->SetLabelL(GraphSearchConstants::kFirstData, 0);
	}
	// Get all edges
	for (uint64_t x = 0; x < mnp.GetMaxStateHash(); x++)
	{
		mnp.GetStateFromHash(start, x);
		mnp.GetActions(start, acts);
		for (auto a : acts)
		{
			double cost = mnp.GCost(start, a);
			mnp.ApplyAction(start, a);
			uint64_t child = mnp.GetStateHash(start);
			if (g->FindEdge(x, child) == 0)
			{
				g->AddEdge(new edge(x, child, cost));
				//printf("Adding edge between %d and %d [%f]\n", x, child, cost);
			}
			mnp.UndoAction(start, a);
		}
	}
	
	// find distance to all states
	astar.SetStopAfterGoal(false);
	std::vector<graphState> tmp;
	astar.GetPath(ge, root, root, tmp);
	double solutionCost;
	astar.GetClosedListGCost(0, solutionCost);
	printf("Solution cost: %1.2f\n", solutionCost);

	//
	
	// 1. Do cost-limited DFS to get ordering of states
	std::deque<graphState> queue;
	std::vector<int> orderedLeaves;
	queue.push_back(root);
	g->GetNode(root)->SetLabelL(GraphSearchConstants::kTemporaryLabel, 1);
	double maximumGCost = 0;
	g->GetNode(root)->SetLabelL(GraphSearchConstants::kFirstData+1, root); // parentid

//	printf("Leaf order [root %d]: \n", root);
	while (queue.size() > 0)
	{
		graphState next = queue.back();
		queue.pop_back();
		//printf("[%d] ", next);
		double g1;
		
		astar.GetClosedListGCost(next, g1);
		// TODO: Option to make these leaves
		if (layoutFullTree == false)
		{
			if (fgreater(g1+searchHeuristic.HCost(next, 0), solutionCost)) // leaf node
			{
				//			printf("%d exceeds cost - is leaf\n", next);
				g->GetNode(next)->SetLabelL(GraphSearchConstants::kFirstData, 1);
				maximumGCost = std::max(maximumGCost, g1);
				orderedLeaves.push_back(next);
				continue;
			}
		}

		node *n = g->GetNode(next);
		
		edge_iterator ei = n->getEdgeIter();
		for (edge *e = n->edgeIterNext(ei); e; e = n->edgeIterNext(ei))
		{
			unsigned int neighbor = e->getFrom();
			if (neighbor == n->GetNum())
				neighbor = e->getTo();
			if (g->GetNode(neighbor)->GetLabelL(GraphSearchConstants::kTemporaryLabel) == 1) // already seen - discard
				continue;

			double g2;
			astar.GetClosedListGCost(neighbor, g2);
			if (fgreater(g2, g1))
			{
				g->GetNode(neighbor)->SetLabelL(GraphSearchConstants::kFirstData+1, next); // parentid
				g->GetNode(neighbor)->SetLabelL(GraphSearchConstants::kTemporaryLabel, 1);
				tmp.push_back(neighbor);
			}
		}
		if (tmp.size() == 0)
		{
//			printf("%d has no children - is leaf\n", next);
			maximumGCost = std::max(maximumGCost, g1);
			g->GetNode(next)->SetLabelL(GraphSearchConstants::kFirstData, 1);
			orderedLeaves.push_back(next);
		}
		while (tmp.size() > 0)
		{
			queue.push_back(tmp.back());
			tmp.pop_back();
		}
	}
//	printf("%d leaves found\n", orderedLeaves.size());

//	for (int x = 0; x < g->GetNumNodes(); x++)
//	{
//		node *n = g->GetNode(x);
//		if (n->GetLabelL(GraphSearchConstants::kFirstData) == 1)
//		{
//			graphState s = x;
//			astar.ExtractPathToStart(s, tmp);
////			printf("[%d] ", x);
//			for (auto t : tmp)
//			{
////				printf("%d ", t);
//			}
////			printf("\n");
//		}
//	}
//	printf("%d reduced leaf nodes; maximum g: %1.2f\n", orderedLeaves.size(), maximumGCost);


	// 1. leaves are placed at their g-cost (y) and order (x)
	for (int x = 0; x < orderedLeaves.size(); x++)
	{
		node *n = g->GetNode(orderedLeaves[x]);
		double depth;
		astar.GetClosedListGCost(orderedLeaves[x], depth);
		
		n->SetLabelF(GraphSearchConstants::kXCoordinate,
					 ((2.0*(x+1.0))/(orderedLeaves.size()+1.0)-1.0)*0.9);
		n->SetLabelF(GraphSearchConstants::kYCoordinate, (-1.0+2.0*((double)depth/maximumGCost))*0.9);
		n->SetLabelF(GraphSearchConstants::kZCoordinate, 0);
//		printf("Put leaf %d [g: %f h: %f] at (%f, %f, %f)\n", orderedLeaves[x], depth, h.HCost(orderedLeaves[x], 0),
//			   n->GetLabelF(GraphSearchConstants::kXCoordinate),
//			   n->GetLabelF(GraphSearchConstants::kYCoordinate),
//			   n->GetLabelF(GraphSearchConstants::kZCoordinate)
//			   );
	}
	
	// 2. parents are distributed at the average y of their children [but only the children that recognize them as parents]
	std::vector<int> currLayer = orderedLeaves;
	std::vector<int> nextLayer;
	std::vector<int> allChildren;
	while (currLayer.size() > 0)
	{
		for (int x = 0; x < currLayer.size(); x++)
		{
			int parent = g->GetNode(currLayer[x])->GetLabelL(GraphSearchConstants::kFirstData+1);
			//int parent = astar.GetParent(currLayer[x]);
			bool ready = true;

			// are all children set?
			node *n = g->GetNode(parent);
			allChildren.resize(0);
			
			// already handled parent
			if (n->GetLabelL(GraphSearchConstants::kFirstData) == 1)
				continue;
			
			double parentCost;
			astar.GetClosedListGCost(parent, parentCost);
			edge_iterator ei = n->getEdgeIter();
			for (edge *e = n->edgeIterNext(ei); e; e = n->edgeIterNext(ei))
			{
				unsigned int neighbor = e->getFrom();
				if (neighbor == n->GetNum())
					neighbor = e->getTo();
				double childCost;
				astar.GetClosedListGCost(neighbor, childCost);

				if (fgreater(childCost, parentCost)) // is actually a child, not a parent
				{
					if (g->GetNode(neighbor)->GetLabelL(GraphSearchConstants::kFirstData+1) != parent)
						continue;
//					if (g->GetNode(currLayer[x])->GetLabelL(GraphSearchConstants::kFirstData+1) != parent)
//						//if (astar.GetParent(neighbor) != parent)
//						continue;
					// is a child, and we are the parent; has its location been set?
					if (g->GetNode(neighbor)->GetLabelL(GraphSearchConstants::kFirstData) != 1)
					{
//						printf("%d can't be handled because child %d isn't set\n", parent, neighbor);
						ready = false;
						nextLayer.push_back(currLayer[x]); // try again later
						break;
					}
					else {
						allChildren.push_back(neighbor);
					}
				}
			}
			if (ready)
			{
//				printf("%d at the average of ", n->GetNum());
				double avg = 0;
				for (int y = 0; y < allChildren.size(); y++)
				{
					avg += g->GetNode(allChildren[y])->GetLabelF(GraphSearchConstants::kXCoordinate);
//					printf("%d ", allChildren[y]);
				}
//				printf("\n");
				
				//node *n = g->GetNode(currLayer[x]);
				double depth;
				astar.GetClosedListGCost(parent, depth);
				
				n->SetLabelF(GraphSearchConstants::kXCoordinate, avg/allChildren.size());
				n->SetLabelF(GraphSearchConstants::kYCoordinate, (-1.0+2.0*((double)depth/maximumGCost))*0.9);
				n->SetLabelF(GraphSearchConstants::kZCoordinate, 0);
				// now we are a leaf
				n->SetLabelL(GraphSearchConstants::kFirstData, 1);
//				printf("%d is set now\n", parent);
				nextLayer.push_back(parent);
			}
		}
		currLayer = nextLayer;
		nextLayer.clear();
	}
	
	// Store cost in tree
	for (uint64_t x = 0; x < mnp.GetMaxStateHash(); x++)
	{
		double cost;
		astar.GetClosedListGCost(x, cost);
		g->GetNode(x)->SetLabelF(GraphSearchConstants::kTemporaryLabel, cost);
		if (g->GetNode(x)->GetLabelF(GraphSearchConstants::kYCoordinate) == 1)
		{
			astar.ExtractPathToStartFromID(x, tmp);
			for (int t = tmp.size()-2; t >= 0; t--)
			{
				if (g->GetNode(tmp[t])->GetLabelF(GraphSearchConstants::kYCoordinate) == 1)
					g->GetNode(tmp[t])->SetLabelF(GraphSearchConstants::kXCoordinate,
												  g->GetNode(tmp[t+1])->GetLabelF(GraphSearchConstants::kXCoordinate));
			}
		}
	}
	

}


double distsquared(unsigned long node, point3d loc)
{
	double dx = g->GetNode(node)->GetLabelF(GraphSearchConstants::kXCoordinate);
	double dy = g->GetNode(node)->GetLabelF(GraphSearchConstants::kYCoordinate);
	
	return (dx-loc.x)*(dx-loc.x) + (dy-loc.y)*(dy-loc.y);
}

double distance(unsigned long n1, unsigned long n2)
{
	double dx1 = g->GetNode(n1)->GetLabelF(GraphSearchConstants::kXCoordinate);
	double dy1 = g->GetNode(n1)->GetLabelF(GraphSearchConstants::kYCoordinate);
	
	double dx2 = g->GetNode(n2)->GetLabelF(GraphSearchConstants::kXCoordinate);
	double dy2 = g->GetNode(n2)->GetLabelF(GraphSearchConstants::kYCoordinate);
	
	return sqrt((dx1-dx2)*(dx1-dx2)+(dy1-dy2)*(dy1-dy2));
}

node *FindClosestNode(Graph *gr, point3d loc)
{
	if (gr->GetNumNodes() == 0)
		return 0;
	unsigned long best = 0;
	double dist = distsquared(0, loc);
	for (unsigned long x = 1; x < gr->GetNumNodes(); x++)
	{
		if (fless(distsquared(x, loc), dist))
		{
			dist = distsquared(x, loc);
			best = x;
		}
	}
	return gr->GetNode(best);
}


bool MyClickHandler(unsigned long , int windowX, int windowY, point3d loc, tButtonType button, tMouseEventType mType)
{
//	if (mType == kMouseDown)
//	{
//		switch (button)
//		{
//			case kRightButton: printf("Right button\n"); break;
//			case kLeftButton: printf("Left button\n"); break;
//			case kMiddleButton: printf("Middle button\n"); break;
//		}
//	}
	//if (button != kLeftButton)
	
	// no mouse right now
	//return false;
	
	switch (mType)
	{
		case kMouseDown:
		{
			//printf("Hit (%f, %f, %f)\n", loc.x, loc.y, loc.z);
			if (running)
				submitTextToBuffer("Stopped running.");
			running = false;
//			ida.InitializeSearch(ge, from, 0, &h, path);
//			ibex.InitializeSearch(ge, from, 0, &h, path);
			RedrawTextDisplay();
			
			currentPath.clear();
			path.clear();
			lastPath.clear();
			hit = FindClosestNode(g, loc)->GetNum();
			printf("Selected node %d parent %d\n", hit, g->GetNode(hit)->GetLabelL(GraphSearchConstants::kFirstData+1));
			RedrawTextDisplay();
			//mnp.GetStateFromHash(start, from);
			return true;
		}
		case kMouseDrag:
		{
			running = false;
			hit = FindClosestNode(g, loc)->GetNum();
			RedrawTextDisplay();
			//mnp.GetStateFromHash(start, from);
			return true;
		}
		case kMouseUp:
		{
			running = false;
			//printf("UnHit at (%f, %f, %f)\n", loc.x, loc.y, loc.z);
			hit = -1;
			RedrawTextDisplay();
			//			from = FindClosestNode(g, loc)->GetNum();
			//			mnp.GetStateFromHash(start, from);
			return true;
		}
	}
	return false;
}

