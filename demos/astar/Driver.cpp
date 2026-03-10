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
#include "GraphEnvironment.h"
#include "TemplateAStar.h"
#include "TextOverlay.h"
#include <string>

enum mode {
	kAddNodes,
	kAddEdges,
	kMoveNodes,
	kFindPath
};

TemplateAStar<graphState, graphMove, GraphEnvironment> astar;
std::vector<graphState> path;

mode m = kAddNodes;

bool recording = false;
bool running = false;

double edgeCost = 1.0;
double weight = 1.0;

Graph *g = 0;
GraphEnvironment *ge;
graphState from=-1, to=-1;
Graphics::point currLoc;

TextOverlay te(30);

void SaveGraph(const char *file);
void LoadGraph(const char *file);
void ShowSearchInfo();
static char name[2] = "a";

double distance(unsigned long n1, unsigned long n2);

class GraphDistHeuristic : public Heuristic<graphState> {
public:
	double HCost(const graphState &a, const graphState &b) const
	{
		return distance(a, b);
	}
};

GraphDistHeuristic searchHeuristic;

std::vector<int> buttons;
typedef enum : int {
	kClearGraphButton = 0,
	kLoadDefaultButton = 1,
	kAddNodesButton = 2,
	kAddEdgesButton = 3,
	kMoveNodesButton = 4,
	kDijkstraButton = 5,
	kAStarButton = 6,
	kWAStar2Button = 7,
	kWAStar100Button = 8,
	kFindPathButton = 9,
	kStepSearchButton = 10,
} buttonID;
void ActivateModeButton(buttonID bId);
void ActivateAlgButton(buttonID bId);
void SetupGUI(int windowID);

int main(int argc, char* argv[])
{
	InstallHandlers();
	RunHOGGUI(argc, argv, 1500, 1000);
	return 0;
}

/**
 * Allows you to install any keyboard handlers needed for program interaction.
 */
void InstallHandlers()
{
	InstallKeyboardHandler(MyDisplayHandler, "Record", "Record a movie", kAnyModifier, 'r');
	InstallKeyboardHandler(MyDisplayHandler, "Picture", "Save a picture of the graph.", kNoModifier, 'p');
	InstallKeyboardHandler(MyDisplayHandler, "Step Simulation", "If the simulation is paused, step forward .1 sec.", kAnyModifier, 'o');
	InstallKeyboardHandler(MyDisplayHandler, "Step Abs Type", "Increase abstraction type", kAnyModifier, ']');
	InstallKeyboardHandler(MyDisplayHandler, "Step Abs Type", "Decrease abstraction type", kAnyModifier, '[');
	InstallKeyboardHandler(MyDisplayHandler, "Clear", "Clear graph", kAnyModifier, '|');
	InstallKeyboardHandler(MyDisplayHandler, "Help", "Draw help", kAnyModifier, '?');
	InstallKeyboardHandler(MyDisplayHandler, "Weight", "Toggle Dijkstra, A* and Weighted A*", kAnyModifier, 'w');

	InstallKeyboardHandler(MyDisplayHandler, "Dijkstra", "Use Dijkstra", kAnyModifier, '1');
	InstallKeyboardHandler(MyDisplayHandler, "A*", "Use A*", kAnyModifier, '2');
	InstallKeyboardHandler(MyDisplayHandler, "A*", "Use WA*(w = 2)", kAnyModifier, '3');
	InstallKeyboardHandler(MyDisplayHandler, "A*", "Use WA*(w = 100)", kAnyModifier, '4');

	InstallKeyboardHandler(MyDisplayHandler, "Find Path", "Move to find path mode", kAnyModifier, 'a');
	InstallKeyboardHandler(MyDisplayHandler, "Add Node", "Move to add node mode", kAnyModifier, 'n');
	InstallKeyboardHandler(MyDisplayHandler, "Add Edge", "Move to add edge mode", kAnyModifier, 'e');
	InstallKeyboardHandler(MyDisplayHandler, "Move Node", "Move to move node mode", kAnyModifier, 'm');

	InstallKeyboardHandler(MyDisplayHandler, "Save", "Save current graph as .svg file", kAnyModifier, 'g');
	InstallKeyboardHandler(MyDisplayHandler, "Save", "Save current graph", kAnyModifier, 's');
	InstallKeyboardHandler(MyDisplayHandler, "Load", "Load last saved graph", kAnyModifier, 'l');
	InstallKeyboardHandler(BuildGraphFromPuzzle, "Default", "Build Deafult Graph", kAnyModifier, 'd');

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
		glClearColor(0.99, 0.99, 0.99, 1.0);
		InstallFrameHandler(MyFrameHandler, windowID, 0);
		
		SetNumPorts(windowID, 3);
		g = new Graph();
		ge = new GraphEnvironment(g);
		ge->SetNodeScale(2.0);
		ge->SetDrawEdgeCosts(true);
		ge->SetDrawNodeLabels(true);
		ge->SetIntegerEdgeCosts(false);
		astar.SetWeight(1.0);
		astar.SetHeuristic(&searchHeuristic);
		te.AddLine("A* algorithm sample code");
		te.AddLine("Current mode: add nodes (click to add node)");
		te.AddLine("Press [ or ] to change modes. '?' for help.");

		// Left half of screen
		ReinitViewports(windowID, {-1, -1, 0.33f, 1}, kScaleToSquare);
		// Split right half into two
		AddViewport(windowID, {0.33f, -1, 1, 0}, kScaleToSquare);  // Buttons
		AddViewport(windowID, {0.33f, 0, 1, 1}, kScaleToSquare);  // Text overlay
		SetupGUI(windowID);
	}
}

void DrawSim(Graphics::Display &display)
{
	if (ge == 0 || g == 0)
		return;
	display.FillRect({-1.0, -1.0, 1.0, 1}, Colors::white);
	ge->SetColor(Colors::black);
	ge->Draw(display);
	
	if (from != -1 && to != -1)
	{
		ge->SetColor(Colors::lightgray);
		auto loc1 = ge->GetLocation(from);
		ge->DrawLine(display, loc1.x, loc1.y, currLoc.x, currLoc.y, 2);
	}
	
	ge->SetColor(Colors::white);
	for (int x = 0; x < g->GetNumNodes(); x++)
		ge->Draw(display, x);

	if (running)
	{
		astar.Draw(display);
	}
	
	if (path.size() > 0)
	{
		ge->SetColor(0, 1, 0);
		for (size_t x = 1; x < path.size(); x++)
		{
			ge->DrawLine(display, path[x-1], path[x], 4);
		}
	}

	return;
}

void SetupGUI(int windowID)
{
	const float borderPaddingX = 0.1f;
	const float borderPaddingY = 0.35f;
	const float verticalSep = 0.25f;
	const float buttonHeight = 0.1f;

	// General graph management
	float horizontalSep = 0.3f;
	float buttonWidth = 0.75f;
	float top = -1+borderPaddingY;
	float bot = top+buttonHeight;
	Graphics::roundedRect graphButton({-1+borderPaddingX, top, -1+borderPaddingX+buttonWidth, bot}, 0.01f);
	Graphics::rect offsetRect(buttonWidth+horizontalSep, 0, buttonWidth+horizontalSep, 0);
	int b = CreateButton(windowID, 1, graphButton, "Clear Graph", '|', 0.01f, Colors::black, Colors::black,
					Colors::white, Colors::lightblue, Colors::lightbluegray);
	buttons.push_back(b);
	graphButton.r += offsetRect;
	b = CreateButton(windowID, 1, graphButton, "Load Default", 'd', 0.01f, Colors::black, Colors::black,
					Colors::white, Colors::lightblue, Colors::lightbluegray);
	buttons.push_back(b);

	// Modifying the graph
	horizontalSep = 0.15f;
	buttonWidth = 0.5f;
	top = bot+verticalSep;
	bot = top+buttonHeight;
	Graphics::roundedRect modeButton({-1+borderPaddingX, top, -1+borderPaddingX+buttonWidth, bot}, 0.01f);
	offsetRect = Graphics::rect(buttonWidth+horizontalSep, 0, buttonWidth+horizontalSep, 0);
	b = CreateButton(windowID, 1, modeButton, "Add Nodes", 'n', 0.01f, Colors::black, Colors::black,
					Colors::yellow, Colors::lightblue, Colors::lightbluegray);
	buttons.push_back(b);
	modeButton.r += offsetRect;
	b = CreateButton(windowID, 1, modeButton, "Add Edges", 'e', 0.01f, Colors::black, Colors::black,
					Colors::white, Colors::lightblue, Colors::lightbluegray);
	buttons.push_back(b);
	modeButton.r += offsetRect;
	b = CreateButton(windowID, 1, modeButton, "Move Nodes", 'm', 0.01f, Colors::black, Colors::black,
					Colors::white, Colors::lightblue, Colors::lightbluegray);
	buttons.push_back(b);

	// Search algorithms
	horizontalSep = 0.1f;
	buttonWidth = 0.375f;
	top = bot+verticalSep;
	bot = top+buttonHeight;
	Graphics::roundedRect algButton({-1+borderPaddingX, top, -1+borderPaddingX+buttonWidth, bot}, 0.01f);
	offsetRect = Graphics::rect(buttonWidth+horizontalSep, 0, buttonWidth+horizontalSep, 0);
	b = CreateButton(windowID, 1, algButton, "Dijkstra", '1', 0.01f, Colors::black, Colors::black,
					Colors::yellow, Colors::lightblue, Colors::lightbluegray);
	buttons.push_back(b);
	algButton.r += offsetRect;
	b = CreateButton(windowID, 1, algButton, "A*", '2', 0.01f, Colors::black, Colors::black,
					Colors::white, Colors::lightblue, Colors::lightbluegray);
	buttons.push_back(b);
	algButton.r += offsetRect;
	b = CreateButton(windowID, 1, algButton, "WA*(2)", '3', 0.01f, Colors::black, Colors::black,
					Colors::white, Colors::lightblue, Colors::lightbluegray);
	buttons.push_back(b);
	algButton.r += offsetRect;
	b = CreateButton(windowID, 1, algButton, "WA*(100)", '4', 0.01f, Colors::black, Colors::black,
					Colors::white, Colors::lightblue, Colors::lightbluegray);
	buttons.push_back(b);

	// Simulating search
	horizontalSep = 0.3f;
	buttonWidth = 0.75f;
	top = bot+verticalSep;
	bot = top+buttonHeight;
	Graphics::roundedRect simulationButton({-1+borderPaddingX, top, -1+borderPaddingX+buttonWidth, bot}, 0.01f);
	offsetRect = Graphics::rect(buttonWidth+horizontalSep, 0, buttonWidth+horizontalSep, 0);
	b = CreateButton(windowID, 1, simulationButton, "Find Path", 'a', 0.01f, Colors::black, Colors::black,
					Colors::white, Colors::lightblue, Colors::lightbluegray);
	buttons.push_back(b);
	simulationButton.r += offsetRect;
	b = CreateButton(windowID, 1, simulationButton, "Step Search", 'o', 0.01f, Colors::black, Colors::black,
					Colors::white, Colors::lightblue, Colors::lightbluegray);
	buttons.push_back(b);
}

void DrawGUI(Graphics::Display &display)
{
	const float e = 0.03f;
	display.FillRect({-1, -1, 1, 1}, Colors::darkgray);
	display.FillRect({-1+e, -1+e, 1-e, 1-e}, Colors::lightgray);
	display.DrawText("Graph: ", {-1+0.1f, -0.8f}, Colors::black, 0.1f, Graphics::textAlignLeft, Graphics::textBaselineTop);
	display.DrawText("Mode: ", {-1+0.1f, -0.45f}, Colors::black, 0.1f, Graphics::textAlignLeft, Graphics::textBaselineTop);
	display.DrawText("Algorithm: ", {-1+0.1f, -0.10f}, Colors::black, 0.1f, Graphics::textAlignLeft, Graphics::textBaselineTop);
	display.DrawText("Simulation: ", {-1+0.1f, 0.25f}, Colors::black, 0.1f, Graphics::textAlignLeft, Graphics::textBaselineTop);
}

void ActivateModeButton(buttonID bId)
{
	SetButtonFillColor(buttons[kAddNodesButton], bId == kAddNodesButton ? Colors::yellow : Colors::white);
	SetButtonFillColor(buttons[kAddEdgesButton], bId == kAddEdgesButton ? Colors::yellow : Colors::white);
	SetButtonFillColor(buttons[kMoveNodesButton], bId == kMoveNodesButton ? Colors::yellow : Colors::white);
	SetButtonFillColor(buttons[kFindPathButton], bId == kFindPathButton ? Colors::yellow : Colors::white);
}

void ActivateAlgButton(buttonID bId)
{
	SetButtonFillColor(buttons[kDijkstraButton], bId == kDijkstraButton ? Colors::yellow : Colors::white);
	SetButtonFillColor(buttons[kAStarButton], bId == kAStarButton ? Colors::yellow : Colors::white);
	SetButtonFillColor(buttons[kWAStar2Button], bId == kWAStar2Button ? Colors::yellow : Colors::white);
	SetButtonFillColor(buttons[kWAStar100Button], bId == kWAStar100Button ? Colors::yellow : Colors::white);
}

int frameCnt = 0;

void MyFrameHandler(unsigned long windowID, unsigned int viewport, void *)
{
	Graphics::Display &display = getCurrentContext()->display;

	if (viewport == 0)
		DrawSim(display);
	else if (viewport == 1)
		DrawGUI(display);
	else
		te.Draw(display);
	
//	if (recording && viewport == GetNumPorts(windowID)-1)
//	{
//		char fname[255];
//		sprintf(fname, "/Users/nathanst/Movies/tmp/astar-%d%d%d%d",
//				(frameCnt/1000)%10, (frameCnt/100)%10, (frameCnt/10)%10, frameCnt%10);
//		SaveScreenshot(windowID, fname);
//		printf("Saved %s\n", fname);
//		frameCnt++;
//		if (path.size() == 0)
//		{
//			MyDisplayHandler(windowID, kNoModifier, 'o');
//		}
//		else {
//			recording = false;
//		}
//	}
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
		case 'a': m = kFindPath; te.AddLine("Current mode: find path"); ActivateModeButton(kFindPathButton); break;
		case 'n': m = kAddNodes; te.AddLine("Current mode: add nodes"); ActivateModeButton(kAddNodesButton); break;
		case 'e': m = kAddEdges; te.AddLine("Current mode: add edges"); ActivateModeButton(kAddEdgesButton); break;
		case 'm': m = kMoveNodes; te.AddLine("Current mode: moves nodes"); ActivateModeButton(kMoveNodesButton); break;
		case '1': // Dijkstra
			te.AddLine("Algorithm: Dijkstra");
			ActivateAlgButton(kDijkstraButton);
			weight = 0.0;
			astar.SetWeight(weight);
			if (running)
			{
				astar.InitializeSearch(ge, astar.start, astar.goal, path);
				ShowSearchInfo();
			}
			else {
				m = kFindPath; te.AddLine("Current mode: find path"); ActivateModeButton(kFindPathButton); break;
			}
			break;
		case '2': // A*
			te.AddLine("Algorithm: A*");
			ActivateAlgButton(kAStarButton);
			weight = 1.0;
			astar.SetWeight(weight);
			if (running)
			{
				astar.InitializeSearch(ge, astar.start, astar.goal, path);
				ShowSearchInfo();
			}
			else {
				m = kFindPath; te.AddLine("Current mode: find path"); ActivateModeButton(kFindPathButton); break;
			}
			break;
		case '3': // WA*(2)
			te.AddLine("Algorithm: WA*(2)");
			ActivateAlgButton(kWAStar2Button);
			weight = 2.0;
			astar.SetWeight(weight);
			if (running)
			{
				astar.InitializeSearch(ge, astar.start, astar.goal, path);
				ShowSearchInfo();
			}
			else {
				m = kFindPath; te.AddLine("Current mode: find path"); ActivateModeButton(kFindPathButton); break;
			}
			break;
		case '4': // WA*(100)
			te.AddLine("Algorithm: WA*(100)");
			ActivateAlgButton(kWAStar100Button);
			weight = 100.0;
			astar.SetWeight(weight);
			if (running)
			{
				astar.InitializeSearch(ge, astar.start, astar.goal, path);
				ShowSearchInfo();
			}
			else {
				m = kFindPath; te.AddLine("Current mode: find path"); ActivateModeButton(kFindPathButton); break;
			}
			break;

		case ']':
		{
			switch (m)
			{
				case kAddNodes: m = kAddEdges; te.AddLine("Current mode: add edges"); ActivateModeButton(kAddEdgesButton); break;
				case kAddEdges: m = kMoveNodes; te.AddLine("Current mode: moves nodes"); ActivateModeButton(kMoveNodesButton); break;
				case kMoveNodes: m = kFindPath; te.AddLine("Current mode: find path"); ActivateModeButton(kFindPathButton); break;
				case kFindPath: m = kAddNodes; te.AddLine("Current mode: add nodes"); ActivateModeButton(kAddNodesButton); break;
			}

		}
			break;
		case '[':
		{
			switch (m)
			{
				case kMoveNodes: m = kAddEdges; te.AddLine("Current mode: add edges"); ActivateModeButton(kAddEdgesButton); break;
				case kFindPath: m = kMoveNodes; te.AddLine("Current mode: moves nodes"); ActivateModeButton(kMoveNodesButton); break;
				case kAddNodes: m = kFindPath; te.AddLine("Current mode: find path"); ActivateModeButton(kFindPathButton); break;
				case kAddEdges: m = kAddNodes; te.AddLine("Current mode: add nodes"); ActivateModeButton(kAddNodesButton); break;
			}
		}
			break;
		case '|':
		{
			name[0] = 'a';
			g->Reset();
			te.AddLine("Current mode: add nodes");
			ActivateModeButton(kAddNodesButton);
			m = kAddNodes;
			path.resize(0);
			running = false;
		}
			break;
			
		case 'w':
			if (weight == 1.0)
				weight = 2.0;
			else if (weight == 2.0)
				weight = 100.0;
			else if (weight == 0.0)
				weight = 1.0;
			else
				weight = 0.0;
			astar.SetWeight(weight);
			if (running)
			{
				astar.InitializeSearch(ge, astar.start, astar.goal, path);
				ShowSearchInfo();
			}
			else {
				m = kFindPath; te.AddLine("Current mode: find path"); ActivateModeButton(kFindPathButton); break;
			}

			break;
		case 'r':
			recording = !recording;
			break;
		case 'p':
		{
			std::fstream svgFile;
			svgFile.open("/Users/nathanst/graph.svg", std::fstream::out | std::fstream::trunc);
			svgFile << ge->SVGHeader();
			svgFile << ge->SVGDraw();
			svgFile << "</svg>";
			svgFile.close();
		}
			break;
		case 'o':
		{
			if (running && path.size() == 0)
			{
				astar.DoSingleSearchStep(path);
				ShowSearchInfo();
			}
		}
			break;
		case '?':
		{
			te.AddLine("Help:");
			te.AddLine("-----");
			te.AddLine("Press '[' and ']' to switch between modes.");
			te.AddLine("Add nodes: click to add a node to the graph");
			te.AddLine("Add edges: drag between nodes to add an edge");
			te.AddLine("           Press 1-9 to change edge weight");
			te.AddLine("Move nodes: drag to move node locations");
			te.AddLine("Find Path: Drag to find path between nodes");
			te.AddLine("           'o' to step pathfinding forward");
			te.AddLine("           Red nodes: closed list");
			te.AddLine("           Green nodes: open list");
			te.AddLine("           Yellow node: next on open list");
			te.AddLine("           Pink node: goal state");
		}
			break;
		case 's':
			SaveGraph("save.graph");
			break;
		case 'l':
			LoadGraph("save.graph");
			break;
		case 'g':
		{
			std::fstream svgFile;
			ge->SetColor(Colors::darkgray.r, Colors::darkgray.g, Colors::darkgray.b);
			svgFile.open("/Users/nathanst/graph.svg", std::fstream::out | std::fstream::trunc);
			svgFile << ge->SVGHeader();
			svgFile << ge->SVGDraw();			
			svgFile << "</svg>";
			svgFile.close();
		}
			break;
		default:
			break;
	}
	
}

void BuildGraphFromPuzzle(unsigned long windowID, tKeyboardModifier mod, char key)
{
	if (key == 'd')
	{
		MyDisplayHandler(windowID, kNoModifier, '|'); // clear current graph
		
		g->AddNode(new node("a"));
		g->AddNode(new node("b"));
		g->AddNode(new node("c"));
		g->AddNode(new node("d"));
		g->AddNode(new node("e"));
		name[0] += 5;
		
		g->GetNode(0)->SetLabelF(GraphSearchConstants::kXCoordinate, -0.5);
		g->GetNode(0)->SetLabelF(GraphSearchConstants::kYCoordinate, -0.65);
		g->GetNode(0)->SetLabelF(GraphSearchConstants::kZCoordinate, 0);

		g->GetNode(1)->SetLabelF(GraphSearchConstants::kXCoordinate, 0.5);
		g->GetNode(1)->SetLabelF(GraphSearchConstants::kYCoordinate, -0.65);
		g->GetNode(1)->SetLabelF(GraphSearchConstants::kZCoordinate, 0);

		g->GetNode(2)->SetLabelF(GraphSearchConstants::kXCoordinate, -0.5);
		g->GetNode(2)->SetLabelF(GraphSearchConstants::kYCoordinate, 0.15);
		g->GetNode(2)->SetLabelF(GraphSearchConstants::kZCoordinate, 0);
		
		g->GetNode(3)->SetLabelF(GraphSearchConstants::kXCoordinate, 0.5);
		g->GetNode(3)->SetLabelF(GraphSearchConstants::kYCoordinate, 0.15);
		g->GetNode(3)->SetLabelF(GraphSearchConstants::kZCoordinate, 0);

		g->GetNode(4)->SetLabelF(GraphSearchConstants::kXCoordinate, 0.0);
		g->GetNode(4)->SetLabelF(GraphSearchConstants::kYCoordinate, 0.75);
		g->GetNode(4)->SetLabelF(GraphSearchConstants::kZCoordinate, 0);
	
		g->AddEdge(new edge(0, 1, distance(0, 1)));
		g->AddEdge(new edge(0, 2, distance(0, 2)));
		g->AddEdge(new edge(1, 3, distance(1, 3)));
		g->AddEdge(new edge(2, 3, distance(2, 3)));
		g->AddEdge(new edge(2, 4, distance(2, 4)));
		g->AddEdge(new edge(3, 4, distance(3, 4)));

		from = to = -1;
		// switch to move nodes
		MyDisplayHandler(windowID, kNoModifier, 'm');

	}
}
#include <iomanip>
#include <sstream>

std::string MyToString(double val)
{
	std::stringstream ss;
	ss << std::fixed << std::setprecision(2) << val;
	return ss.str();
}

void ShowSearchInfo()
{
	const int colWidth = 24;
	std::string s;
	te.Clear();
	s = "-----> Searching from ";
	s +=g->GetNode(astar.start)->GetName();
	s +=" to ";
	s += g->GetNode(astar.goal)->GetName();
	s += " <-----";
	if (weight == 0)
	{
		s += " Dijkstra";
	}
	else if (weight == 1)
	{
		s += " A*";
	}
	else if (weight == 2)
	{
		s += " WA*(2)";
	}
	else if (weight == 100)
	{
		s += " WA*(100)";
	}
	te.AddLine(s.c_str());
	te.AddLine("Press 'o' to advance search.");
	for (int x = 0; x < g->GetNumNodes(); x++)
	{
		double gcost;
		s = g->GetNode(x)->GetName();
		switch (astar.GetStateLocation(x))
		{
			case kClosedList:
			{
				s += ": Closed  (g: ";
				astar.GetClosedListGCost(x, gcost);
				s += MyToString(gcost);
				s += ", h: ";
				s += MyToString(searchHeuristic.HCost(x, astar.goal));
				s += ")";
			}
			break;
			case kOpenList:
			{
				s += ": Open    (g: ";
				astar.GetOpenListGCost(x, gcost);
				s += MyToString(gcost);
				s += ", h: ";
				s += MyToString(searchHeuristic.HCost(x, astar.goal));
				s += ")";
			}
				break;

			case kNotFound:
				s += ": Ungenerated (h: ";
				s += MyToString(searchHeuristic.HCost(x, astar.goal));
				s += ")";
				break;
		}
		
		te.AddLine(s.c_str());
	}

	te.AddLine("");
	te.AddLine("Open List:");
	size_t length = strlen(te.GetLastLine());
	for (int x = length; x < colWidth; x++)
		te.AppendToLine(" ");
	te.AppendToLine("Closed List:");

	std::vector<std::string> open, closed;
	
	for (size_t x = 0; x < astar.GetNumOpenItems(); x++)
	{
		auto item = astar.GetOpenItem(x);
		s = g->GetNode(item.data)->GetName();
		s += ": ";
		s += MyToString(item.g+item.h);
		s += "=";
		s += MyToString(item.g);
		s += "+";
		s += MyToString(item.h);
		s += " p: ";
		s += g->GetNode(astar.GetItem(item.parentID).data)->GetName();
		//te.AddLine(s.c_str());
		open.push_back(s);
	}
	
	for (int x = 0; x < astar.GetNumItems(); x++)
	{
		auto item = astar.GetItem(x);
		if (item.where == kClosedList)
		{
			s = g->GetNode(item.data)->GetName();
			s += ": ";
			s += MyToString(item.g+item.h);
			s += "=";
			s += MyToString(item.g);
			s += "+";
			s += MyToString(item.h);
			s += " p: ";
			s += g->GetNode(astar.GetItem(item.parentID).data)->GetName();
			//te.AddLine(s.c_str());
			closed.push_back(s);
		}
	}
	for (size_t x = 0; x < open.size(); x++)
	{
		te.AddLine(open[x].c_str());
		for (size_t t = open[x].length(); t < colWidth; t++)
			te.AppendToLine(" ");
		if (x < closed.size())
			te.AppendToLine(closed[x].c_str());
	}
	for (size_t x = open.size(); x < closed.size(); x++)
	{
		te.AddLine("                        ");
		te.AppendToLine(closed[x].c_str());
	}
}

double distsquared(unsigned long node, point3d loc)
{
	double dx = g->GetNode(node)->GetLabelF(GraphSearchConstants::kXCoordinate);
	double dy = g->GetNode(node)->GetLabelF(GraphSearchConstants::kYCoordinate);

	return (dx-loc.x)*(dx-loc.x) + (dy-loc.y)*(dy-loc.y);
}

double dist(unsigned long node, point3d loc)
{
	double dx = g->GetNode(node)->GetLabelF(GraphSearchConstants::kXCoordinate);
	double dy = g->GetNode(node)->GetLabelF(GraphSearchConstants::kYCoordinate);
	
	return sqrt((dx-loc.x)*(dx-loc.x) + (dy-loc.y)*(dy-loc.y));
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


bool MyClickHandler(unsigned long , int viewport, int windowX, int windowY, point3d loc, tButtonType button, tMouseEventType mType)
{
	// Prevent button clicks from drawing on the graph.
	if (viewport != 0)
		return false;

	if (mType == kMouseDown)
	{
		switch (button)
		{
			case kRightButton: printf("Right button\n"); break;
			case kLeftButton: printf("Left button\n"); break;
			case kMiddleButton: printf("Middle button\n"); break;
		}
	}
	if (button != kLeftButton)
		return false;
	switch (mType)
	{
		case kMouseDown:
		{
			printf("Hit (%f, %f, %f)\n", loc.x, loc.y, loc.z);
			if (m == kAddNodes)
			{
				if (loc.x > 1 || loc.x < -1 || loc.y > 1 || loc.y < -1)
					return false;
				node *n = new node(name);
				name[0]++;
				g->AddNode(n);
				n->SetLabelF(GraphSearchConstants::kXCoordinate, loc.x);
				n->SetLabelF(GraphSearchConstants::kYCoordinate, loc.y);
				n->SetLabelF(GraphSearchConstants::kZCoordinate, 0);
				printf("Added node %d to graph\n", g->GetNumNodes());
			}
			if (m == kAddEdges || m == kFindPath)
			{
				node *n = FindClosestNode(g, loc);
				if (n)
				{
					from = to = n->GetNum();
					currLoc = ge->GetLocation(from);
				}
			}
			if (m == kMoveNodes)
			{
				from = to = FindClosestNode(g, loc)->GetNum();
				if (loc.x > 1) loc.x = 1;
				if (loc.x < -1) loc.x = -1;
				if (loc.y > 1) loc.y = 1;
				if (loc.y < -1) loc.y = -1;
				g->GetNode(from)->SetLabelF(GraphSearchConstants::kXCoordinate, loc.x);
				g->GetNode(from)->SetLabelF(GraphSearchConstants::kYCoordinate, loc.y);
				g->GetNode(from)->SetLabelF(GraphSearchConstants::kZCoordinate, 0);
				currLoc = ge->GetLocation(from);
				edge_iterator i = g->GetNode(from)->getEdgeIter();
				for (edge *e = g->GetNode(from)->edgeIterNext(i); e; e = g->GetNode(from)->edgeIterNext(i))
					e->setWeight(distance(e->getFrom(), e->getTo()));
			}
			return true;
		}
		case kMouseDrag:
		{
			if (m == kAddEdges || m == kFindPath)
			{
				node *n = FindClosestNode(g, loc);
				if (n == 0)
					return false;
				to = n->GetNum();
				currLoc = loc;
				if (dist(to, loc) < 0.2 && to != from)
				{
					currLoc = ge->GetLocation(to);
				}
				else {
					to = from;
				}
			}
			if (m == kMoveNodes)
			{
				if (loc.x > 1) loc.x = 1;
				if (loc.x < -1) loc.x = -1;
				if (loc.y > 1) loc.y = 1;
				if (loc.y < -1) loc.y = -1;
				g->GetNode(from)->SetLabelF(GraphSearchConstants::kXCoordinate, loc.x);
				g->GetNode(from)->SetLabelF(GraphSearchConstants::kYCoordinate, loc.y);
				g->GetNode(from)->SetLabelF(GraphSearchConstants::kZCoordinate, 0);
				currLoc = ge->GetLocation(from);
				edge_iterator i = g->GetNode(from)->getEdgeIter();
				for (edge *e = g->GetNode(from)->edgeIterNext(i); e; e = g->GetNode(from)->edgeIterNext(i))
					e->setWeight(distance(e->getFrom(), e->getTo()));
			}
			return true;
		}
		case kMouseUp:
		{
			printf("UnHit at (%f, %f, %f)\n", loc.x, loc.y, loc.z);
			if (m == kAddEdges)
			{
				to = FindClosestNode(g, loc)->GetNum();
				if (from != to && dist(to, loc) < 0.2)
				{
					edge *e;
					if ((e = g->FindEdge(from, to)) != 0)
					{
						//e->setWeight(distance(from, to));
					}
					else {
						g->AddEdge(new edge(from, to, distance(from, to)));
					}
				}
			}
			if (m == kFindPath)
			{
				to = FindClosestNode(g, loc)->GetNum();
				currLoc = ge->GetLocation(to);
				if (from != to && dist(to, loc) < 0.2)
				{
					//weight = 1.0;
					astar.SetWeight(weight);
					astar.InitializeSearch(ge, from, to, path);
					ShowSearchInfo();
					
					running = true;
				}
			}
			if (m == kMoveNodes)
			{
				if (loc.x > 1) loc.x = 1;
				if (loc.x < -1) loc.x = -1;
				if (loc.y > 1) loc.y = 1;
				if (loc.y < -1) loc.y = -1;

				g->GetNode(from)->SetLabelF(GraphSearchConstants::kXCoordinate, loc.x);
				g->GetNode(from)->SetLabelF(GraphSearchConstants::kYCoordinate, loc.y);
				g->GetNode(from)->SetLabelF(GraphSearchConstants::kZCoordinate, 0);
				currLoc = ge->GetLocation(from);

				edge_iterator i = g->GetNode(from)->getEdgeIter();
				for (edge *e = g->GetNode(from)->edgeIterNext(i); e; e = g->GetNode(from)->edgeIterNext(i))
					e->setWeight(distance(e->getFrom(), e->getTo()));
			}
			from = to = -1;
			return true;
		}
	}
	return false;
}

void SaveGraph(const char *file)
{
	FILE *f = fopen(file, "w+");
	if (f)
	{
		fprintf(f, "%d %d\n", g->GetNumNodes(), g->GetNumEdges());
		for (int x = 0; x < g->GetNumNodes(); x++)
		{
			fprintf(f, "%d %f %f %f %s\n", x,
					g->GetNode(x)->GetLabelF(GraphSearchConstants::kXCoordinate),
					g->GetNode(x)->GetLabelF(GraphSearchConstants::kYCoordinate),
					g->GetNode(x)->GetLabelF(GraphSearchConstants::kZCoordinate),
					g->GetNode(x)->GetName()
					);
		}
		edge_iterator ei = g->getEdgeIter();
		while (1)
		{
			edge *e = (edge *)g->edgeIterNext(ei);
			if (e)
				fprintf(f, "%d %d %f\n", e->getFrom(), e->getTo(), e->GetWeight());
			else
				break;
		}
		fclose(f);
	}
}

void LoadGraph(const char *file)
{
	g->Reset();
	char nodeName[255];
	FILE *f = fopen(file, "r");
	if (f)
	{
		int numNodes, numEdges;
		fscanf(f, "%d %d\n", &numNodes, &numEdges);
		for (int n = 0; n < numNodes; n++)
		{
			int which;
			float x, y, z;
			fscanf(f, "%d %f %f %f %s\n", &which, &x, &y, &z, nodeName);
			assert(which == n);
			node *next = new node(nodeName);
			next->SetLabelF(GraphSearchConstants::kXCoordinate, x);
			next->SetLabelF(GraphSearchConstants::kYCoordinate, y);
			next->SetLabelF(GraphSearchConstants::kZCoordinate, z);
			g->AddNode(next);
		}
		name[0] += numNodes;
		for (int e = 0; e < numEdges; e++)
		{
			int from, to;
			float weight;
			fscanf(f, "%d %d %f", &from, &to, &weight);
			g->AddEdge(new edge(from, to, distance(from, to)));
		}
		fclose(f);
	}
}
