//
//  IncrementalDFID.h
//  hog2 glut
//
//  Created by Nathan Sturtevant on 3/24/16.
//  Copyright © 2016 University of Denver. All rights reserved.
//

#ifndef IncrementalDFID_h
#define IncrementalDFID_h

template <class state, class action>
class IncrementalDFID {
public:
	IncrementalDFID(int initialBound = 0) :bound(initialBound), initialBound(initialBound) {}
	void GetPath(SearchEnvironment<state, action> *env, state from, state to,
				 std::vector<state> &thePath);
	void GetPath(SearchEnvironment<state, action> *env, state from, state to,
				 std::vector<action> &thePath);
	bool DoSingleSearchStep(SearchEnvironment<state, action> *env, state from, state to,
							std::vector<state> &thePath);
	uint64_t GetNodesExpanded() { return nodesExpanded; }
	uint64_t GetNodesTouched() { return nodesTouched; }
	void ResetNodeCount() { nodesExpanded = nodesTouched = 0; }
	void Reset() { bound = initialBound; path.clear(); history.clear(); }
//	void OpenGLDraw();
	void Draw(Graphics::Display &display) const;
	state GetCurrentState() const { return path.back(); }
private:
	unsigned long nodesExpanded, nodesTouched;
	
	bool DoIteration(SearchEnvironment<state, action> *env,
					 state parent, state currState,
					 std::vector<state> &thePath, double bound, double g);
	bool DoIteration(SearchEnvironment<state, action> *env,
					 action forbiddenAction, state &currState,
					 std::vector<action> &thePath, double bound, double g);
	
	std::vector<std::pair<state, int>> history;
	std::vector<state> path;
	state goal;
	int bound;
	int initialBound;
	//double nextBound;
	SearchEnvironment<state, action> *env;
	std::vector<state> succ;
};

template <class state, class action>
void IncrementalDFID<state, action>::GetPath(SearchEnvironment<state, action> *env, state from, state to,
			 std::vector<state> &thePath)
{
	while (!DoSingleSearchStep(env, from, to, thePath))
	{}
}

template <class state, class action>
void IncrementalDFID<state, action>::GetPath(SearchEnvironment<state, action> *env, state from, state to,
			 std::vector<action> &thePath)
{
	
}

template <class state, class action>
bool IncrementalDFID<state, action>::DoSingleSearchStep(SearchEnvironment<state, action> *env, state from, state to,
						std::vector<state> &thePath)
{
	if (history.size() == 0)
	{
		history.push_back({from, 0});
		bound++;
	}

	this->env = env;
	int depth = history.back().second;
	env->GetSuccessors(history.back().first, succ);
	//printf("Working at depth %d, bound %d\n", depth, bound);
	// backtracking step
	if (path.size() > depth)
	{
		path.pop_back();

		// terminating on next step - for visualization only! (avoids flicker)
		if (path.size() <= depth && env->GoalTest(history.back().first, to))
			path.push_back(history.back().first);
		return false;
	}
	path.push_back(history.back().first);

	// Later than normal - for the purposes of drawing nicely
	if (env->GoalTest(history.back().first, to))
	{
		//printf("Done!");
		return true;
	}
	
	history.pop_back();
	if (depth < bound)
	{
		for (int x = succ.size()-1; x >= 0; x--)
		{
			if (path.size() < 2 || path[path.size()-2] != succ[x])
				history.push_back({succ[x], depth+1});
		}
	}
	return false;
}

//template <class state, class action>
//void IncrementalDFID<state, action>::OpenGLDraw()
//{
////	for (auto x : history)
////		env->OpenGLDraw(x.first);
//	for (int x = 1; x < path.size(); x++)
//		env->GLDrawLine(path[x-1], path[x]);
//}

template <class state, class action>
void IncrementalDFID<state, action>::Draw(Graphics::Display &display) const
{
	for (int x = 1; x < path.size(); x++)
		env->DrawLine(display, path[x-1], path[x], 2.0);
}



#endif /* IncrementalDFID_h */
