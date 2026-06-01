//
//  IncrementalBFS.h
//  hog2 glut
//
//  Created by Nathan Sturtevant on 3/24/16.
//  Copyright © 2016 University of Denver. All rights reserved.
//

#ifndef IncrementalBFS_h
#define IncrementalBFS_h

#include <deque>

template <class state, class action>
class IncrementalBFS {
public:
	void GetPath(SearchEnvironment<state, action> *env, state from, state to,
				 std::vector<state> &thePath);
	void GetPath(SearchEnvironment<state, action> *env, state from, state to,
				 std::vector<action> &thePath);
	bool DoSingleSearchStep(SearchEnvironment<state, action> *env, state from, state to,
							std::vector<state> &thePath);
	uint64_t GetNodesExpanded() { return nodesExpanded; }
	uint64_t GetNodesTouched() { return nodesTouched; }
	void ResetNodeCount() { nodesExpanded = nodesTouched = 0; }
	void Reset() { history.clear(); }
//	void OpenGLDraw();
	void Draw(Graphics::Display &display) const;
private:
	unsigned long nodesExpanded, nodesTouched;
	
	bool DoIteration(SearchEnvironment<state, action> *env,
					 state parent, state currState,
					 std::vector<state> &thePath, double bound, double g);
	bool DoIteration(SearchEnvironment<state, action> *env,
					 action forbiddenAction, state &currState,
					 std::vector<action> &thePath, double bound, double g);
	
	std::deque<std::pair<state, int>> history;
	state goal;
	SearchEnvironment<state, action> *env;
	std::vector<state> succ;
};

template <class state, class action>
void IncrementalBFS<state, action>::GetPath(SearchEnvironment<state, action> *env, state from, state to,
											 std::vector<state> &thePath)
{
	while (!DoSingleSearchStep(env, from, to, thePath))
	{}
}

template <class state, class action>
void IncrementalBFS<state, action>::GetPath(SearchEnvironment<state, action> *env, state from, state to,
											 std::vector<action> &thePath)
{
	
}

template <class state, class action>
bool IncrementalBFS<state, action>::DoSingleSearchStep(SearchEnvironment<state, action> *env, state from, state to,
														std::vector<state> &thePath)
{
	if (history.size() == 0)
	{
		history.push_back({from, 0});
	}
	
	this->env = env;
	int depth = history.front().second;
	env->GetSuccessors(history.front().first, succ);
	
	// Later than normal - for the purposes of drawing nicely
	if (env->GoalTest(history.front().first, to))
		return true;
	
	history.pop_front();
	for (int x = 0; x < succ.size(); x++)
	{
		history.push_back({succ[x], depth+1});
	}
	return false;
}

//template <class state, class action>
//void IncrementalBFS<state, action>::OpenGLDraw()
//{
//	for (auto x : history)
//		env->OpenGLDraw(x.first);
//}

template <class state, class action>
void IncrementalBFS<state, action>::Draw(Graphics::Display &display) const
{
	for (auto x : history)
		env->Draw(display, x.first);
}


#endif /* IncrementalBFS_h */
