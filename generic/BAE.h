/**
 * @file BAE.h
 * @package hog2
 * @brief A templated version of BAE*
 * @author Vidal Alcázar
 * @author Lior Siag
 * @date 7/25/19, modified 7/24/24
 *
 * This is a templated version of BAE*, implemented from algorithm presented in the paper "Bidirectional Heuristic
 * Search based on Error Estimate" by Samir K Sadhukhan (2013).
 * Slight modifications and bug-fixes, cleanup, and documentation was done by Lior before integrating into HOG2.
 * The code was not attempted on directed domains, and as such can have still bugs when dealing with them.
 */

#ifndef BAE_H
#define BAE_H

#include "AStarOpenClosed.h"
#include "FPUtil.h"
#include "Timer.h"
#include <unordered_map>
#include <cmath>
#include <iostream>
#include "Heuristic.h"
#include <vector>
#include <algorithm>

template<class state>
struct BAECompare {
    bool operator()(const AStarOpenClosedData<state> &i1, const AStarOpenClosedData<state> &i2) const {
        // Note that h here is used to contain the b-value of a node, which is the priority of it.
        double p1 = i1.h;
        double p2 = i2.h;
        if (fequal(p1, p2)) {
            return (fless(i1.g, i2.g)); // high g-cost over low
        }
        return (fgreater(p1, p2)); // low priority over high
    }
};

/**
 * A class which implements the BAE algorithm. This implementation uses two papers' details:
 * A. "Bidirectional Heuristic Search based on Error Estimate" by Samir K Sadhukhan (2013).
 * B. "A Unifying View on Individual Bounds and Heuristic Inaccuracies in Bidirectional Search" by Vidal Alcázar, Pat
 * Riddle, Mike Barley (2020).
 */
template<class state, class action, class environment, class priorityQueue = AStarOpenClosed<state, BAECompare<state>>>
class BAE {
public:
    /**
     *
     * @param alternating_ Is the side choosing policy alternating or Pohl's cardinality criterion
     * @param epsilon_ Cost of the least-cost edge
     * @param gcd_ Greatest common denominator between all edges. Note that for edges e.g., 1 and 1.5, the gcd is 0.5.
     */

    BAE(bool alternating_ = true, double epsilon_ = 1.0, double gcd_ = 1.0) {
        forwardHeuristic = 0;
        backwardHeuristic = 0;
        env = 0;
        ResetNodeCount();
        epsilon = epsilon_;
        gcd = gcd_;
        alternating = alternating_;
    }

    virtual ~BAE() {}

    void GetPath(environment *env, const state &from, const state &to,
                 Heuristic<state> *forward, Heuristic<state> *backward, std::vector<state> &thePath);

    bool InitializeSearch(environment *env, const state &from, const state &to,
                          Heuristic<state> *forward, Heuristic<state> *backward, std::vector<state> &thePath);

    bool DoSingleSearchStep(std::vector<state> &thePath);

    virtual const char *GetName() { return "BAE"; }

    void ResetNodeCount() { nodesExpanded = nodesTouched = uniqueNodesExpanded = 0; }

    inline const int GetNumForwardItems() { return forwardQueue.size(); }

    inline const AStarOpenClosedData<state> &GetForwardItem(unsigned int which) { return forwardQueue.Lookat(which); }

    inline const int GetNumBackwardItems() { return backwardQueue.size(); }

    inline const AStarOpenClosedData<state> &GetBackwardItem(unsigned int which) {
        return backwardQueue.Lookat(which);
    }

    uint64_t GetUniqueNodesExpanded() const { return uniqueNodesExpanded; }

    uint64_t GetNodesExpanded() const { return nodesExpanded; }

    uint64_t GetNodesTouched() const { return nodesTouched; }

    uint64_t GetNecessaryExpansions() {
        uint64_t necessary = 0;
        for (const auto &count: counts) {
            if (count.first < currentCost)
                necessary += count.second;
        }
        return necessary;
    }
	void Draw(Graphics::Display &d) const;
	void Draw(Graphics::Display &d, const priorityQueue &q) const;

private:

    void Nip(const state &, priorityQueue &reverse);

    void ExtractPathToGoal(state &node, std::vector<state> &thePath) {
        uint64_t theID;
        backwardQueue.Lookup(env->GetStateHash(node), theID);
        ExtractPathToGoalFromID(theID, thePath);
    }

    void ExtractPathToGoalFromID(uint64_t node, std::vector<state> &thePath) {
        do {
            thePath.push_back(backwardQueue.Lookup(node).data);
            node = backwardQueue.Lookup(node).parentID;
        } while (backwardQueue.Lookup(node).parentID != node);
        thePath.push_back(backwardQueue.Lookup(node).data);
    }

    void ExtractPathToStart(state &node, std::vector<state> &thePath) {
        uint64_t theID;
        forwardQueue.Lookup(env->GetStateHash(node), theID);
        ExtractPathToStartFromID(theID, thePath);
    }

    void ExtractPathToStartFromID(uint64_t node, std::vector<state> &thePath) {
        do {
            thePath.push_back(forwardQueue.Lookup(node).data);
            node = forwardQueue.Lookup(node).parentID;
        } while (forwardQueue.Lookup(node).parentID != node);
        thePath.push_back(forwardQueue.Lookup(node).data);
    }

    void Expand(priorityQueue &current, priorityQueue &opposite, Heuristic<state> *heuristic,
                Heuristic<state> *reverse_heuristic, const state &target, const state &source);

    double getLowerBound();

    priorityQueue forwardQueue, backwardQueue;
    state goal, start;
    uint64_t nodesTouched, nodesExpanded, uniqueNodesExpanded;
    state middleNode; // Meeting point of the current solution
    double currentCost; // Cost of the best solution found so far

    std::vector<state> neighbors;
    environment *env;
    Heuristic<state> *forwardHeuristic;
    Heuristic<state> *backwardHeuristic;

    double epsilon; // Cost of the least-cost edge
    double gcd; // Greatest common denominator between all edges

    bool alternating; // Is the side-choosing policy alternating or not
    bool expandForward; // Is the current expansion direction forward. This is used for the alternating policy

    std::unordered_map<double, int> counts;
};

/**
 * Calculates the current lower-bound bases on min b in both sides, uses the gcd trick from Alcázar et al.
 * @return Current lower-bound on the search
 */
template<class state, class action, class environment, class priorityQueue>
double BAE<state, action, environment, priorityQueue>::getLowerBound() {
    if (forwardQueue.OpenSize() == 0 || backwardQueue.OpenSize() == 0)
        return DBL_MAX;

    double totalErrorForward = forwardQueue.Lookup(forwardQueue.Peek()).h;
    double totalErrorBackward = backwardQueue.Lookup(backwardQueue.Peek()).h;
    double unroundedLowerBound = (totalErrorForward + totalErrorBackward) / 2;

    // round up to the next multiple of gcd
    return ceil(unroundedLowerBound / gcd) * gcd;
}

/**
 * Given a start and a goal, find a path between them
 * @param env A pointer to the domain environment
 * @param from Initial state of the search
 * @param to The goal state
 * @param forward A forward heuristic, i.e., a heuristic from some node n to the goal
 * @param backward A backward heuristic, i.e., a heuristic from the start to some node n
 * @param thePath The solution path which we will fill when the search is done
 */
template<class state, class action, class environment, class priorityQueue>
void BAE<state, action, environment, priorityQueue>::GetPath(environment *env, const state &from, const state &to,
                                                             Heuristic<state> *forward, Heuristic<state> *backward,
                                                             std::vector<state> &thePath) {
    if (InitializeSearch(env, from, to, forward, backward, thePath) == false)
        return;
    while (!DoSingleSearchStep(thePath)) {}
}

/**
 * Initializes the internal structure and data for a new search
 * @param env A pointer to the domain environment
 * @param from Initial state of the search
 * @param to The goal state
 * @param forward A forward heuristic, i.e., a heuristic from some node n to the goal
 * @param backward A backward heuristic, i.e., a heuristic from the start to some node n
 * @param thePath The solution path which we will fill when the search is done
 * @return whether the start and goal are not the same (false means they are the same)
 */
template<class state, class action, class environment, class priorityQueue>
bool BAE<state, action, environment, priorityQueue>::InitializeSearch(environment *env, const state &from,
                                                                      const state &to,
                                                                      Heuristic<state> *forward,
                                                                      Heuristic<state> *backward,
                                                                      std::vector<state> &thePath) {
    this->env = env;
    forwardHeuristic = forward;
    backwardHeuristic = backward;
    currentCost = DBL_MAX;
    forwardQueue.Reset();
    backwardQueue.Reset();
    ResetNodeCount();
    thePath.resize(0);
    start = from;
    goal = to;
    if (start == goal)
        return false;

    forwardQueue.AddOpenNode(start, env->GetStateHash(start), 0, forwardHeuristic->HCost(start, goal));
    backwardQueue.AddOpenNode(goal, env->GetStateHash(goal), 0, backwardHeuristic->HCost(goal, start));

    expandForward = true;
    return true;
}

/**
 * Do a single cycle of search, which means check if the search is over, and if not expand a single node
 * @param thePath The solution path which we will fill when the search is done
 * @return Wether the search was done or not
 */
template<class state, class action, class environment, class priorityQueue>
bool BAE<state, action, environment, priorityQueue>::DoSingleSearchStep(std::vector<state> &thePath) {
    if ((forwardQueue.OpenSize() == 0 || backwardQueue.OpenSize() == 0) && currentCost == DBL_MAX) {
        std::cerr << " !! Problem with no solution?? Expanded: " << nodesExpanded << std::endl;
        exit(0);
    }

    // This means that the best solution so far is better (or equal) than any solution we will be able to achieve from
    // this point forward, i.e., we are done
    if (currentCost <= getLowerBound()) {
        std::vector<state> pFor, pBack;
        ExtractPathToGoal(middleNode, pBack);
        ExtractPathToStart(middleNode, pFor);
        std::reverse(pFor.begin(), pFor.end());
        thePath = pFor;
        thePath.insert(thePath.end(), pBack.begin() + 1, pBack.end());

        return true;
    }

    // If we are not done, expand a single node based on the side-choosing policy set by the user
    if (alternating) { // original BAE* definition
        if (expandForward) {
            Expand(forwardQueue, backwardQueue, forwardHeuristic, backwardHeuristic, goal, start);
            expandForward = false;
        } else {
            Expand(backwardQueue, forwardQueue, backwardHeuristic, forwardHeuristic, start, goal);
            expandForward = true;
        }
    } else { // BS* policy, roughly Pohl's criterion
        if (forwardQueue.OpenSize() > backwardQueue.OpenSize())
            Expand(backwardQueue, forwardQueue, backwardHeuristic, forwardHeuristic, start, goal);
        else
            Expand(forwardQueue, backwardQueue, forwardHeuristic, backwardHeuristic, goal, start);
    }

    return false;
}

/**
 * Expands a single node
 * @param current The current open list from which we will take our node
 * @param opposite The opposite openClosed list (e.g., if current is forward, this is backward)
 * @param heuristic The heuristic for the current direction
 * @param reverse_heuristic The heuristic for the opposite direction
 * @param target The node we are aiming for (goal in forward, start in backward)
 * @param source The node we started from, opposite of target
 */
template<class state, class action, class environment, class priorityQueue>
void BAE<state, action, environment, priorityQueue>::Expand(priorityQueue &current, priorityQueue &opposite,
                                                            Heuristic<state> *heuristic,
                                                            Heuristic<state> *reverse_heuristic,
                                                            const state &target, const state &source) {
    uint64_t nextID;
    bool success = false;
    // This does lazy nipping, which means we do not we do not expand a node that was already closed in the opposite
    // direction. We search until we find one that is not closed
    while (current.OpenSize() > 0) {
        nextID = current.Close();
        uint64_t reverseLoc;
        auto loc = opposite.Lookup(env->GetStateHash(current.Lookup(nextID).data), reverseLoc);
        if (loc != kClosedList) {
            success = true;
            break;
        }
    }

    // This can only fail if we found no nodes like that, meaning the current openList is empty as well
    if (!success)
        return;

    bool foundBetterSolution = false;
    nodesExpanded++;

    // This is to update for necessary expansions
    counts[getLowerBound()] += 1;

    if (current.Lookup(nextID).reopened == false)
        uniqueNodesExpanded++;

    env->GetSuccessors(current.Lookup(nextID).data, neighbors);
    for (auto &succ: neighbors) {
        nodesTouched++;
        uint64_t childID;
        uint64_t hash = env->GetStateHash(succ);
        auto loc = current.Lookup(hash, childID);
        auto &childData = current.Lookup(childID);
        auto &parentData = current.Lookup(nextID);

        double edgeCost = env->GCost(parentData.data, succ);

        // ignore states with greater cost than best solution
        if (fgreatereq(parentData.g + edgeCost+heuristic->HCost(succ, target), currentCost))
            continue;

        switch (loc) {
            case kClosedList: // Since BAE* requires a consistent heuristic, this situation should be impossible
                if (fless(parentData.g + edgeCost, childData.g)) {
                    std::cerr << "  Expanded with non optimal g??????? " << std::endl;
                    exit(0);
                    childData.h = std::max(childData.h, parentData.h - edgeCost);
                    childData.parentID = nextID;
                    childData.g = parentData.g + edgeCost;
                    current.Reopen(childID);
                }
                break;
            case kOpenList: // Update cost if needed
            {
                if (fless(parentData.g + edgeCost, childData.g)) {
                    childData.parentID = nextID;
                    double gDiff = childData.g - (parentData.g + edgeCost);
                    childData.g = parentData.g + edgeCost;
                    // Modify total error accordingly. Since b uses 2g in the formula, and we reduced g (as h and h_r
                    // are static), all that needs to change is 2 times the difference
                    childData.h = childData.h - (2 * gDiff);
                    current.KeyChanged(childID);

                    // Check if we found a potential solution
                    uint64_t reverseLoc;
                    auto loc = opposite.Lookup(hash, reverseLoc);
                    if (loc == kOpenList) {
                        if (fless(parentData.g + edgeCost + opposite.Lookup(reverseLoc).g, currentCost)) {
                            foundBetterSolution = true;
                            currentCost = parentData.g + edgeCost + opposite.Lookup(reverseLoc).g;
                            middleNode = succ;
                        }
                    }
                }
            }
                break;
            case kNotFound: {
                double g = parentData.g + edgeCost;
                double h = std::max(heuristic->HCost(succ, target), epsilon);

                // Ignore nodes that don't have lower f-cost than the incumbent solution
                if (!fless(g + h, currentCost))
                    break;

                double totalError = (2 * g) + h - reverse_heuristic->HCost(succ, source);

                current.AddOpenNode(succ, hash, g, totalError, nextID);

                // Check if we found a potential solution
                uint64_t reverseLoc;
                auto loc = opposite.Lookup(hash, reverseLoc);
                if (loc == kOpenList) {
                    if (fless(current.Lookup(nextID).g + edgeCost + opposite.Lookup(reverseLoc).g, currentCost)) {
                        foundBetterSolution = true;
                        currentCost = current.Lookup(nextID).g + edgeCost + opposite.Lookup(reverseLoc).g;
                        middleNode = succ;
                    }
                }
            }
        }
    }

}

/**
 * A function that does an active nipping
 * @param s The state to nip the successors of
 * @param reverse The openClosed list in the opposite direction to which the state was expanded from
 * @deprecated
 */
template<class state, class action, class environment, class priorityQueue>
void BAE<state, action, environment, priorityQueue>::Nip(const state &s, priorityQueue &reverse)
{
    assert(!"Not using this code currently - the correct implementation of 'remove' is unclear from BS*");
    // At this point parent has been removed from open
    // Need to find any successors that have a parent id of parent & recursively remove them from open

    std::vector<state> n;

    uint64_t parentID;
    auto loc = reverse.Lookup(env->GetStateHash(s), parentID);
    assert(loc == kClosedList);
    env->GetSuccessors(s, n);
    for (auto &succ: n) {
        uint64_t childID;
        uint64_t hash = env->GetStateHash(succ);
        auto loc = reverse.Lookup(hash, childID);
        auto &childData = reverse.Lookup(childID);
        if (loc == kClosedList && childData.parentID == parentID) {
            Nip(childData.data, reverse);
        } else if (loc == kOpenList && (childData.parentID == parentID)) {
            if (childData.data == middleNode) {
                std::cout << "Error - removing middle node\n";
                if (&reverse == &forwardQueue)
                    std::cout << "In backward search - removing from for\n";
                else
                    std::cout << "In forward search - removing from back\n";
                std::cout << "Parent: " << s << "\n";
                std::cout << "Middle: " << middleNode << "\n";
                std::vector<state> pFor, pBack, final;
                ExtractPathToGoal(middleNode, pBack);
                ExtractPathToStart(middleNode, pFor);
                reverse(pFor.begin(), pFor.end());
                std::cout << "Path forward: \n";

                for (auto &s: pFor)
                    std::cout << s << "\n";
                std::cout << "Path backward: \n";
                for (auto &s: pBack)
                    std::cout << s << "\n";

                exit(0);
            }
            reverse.Remove(env->GetStateHash(childData.data));
        }
    }
}
template<class state, class action, class environment, class priorityQueue>
void BAE<state, action, environment, priorityQueue>::Draw(Graphics::Display &disp) const
{
	Draw(disp, forwardQueue);
	Draw(disp, backwardQueue);
}

template<class state, class action, class environment, class priorityQueue>
void BAE<state, action, environment, priorityQueue>::Draw(Graphics::Display &disp, const priorityQueue &q) const
{
	double transparency = 1.0;
	if (q.size() == 0)
		return;
	uint64_t top = -1;
	//	double minf = 1e9, maxf = 0;
	if (q.OpenSize() > 0)
	{
		top = q.Peek();
	}
	for (unsigned int x = 0; x < q.size(); x++)
	{
		const auto &data = q.Lookat(x);
		if (x == top)
		{
			env->SetColor(1.0, 1.0, 0.0, transparency);
			env->Draw(disp, data.data);
		}
		else if ((data.where == kOpenList) && (data.reopened))
		{
			env->SetColor(0.0, 0.5, 0.5, transparency);
			env->Draw(disp, data.data);
		}
		else if (data.where == kOpenList)
		{
			env->SetColor(0.0, 1.0, 0.0, transparency);
			env->Draw(disp, data.data);
		}
		else if ((data.where == kClosedList) && (data.reopened))
		{
			env->SetColor(0.5, 0.0, 0.5, transparency);
			env->Draw(disp, data.data);
		}
		else if (data.where == kClosedList)
		{
			if (&q != &forwardQueue)
				env->SetColor(0.25, 0.5, 1.0, transparency);
			else
				env->SetColor(1.0, 0.0, 0.0, transparency);
			env->Draw(disp, data.data);

//			if (data.parentID == x)
//				env->SetColor(1.0, 0.5, 0.5, transparency);
//			else
//				env->SetColor(1.0, 0.0, 0.0, transparency);
//			//			}
//			env->Draw(disp, data.data);
		}
	}
	env->SetColor(1.0, 0.5, 1.0, 0.5);
	env->Draw(disp, goal);
}

#endif //BAE_H
