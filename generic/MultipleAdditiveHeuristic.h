#include "Heuristic.h"
#include <vector>

using namespace std;

template <class state>
class MultipleAdditiveHeuristic : public Heuristic<state>
{
public:
    vector<Heuristic<state>*> items;
    vector<vector<int>> groups;
    double HCost(const state &node1, const state &node2) const
    {
        double res = 0;
        for (auto group : groups)
        {
            double temp = 0;
            for (auto index : group)
            {
                temp += abs(items[index]->HCost(node1, node1) - items[index]->HCost(node2, node2));
            }
            res = max(res, temp);
        }
        return res;
    }

    void AddGroup(vector<int> group)
    {
        groups.push_back(group);
    }

    void AddHeuristic(Heuristic<state> *h)
    {
        items.push_back(h);
    }
};