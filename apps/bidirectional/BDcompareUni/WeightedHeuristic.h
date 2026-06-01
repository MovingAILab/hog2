#ifndef WEIGHTED_HEURISTIC
#define WEIGHTED_HEURISTIC

#include"Heuristic.h"

template <class state>
class WeightedHeuristic : public Heuristic<state> {
public:

	WeightedHeuristic(Heuristic<state>* o, double w)
		:original(o),weight(w)
	{
	}
	double HCost(const state &a, const state &b) const 
	{
		return weight * original->HCost(a, b);
	}
private:
	double weight;
	Heuristic<state>* original;
};

#endif // !WEIGHTED_HEURISTIC


