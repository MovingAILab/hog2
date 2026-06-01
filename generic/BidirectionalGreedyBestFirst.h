#include "TemplateAStar.h"
#include "AStarOpenClosed.h"
#include "BidirGreedyBestFirstFrontier.h"
#include <set>



template <class state, class action, class environment, class openList = AStarOpenClosed<state, AStarCompareWithF<state>, AStarOpenClosedDataWithF<state>> >
class BidirectionalGreedyBestFirst {
public:
	std::set<uint64_t> visited;
    BidirGreedyBestFirstFrontier<state, action, environment, openList> f;// = new CollisionTemplateAStar<state, action, environment>();
	BidirGreedyBestFirstFrontier<state, action, environment, openList> b;// = new CollisionTemplateAStar<state, action, environment>();
	BidirectionalGreedyBestFirst() {
		f.other = &b;
		b.other = &f;
	}
	virtual ~BidirectionalGreedyBestFirst() {}
	void SetPhi(std::function<double(double, double)> p)
	{
		f.SetPhi(p);
		b.SetPhi(p);
	}
	void SetForwardHeuristic(Heuristic<state> *h)
	{
		f.SetHeuristic(h);
	}
	void SetBackwardHeuristic(Heuristic<state> *h)
	{
		b.SetHeuristic(h);
	}
	void SetHeuristic(Heuristic<state> *h) {
		f.SetHeuristic(h);
		b.SetHeuristic(h);
	}
	void GetPath(environment *_env, const state& from, const state& to, std::vector<state> &fPath, std::vector<state> &bPath);
	int GetNodesExpanded(){
		return f.GetNodesExpanded() + b.GetNodesExpanded();
	}
private:
	
};

template <class state, class action, class environment, class openList>
void BidirectionalGreedyBestFirst<state,action,environment,openList>::GetPath(environment *_env, const state& from, const state& to, std::vector<state> &fPath, std::vector<state> &bPath)
{
	//visited.resize(_env->GetMaxHash());
	visited.clear();
	auto f_i = f.InitializeSearch(_env, from, to, fPath);
	auto b_i = b.InitializeSearch(_env, to, from, bPath);
	if (!f_i || !b_i)
  	{	
  		return;
  	}
  	while (true)//!DoSingleSearchStep(thePath))
	{
		auto res_f = f.DoSingleSearchStep(fPath, visited);
		auto res_b = b.DoSingleSearchStep(bPath, visited);
		state rend;
		if (res_f || res_b)
		{
			if (res_f)
			{
				rend = f.GetRendezvous();
			}
			if (res_b)
			{
				rend = b.GetRendezvous();
			}
			//std::cout << "found" << std::endl;
			//std::vector<state> _fPath, _bPath;
			//std::cout << "path" << std::endl;
			//f.ExtractPathToStart(rend, _fPath);
			//std::cout << "path1" << std::endl;
			//b.ExtractPathToStart(rend, _bPath);
			//std::cout << "path2" << std::endl;
//
			//cout << "REND: " << rend << endl;
			//std::cout << "------------------ solution ------------------" << std::endl;
			//for(int i = _fPath.size() - 1; i >= 0; i--)
    		//{
        	//	std::cout << _fPath[i] << std::endl;
    		//}
			//std::cout << "	===" << std::endl;
			//
			//for(int i = 0; i <= _bPath.size() - 1; i++)
    		//{
        	//	std::cout << _bPath[i] << std::endl;
    		//}
			
			if (!f.done && !b.done)
			{
				fPath.resize(0);
				bPath.resize(0);
			}
			else
			{
				f.ExtractPathToStart(rend, fPath);
				b.ExtractPathToStart(rend, bPath);
			}
			break;
		}
	}
}