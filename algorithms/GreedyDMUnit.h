#include "Unit.h"
#include "WeightedMap2DEnvironment.h"

template <class environment>
class GreedyDMUnit : public Unit<xyLoc,tDirection,environment>
{
	public:
		GreedyDMUnit(xyLoc &startLoc) :loc(startLoc) {}
		~GreedyDMUnit() {}
		
		virtual const char *GetName() { return "Greedy DM following unit"; }
		
		virtual bool MakeMove(environment *theEnv, OccupancyInterface<xyLoc, tDirection> *, SimulationInfo<xyLoc,tDirection,environment> *, tDirection& a)
		{
//			if (random()%10)
//			{ a = kStay; return true; }
			
			std::vector<tDirection> directions;
			directions.push_back(kN);
			directions.push_back(kNE);
			directions.push_back(kE);
			directions.push_back(kSE);
			directions.push_back(kS);
			directions.push_back(kSW);
			directions.push_back(kW);
			directions.push_back(kNW);
	
			Vector2D nodeVec = wme->GetAngle(loc);
		
			// if no angle stored, pick random
			if (nodeVec.x == 0 && nodeVec.y == 0)
			{ 	
				a = directions[random()%directions.size()];
				return true;
			}		
					
			double bestDotProd = -1;
			double secondBestValue = -1;
			tDirection bestDir = kStay;		
			tDirection secondBest = kStay;	
			
			//std::vector<double> dotProds;
				
			for (unsigned int i=0; i<directions.size(); i++)
			{	 		
				xyLoc nextThisDir;
				wme->GetNextState(loc,directions[i],nextThisDir);
				//if (prevLoc == nextThisDir)
				//	continue;	
				if (theEnv->GetMap()->CanStep(loc.x, loc.y, nextThisDir.x, nextThisDir.y))
				{
					if (theEnv->GetOccupancyInfo() && theEnv->GetOccupancyInfo()->GetStateOccupied(nextThisDir))
						continue;
					Vector2D dirVec = wme->GetAngleFromDirection(directions[i]);
					Vector2D nextVec = wme->GetAngle(nextThisDir);
					
					double dotProd1 = (nodeVec.x * dirVec.x) + (nodeVec.y * dirVec.y);
					double dotProd2 = (dirVec.x * nextVec.x) + (dirVec.y * nextVec.y);
					
					double dotProd = 0.5 * dotProd1 + 0.5 * dotProd2;
					
					if (nextVec.x == 0 && nextVec.y == 0)
						dotProd = -1;

					//dotProds.push_back(dotProd);
					
					if (dotProd >= bestDotProd)
					{
						//if (abs(bestDotProd-dotProd) < 0.1)
						secondBest = bestDir;
						secondBestValue = bestDotProd;
						bestDotProd = dotProd;
						
						bestDir = directions[i];
					}	
					
					
				}		
			}
			
			// find all dot products within epsilon of best, then randomly choose one of them.
			
			
// 			xyLoc next;
// 			wme->GetNextState(loc, bestDir, next);
// 			
// 			//std::cout<<"loc "<<loc<<" next "<<next<<std::endl;
// 			if (!(env->GetMap()->CanStep(loc.x, loc.y, next.x, next.y))) // make best possible move
// 			{
// 				
// 				//std::cout<<"can't go here - random\n";
// 				a = directions[random()%directions.size()];
// 			}	
// 			else
	
	
			if (secondBest == kStay)
				a = bestDir;
			else
			{
				if ((bestDotProd - secondBestValue < 0.1) && (random()%4 == 1))
					a = secondBest;
				else
					a = bestDir;
			}
// 			else
// 				a = directions[random()%directions.size()];
			
			return true;
		}
		
		
		virtual void UpdateLocation(environment *, xyLoc &newloc, bool success, SimulationInfo<xyLoc,tDirection,environment> *)
		{
			if (success)
			{
				prevLoc = loc;
				loc = newloc;
			}
		}
		
		virtual void GetLocation(xyLoc &l)
		{ l = loc; }
	
//		virtual void OpenGLDraw(const environment *theEnv, const SimulationInfo<xyLoc,tDirection,environment> *) const
//		{ theEnv->OpenGLDraw(loc); }
		
		virtual void GetGoal(xyLoc &s)
		{ s = loc; }
		
		void SetEnvironment(WeightedMap2DEnvironment *w){wme = w;}
		
	private:
		xyLoc loc, prevLoc;
		WeightedMap2DEnvironment *wme;
};
