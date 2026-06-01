/**
*
* @file WeightedUnitGroup.h
* @author Renee Jansen
* @date 09/19/2007
*/
#include "UnitGroup.h"
#include "WeightedMap2DEnvironment.h"

template <class state, class action, class environment>
class WeightedUnitGroup : public UnitGroup<state,action,environment>
{
	public:
		WeightedUnitGroup(environment* e){wme = new WeightedMap2DEnvironment(e); wt = -1; prop = -1; useWindow = false; windowSize = -1; localWeights = false; currDrawEnv = 0; noweighting = false; updateOnQuery = false; updateSurrounding = false; usePerceptron = false; learningRate = 0; }
//		WeightedUnitGroup(double weight, double proportion) {wt = weight; prop = proportion;}
		
		void SetWeight(double wght) {wt = wght;}
		void SetProportion(double proport) { prop=proport;}
		void SetUseWindow(bool b) {useWindow = b;}
		void SetWindowSize(double d) {windowSize = d;}
		void UseLocalWeights(bool b) {localWeights = b;}
		void SetLocalWeightRadius (double r) {localRadius = r;}
		void SetNoWeighting(bool b) {noweighting = b;}
		void SetUpdateOnQuery(double d) {updateOnQuery = true; queryProp = d; }
		void SetUpdateSurrounding(double d) { updateSurrounding = true; surrProp = d; }
		void UsePerceptron(double lr)	{ usePerceptron = true;	learningRate = lr; }

		WeightedMap2DEnvironment* GetWeightedEnvironment() { return wme; }
		virtual ~WeightedUnitGroup() 
		{	
			delete wme; 
			for (unsigned int i=0; i<unitWme.size(); i++) 
			delete unitWme[i];
		}
			
		virtual const char *GetName() { return "WeightedUnitGroup"; }
	
		virtual bool MakeMove(Unit<state, action, environment> *u, environment *e, SimulationInfo<state,action,environment> *si, action& a)
		{
			if (myE != e)
			{
				wme = new WeightedMap2DEnvironment(e);
				
				if (localWeights)
				{	
					for (unsigned int x = 0; x < this->GetMembers().size(); x++)
					{
						unitWme.push_back(new WeightedMap2DEnvironment(e));
						if (wt != -1)
							unitWme[unitWme.size()-1]->SetWeight(wt);
						if (prop != -1)
							unitWme[unitWme.size()-1]->SetProportionOld(prop);
						if ((useWindow)&&(windowSize >= 0))
						{
							unitWme[unitWme.size()-1]->UseWindow(true);
							unitWme[unitWme.size()-1]->SetWindowSize(windowSize);
						}
						if (noweighting)
							unitWme[unitWme.size()-1]->SetNoWeighting(true);
							
						if (updateOnQuery)
						{
							unitWme[unitWme.size()-1]->SetUpdateOnQuery(true);
							unitWme[unitWme.size()-1]->SetQueryProportionOld(queryProp);
						}
						if (updateSurrounding)
						{
							unitWme[unitWme.size()-1]->SetUpdateSurrounding(true);
							unitWme[unitWme.size()-1]->SetSurroundingProportion(surrProp);		
						}	
						if (usePerceptron)
						{
							unitWme[unitWme.size()-1]->UsePerceptron(learningRate);
						}		
								
						environment* uenv = unitWme[unitWme.size()-1];
						unitEnv.push_back(uenv);
					}
					currDrawEnv = unitWme.size();
				}
				
				if (wt != -1)
					wme->SetWeight(wt);
				if (prop != -1)
					wme->SetProportionOld(prop);
				if ((useWindow)&&(windowSize >= 0))
				{
					wme->UseWindow(true);
					wme->SetWindowSize(windowSize);
				}
				if (noweighting)
					wme->SetNoWeighting(true);
					
				if (updateOnQuery)
				{
					wme->SetUpdateOnQuery(true);
					wme->SetQueryProportionOld(queryProp);
				}
				if (updateSurrounding)
				{
					wme->SetUpdateSurrounding(true);
					wme->SetSurroundingProportion(surrProp);		
				}		
				if (usePerceptron)
				{
					wme->UsePerceptron(learningRate);
				}	
				env = wme;
				myE = e;
			}
			if (useWindow) // Set the unit's current location as center for the window 
			{
				xyLoc x;
				u->GetLocation(x);
				wme->SetWindowCenter(x);
			}
			if (localWeights)
			{
				while(unitWme.size() < this->GetMembers().size())
				{
					unitWme.push_back(new WeightedMap2DEnvironment(e));
					environment* uenv = unitWme[unitWme.size()-1];
					unitEnv.push_back(uenv);
				}
				// Update local weights in this unit's environment
				UpdateLocalWeights(u);
				
				environment* thisUnitsEnv = unitEnv[GetUnitIndex(u)];
								
				return (u->MakeMove(thisUnitsEnv, thisUnitsEnv->GetOccupancyInfo(), si,a));
			}
			
			
			//u->GetAlgorithm()->SetWeightedEnvironment(wme);
			
			return (u->MakeMove(env, env->GetOccupancyInfo(), si,a));
		}

		void UpdateLocalWeights(Unit<state,action,environment> *u)
		{
			WeightedMap2DEnvironment* uwme = unitWme[GetUnitIndex(u)];
			xyLoc currLoc;
			u->GetLocation(currLoc);
			
			//std::cout<<std::endl<<"I'm at "<<currLoc<<std::endl;
			for (int x = currLoc.x - (int)localRadius; x<=currLoc.x + (int)localRadius; x++)
			{
			 	if ((x<0)||(x>=uwme->GetMap()->GetMapWidth()))
			 		continue;
				for (int y = currLoc.y - (int)localRadius; y<=currLoc.y + (int)localRadius; y++)
			 	{
			 		if ((y<0)||(y>=uwme->GetMap()->GetMapHeight()))
			 			continue;
			 			
			 		// if this location is within localRadius, update angle
			 		xyLoc thisloc;
			 		thisloc.x = x;
			 		thisloc.y = y;
			 		
			 		if (uwme->HCost(currLoc, thisloc) <= localRadius)
			 		{
			 			//std::cout<<"updating "<<thisloc<<std::endl;
			 			Vector2D angle = wme->GetAngle(thisloc);
			 			uwme->SetAngle(thisloc, angle);
			 		}
			 		
			 	}
			}
		}

		int GetUnitIndex(Unit<state,action,environment> *u)
		{
			for (unsigned int x = 0; x < this->GetMembers().size(); x++)
			{
				if (this->GetMembers()[x] == u)
					return x;
			}
			return -1;
		}

		virtual void UpdateLocation(Unit<state, action, environment> *u, environment *e, state &loc, bool success, SimulationInfo<state,action,environment> *si)
		{
			if (myE != e)
			{
				if (localWeights)
				{	
					for (unsigned int x = 0; x < this->GetMembers().size(); x++)
					{
						unitWme.push_back(new WeightedMap2DEnvironment(e));
						if (wt != -1)
							unitWme[unitWme.size()-1]->SetWeight(wt);
						if (prop != -1)
							unitWme[unitWme.size()-1]->SetProportionOld(prop);
						if ((useWindow)&&(windowSize >= 0))
						{
							unitWme[unitWme.size()-1]->UseWindow(true);
							unitWme[unitWme.size()-1]->SetWindowSize(windowSize);
						}
						if (noweighting)
							unitWme[unitWme.size()-1]->SetNoWeighting(true);
							
						if (updateOnQuery)
						{
							unitWme[unitWme.size()-1]->SetUpdateOnQuery(true);
							unitWme[unitWme.size()-1]->SetQueryProportionOld(queryProp);
						}
						if (updateSurrounding)
						{
							unitWme[unitWme.size()-1]->SetUpdateSurrounding(true);
							unitWme[unitWme.size()-1]->SetSurroundingProportion(surrProp);		
						}	
						if (usePerceptron)
						{
							unitWme[unitWme.size()-1]->UsePerceptron(learningRate);
						}	
						environment* uenv = unitWme[unitWme.size()-1];
						unitEnv.push_back(uenv);
					}
					currDrawEnv = unitWme.size();
				}
				
				wme = new WeightedMap2DEnvironment(e);
				if (wt != -1)
					wme->SetWeight(wt);
				if (prop != -1)
					wme->SetProportionOld(prop);
				if ((useWindow)&&(windowSize >= 0))
				{
					wme->UseWindow(true);
					wme->SetWindowSize(windowSize);
				}	
				if (noweighting)
					wme->SetNoWeighting(true);
					
				if (updateOnQuery)
				{
					wme->SetUpdateOnQuery(true);
					wme->SetQueryProportionOld(queryProp);
				}
				if (updateSurrounding)
				{
					wme->SetUpdateSurrounding(true);
					wme->SetSurroundingProportion(surrProp);		
				}				
				if (usePerceptron)
				{
					wme->UsePerceptron(learningRate);
				}				
				env = wme;
				myE = e;
			}
			xyLoc s;
			u->GetLocation(s);
			wme->UpdateAngle(s, loc,si->GetSimulationTime());
			u->UpdateLocation(env, loc, success, si);
		}
		
		void DrawNextEnvironment()
		{
			if (currDrawEnv != unitWme.size())
				this->GetMembers()[currDrawEnv]->SetColor(1,0,0);
			currDrawEnv = (currDrawEnv + 1)%(unitWme.size()+1);
		}
		
//		virtual void OpenGLDraw(const environment *, const SimulationInfo<state,action,environment> *)  const
//		{ 
//			if (currDrawEnv == unitWme.size())
//				wme->OpenGLDraw();
//			else
//			{
//				//this->GetMembers()[currDrawEnv]->SetColor(0,0,1);
//				unitWme[currDrawEnv]->OpenGLDraw();
//			}
//		}
		
		double ComputeArrowMetric(bool b, double t,bool b2, double mt)
		{
			return wme->ComputeArrowMetric(b,t,b2,mt);
		}
		

		
		
	private:
		WeightedMap2DEnvironment* wme;
		std::vector<WeightedMap2DEnvironment*> unitWme;
		std::vector<environment*> unitEnv;
		environment* env; 
		environment* myE;
		double wt; 
		double prop;
		bool useWindow;
		double windowSize;
		bool localWeights;
		double localRadius;
		unsigned int currDrawEnv;
		bool noweighting;
		bool updateOnQuery;
		double queryProp;
		bool updateSurrounding;
		double surrProp;
		bool usePerceptron;
		double learningRate;
};

