#include <cstdio>
#include <cstring>
#include <vector>
#include <string>
#include <time.h>
#include <math.h>
#include <sys/time.h>
//#include "MNPuzzle.h"
#include "TopSpin.h"
#include <deque>
#include "Propagation.h"
#include "GraphEnvironment.h"
#include "AStarDelay.h"

using namespace std;

//extern int THmode;

void BuildPDB(int psize, int flipsize, int PDB) 
{
	char filename[256];
	sprintf(filename, "TS_%d_%d_%d.db", psize, flipsize, PDB);
	FILE *f = fopen(filename, "r");

	// check if file exists, if so exit
	if(f) {
		fclose(f);
		return;
	}

	//int psize = 12;
	//static int PDB = 1;
	//PDB++;
	TopSpin *ts = new TopSpin(psize, flipsize);

	std::vector<int> P1(psize);
	for (int x = 0; x < psize; x++)
		P1[x] = x;
	graphState s = ts->GetState(P1);
	
	std::vector<uint8_t> DB(ts->GetPDBSize(psize, PDB));
	for (unsigned int x = 0; x < DB.size(); x++)
		DB[x] = 255;
	int entries = 0;
	
	std::cout << s << std::endl;
	std::cout << ts->GetStateHash(s) << std::endl;
	std::cout << ts->GetPDBHash(s, PDB) << std::endl;
	std::cout << "PDB size: " << ts->GetPDBSize(psize, PDB) << std::endl;
	
	std::deque<graphState> q;  // start from goal, using BFS
	std::vector<graphState> moves;
	q.push_back(s);
	DB[ts->GetPDBHash(s, PDB)] = 0;
	entries++;
	while (entries < (int)DB.size())
	{
		graphState next = q.front();
//		printf("Expanding %lu\n", next);
		q.pop_front();
		ts->GetSuccessors(next, moves);
		for (unsigned int x = 0; x < moves.size(); x++)
		{
//			printf("   Child %d: val %lu -- #%d - (db: --) hash: %d\n",
//				   x, moves[x], entries,
//				   ts->GetPDBHash(moves[x], PDB));
			
			if (DB[ts->GetPDBHash(moves[x], PDB)] == 255)
			{
				DB[ts->GetPDBHash(moves[x], PDB)] = DB[ts->GetPDBHash(next, PDB)]+1;
				q.push_back(moves[x]);
				entries++;

				if ((entries%10000) == 0)
					std::cout << entries << std::endl;
			}
		}
		if (q.size() == 0)
		{
			printf("Error! q-size is 0!\n");
			break;
		}
	}
	printf("%d of %d entries found\n", entries, (int)DB.size());
	
	f = fopen(filename, "w+");
	if (f)
    {
		fwrite(&(DB[0]), sizeof(uint8_t), DB.size(), f);
		fclose(f);
    }

	delete ts;
}

graphState randomWalk(int step, std::vector<int>& P1, TopSpin* ts)
{
	std::vector<int> P2;
	P2 = P1;
	graphState s = ts->GetState(P2);
	graphState parent = s, current;

	std::vector<graphState> succ;
	for (int x = 0; x < step; x++)
	{
		ts->GetSuccessors(s, succ);
		if(succ.size() <= 1) break;

		current = s;

		while(true) {
			s = succ[random()%succ.size()];
			if(s != parent)  // disallow the parent
				break;
		}

		parent = current;
	}

	return s;
}

double calcTime(timeval t0, timeval t1) {
	return t1.tv_sec-t0.tv_sec + (t1.tv_usec-t0.tv_usec)/1000000.0;
}

int main(int argc, char* argv[])
{
	int psize  = 12;
	int flipsize  = 4;
	int PDB  = 6;
	int walk = 60;

	char filepath[100];
	struct timeval t0,t1;

	/*if(argc >= 5) {
		psize = atoi(argv[1]);
		flipsize = atoi(argv[2]);
		PDB = atoi(argv[3]);
		walk = atoi(argv[4]);
	}
	else {
		printf("\n\n  ./mymain [psize=]12 [flipsize=]4 [PDB=]6 [walk=]60\n");
	}*/

	//for( walk = 60; walk <= 180; walk += 30)  {
	for(psize = 9; psize <= 14; psize++)
	{
		flipsize = 4;
		PDB = psize / 2;
		walk = psize*10; //psize+2;

		BuildPDB(psize,flipsize,PDB);

		//int sum0=0, sum1=0, sum2=0;
		//sprintf(filepath,"TS.normal.A");
		FILE* ANormaldata = fopen("outputs/TS.normal.A","a");
		FILE* Adata = fopen("outputs/TS.dual.A","a");
		FILE* Bdata = fopen("outputs/TS.dual.B","a");
		FILE* BPdata = fopen("outputs/TS.dual.BP","a");
		FILE* BPMXdata = fopen("outputs/TS.dual.bpmx","a");
		FILE* BPMX2data = fopen("outputs/TS.dual.bpmx2","a");
		FILE* BPMX3data = fopen("outputs/TS.dual.bpmx3","a");
		FILE* BPMXidata = fopen("outputs/TS.dual.bpmxinf","a");

		FILE* ASD2data = fopen("outputs/TS.dual.asd.2","a");
		FILE* ASDlogdata = fopen("outputs/TS.dual.asd.log","a");
		FILE* ASD2bpmxdata = fopen("outputs/TS.dual.asd.2.bpmx","a");
		FILE* ASDlogbpmxdata = fopen("outputs/TS.dual.asd.log.bpmx","a");
		FILE* ASD2bpmxinfdata = fopen("outputs/TS.dual.asd.2.bpmxinf","a");
		FILE* ASDlogbpmxinfdata = fopen("outputs/TS.dual.asd.log.bpmxinf","a");

		FILE* DPdata = fopen("outputs/TS.dual.DP","a");
		FILE* DPbpmxdata = fopen("outputs/TS.dual.DP.bpmx","a");

		FILE* DPDL2MXdata = fopen("outputs/TS.dual.DPDL2MX","a");

		for(int round=0;round<1000;round++)
		{
			//BuildPDB(psize,flipsize,PDB);

			TopSpinGraphHeuristic *tsh;
			TopSpin* ts = new TopSpin(psize, flipsize, tsh = new TopSpinGraphHeuristic(psize, flipsize, PDB));
			tsh->SetState(ts);

			std::vector<int> P1(psize);
			for (int x = 0; x < psize; x++)
				P1[x] = x;
			graphState goal = ts->GetState(P1);

			graphState start = randomWalk(walk,P1,ts);

			std::vector<graphState> path;

			Prop A(PROP_A);
			Prop B(PROP_B);
			Prop BP(PROP_BP);
			
			Prop BPMX(PROP_BPMX);
			Prop BPMX2(PROP_BPMX);
			BPMX2.bpmxLevel = 2;
			Prop BPMX3(PROP_BPMX);
			BPMX3.bpmxLevel = 3;
			Prop BPMXinf(PROP_BPMX);
			BPMXinf.bpmxLevel = 65536;
			//AStarDelay alg2(1);

			AStarDelay asd2(0);
			asd2.fD = D_TWO;
			AStarDelay asdlog(0);
			asdlog.fD = D_LOG2;
			AStarDelay asd2bpmx(1);
			asd2bpmx.fD = D_TWO; 
			AStarDelay asdlogbpmx(1);
			asdlogbpmx.fD = D_LOG2;
			AStarDelay asd2bpmxinf(65536);
			asd2bpmxinf.fD = D_TWO;
			AStarDelay asdlogbpmxinf(65536);
			asdlogbpmxinf.fD = D_LOG2;

			Prop DP(PROP_DP);
			Prop DPMX(PROP_DP);
			DPMX.bpmxLevel = 1;

			Prop DPDL2MX(PROP_DPDLMX);
			
			TopSpinGraphHeuristic::THmode = 0;

			// the first run is for caching !
			A.GetPath((GraphEnvironment*)ts, 0, start, goal, path);

			gettimeofday(&t0,0);
			A.GetPath((GraphEnvironment*)ts, 0, start, goal, path);
			gettimeofday(&t1,0);

			fprintf(ANormaldata,"%d %d %d %d %ld %lf %lf %lu %lu %lu %lu %lu %lu\n", psize, flipsize, PDB, walk, A.GetNodesExpanded(), calcTime(t0,t1), A.solutionCost, (unsigned long)A.nNewExp,(unsigned long)A.nReExp,(unsigned long)A.nBPMX,A.tickNewExp,A.tickReExp,A.tickBPMX);

			TopSpinGraphHeuristic::THmode = 1;
			
			A.GetPath((GraphEnvironment*)ts, 0, start, goal, path);
			gettimeofday(&t0,0);
			A.GetPath((GraphEnvironment*)ts, 0, start, goal, path);
			gettimeofday(&t1,0);

			fprintf(Adata,"%d %d %d %d %ld %lf %lf %lu %lu %lu %lu %lu %lu\n", psize, flipsize, PDB, walk, A.GetNodesExpanded(), calcTime(t0,t1), A.solutionCost, (unsigned long)A.nNewExp,(unsigned long)A.nReExp,(unsigned long)A.nBPMX,A.tickNewExp,A.tickReExp,A.tickBPMX);

			B.GetPath((GraphEnvironment*)ts, 0, start, goal, path);
			gettimeofday(&t0,0);
			B.GetPath((GraphEnvironment*)ts, 0, start, goal, path);
			gettimeofday(&t1,0);

			fprintf(Bdata,"%d %d %d %d %ld %lf %lf %lu %lu %lu %lu %lu %lu\n", psize, flipsize, PDB, walk, B.GetNodesExpanded(), calcTime(t0,t1), B.solutionCost, (unsigned long)B.nNewExp,(unsigned long)B.nReExp,(unsigned long)B.nBPMX,B.tickNewExp,B.tickReExp,B.tickBPMX);

			BP.GetPath((GraphEnvironment*)ts, 0, start, goal, path);
			gettimeofday(&t0,0);
			BP.GetPath((GraphEnvironment*)ts, 0, start, goal, path);
			gettimeofday(&t1,0);

			fprintf(BPdata,"%d %d %d %d %ld %lf %lf %lu %lu %lu %lu %lu %lu\n", psize, flipsize, PDB, walk, BP.GetNodesExpanded(), calcTime(t0,t1), BP.solutionCost, (unsigned long)BP.nNewExp,(unsigned long)BP.nReExp,(unsigned long)BP.nBPMX,BP.tickNewExp,BP.tickReExp,BP.tickBPMX);

			BPMX.GetPath((GraphEnvironment*)ts, 0, start, goal, path);
			gettimeofday(&t0,0);
			BPMX.GetPath((GraphEnvironment*)ts, 0, start, goal, path);
			gettimeofday(&t1,0);

			fprintf(BPMXdata,"%d %d %d %d %ld %lf %lf %lu %lu %lu %lu %lu %lu\n", psize, flipsize, PDB, walk, BPMX.GetNodesExpanded(), calcTime(t0,t1), BPMX.solutionCost, (unsigned long)BPMX.nNewExp,(unsigned long)BPMX.nReExp,(unsigned long)BPMX.nBPMX,BPMX.tickNewExp,BPMX.tickReExp,BPMX.tickBPMX);

			BPMX2.GetPath((GraphEnvironment*)ts, 0, start, goal, path);
			gettimeofday(&t0,0);
			BPMX2.GetPath((GraphEnvironment*)ts, 0, start, goal, path);
			gettimeofday(&t1,0);

			fprintf(BPMX2data,"%d %d %d %d %ld %lf %lf %lu %lu %lu %lu %lu %lu\n", psize, flipsize, PDB, walk, BPMX2.GetNodesExpanded(), calcTime(t0,t1), BPMX2.solutionCost, (unsigned long)BPMX2.nNewExp,(unsigned long)BPMX2.nReExp,(unsigned long)BPMX2.nBPMX,BPMX2.tickNewExp,BPMX2.tickReExp,BPMX2.tickBPMX);

			BPMX3.GetPath((GraphEnvironment*)ts, 0, start, goal, path);
			gettimeofday(&t0,0);
			BPMX3.GetPath((GraphEnvironment*)ts, 0, start, goal, path);
			gettimeofday(&t1,0);

			fprintf(BPMX3data,"%d %d %d %d %ld %lf %lf %lu %lu %lu %lu %lu %lu\n", psize, flipsize, PDB, walk, BPMX3.GetNodesExpanded(), calcTime(t0,t1), BPMX3.solutionCost, (unsigned long)BPMX3.nNewExp,(unsigned long)BPMX3.nReExp,(unsigned long)BPMX3.nBPMX,BPMX3.tickNewExp,BPMX3.tickReExp,BPMX3.tickBPMX);

			BPMXinf.GetPath((GraphEnvironment*)ts, 0, start, goal, path);
			gettimeofday(&t0,0);
			BPMXinf.GetPath((GraphEnvironment*)ts, 0, start, goal, path);
			gettimeofday(&t1,0);

			fprintf(BPMXidata,"%d %d %d %d %ld %lf %lf %lu %lu %lu %lu %lu %lu\n", psize, flipsize, PDB, walk, BPMXinf.GetNodesExpanded(), calcTime(t0,t1), BPMXinf.solutionCost, (unsigned long)BPMXinf.nNewExp,(unsigned long)BPMXinf.nReExp,(unsigned long)BPMXinf.nBPMX,BPMXinf.tickNewExp,BPMXinf.tickReExp,BPMXinf.tickBPMX);

			//printf("Solution:\n");
			//for (unsigned int x = 0; x < path.size(); x++)
			//	printf("%lu   ", path[x]);
			//printf("\n");
			//printf("%ld nodes expanded\n", alg0.GetNodesExpanded());

			asd2.GetPath((GraphEnvironment*)ts, 0, start, goal, path);
			gettimeofday(&t0,0);
			asd2.GetPath((GraphEnvironment*)ts, 0, start, goal, path);
			gettimeofday(&t1,0);

			fprintf(ASD2data,"%d %d %d %d %ld %lf %lf %lu %lu %lu %lu %lu %lu\n", psize, flipsize, PDB, walk, asd2.GetNodesExpanded(), calcTime(t0,t1), asd2.solutionCost, (unsigned long)asd2.nNewExp,(unsigned long)asd2.nReExp,(unsigned long)asd2.nBPMX,asd2.tickNewExp,asd2.tickReExp,asd2.tickBPMX);

			asdlog.GetPath((GraphEnvironment*)ts, 0, start, goal, path);
			gettimeofday(&t0,0);
			asdlog.GetPath((GraphEnvironment*)ts, 0, start, goal, path);
			gettimeofday(&t1,0);

			fprintf(ASDlogdata,"%d %d %d %d %ld %lf %lf %lu %lu %lu %lu %lu %lu\n", psize, flipsize, PDB, walk, asdlog.GetNodesExpanded(), calcTime(t0,t1), asdlog.solutionCost, (unsigned long)asdlog.nNewExp,(unsigned long)asdlog.nReExp,(unsigned long)asdlog.nBPMX,asdlog.tickNewExp,asdlog.tickReExp,asdlog.tickBPMX);

			asd2bpmx.GetPath((GraphEnvironment*)ts, 0, start, goal, path);
			gettimeofday(&t0,0);
			asd2bpmx.GetPath((GraphEnvironment*)ts, 0, start, goal, path);
			gettimeofday(&t1,0);

			fprintf(ASD2bpmxdata,"%d %d %d %d %ld %lf %lf %lu %lu %lu %lu %lu %lu\n", psize, flipsize, PDB, walk, asd2bpmx.GetNodesExpanded(), calcTime(t0,t1), asd2bpmx.solutionCost, (unsigned long)asd2bpmx.nNewExp,(unsigned long)asd2bpmx.nReExp,(unsigned long)asd2bpmx.nBPMX,asd2bpmx.tickNewExp,asd2bpmx.tickReExp,asd2bpmx.tickBPMX);

			asdlogbpmx.GetPath((GraphEnvironment*)ts, 0, start, goal, path);
			gettimeofday(&t0,0);
			asdlogbpmx.GetPath((GraphEnvironment*)ts, 0, start, goal, path);
			gettimeofday(&t1,0);

			fprintf(ASDlogbpmxdata,"%d %d %d %d %ld %lf %lf %lu %lu %lu %lu %lu %lu\n", psize, flipsize, PDB, walk, asdlogbpmx.GetNodesExpanded(), calcTime(t0,t1), asdlogbpmx.solutionCost, (unsigned long)asdlogbpmx.nNewExp,(unsigned long)asdlogbpmx.nReExp,(unsigned long)asdlogbpmx.nBPMX,asdlogbpmx.tickNewExp,asdlogbpmx.tickReExp,asdlogbpmx.tickBPMX);

			asd2bpmxinf.GetPath((GraphEnvironment*)ts, 0, start, goal, path);
			gettimeofday(&t0,0);
			asd2bpmxinf.GetPath((GraphEnvironment*)ts, 0, start, goal, path);
			gettimeofday(&t1,0);

			fprintf(ASD2bpmxinfdata,"%d %d %d %d %ld %lf %lf %lu %lu %lu %lu %lu %lu\n", psize, flipsize, PDB, walk, asd2bpmxinf.GetNodesExpanded(), calcTime(t0,t1), asd2bpmxinf.solutionCost, (unsigned long)asd2bpmxinf.nNewExp,(unsigned long)asd2bpmxinf.nReExp,(unsigned long)asd2bpmxinf.nBPMX,asd2bpmxinf.tickNewExp,asd2bpmxinf.tickReExp,asd2bpmxinf.tickBPMX);

			asdlogbpmxinf.GetPath((GraphEnvironment*)ts, 0, start, goal, path);
			gettimeofday(&t0,0);
			asdlogbpmxinf.GetPath((GraphEnvironment*)ts, 0, start, goal, path);
			gettimeofday(&t1,0);

			fprintf(ASDlogbpmxinfdata,"%d %d %d %d %ld %lf %lf %lu %lu %lu %lu %lu %lu\n", psize, flipsize, PDB, walk, asdlogbpmxinf.GetNodesExpanded(), calcTime(t0,t1), asdlogbpmxinf.solutionCost, (unsigned long)asdlogbpmxinf.nNewExp,(unsigned long)asdlogbpmxinf.nReExp,(unsigned long)asdlogbpmxinf.nBPMX,asdlogbpmxinf.tickNewExp,asdlogbpmxinf.tickReExp,asdlogbpmxinf.tickBPMX);

			DP.GetPath((GraphEnvironment*)ts, 0, start, goal, path);
			gettimeofday(&t0,0);
			DP.GetPath((GraphEnvironment*)ts, 0, start, goal, path);
			gettimeofday(&t1,0);

			fprintf(DPdata,"%d %d %d %d %ld %lf %lf %lu %lu %lu %lu %lu %lu\n", psize, flipsize, PDB, walk, DP.GetNodesExpanded(), calcTime(t0,t1), DP.solutionCost, (unsigned long)DP.nNewExp,(unsigned long)DP.nReExp,(unsigned long)DP.nBPMX,DP.tickNewExp,DP.tickReExp,DP.tickBPMX);

			DPMX.GetPath((GraphEnvironment*)ts, 0, start, goal, path);
			gettimeofday(&t0,0);
			DPMX.GetPath((GraphEnvironment*)ts, 0, start, goal, path);
			gettimeofday(&t1,0);

			fprintf(DPbpmxdata,"%d %d %d %d %ld %lf %lf %lu %lu %lu %lu %lu %lu\n", psize, flipsize, PDB, walk, DPMX.GetNodesExpanded(), calcTime(t0,t1), DPMX.solutionCost, (unsigned long)DPMX.nNewExp,(unsigned long)DPMX.nReExp,(unsigned long)DPMX.nBPMX,DPMX.tickNewExp,DPMX.tickReExp,DPMX.tickBPMX);

			gettimeofday(&t0,0);
			//DPDL2MX.GetPath((GraphEnvironment*)ts, 0, start, goal, path);
			gettimeofday(&t1,0);

			//fprintf(DPDL2MXdata,"%d %d %d %d %ld %lf %lf\n", psize, flipsize, PDB, walk, DPDL2MX.GetNodesExpanded(), calcTime(t0,t1), DPDL2MX.solutionCost);



			//sum0 += alg0.GetNodesExpanded();
			//sum1 += alg1.GetNodesExpanded();
			//sum2 += alg2.GetNodesExpanded();

			delete ts;
			//delete tsh;
		}

		fclose(ANormaldata);
		fclose(Adata);
		fclose(Bdata);
		fclose(BPdata);
		fclose(BPMXdata);
		fclose(BPMX2data);
		fclose(BPMX3data);
		fclose(BPMXidata);
		//FILE *f = fopen("data.log","a");
		
		//fprintf(f, "%ld %ld %ld \n", sum0/50,sum1/50,sum2/50);
		fclose(ASD2data);
		fclose(ASDlogdata);
		fclose(ASD2bpmxdata);
		fclose(ASDlogbpmxdata);
		fclose(ASD2bpmxinfdata);
		fclose(ASDlogbpmxinfdata);
		fclose(DPdata);
		fclose(DPbpmxdata);

		fclose(DPDL2MXdata);
		//fclose(f);
	}

	return 0;
}

// it is unclear which K of delay is best, mostly gives identical expansions.

