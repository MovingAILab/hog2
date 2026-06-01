
#include <cstdio>
#include <cstring>
#include <vector>
#include <string>
#include <unordered_map>
#include <time.h>
#include <math.h>
#include <sys/time.h>
#include "Propagation.h"
#include "GraphEnvironment.h"
#include "MapAbstraction.h"
#include "AStarDelay.h"
#include "ScenarioLoader.h"
//#include "Map2DEnvironment.h"

using namespace std;
using namespace __gnu_cxx;

//extern int hmode;  // in GraphEnvironment.cpp

char filelist[1024] = {0}; // this is a filename
double delta=0;
//extern int HN;

#define MAX_ALG_ID 0

void processArgs(int argc, char* argv[]) {
	// -list scenfilelist -HN hn -delta delta

	filelist[0] = 0;
	int i = 1;
	while(i<argc) 
	{
		if(strcmp(argv[i],"-list")==0) 
		{
			strncpy(filelist, argv[i+1], 1024);
			i += 2;
		}
		else if(strcmp(argv[i],"-HN")==0) 
		{
			GraphMapInconsistentHeuristic::HN = atoi(argv[i+1]);
			i += 2;
		}
		else if(strcmp(argv[i],"-delta")==0)
		{
			sscanf(argv[i+1],"%lf",&delta);
			i += 2; 
		}
		else 
		{
			i++;
		}
	}
	
	if(strlen(filelist)) 
	{
		FILE * mf = fopen(filelist,"r");
		if(!mf)
		{
			printf("Catelog file not exists.\n");
			exit(-2);
		}
		else
		{
			fclose(mf);
		}
	}
	else 
	{
		//printf("Map file not specified.\n");
		//exit(-1);
		//g = PropUtil::graphGenerator::genFig1(N);
	}
}

struct IncStat {
	int expanded;
	double time;

	IncStat() {
		expanded = 0;
		time = 0;
	}
	IncStat(int e, double t) {
		expanded = e;
		time = t;
	}
};

double calcTime(timeval t0, timeval t1) {
	return t1.tv_sec-t0.tv_sec + (t1.tv_usec-t0.tv_usec)/1000000.0;
}

void remove_newline(char* buf) {  // checked
	if(!strlen(buf))
		return;
	if(buf[strlen(buf)-1] == '\n')
		buf[strlen(buf)-1] = 0;
	while(strlen(buf) && buf[strlen(buf)-1]==' ') { // remove tailing space as well
		buf[strlen(buf)-1] = 0;
	}
	if(!strlen(buf))
		return;
	int sht = 0;
	while(buf[sht]==' ') {
		sht++;
	}
	memmove(buf,buf+sht,strlen(buf)-sht + 1);
}

int main(int argc, char* argv[])
{
	char scenfile[1024];
	char line[1024];

	processArgs(argc,argv);

	// Note: bid*4 is the actual bucket value

	// init stat collection here !
	hash_map<int, vector<vector<IncStat> > > stat;  // [bid][alg][scenid]

	hash_map<int, int> maxDiffX; // [bid]
	hash_map<int, int> maxDiffY; // [bid]

	FILE* catelog = fopen(filelist,"r");
	if(catelog == 0)
		exit(-1);

	while(1) {
		if(fgets(line,1024,catelog)==0)
			break;
		remove_newline(line);
		if(strlen(line)==0)
			continue;
		sprintf(scenfile,"scenarios/%s",line);

		ScenarioLoader* sl;
		sl = new ScenarioLoader(scenfile);
		if(sl->GetNumExperiments()==0)
		{
			std::cout<<"No experiments in this scenario file or invalid file:"<<scenfile<<endl;
			//exit(1);
			delete sl;
			continue;
		}

		//fprintf(stderr,"line=%s\n",line);

		vector<IncStat> tmp;
		for(int iexp=0;iexp<sl->GetNumExperiments();iexp++) {
			Experiment exp = sl->GetNthExperiment(iexp);
			if(stat[exp.GetBucket()].size() > 0)
				continue;

			for(int i=0;i<=MAX_ALG_ID;i++) {// we have 9 algs 
				stat[exp.GetBucket()].push_back(tmp);
			}
		}
		delete sl;
	}

	Graph* g=0;
	Map* mp=0;
	GraphMapInconsistentHeuristic* heuristic = 0;
	GraphEnvironment* env=0;

	string currentMap;
	int currentScaleX=-1;
	int currentScaleY=-1;

	fclose(catelog);
	catelog = fopen(filelist,"r");

	while(1) 
	{
		if(fgets(line,1024,catelog)==0)
			break;
		remove_newline(line);
		if(strlen(line)==0)
			continue;
		sprintf(scenfile,"scenarios/%s",line);

		ScenarioLoader* sl;
		vector<graphState> thePath;
		
		sl = new ScenarioLoader(scenfile);

		if(sl->GetNumExperiments()==0)
		{
			std::cout<<"No experiments in this scenario file or invalid file.\n";
			//exit(1);
			delete sl;
			continue;
		}

		// init buckets
		//hash_map<int, vector<vector<InconsistStat> > > statBuckets; // [bucket][alg][item]
		//for(int iexp=0;iexp<sl->GetNumExperiments();iexp++) {
		//	Experiment exp = sl->GetNthExperiment(iexp);
		//	if(statBuckets[exp.GetBucket()].size() > 0)
		//		continue;
		//	for(int i=0;i<10;i++) {// we have 9 algs 
		//		vector<InconsistStat> tmp;
		//		statBuckets[exp.GetBucket()].push_back(tmp);
		//	}
		//}

		char datapath[100];
		sprintf(datapath,"outputs/DB%d.dat.AOct",GraphMapInconsistentHeuristic::HN);
		FILE *AOctdata = fopen(datapath,"a");

		sprintf(datapath,"outputs/DB%d.dat.AMax",GraphMapInconsistentHeuristic::HN);
		FILE *AMaxdata = fopen(datapath,"a");

		sprintf(datapath,"outputs/DB%d.dat.A",GraphMapInconsistentHeuristic::HN);
		FILE *Adata = fopen(datapath,"a");

		sprintf(datapath,"outputs/DB%d.dat.B",GraphMapInconsistentHeuristic::HN);
		FILE *Bdata = fopen(datapath,"a");

		sprintf(datapath,"outputs/DB%d.dat.BP",GraphMapInconsistentHeuristic::HN);
		FILE *BPdata = fopen(datapath,"a");

		sprintf(datapath,"outputs/DB%d.dat.bpmx",GraphMapInconsistentHeuristic::HN);
		FILE *BPMXdata = fopen(datapath,"a");

		sprintf(datapath,"outputs/DB%d.dat.bpmx2",GraphMapInconsistentHeuristic::HN);
		FILE *BPMX2data = fopen(datapath,"a");

		sprintf(datapath,"outputs/DB%d.dat.bpmx3",GraphMapInconsistentHeuristic::HN);
		FILE *BPMX3data = fopen(datapath,"a");

		sprintf(datapath,"outputs/DB%d.dat.bpmxinf",GraphMapInconsistentHeuristic::HN);
		FILE *BPMXidata = fopen(datapath,"a");

		sprintf(datapath,"outputs/DB%d.dat.asd.2",GraphMapInconsistentHeuristic::HN);
		FILE *ASD2data = fopen(datapath,"a");

		sprintf(datapath,"outputs/DB%d.dat.asd.2.bpmx",GraphMapInconsistentHeuristic::HN);
		FILE *ASD2bpmxdata = fopen(datapath,"a");

		sprintf(datapath,"outputs/DB%d.dat.asd.2.bpmxinf",GraphMapInconsistentHeuristic::HN);
		FILE *ASD2bpmxinfdata = fopen(datapath,"a");

		sprintf(datapath,"outputs/DB%d.dat.DP",GraphMapInconsistentHeuristic::HN);
		FILE *DPdata = fopen(datapath,"a");

		sprintf(datapath,"outputs/DB%d.dat.DP.bpmx",GraphMapInconsistentHeuristic::HN);
		FILE *DPbpmxdata = fopen(datapath,"a");

		sprintf(datapath,"outputs/DB%d.dat.DPDL2MX",GraphMapInconsistentHeuristic::HN);
		FILE *DPDL2MXdata = fopen(datapath,"a");

		sprintf(datapath,"outputs/DB%d.dat.NULL",GraphMapInconsistentHeuristic::HN);
		FILE *NULLdata = fopen(datapath,"a");

		for(int iexp=0;iexp<sl->GetNumExperiments();iexp++)
		{
			Experiment exp = sl->GetNthExperiment(iexp);

			char mname[1024];
			//xyLoc locstart, locgoal;
			struct timeval t0,t1;

			exp.GetMapName(mname);

			if((string)(mname)==currentMap && exp.GetXScale()==currentScaleX && exp.GetYScale()==currentScaleY){
				// rebuild the map,g,env only when necessary
			}
			else {
				if(mp != 0) {
					delete mp;
					//delete g;
					//delete heuristic;
					delete env;
				}
				mp = new Map(mname);
				if(exp.GetXScale() <= 0) {
					printf("Invalid scaling\n");
					//exit(-1);
					delete mp;
					continue;
				}
				currentMap = (string)(mname);
				currentScaleX = exp.GetXScale();
				currentScaleY = exp.GetYScale();
				mp->scale(exp.GetXScale(), exp.GetYScale());
				g = GraphSearchConstants::GetGraph(mp);
				heuristic = new GraphMapInconsistentHeuristic(mp, g);
				env = new GraphEnvironment(g,heuristic);  // 
			}


			//mp = new Map(mname);
			//// need to scale the map !
			//if(exp.GetXScale() <= 0) {
			//	printf("Invalid scaling\n");
			//	exit(-1);
			//}
			//mp->scale(exp.GetXScale(), exp.GetYScale());

			//g = GetMapGraph(mp);


			// identify from & to
			graphState from, to;

			from = mp->getNodeNum(exp.GetStartX(),exp.GetStartY());
			to = mp->getNodeNum(exp.GetGoalX(),exp.GetGoalY());

			int bid = exp.GetBucket();

			//maxDiffX[bid] = (int)max(maxDiffX[bid],abs(exp.GetStartX()-exp.GetGoalX()));
			//maxDiffY[bid] = (int)max(maxDiffY[bid],abs(exp.GetStartY()-exp.GetGoalY()));

	//printf("scene=%s,iexp=%d\n",scenfile,iexp);

			Prop A(PROP_A);
			Prop B(PROP_B);
			Prop BP(PROP_BP);
			Prop BPMX1(PROP_BPMX);
			Prop BPMX2(PROP_BPMX);
			BPMX2.bpmxLevel = 2;
			Prop BPMX3(PROP_BPMX);
			BPMX3.bpmxLevel = 3;
			Prop BPMXi(PROP_BPMX);
			BPMXi.bpmxLevel = 65536;

			AStarDelay asd(0);
			AStarDelay asdmx(1);
			AStarDelay asdmxinf(65536);

			Prop DP(PROP_DP);
			DP.bpmxLevel = 0;

			Prop DPMX(PROP_DP);
			DPMX.bpmxLevel = 1;

			Prop DPDL2MX(PROP_DPDLMX);

			//alg0.bpmxLevel = 1;
			//AStarDelay alg0(1);
			//Prop alg1(PROP_A);
			//Prop alg2(PROP_A);
			//Prop alg2(2);
			//Prop alg3(3,delta);
			//Prop alg4(4);
			//Prop alg5(5);
			//AStarDelay alg6;
			//Prop alg7(7);
			//Prop alg8(8);
			//Prop alg9(9);
			// the following are special ! A* with a different heuristic
			//Prop alg10(0); // upper bound
			//Prop alg11(0); // lower bound ??

			GraphMapInconsistentHeuristic::hmode = 0;

			gettimeofday(&t0,0);
			A.GetPath(env,g,from,to,thePath);
			gettimeofday(&t1,0);

			//fprintf(NULLdata,"%ld %lf %d\n",A.GetNodesExpanded(), A.solutionCost, bid);
			fprintf(AOctdata,"%ld %lf %d %lu %lu %lu %lu %lu %lu\n",A.GetNodesExpanded(), calcTime(t0,t1), bid, (unsigned long)A.nNewExp,(unsigned long)A.nReExp,(unsigned long)A.nBPMX,A.tickNewExp,A.tickReExp,A.tickBPMX);

			GraphMapInconsistentHeuristic::hmode = 2;

			gettimeofday(&t0,0);
			A.GetPath(env,g,from,to,thePath);
			gettimeofday(&t1,0);

			fprintf(AMaxdata,"%ld %lf %d %lu %lu %lu %lu %lu %lu\n",A.GetNodesExpanded(), calcTime(t0,t1), bid, (unsigned long)A.nNewExp,(unsigned long)A.nReExp,(unsigned long)A.nBPMX,A.tickNewExp,A.tickReExp,A.tickBPMX);

			GraphMapInconsistentHeuristic::hmode = 1;  // careful !!!!

			gettimeofday(&t0,0);
			A.GetPath(env,g,from,to,thePath);
			gettimeofday(&t1,0);

			fprintf(Adata,"%ld %lf %d %lu %lu %lu %lu %lu %lu\n", A.GetNodesExpanded(), calcTime(t0,t1), bid, (unsigned long)A.nNewExp,(unsigned long)A.nReExp,(unsigned long)A.nBPMX,A.tickNewExp,A.tickReExp,A.tickBPMX);

			gettimeofday(&t0,0);
			B.GetPath(env,g,from,to,thePath);
			gettimeofday(&t1,0);

			fprintf(Bdata,"%ld %lf %d %lu %lu %lu %lu %lu %lu\n",B.GetNodesExpanded(), calcTime(t0,t1), bid, (unsigned long)B.nNewExp,(unsigned long)B.nReExp,(unsigned long)B.nBPMX,B.tickNewExp,B.tickReExp,B.tickBPMX);

			gettimeofday(&t0,0);
			BP.GetPath(env,g,from,to,thePath);
			gettimeofday(&t1,0);

			fprintf(BPdata,"%ld %lf %d %lu %lu %lu %lu %lu %lu\n",BP.GetNodesExpanded(), calcTime(t0,t1), bid, (unsigned long)BP.nNewExp,(unsigned long)BP.nReExp,(unsigned long)BP.nBPMX,BP.tickNewExp,BP.tickReExp,BP.tickBPMX);

			gettimeofday(&t0,0);
			BPMX1.GetPath(env,g,from,to,thePath);
			gettimeofday(&t1,0);

			fprintf(BPMXdata,"%ld %lf %d %lu %lu %lu %lu %lu %lu\n",BPMX1.GetNodesExpanded(), calcTime(t0,t1), bid, (unsigned long)BPMX1.nNewExp,(unsigned long)BPMX1.nReExp,(unsigned long)BPMX1.nBPMX,BPMX1.tickNewExp,BPMX1.tickReExp,BPMX1.tickBPMX);

			gettimeofday(&t0,0);
			BPMX2.GetPath(env,g,from,to,thePath);
			gettimeofday(&t1,0);

			fprintf(BPMX2data,"%ld %lf %d %lu %lu %lu %lu %lu %lu\n",BPMX2.GetNodesExpanded(), calcTime(t0,t1), bid, (unsigned long)BPMX2.nNewExp,(unsigned long)BPMX2.nReExp,(unsigned long)BPMX2.nBPMX,BPMX2.tickNewExp,BPMX2.tickReExp,BPMX2.tickBPMX);

			gettimeofday(&t0,0);
			BPMX3.GetPath(env,g,from,to,thePath);
			gettimeofday(&t1,0);

			fprintf(BPMX3data,"%ld %lf %d %lu %lu %lu %lu %lu %lu\n",BPMX3.GetNodesExpanded(), calcTime(t0,t1), bid, (unsigned long)BPMX3.nNewExp,(unsigned long)BPMX3.nReExp,(unsigned long)BPMX3.nBPMX,BPMX3.tickNewExp,BPMX3.tickReExp,BPMX3.tickBPMX);

			gettimeofday(&t0,0);
			BPMXi.GetPath(env,g,from,to,thePath);
			gettimeofday(&t1,0);

			fprintf(BPMXidata,"%ld %lf %d %lu %lu %lu %lu %lu %lu\n",BPMXi.GetNodesExpanded(), calcTime(t0,t1), bid, (unsigned long)BPMXi.nNewExp,(unsigned long)BPMXi.nReExp,(unsigned long)BPMXi.nBPMX,BPMXi.tickNewExp,BPMXi.tickReExp,BPMXi.tickBPMX);

			gettimeofday(&t0,0);
			asd.GetPath(env,g,from,to,thePath);
			gettimeofday(&t1,0);

			fprintf(ASD2data,"%ld %lf %d %lu %lu %lu %lu %lu %lu\n",asd.GetNodesExpanded(), calcTime(t0,t1), bid, (unsigned long)asd.nNewExp,(unsigned long)asd.nReExp,(unsigned long)asd.nBPMX,asd.tickNewExp,asd.tickReExp,asd.tickBPMX);

			gettimeofday(&t0,0);
			asdmx.GetPath(env,g,from,to,thePath);
			gettimeofday(&t1,0);

			fprintf(ASD2bpmxdata,"%ld %lf %d %lu %lu %lu %lu %lu %lu\n",asdmx.GetNodesExpanded(), calcTime(t0,t1), bid, (unsigned long)asdmx.nNewExp,(unsigned long)asdmx.nReExp,(unsigned long)asdmx.nBPMX,asdmx.tickNewExp,asdmx.tickReExp,asdmx.tickBPMX);

			gettimeofday(&t0,0);
			asdmxinf.GetPath(env,g,from,to,thePath);
			gettimeofday(&t1,0);

			fprintf(ASD2bpmxinfdata,"%ld %lf %d %lu %lu %lu %lu %lu %lu\n",asdmxinf.GetNodesExpanded(), calcTime(t0,t1), bid, (unsigned long)asdmxinf.nNewExp,(unsigned long)asdmxinf.nReExp,(unsigned long)asdmxinf.nBPMX,asdmxinf.tickNewExp,asdmxinf.tickReExp,asdmxinf.tickBPMX);

			gettimeofday(&t0,0);
			DP.GetPath(env,g,from,to,thePath);
			gettimeofday(&t1,0);

			fprintf(DPdata,"%ld %lf %d %lu %lu %lu %lu %lu %lu\n",DP.GetNodesExpanded(), calcTime(t0,t1), bid, (unsigned long)DP.nNewExp,(unsigned long)DP.nReExp,(unsigned long)DP.nBPMX,DP.tickNewExp,DP.tickReExp,DP.tickBPMX);

			gettimeofday(&t0,0);
			DPMX.GetPath(env,g,from,to,thePath);
			gettimeofday(&t1,0);

			fprintf(DPbpmxdata,"%ld %lf %d %lu %lu %lu %lu %lu %lu\n",DPMX.GetNodesExpanded(), calcTime(t0,t1), bid, (unsigned long)DPMX.nNewExp,(unsigned long)DPMX.nReExp,(unsigned long)DPMX.nBPMX,DPMX.tickNewExp,DPMX.tickReExp,DPMX.tickBPMX);

			gettimeofday(&t0,0);
			//DPDL2MX.GetPath(env,g,from,to,thePath);
			gettimeofday(&t1,0);

			//fprintf(DPDL2MXdata,"%ld %lf %d\n",DPDL2MX.GetNodesExpanded(), calcTime(t0,t1), bid);

			//====================================================
	
			//gettimeofday(&t0,0);
			//alg1.GetPath(env,g,from,to,thePath);
			//gettimeofday(&t1,0);

			//hmode = 2;

			//gettimeofday(&t0,0);
			//alg1.GetPath(env,g,from,to,thePath);
			//gettimeofday(&t1,0);
			//stat[bid][1].push_back(IncStat(alg1.GetNodesExpanded(),calcTime(t0,t1)));

			//if(! (alg0.solutionCost == alg1.solutionCost )) 
			//{
				//printf("scenfile= %s , exp = %d, %lf, %lf  \n", line, iexp, alg0.solutionCost, alg1.solutionCost);
				//exit(-1);
			//}

			//if(strcmp(line,"AR0205SR.map.txt") == 0 && iexp == 498) {
			//	printf("The cost is %lf \n", alg0.solutionCost);
			//	exit(-1);
			//}

			//gettimeofday(&t0,0);
			//alg2.GetPath(env,g,from,to,thePath);
			//gettimeofday(&t1,0);
			//stat[bid][2].push_back(IncStat(alg2.GetNodesExpanded(),calcTime(t0,t1)));

			//gettimeofday(&t0,0);
			//alg3.GetPath(env,g,from,to,thePath);
			//gettimeofday(&t1,0);
			//stat[bid][3].push_back(IncStat(alg3.GetNodesExpanded(),calcTime(t0,t1)));

			////alg4.GetPath(env,g,from,to,thePath);

			////alg5.GetPath(env,g,from,to,thePath);

			//gettimeofday(&t0,0);
			//alg6.GetPath(env,g,from,to,thePath);
			//gettimeofday(&t1,0);
			//stat[bid][6].push_back(IncStat(alg6.GetNodesExpanded(),calcTime(t0,t1)));

			//gettimeofday(&t0,0);
			//alg7.GetPath(env,g,from,to,thePath);
			//gettimeofday(&t1,0);
			//stat[bid][7].push_back(IncStat(alg7.GetNodesExpanded(),calcTime(t0,t1)));

			//gettimeofday(&t0,0);
			//alg8.GetPath(env,g,from,to,thePath);
			//gettimeofday(&t1,0);
			//stat[bid][8].push_back(IncStat(alg8.GetNodesExpanded(),calcTime(t0,t1)));

			//gettimeofday(&t0,0);
			//alg9.GetPath(env,g,from,to,thePath);
			//gettimeofday(&t1,0);
			//stat[bid][9].push_back(IncStat(alg9.GetNodesExpanded(),calcTime(t0,t1)));

			//hmode = 0;
			//gettimeofday(&t0,0);
			//alg10.GetPath(env,g,from,to,thePath);
			//gettimeofday(&t1,0);
			//stat[bid][10].push_back(IncStat(alg10.GetNodesExpanded(),calcTime(t0,t1)));
		
			//hmode = 2;
			//gettimeofday(&t0,0);
			//alg11.GetPath(env,g,from,to,thePath);
			//gettimeofday(&t1,0);
			//stat[bid][11].push_back(IncStat(alg11.GetNodesExpanded(),calcTime(t0,t1)));

			//delete env;
			//delete g;
			//delete mp;
		}

		fclose(AOctdata);
		fclose(AMaxdata);
		fclose(Adata);
		fclose(Bdata);
		fclose(BPdata);
		fclose(BPMXdata);
		fclose(BPMX2data);
		fclose(BPMX3data);
		fclose(BPMXidata);

		fclose(ASD2data);
		fclose(ASD2bpmxdata);
		fclose(ASD2bpmxinfdata);
		fclose(DPdata);
		fclose(DPbpmxdata);

		fclose(DPDL2MXdata);

		fclose(NULLdata);

		// Note: bid*4 is the actual bucket value

		// summary of the stat

		// write to file, 
		//FILE *outlog,*outtable,*outstat;
		//char log[1024],table[1024],st[1024];
		//sprintf(log,"outputs/run.hn%d.log",HN);
		//sprintf(table,"outputs/run.hn%d.table",HN);
		//sprintf(st,"outputs/run.hn%d.stat",HN);
		//outlog = fopen(log,"a"); // append
		//outtable = fopen(table,"w");
		//outstat = fopen(st,"w");

		//for(hash_map<int, vector<vector<IncStat> > >::iterator biter=stat.begin();biter!=stat.end();biter++)
		//{
		//	int vid=0;
		//	for(vector<vector<IncStat> >::iterator viter=biter->second.begin();viter != biter->second.end(); viter++,vid++) 
		//	{
		//		if(vid == 4 || vid == 5)
		//			continue;
		//		// compute the ave
		//		double expanded=0,time=0; // total
		//		for(vector<IncStat>::iterator iter=viter->begin();iter!=viter->end();iter++) 
		//		{
		//			expanded += iter->expanded;
		//			time += iter->time;
		//		}
		//		expanded /= viter->size();
		//		time /= viter->size();
		//		fprintf(outstat,"%d %d %lf %lf\n",4*biter->first,vid,expanded,time); // bucket = 4*bid

		//		// compute std, max/min
		//		double maxExp=0,maxTime=0;
		//		double minExp=DBL_MAX,minTime=DBL_MAX;
		//		double stdExp =0;
		//		double stdTime = 0;
		//		for(vector<IncStat>::iterator iter=viter->begin();iter!=viter->end();iter++)
		//		{
		//			maxExp = max(maxExp,iter->expanded);
		//			maxTime = max(maxTime,iter->time);
		//			minExp = min(minExp,iter->expanded);
		//			minTime = min(minTime,iter->time);
		//			stdExp += pow(expanded-iter->expanded,2);
		//			stdTime += pow(time-iter->time,2);
		//		}
		//		stdExp = sqrt(stdExp/viter->size());
		//		stdTime = sqrt(stdTime/viter->size());
		//		fprintf(outtable,"%d %d %lf %lf %d %lf %d %lf %d %d\n",4*biter->first,vid,stdExp,stdTime,(int)maxExp,maxTime,(int)minExp,minTime,maxDiffX[biter->first],maxDiffY[biter->first]);
		//	}
		//}

		//fprintf(outlog,"%s\n",scenfile); // what file has been processed

		delete sl;

		//fclose(outlog);
		//fclose(outtable);
		//fclose(outstat);
	}

	fclose(catelog);

	if(mp != 0) {
		delete mp;
		//delete g;
		//delete heuristic;
		delete env;
	}

	return 0;
}

/* how to graph:
1) expansion graph: 
   x-axis: buckets
   y-axis: expanded
   each alg has 1 line
2) time graph:
   x-axis: buckets
   y-axis: time
   each alg has 1 line
*/
