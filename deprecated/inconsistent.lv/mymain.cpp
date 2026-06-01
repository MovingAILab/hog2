/*
* mymain.cpp
*
* by Zhifu Zhang
*
*/

#include <cstdio>
#include <cstring>
#include <vector>
#include "Propagation.h"
#include "GraphEnvironment.h"
#include "MapAbstraction.h"
#include "AStarDelay.h"

void processArgs(int argc, char* argv[], char* mfilename, unsigned int &vid, double &delta, int &N, int &fig, bool &singleV) {
	// -map filename -delta del -v vid 

	mfilename[0] = 0;
	int i = 1;
	while(i<argc) 
	{
		if(strcmp(argv[i],"-map")==0) 
		{
			strncpy(mfilename, argv[i+1], 50);
			i += 2;
		}
		else if(strcmp(argv[i],"-v")==0) 
		{
			vid = atoi(argv[i+1]);
			i += 2;
			singleV = true;
		}
		else if(strcmp(argv[i],"-delta")==0)
		{
			sscanf(argv[i+1],"%lf",&delta);
			i += 2; 
		}
		else if(strcmp(argv[i],"-N")==0)
		{
			N = atoi(argv[i+1]);
			i += 2;
		}
		else if(strcmp(argv[i],"-fig")==0)
		{
			fig = atoi(argv[i+1]);
			i += 2;
		}
		else 
		{
			i++;
		}
	}
	
	if(strlen(mfilename)) 
	{
		FILE * mf = fopen(mfilename,"r");
		if(!mf)
		{
			printf("Map file not exists.\n");
			exit(-1);
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

int main(int argc, char* argv[])
{
	char mfname[200];
	unsigned int vid=1;
	double delta=0;
	int N = 5;
	int fig = 1;
	bool singleV = false;

	graphState from=0,to=0;
	std::vector<graphState> thePath;

	processArgs(argc,argv,mfname,vid,delta,N,fig,singleV);

	Graph* g=0;
	Map* mp=0;
	GraphEnvironment* env=0;

	if(strlen(mfname))
	{
		mp = new Map(mfname);
		//g = GetMapGraph(mp);
		g = GraphSearchConstants::GetGraph(mp);
		env = new GraphEnvironment(g, new GraphMapInconsistentHeuristic(mp, g));

		while(from==to) {
			from = g->GetRandomNode()->GetNum();
			to = g->GetRandomNode()->GetNum();
		}
	}
	else 
	{
		if(fig==1) 
		{
			g = PropUtil::graphGenerator::genFig1(N);
			from = N;
			to = 0;
			env = new GraphEnvironment(g, new GraphLabelHeuristic(g, to));
		}
		else if(fig==2)
		{
			g = PropUtil::graphGenerator::genFig2(N);
			from = N;
			to = 0;
			env = new GraphEnvironment(g, new GraphLabelHeuristic(g, to));
		}
		else {
			g = PropUtil::graphGenerator::genFig3(N);
			from = 0;
			to = 2*N - 1;
			env = new GraphEnvironment(g, new GraphLabelHeuristic(g, to));
		}
	}
	
	printf("Environment ready.\n");

	//GraphEnvironment* env = new GraphEnvironment(g);
	Prop alg0(0);
	Prop alg1(1);
	Prop alg2(2);
	Prop alg3(3,delta);
	Prop alg4(4);
	Prop alg5(5);
	AStarDelay alg6;
	Prop alg7(7);
	Prop alg8(8);
	Prop alg9(9);
	Prop alg10(10);

	if(singleV) {
		if(vid<=10 && vid!=6) {
			Prop alg(vid,delta);
			alg.GetPath(env,g,from,to,thePath);
		}
		else if(vid==6) {
			AStarDelay alg;
			alg.GetPath(env,g,from,to,thePath);
		}
		else {
			printf("Wrong vid!\n");
			exit(-1);
		}
		printf("from=%ld,to=%ld\n",from,to);
	}
	else {
		// non-interactive
		alg0.GetPath(env,g,from,to,thePath);
		printf("from=%ld,to=%ld\n",from,to);

		alg1.GetPath(env,g,from,to,thePath);
		printf("from=%ld,to=%ld\n",from,to);

		alg2.GetPath(env,g,from,to,thePath);
		printf("from=%ld,to=%ld\n",from,to);

		alg3.GetPath(env,g,from,to,thePath);
		printf("from=%ld,to=%ld\n",from,to);

		alg4.GetPath(env,g,from,to,thePath);
		printf("from=%ld,to=%ld\n",from,to);

		alg5.GetPath(env,g,from,to,thePath);
		printf("from=%ld,to=%ld\n",from,to);

		alg6.GetPath(env,g,from,to,thePath);
		printf("from=%ld,to=%ld\n",from,to);

		alg7.GetPath(env,g,from,to,thePath);
		printf("from=%ld,to=%ld\n",from,to);

		alg8.GetPath(env,g,from,to,thePath);
		printf("from=%ld,to=%ld\n",from,to);

		alg9.GetPath(env,g,from,to,thePath);
		printf("from=%ld,to=%ld\n",from,to);

		alg10.GetPath(env,g,from,to,thePath);
		printf("from=%ld,to=%ld\n",from,to);
	}

	delete env;
	delete g;
	if(mp)
		delete mp;
	
}
