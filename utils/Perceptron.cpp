/*
 *  Perceptron.cpp
 *  games
 *
 *  Created by Nathan Sturtevant on 3/2/05.
 *  Copyright 2005 __MyCompanyName__. All rights reserved.
 *
 */

#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include "Perceptron.h"

static const float VERSION = 1.0;

perceptron::perceptron(int _inputs, int _outputs, double _rate)
{
	outputActivation = kLinear;
	inputs = _inputs;
	outputs = _outputs;
	rate = _rate;
	weight.resize(0);
	//weight = 0;
	allocateMemory();
}

perceptron::perceptron(perceptron *perp)
{
	inputs = perp->inputs;
	outputs = perp->outputs;
	rate = perp->rate;
	outputActivation = perp->outputActivation;
	weight.resize(0);
	//weight = 0;
	allocateMemory();
}

perceptron::perceptron(char *f)
{
	weight.resize(0);
	//weight = 0;
	inputs = outputs = -1;
	load(f);
}

perceptron::perceptron(FunctionApproximator *fa)
{
	perceptron *perp = (perceptron *)fa;
	inputs = perp->inputs;
	outputs = perp->outputs;
	outputActivation = perp->outputActivation;	
	rate = perp->rate;
	weight.resize(0);
	//weight = 0;
	allocateMemory();
	load((perceptron*)fa);
}

perceptron::~perceptron()
{
	freeMemory();
}

void perceptron::allocateMemory()
{
//	if (weight.size() != 0)
//		freeMemory();
	fflush(stdin);
	weight.resize(outputs);// = new double*[outputs];
	for (int x = 0; x < outputs; x++)
	{
		fflush(stdin);
		weight[x].resize(inputs+1);// = new double[inputs+1];
		for (int y = 0; y <= inputs; y++)
			weight[x][y] = ((double)2*random()/RAND_MAX-1)/(double)inputs;
	}
	output.resize(outputs);// = new double[outputs];
}

void perceptron::freeMemory()
{
//	if (weight != 0)
//	{
////		for (int x = 0; x < outputs; x++)
////			delete [] weight[x];
////		delete [] weight;
////		delete [] output;
		weight.resize(0);
//		//weight = 0;
//	}
}

void perceptron::load(const char *fname)
{
	FILE *f;
	f = fopen(fname, "r");
	if (f == 0)
	{
		fprintf(stderr, "PERP Error: could not open file '%s' for loading; trying once more.\n", fname);
		sleep(1);
		f = fopen(fname, "r");
		if (f == 0)
		{
			fprintf(stderr, "PERP Error: could not open file '%s' for loading.\n", fname);
			exit(0);
			return;
		}
	}
	load(f);
	fclose(f);
}

void perceptron::load(FILE *f)
{
	int inputs1, outputs1;
	float version;
	int res = fscanf(f, "PERP %f %d %d\n", &version, &inputs1, &outputs1);
	if (res != 3)
	{
		printf("Error: unrecognized perceptron file. Expected header 'PERP <version> <inputs> <outputs>'.");
		exit(0);
	}
	if (version > VERSION)
	{
		printf("Error: loaded network is %1.2f, but code can only handle %1.2f.",
					 version, VERSION);
		exit(0);
	}
	if ((inputs1 != inputs) || (outputs1 != outputs))
	{
		freeMemory();
		inputs = inputs1;
		outputs = outputs1;
		allocateMemory();
	}

	for (int y = 0; y < outputs; y++)
	{
		for (int x = 0; x <= inputs; x++)
		{
			fscanf(f, "%le ", &weight[y][x]);
		}
	}	
}

void perceptron::load(const perceptron *p)
{
	if (p && ((p->inputs != inputs) || (p->outputs != outputs)))
	{
		freeMemory();
		inputs = p->inputs;
		outputs = p->outputs;
		outputActivation = p->outputActivation;

		allocateMemory();
	}
	for (int y = 0; y < outputs; y++)
	{
		for (int x = 0; x <= inputs; x++)
		{
			weight[y][x] = p->weight[y][x];
		}
	}
}

bool perceptron::validSaveFile(char *fname)
{
	FILE *f;
	f = fopen(fname, "r");
	if (f == 0)
	{
		return false;
	}
	int finput, foutput;
	float version;
	int res = fscanf(f, "PERP %f %d %d\n", &version, &finput, &foutput);
	fclose(f);
	if (res != 3)
	{
		return false;
	}
	if (version > VERSION)
	{
		return false;
	}
	return true;
}

void perceptron::save(const char *fname)
{
	FILE *f;
	f = fopen(fname, "w+");
	if (f == 0)
	{
		fprintf(stderr, "Error: could not open file for saving.\n");
		return;
	}
	save(f);
	fclose(f);
}

void perceptron::save(FILE *f)
{
	fprintf(f, "PERP %1.2f %d %d\n", VERSION, inputs, outputs);
	
	for (int y = 0; y < outputs; y++)
	{
		for (int x = 0; x <= inputs; x++)
		{
			fprintf(f, "%le ", weight[y][x]);
		}
		fprintf(f, "\n");
	}
}

//double perceptron::g(double a)
//{
//	return (1/(1+exp(-a)));
//}
//
//double perceptron::dg(double a)
//{
//	double g_a = g(a);
//	return g_a*(1-g_a);
////	return a*(1-a);
//}

double perceptron::outputerr(const std::vector<double> &out, const std::vector<double> &expected, int which)
{
	double err = (out[which]-expected[which]);

	return err;
}

double perceptron::train(std::vector<double> &input, std::vector<double> &target)
{
	double totalErr = 0;
	test(input);
	for (int x = 0; x < outputs; x++)
	{
		double err = outputerr(output,target,x);
		totalErr+=err*err;
		for (int y = 0; y < inputs; y++)
		{
			weight[x][y] -= rate*err*input[y];
		}
		weight[x][inputs] -= rate*err*(1); // bias
	}
	return totalErr;
}

double perceptron::train(std::vector<unsigned int> &input, std::vector<double> &target)
{
	double totalErr = 0;
	test(input);
	for (int x = 0; x < outputs; x++)
	{
		double err = outputerr(output,target,x);
		totalErr+=err*err;
		double rateTimesError = rate*err;
		for (unsigned int y = 0; y < input.size(); y++)
		{
			weight[x][input[y]] -= rateTimesError;
		}
		weight[x][inputs] -= rateTimesError; // bias
	}
	return totalErr;
}

double *perceptron::test(const std::vector<double> &input)
{
	for (int y = 0; y < outputs; y++)
	{
		output[y] = weight[y][inputs];
		for (int x = 0; x < inputs; x++)
		{
			output[y] += weight[y][x]*input[x];
		}
		if (outputActivation == kStep)
		{
			if(output[y] > .5)
				output[y] = 1.0;
			else
				output[y] = 0.0;
		}

	}
	return &output[0];
}

double *perceptron::test(const std::vector<unsigned int> &input)
{
	for (int y = 0; y < outputs; y++)
	{
		output[y] = weight[y][inputs]; // bias
		for (unsigned int x = 0; x < input.size(); x++)
		{
			output[y] += weight[y][input[x]];
		}
	}
	return &output[0];
}

void perceptron::Print()
{
	for (int y = 0; y < outputs; y++)
		for (int x = 0; x <= inputs; x++)
		{
			printf("%1.3f  ", weight[y][x]);
		}
	printf("\n");
}
