#include <iostream>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <cassert>
#include "NN.h"


/*
 class NN
 double[][][] weights;
 double[][][] errors;
 double[] hidden;
 double[] output;
 double rate;
 */

static const float VERSION = 1.0;

using namespace std;

NN::NN(int _inputs, int _hiddens, int _outputs, double learnrate)
:inputs(_inputs), hiddens(_hiddens), outputs(_outputs)
{
	outputActivation = kExponential;
	rate = learnrate;
	allocateMemory();
}

NN::~NN()
{
	freeMemory();
}

NN::NN(NN *nn)
{
	inputs = nn->inputs;
	hiddens = nn->hiddens;
	outputs = nn->outputs;
	rate = nn->rate;
	outputActivation = nn->outputActivation;
	
	allocateMemory();
}

NN::NN(char *f)
{
	inputs = -1;
	load(f);
}

NN::NN(FunctionApproximator *fa)
{
	NN *nn = (NN*)fa;
	inputs = nn->inputs;
	hiddens = nn->hiddens;
	outputs = nn->outputs;
	rate = nn->rate;
	outputActivation = nn->outputActivation;	
	allocateMemory();
}

void NN::allocateMemory(const NN *nn)
{
	hidden.resize(hiddens);
	output.resize(outputs);
	
		
	weights.resize(2);
	updatedweights.resize(2);
	errors.resize(2);// = new double**[2];
	
	weights[0].resize(hiddens);
	updatedweights[0].resize(hiddens);
	errors[0].resize(hiddens);
	for (int x = 0; x < hiddens; x++)
	{
		weights[0][x].resize(inputs+1);
		updatedweights[0][x].resize(inputs+1);
		errors[0][x].resize(inputs+1);
	}
	
	for (int x = 0; x < hiddens; x++)
	{
		for (int y = 0; y < inputs+1; y++)
		{
			if (nn)
			{
				weights[0][x][y] = nn->weights[0][x][y];
			}
			else {
				weights[0][x][y] = ((double)2*random()/RAND_MAX-1)/3;
			}
			errors[0][x][y] = 0;
		}
	}
	weights[1].resize(outputs);
	updatedweights[1].resize(outputs);
	errors[1].resize(outputs);// = new double*[outputs];
	for (int x = 0; x < outputs; x++)
	{
		weights[1][x].resize(hiddens+1);
		updatedweights[1][x].resize(hiddens+1);
		errors[1][x].resize(hiddens+1);// = new double[hiddens+1];
	}
	for (int x = 0; x < outputs; x++)
	{
		for (int y = 0; y < hiddens+1; y++)
		{
			if (nn)
				weights[1][x][y] = weights[1][x][y];
			else
				weights[1][x][y] = ((double)2*random()/RAND_MAX-1)/3;
			errors[1][x][y] = 0;
		}
	}
}

void NN::freeMemory()
{
//	for (int x = 0; x < outputs; x++)
//	{
//		delete [] weights[1][x];// = new double[hiddens+1];
//		delete [] errors[1][x];// = new double[hiddens+1];
//	}
//	delete [] weights[1];// = new double*[outputs];
//	delete [] errors[1];// = new double*[outputs];
//	
//	for (int x = 0; x < hiddens; x++)
//	{
//		delete weights[0][x];// = new double[inputs+1];
//		delete errors[0][x];// = new double[inputs+1];
//	}
//	delete [] weights[0];// = (double**)new double[hiddens];
//	delete [] errors[0];// = (double **)new double[hiddens];
//	
//	delete [] weights;// = new double**[2];
//	delete [] errors;// = new double**[2];
//
//	delete [] hidden;// = new double[hiddens];
//	delete [] output;// = new double[outputs];
}

void NN::load(const NN *nn)
{
	if ((nn->inputs != inputs) || (nn->hiddens != hiddens) || (nn->outputs != outputs))
	{
		freeMemory();
		outputActivation = nn->outputActivation;
		allocateMemory(nn);
	}
	else {
		for (int x = 0; x < hiddens; x++)
		{
			for (int y = 0; y < inputs+1; y++)
			{
				weights[0][x][y] = nn->weights[0][x][y];
				errors[0][x][y] = 0;
			}
		}
		for (int x = 0; x < outputs; x++)
		{
			for (int y = 0; y < hiddens+1; y++)
			{
				weights[1][x][y] = weights[1][x][y];
				errors[1][x][y] = 0;
			}
		}
	}
}

bool NN::validSaveFile(char *fname)
{
	FILE *f;
	f = fopen(fname, "r");
	if (f == 0)
	{
		return false;
	}
	int finput, fhidden, foutput;
	float version;
	int res = fscanf(f, "NN %f %d %d %d\n", &version, &finput, &fhidden, &foutput);
	fclose(f);
	if (res != 4)
	{
		return false;
	}
	if (version > VERSION)
	{
		return false;
	}
	return true;
}

void NN::load(const char *fname)
{
	FILE *f;
	f = fopen(fname, "r");
	if (f == 0)
	{
		fprintf(stderr, "Error: could not load file %s\n", fname);
		return;
	}
	load(f);
	fclose(f);
}

void NN::load(FILE *f)
{
	int finput, fhidden, foutput;
	float version;
	int res = fscanf(f, "NN %f %d %d %d\n", &version, &finput, &fhidden, &foutput);
	if (res != 4)
	{
		printf("Error: unrecognized neural network file. Expected header 'NN <version> <inputs> <hidden> <outputs>'.");
		exit(0);
	}
	if (version > VERSION)
	{
		printf("Error: loaded network is %1.2f, but code can only handle %1.2f.",
					 version, VERSION);
		exit(0);
	}
	if ((finput != inputs) || (fhidden != hiddens) || (foutput != outputs))
	{
		if (inputs != -1)
			freeMemory();
		inputs = finput;
		hiddens = fhidden;
		outputs = foutput;
		allocateMemory();
	}

	for (int x = 0; x < hiddens; x++)
	{
		for (int y = 0; y < inputs+1; y++)
		{
			fscanf(f, "%lf", &weights[0][x][y]);
			//fread(&weights[0][x][y], sizeof(double), 1, f);
			errors[0][x][y] = 0;
		}
	}
	for (int x = 0; x < outputs; x++)
	{
		for (int y = 0; y < hiddens+1; y++)
		{
			fscanf(f, "%lf ", &weights[1][x][y]);
			//fread(&weights[1][x][y], sizeof(double), 1, f);
			errors[1][x][y] = 0;
		}
	}
}

void NN::save(const char *fname)
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

void NN::save(FILE *f)
{
	fprintf(f, "NN %1.2f %d %d %d\n", VERSION, inputs, hiddens, outputs);
	for (int x = 0; x < hiddens; x++)
	{
		for (int y = 0; y < inputs+1; y++)
		{
			//fwrite(&weights[0][x][y], sizeof(double), 1, f);
			fprintf(f, "%lf ", weights[0][x][y]);
		}
		fprintf(f, "\n");
	}
	//printf("Done...with internal\n");
	for (int x = 0; x < outputs; x++)
	{
		for (int y = 0; y < hiddens+1; y++)
		{
			//fwrite(&weights[1][x][y], sizeof(double), 1, f);
			fprintf(f, "%lf ", weights[1][x][y]);
		}
		fprintf(f, "\n");
	}
	//printf("Done...with external\n");
}

double NN::g(double a)
{
	return (1/(1+exp(-a)));
}

double NN::dg(double a)
{
	double g_a = a;
	return g_a*(1-g_a);
}

double NN::outputerr(const std::vector<double> &out, const std::vector<double> &expected, int which)
{
	double err = (expected[which]-out[which]);
	if (outputActivation == kExponential)
		err *= dg(out[which]);

	return err;
}

double NN::internalerr(const std::vector<double> &out, const std::vector<double> &expected, int which)
{
	double answer = 0;

	for (int x = 0; x < outputs; x++) // weights[1].length
	  answer+=weights[1][x][which]*outputerr(out, expected, x);
	answer *= dg(hidden[which]);

	return answer;
}

double NN::internalinputerr(const std::vector<double> &out, const std::vector<double> &expected, int which)
{
	double answer = 0;
	
	for (int x = 0; x < outputs; x++)
		answer+=hidden[which]*outputerr(out, expected, x)*dg(weights[1][x][which]);
	
	return answer;
}

double NN::error(const std::vector<double> &target)
{
	double answer = 0, t;
	for (int x = 0; x < outputs; x++) // output.length
	{
		t = (output[x]-target[x]);
		answer += t*t;
	}
	return answer;
}

double NN::train(std::vector<double> &input, std::vector<double> &target)
{
	test(input);

	// calulate error in output layer
	for (int x = 0; x < outputs; x++) // weights[1].length
	{
		double xoutputerror = rate*outputerr(output, target, x);
		errors[1][x][0] = xoutputerror;
		updatedweights[1][x][0] = weights[1][x][0] + errors[1][x][0];
		for (int y = 0; y < hiddens; y++)
		{
			errors[1][x][y+1] = hidden[y]*xoutputerror;
			updatedweights[1][x][y+1] = weights[1][x][y+1]+errors[1][x][y+1];
		}
	}

	// calulate error in hidden layer
	for (int x = 0; x < hiddens; x++) // weights[0].length
	{
		double xinternalerror = rate*internalerr(output, target, x);
		errors[0][x][0] = xinternalerror;
		updatedweights[0][x][0] = weights[0][x][0]+errors[0][x][0];
		for (int y = 0; y < inputs; y++)
		{
			errors[0][x][y+1] = input[y]*xinternalerror;
			updatedweights[0][x][y+1] = weights[0][x][y+1]+errors[0][x][y+1];
		} 
	}
	updatedweights.swap(weights);
	return error(target);
}

double NN::train(std::vector<unsigned int> &input, std::vector<double> &target)
{
	assert(!"Code needs updating");
	test(input);
	
	// calulate error in output layer
	for (int x = 0; x < outputs; x++) // weights[1].length
	{
		double xoutputerror = rate*outputerr(output, target, x);
		errors[1][x][0] = xoutputerror;
		weights[1][x][0] += errors[1][x][0];
		for (int y = 0; y < hiddens; y++) // weights[1][x].length-1
		{
			errors[1][x][y+1] = hidden[y]*xoutputerror;
			weights[1][x][y+1] += errors[1][x][y+1];
		}
	}
	
	// calulate error in hidden layer
	for (int x = 0; x < hiddens; x++) // weights[0].length
	{
		double xinternalerror = rate*internalerr(output, target, x);
		errors[0][x][0] = xinternalerror;
		weights[0][x][0] += errors[0][x][0];
		for (unsigned int y = 0; y < input.size(); y++) //w[0][x].length-1
		{
			errors[0][x][input[y]+1] = (1)*xinternalerror;
			weights[0][x][input[y]+1] += errors[0][x][input[y]+1];
		} 
	}
	return error(target);
}

/*
 * Given the target output, find input that would maximize this classification
 */
double NN::GetInput(std::vector<double> &input, const std::vector<double> &target)
{
	test(input);
	
	// calulate error in hidden layer
	for (int x = 0; x < hiddens; x++) // weights[0].length
	{
		double xinternalerror = internalinputerr(output, target, x);
		for (int y = 0; y < inputs; y++)
		{
			input[y] += weights[0][x][y+1]*xinternalerror;
		}
	}
	return error(target);
}

double *NN::test(const std::vector<double> &input)
{
	// calculate the input value for the node
	for (int y = 0; y < hiddens; y++)
		hidden[y] = weights[0][y][0]*(1);

	for (int x = 0; x < inputs; x++) //input.length
		if (input[x] != 0)
			for (int y = 0; y < hiddens; y++) // hidden.length
				hidden[y] += weights[0][y][x+1]*input[x];

	// pass value through activation function
	for (int y = 0; y < hiddens; y++)
		hidden[y] = g(hidden[y]);

	for (int y = 0; y < outputs; y++) // output.length
	{
		// calculate the input value for the node
		output[y] = weights[1][y][0]*(1);
		for (int x = 0; x < hiddens; x++) // hidden.length
			output[y] += weights[1][y][x+1]*hidden[x];

		if (outputActivation == kExponential)
			output[y] = g(output[y]);
		else if (outputActivation == kStep)
		{
			if (output[y] > .5)
				output[y] = 1.0;
			else
				output[y] = 0.0;
		}
		else // kLinear
			output[y] = output[y];
	}
	return &output[0];
}

double *NN::test(const std::vector<unsigned int> &input)
{
	// calculate the input value for the node
	for (int y = 0; y < hiddens; y++)
		hidden[y] = weights[0][y][0]*(1);
	
	for (unsigned int x = 0; x < input.size(); x++) //input.length
		for (int y = 0; y < hiddens; y++) // hidden.length
			hidden[y] += weights[0][y][input[x]+1]*(1);
	
	// pass value through activation function
	for (int y = 0; y < hiddens; y++)
		hidden[y] = g(hidden[y]);
	
	for (int y = 0; y < outputs; y++) // output.length
	{
		// calculate the input value for the node
		output[y] = weights[1][y][0]*(1);
		for (int x = 0; x < hiddens; x++) // hidden.length
			output[y] += weights[1][y][x+1]*hidden[x];
		
		if (outputActivation == kExponential)
			output[y] = g(output[y]);
		else if (outputActivation == kStep)
		{
			if (output[y] > .5)
				output[y] = 1.0;
			else
				output[y] = 0.0;
		}
		else // kLinear
			output[y] = output[y];
	}
	return &output[0];
}

void NN::Print()
{
	cout << "Neural network weights:" << endl;
	for (int x = 0; x < outputs; x++) // weights[1].length
	{
      cout << "Input weights to output " << x << ":";
		for (int y = 0; y < hiddens+1; y++)
			cout << " " << weights[1][x][y];
		cout << endl;
	}
	for (int x = 0; x < outputs; x++)
	{
      cout << "Input weights to hidden node " << x << ":";
      for (int y = 0; y < hiddens+1; y++)
			cout << " " << weights[0][x][y];
      cout << endl;
	}
}


