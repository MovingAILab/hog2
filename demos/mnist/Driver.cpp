/*
 *  $Id: sample.cpp
 *  hog2
 *
 *  Created by Nathan Sturtevant on 5/31/05.
 *  Modified by Nathan Sturtevant on 02/29/20.
 *
 * This file is part of HOG2. See https://github.com/nathansttt/hog2 for licensing information.
 *
 */

#include "Common.h"
#include "Driver.h"
#include <string.h>
#include <iostream>
#include "LinearRegression.h"
#include "LogisticRegression.h"
#include "NN.h"

bool recording = false;
std::string basePath;
bool trained = false;
struct image {
	uint8_t get(int x, int y) const
	{ return 255-pixel[x+y*28]; }
	uint8_t pixel[28*28];
	uint8_t value;
};

std::vector<image> train;
std::vector<image> test;

FunctionApproximator *lr = 0;

int main(int argc, char* argv[])
{
	InstallHandlers();
	RunHOGGUI(argc, argv, 1200, 1200);
}

/**
 * Allows you to install any keyboard handlers needed for program interaction.
 */
void InstallHandlers()
{
	InstallKeyboardHandler(MyDisplayHandler, "Record", "Record a movie", kAnyModifier, 'r');
	InstallKeyboardHandler(MyDisplayHandler, "Toggle Abstraction", "Toggle display of the ith level of the abstraction", kAnyModifier, '0', '9');
	InstallKeyboardHandler(MyDisplayHandler, "Cycle Abs. Display", "Cycle which group abstraction is drawn", kAnyModifier, '\t');
	InstallKeyboardHandler(MyDisplayHandler, "Pause Simulation", "Pause simulation execution.", kNoModifier, 'p');
	InstallKeyboardHandler(MyDisplayHandler, "Step Simulation", "If the simulation is paused, step forward .1 sec.", kAnyModifier, 'o');
	InstallKeyboardHandler(MyDisplayHandler, "Step History", "If the simulation is paused, step forward .1 sec in history", kAnyModifier, '}');
	InstallKeyboardHandler(MyDisplayHandler, "Step History", "If the simulation is paused, step back .1 sec in history", kAnyModifier, '{');
	InstallKeyboardHandler(MyDisplayHandler, "Step Abs Type", "Increase abstraction type", kAnyModifier, ']');
	InstallKeyboardHandler(MyDisplayHandler, "Step Abs Type", "Decrease abstraction type", kAnyModifier, '[');
	InstallKeyboardHandler(MyDisplayHandler, "Clear", "Clear graph", kAnyModifier, '|');
	InstallKeyboardHandler(MyDisplayHandler, "Help", "Draw help", kAnyModifier, '?');
	InstallKeyboardHandler(MyDisplayHandler, "Weight", "Toggle Dijkstra & A*", kAnyModifier, 'w');
	InstallKeyboardHandler(MyDisplayHandler, "Save", "Save current graph", kAnyModifier, 's');
	InstallKeyboardHandler(MyDisplayHandler, "Load", "Load last saved graph", kAnyModifier, 'l');
	InstallKeyboardHandler(MyDisplayHandler, "Train", "Train Linear Regression", kAnyModifier, 't');

	InstallCommandLineHandler(MyCLHandler, "-data", "-data <filename>", "Load the MNIST data.");
	
	InstallWindowHandler(MyWindowHandler);

	InstallMouseClickHandler(MyClickHandler);
}

void SwapEndian(uint32_t &val)
{
	val = ((val & 0xFF) << 24) | ((val & 0xFF00) << 8) | ((val >> 8) & 0xFF00) | ((val >> 24) & 0xFF);
}

void LoadData(std::vector<image> &images, const char *imageFile, const char *labelFile)
{
	uint32_t dataCount;
	std::string path = basePath;
	if (path.back() != '/' && path.size() > 1)
		path.push_back('/');
	path += imageFile;
	FILE *f = fopen(path.c_str(), "r");
	uint32_t tmp;
	
	fread(&tmp, sizeof(tmp), 1, f);
	SwapEndian(tmp);
	assert(tmp == 0x00000803); // magic number
	printf("Read: 0x%X\n", tmp);
	
	fread(&dataCount, sizeof(dataCount), 1, f);
	SwapEndian(dataCount);
	printf("Read: 0x%X\n", tmp); // num entries
	
	fread(&tmp, sizeof(tmp), 1, f);
	SwapEndian(tmp);
	assert(tmp == 28);
	printf("Read: 0x%X\n", tmp); // image size
	
	fread(&tmp, sizeof(tmp), 1, f);
	SwapEndian(tmp);
	assert(tmp == 28);
	printf("Read: 0x%X\n", tmp); // image size
	
	image i;
	for (int x = 0; x < dataCount; x++) // read image data
	{
		size_t read = fread(i.pixel, sizeof(uint8_t), 28*28, f);
		assert(read == 28*28);
		images.push_back(i);
	}
	fclose(f);
	
	
	path = basePath;
	if (path.back() != '/' && path.size() > 1)
		path.push_back('/');
	path += labelFile;
	fopen(path.c_str(), "r");
	
	fread(&tmp, sizeof(tmp), 1, f);
	SwapEndian(tmp);
	assert(tmp == 0x00000801); // magic number
	printf("Read: 0x%X\n", tmp);
	
	fread(&tmp, sizeof(tmp), 1, f);
	SwapEndian(tmp);
	assert(tmp == dataCount); // num instances
	printf("Read: 0x%X\n", tmp);
	
	for (int x = 0; x < dataCount; x++) // labels
	{
		size_t read = fread(&images[x].value, sizeof(uint8_t), 1, f);
		assert(read == 1);
	}
	fclose(f);
}

void MyWindowHandler(unsigned long windowID, tWindowEventType eType)
{
	if (eType == kWindowDestroyed)
	{
		printf("Window %ld destroyed\n", windowID);
		RemoveFrameHandler(MyFrameHandler, windowID, 0);
	}
	else if (eType == kWindowCreated)
	{
		printf("Window %ld created\n", windowID);
		//glClearColor(0.99, 0.99, 0.99, 1.0);
		InstallFrameHandler(MyFrameHandler, windowID, 0);
		SetNumPorts(windowID, 1);

		LoadData(train, "train-images.idx3-ubyte", "train-labels.idx1-ubyte");
		LoadData(test, "t10k-images.idx3-ubyte", "t10k-labels.idx1-ubyte");
		
		
		lr = new NN(784, 500, 10, 0.01);
//		lr = new LogisticRegression(784, 10, 0.01);
//		lr = new LinearRegression(784, 10, 0.01);
	}
}

int frameCnt = 0;

void ShowImage(const image &i, bool rebuild = false)
{
	static image const *lastImage = 0;
	static recColor cache[2][10][28*28];

	// rebuild cache
	if (lastImage != &i || rebuild)
	{
		printf("Rebuilding cache\n");
		lastImage = &i;
		
		std::vector<double> input(28*28);
		double minVal[10], maxVal[10];
		double minWeight[10], maxWeight[10];
		for (int x = 0; x < 10; x++)
		{
			minVal[x] = minWeight[x] = 1000;
			maxVal[x] = maxWeight[x] = -1000;
		}
		double *tmp = lr->test(input);
		double bias[10];
		double imageSum = 0;
		for (int x = 0; x < 10; x++)
			bias[x] = 0;//tmp[x];
		for (int x = 0; x < 28*28; x++)
			imageSum += i.pixel[x]/255.0;
		for (int x = 0; x < 28*28; x++)
		{
			input[x] = 1;
			double result[10];
			for (int y = 0; y < 10; y++) // bad caching perf but oh well
				result[y] = lr->getInputWeight(x, y);//lr->test(input);
			for (int y = 0; y < 10; y++) // bad caching perf but oh well
			{
				cache[0][y][x].r = result[y]-bias[y]+bias[y]/(28.0*28.0);
				//printf("%1.2f (res) %1.2f (weight)\n", lr->getInputWeight(x, y)+lr->getInputWeight(28*28, y), result[y]);
				//assert(result[y] == lr->getInputWeight(x, y)+lr->getInputWeight(28*28, y));
				if (cache[0][y][x].r < minVal[y])
					minVal[y] = cache[0][y][x].r;
				if (cache[0][y][x].r > maxVal[y])
					maxVal[y] = cache[0][y][x].r;
			}

			input[x] = i.pixel[x]/255.0;
			for (int y = 0; y < 10; y++) // bad caching perf but oh well
				result[y] = lr->getInputWeight(x, y)*input[x];//lr->test(input);
			for (int y = 0; y < 10; y++)
			{
				cache[1][y][x].r = result[y]-bias[y]+input[x]*bias[y]/(imageSum);
				//cache[1][y][x].r = result[y]-bias[y];
				if (cache[1][y][x].r < minWeight[y])
					minWeight[y] = cache[1][y][x].r;
				if (cache[1][y][x].r > maxWeight[y])
					maxWeight[y] = cache[1][y][x].r;
			}
			input[x] = 0;
		}
		
		// normalize by max/min weight
		for (int x = 0; x < 28*28; x++)
		{
			for (int y = 0; y < 10; y++)
			{
				if (cache[0][y][x].r > 0)
				{
					GLfloat color = cache[0][y][x].r/maxVal[y];
					cache[0][y][x].r = 1-color;
					cache[0][y][x].g = 1-color;
					cache[0][y][x].b = 1;
				}
				else {
					GLfloat color = cache[0][y][x].r/minVal[y];
					cache[0][y][x].r = 1;
					cache[0][y][x].g = 1-color;
					cache[0][y][x].b = 1-color;
				}
				if (cache[1][y][x].r > 0)
				{
					GLfloat color = cache[1][y][x].r/maxWeight[y];
					cache[1][y][x].r = 1-color;
					cache[1][y][x].g = 1-color;
					cache[1][y][x].b = 1;
				}
				else {
					GLfloat color = cache[1][y][x].r/minWeight[y];
					cache[1][y][x].r = 1;
					cache[1][y][x].g = 1-color;
					cache[1][y][x].b = 1-color;
				}
			}
		}
	}
	
	// draw image
	for (int x = 0; x < 28; x++)
	{
		for (int y = 0; y < 28; y++)
		{
			GLfloat color = i.get(x, y)/255.0;
			glColor3f(color, color, color);
			DrawSquare(-1.0+x/28.0, -0.5+y/28.0, 0, 1/28.0);
		}
	}
	
//	reverse engineer the best image
//	
	static std::vector<double> input(28*28, 0.0);
	if (rebuild)
	{
		input.clear();
		input.resize(28*28);
	}
	std::vector<double> output(10);
	output[i.value] = 1.0;
	
	for (int x = 0; x < 20; x++)
	{
		lr->GetInput(input, output);
	}
//	for (int x = 0; x < 28*28; x++)
//	{
//		if (input[x] > 1)
//			input[x] = 1;
//		if (input[x] < 0)
//			input[x] = 0;
//	}

//	for (auto x : input)
//		std::cout << (int)(x*100) << " ";
//	std::cout << "\n";

	
	for (int x = 0; x < 28; x++)
	{
		for (int y = 0; y < 28; y++)
		{
			glColor3f(input[x+y*28],
					  input[x+y*28],
					  input[x+y*28]);
			DrawSquare(-0.0+x/56.0, -0.5+y/56.0, 0, 1/56.0);
		}
	}
	double *tmp = lr->test(input);
	for (int x = 0; x < 10; x++)
	{
		glColor3f(1-tmp[x], tmp[x], 0);
		DrawSquare(1/40.0+x/20.0, -0.55, 0.0, 1/40.0);
	}
//
//	{
//		double maxWeight = -1000;
//		double minWeight = 1000;
//		for (int i = 0; i < 10; i++)
//		{
//			for (int y = 0; y < 28*28; y++)
//			{
//				double val = lr->getInputWeight(y, i)*input[y]+lr->getInputWeight(28*28, i)/(sum);
//				if (val > maxWeight)
//					maxWeight = val;
//				if (val < minWeight)
//					minWeight = val;
//			}
//		}
		for (int i = 0; i < 10; i++)
		{
			if (0) // TODO: Draw green if correct classification and draw wrong
			{
				glColor3f(0.0, 0.0, 1.0);
				glLineWidth(4.0);
				glBegin(GL_LINE_LOOP);
				glVertex3f(-1+0.2*i+14.0/140.0+14./140., 0.5+14.0/140.0+14./140., -0.02);
				glVertex3f(-1+0.2*i+14.0/140.0+14./140., 0.5-14.0/140.0+14./140., -0.02);
				glVertex3f(-1+0.2*i-14.0/140.0+14./140., 0.5-14.0/140.0+14./140., -0.02);
				glVertex3f(-1+0.2*i-14.0/140.0+14./140., 0.5+14.0/140.0+14./140., -0.02);
				glEnd();
				glLineWidth(4.0);
				//DrawSquare(-1+0.2*i+x/140.0, 0.5+y/140.0, 0, 1/140.0);
				
			}
			for (int x = 0; x < 28; x++)
			{
				for (int y = 0; y < 28; y++)
				{
					glColor3f(cache[1][i][x+28*y].r , cache[1][i][x+28*y].g , cache[1][i][x+28*y].b );
					DrawSquare(-1+0.2*i+x/140.0, 0.5+y/140.0, 0, 1/140.0);

					glColor3f(cache[0][i][x+28*y].r , cache[0][i][x+28*y].g , cache[0][i][x+28*y].b );
					DrawSquare(-1+0.2*i+x/140.0, 0.7+y/140.0, 0, 1/140.0);
				}
			}
		}
//	}
//	{
//		double maxWeight = -1000;
//		double minWeight = 1000;
//		for (int i = 0; i < 10; i++)
//		{
//			for (int y = 0; y < 28*28; y++)
//			{
//				double val = lr->getInputWeight(y, i);
//				if (val > maxWeight)
//					maxWeight = val;
//				if (val < minWeight)
//				minWeight = val;
//			}
//		}
//		for (int i = 0; i < 10; i++)
//		{
//			for (int x = 0; x < 28; x++)
//			{
//				for (int y = 0; y < 28; y++)
//				{
//					double v = lr->getInputWeight(x+y*28, i);
//					if (v > 0)
//					{
//						GLfloat color = v/maxWeight;
//						glColor3f(1-color, 1-color, 1);
//					}
//					else if (v < 0){
//						GLfloat color = v/minWeight;
//						glColor3f(1, 1-color, 1-color);
//					}
//					DrawSquare(-1+0.2*i+x/140.0, 0.7+y/140.0, 0, 1/140.0);
//				}
//			}
//		}
//	}
	
	std::string label;
	label = "Actual image ";
	label += std::to_string(+i.value);
//	label += " trained value: ";
//	label += std::to_string(maxVal);
//	label += " [";
//	label += std::to_string(result[i.value]);
//	label += ", ";
//	label += std::to_string(result[maxVal]);
//	label += "]";
	
	submitTextToBuffer(label.c_str());
}

void MyFrameHandler(unsigned long windowID, unsigned int viewport, void *)
{
//	frameCnt++;
	ShowImage(test[frameCnt%test.size()], trained);
	trained = false;
}

int MyCLHandler(char *argument[], int maxNumArgs)
{
	if (maxNumArgs <= 1)
		return 0;
	basePath = argument[1];
	//strncpy(gDefaultMap, argument[1], 1024);
	return 0;
}

void MyDisplayHandler(unsigned long windowID, tKeyboardModifier mod, char key)
{
	switch (key)
	{
		case 't':
		{
			std::vector<double> input(28*28);
			std::vector<double> output(10);
			for (int x = 0; x < train.size(); x++)
			{
				for (int y = 0; y < 28*28; y++)
				{
					input[y] = train[x].pixel[y]/255.0;
				}
				output[train[x].value] = 1.0;

				lr->train(input, output);
				
				output[train[x].value] = 0.0;
			}

			int numCorrect = 0;
			for (int x = 0; x < test.size(); x++)
			{
				for (int y = 0; y < 28*28; y++)
				{
					input[y] = train[x].pixel[y]/255.0;
				}

				const double *result = lr->test(input);
				bool correct = true;
				for (int t = 0; t < 10; t++)
				{
					if (x == train[t].value)
						continue;
					if (result[t] > result[train[x].value])
						correct = false;
				}
				if (correct)
					numCorrect++;
			}
			printf("%d of %d classified correctly\n", numCorrect, test.size());

			trained = true;
			break;
		}
		case '{':
		{
			break;
		}
		case ']':
		{
			frameCnt++;
		}
			break;
		case '[':
		{
			frameCnt--;
			if (frameCnt < 0)
				frameCnt += 60000;
		}
			break;
		case '|':
		{
		}
			break;
		case 'w':
			break;
		case 'r':
			recording = !recording;
			break;
		case '0': break;
		case '1': break;
		case '2': break;
		case '3': break;
		case '4': break;
		case '5': break;
		case '6': break;
		case '7': break;
		case '8': break;
		case '9': break;
		case '\t':
			break;
		case 'p':
			//running = !running;
			break;
		case 'o':
		{
		}
			break;
		case '?':
		{
		}
			break;
		case 's':
			break;
		case 'l':
			break;
		default:
			break;
	}
	
}


bool MyClickHandler(unsigned long , int windowX, int windowY, point3d loc, tButtonType button, tMouseEventType mType)
{
	if (mType == kMouseDown)
	{
		switch (button)
		{
			case kRightButton: printf("Right button\n"); break;
			case kLeftButton: printf("Left button\n"); break;
			case kMiddleButton: printf("Middle button\n"); break;
		}
	}
	if (button != kLeftButton)
		return false;
	switch (mType)
	{
		case kMouseDown:
		{
			break;
		}
		case kMouseDrag:
		{
			break;
		}
		case kMouseUp:
		{
			break;
		}
	}
	return false;
}


