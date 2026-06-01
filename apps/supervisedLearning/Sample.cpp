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
#include "Sample.h"
#include "LinearRegression.h"
#include "LogisticRegression.h"
#include "NN.h"

bool CLASSIFIER = 0;
bool NN = true;
//#define KERNEL
const double scale = 2.0;
bool runTraining = false;
int numInputs = 1;
struct data
{
	point3d loc;
	bool val;
};

std::vector<data> points;

FunctionApproximator *cl = new LinearRegression(11, 1, 0.01);
FunctionApproximator *l = new LinearRegression(10, 1, 0.01);

bool threshold = false;

int main(int argc, char* argv[])
{
	InstallHandlers();
	RunHOGGUI(argc, argv);
}


/**
 * Allows you to install any keyboard handlers needed for program interaction.
 */
void InstallHandlers()
{
	InstallKeyboardHandler(MyDisplayHandler, "Toggle Abstraction", "Toggle display of the ith level of the abstraction", kAnyModifier, '0', '9');
	InstallKeyboardHandler(MyDisplayHandler, "Cycle Abs. Display", "Cycle which group abstraction is drawn", kAnyModifier, 't');
	InstallKeyboardHandler(MyDisplayHandler, "Toggle classifier", "toggle classifier", kAnyModifier, 'c');
	InstallKeyboardHandler(MyDisplayHandler, "Toggle Neural Network", "toggle Neural Network", kAnyModifier, 'n');
	InstallKeyboardHandler(MyDisplayHandler, "Pause Simulation", "Pause simulation execution.", kNoModifier, 'p');
	InstallKeyboardHandler(MyDisplayHandler, "Step Simulation", "If the simulation is paused, step forward .1 sec.", kAnyModifier, 'o');
	InstallKeyboardHandler(MyDisplayHandler, "Toggle threshold", "Toggle whether output is thresholded for display.", kAnyModifier, ']');
	InstallKeyboardHandler(MyDisplayHandler, "Toggle threshold", "Toggle whether output is thresholded for display.", kAnyModifier, '[');
	InstallKeyboardHandler(MyDisplayHandler, "Run training", "Runtraining continuously.", kAnyModifier, 'r');
	InstallKeyboardHandler(MyDisplayHandler, "Logistic", "Logistic regression.", kAnyModifier, 'l');
	
	InstallKeyboardHandler(MyPathfindingKeyHandler, "Mapbuilding Unit", "Deploy unit that paths to a target, building a map as it travels", kNoModifier, 'd');
	InstallKeyboardHandler(MyRandomUnitKeyHandler, "Add A* Unit", "Deploys a simple a* unit", kNoModifier, 'a');
	InstallKeyboardHandler(MyRandomUnitKeyHandler, "Add simple Unit", "Deploys a randomly moving unit", kShiftDown, 'a');
	InstallKeyboardHandler(MyRandomUnitKeyHandler, "Add simple Unit", "Deploys a right-hand-rule unit", kControlDown, '1');
	
	InstallCommandLineHandler(MyCLHandler, "-map", "-map filename", "Selects the default map to be loaded.");
	InstallCommandLineHandler(MyCLHandler, "-problems", "-problems filename sectorMultiplier", "Selects the problem set to run.");
	InstallCommandLineHandler(MyCLHandler, "-problems2", "-problems2 filename sectorMultiplier", "Selects the problem set to run.");
	InstallCommandLineHandler(MyCLHandler, "-size", "-batch integer", "If size is set, we create a square maze with the x and y dimensions specified.");
	
	InstallWindowHandler(MyWindowHandler);
	
	InstallMouseClickHandler(MyClickHandler);
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
		InstallFrameHandler(MyFrameHandler, windowID, 0);
		SetNumPorts(windowID, 1);
	}
}

void MyFrameHandler(unsigned long windowID, unsigned int viewport, void *)
{
	if (runTraining)
	{
		if (CLASSIFIER)
			MyDisplayHandler(windowID, kNoModifier, 't');
		else
			MyDisplayHandler(windowID, kNoModifier, 'o');
	}
	
	glColor3f(0.0, 0.0, 0.0);
	glBegin(GL_TRIANGLE_STRIP);
	glVertex3f(-1, -1, 0.01);
	glVertex3f(-1,  1, 0.01);
	glVertex3f( 1, -1, 0.01);
	glVertex3f( 1,  1, 0.01);
	glEnd();
	glLineWidth(8.0);
	glColor3f(1, 1, 1);
	glBegin(GL_LINES);
	glVertex3f(-1, 0, 0);
	glVertex3f(1, 0, 0);
	glVertex3f(0, 1, 0);
	glVertex3f(0, -1, 0);
	
	for (unsigned int x = 0; x < points.size(); x++)
	{
		if (points[x].val)
			glColor3f(0, 1, 0);
		else
			glColor3f(1, 0, 0);
		glVertex3f(points[x].loc.x-0.02, points[x].loc.y-0.02, 0);
		glVertex3f(points[x].loc.x+0.02, points[x].loc.y+0.02, 0);
		glVertex3f(points[x].loc.x-0.02, points[x].loc.y+0.02, 0);
		glVertex3f(points[x].loc.x+0.02, points[x].loc.y-0.02, 0);
	}
	glEnd();
	
	if (!CLASSIFIER)
	{
		glBegin(GL_LINE_STRIP);
		for (double x = -1; x <= 1; x += 1.0/64.0)
		{
			glColor3f(1, 1, 0);
			std::vector<double> in(11);
			in[0] = x;
			for (int x = 1; x < numInputs; x++)
				in[x] = in[0]*in[x-1];
			double res = *l->test(in);
			res = (res-0.5)*scale;
			glVertex3f(x, res, 0);
		}
		glEnd();
	}
	else {
		glPointSize(4.0);
		glBegin(GL_POINTS);
		std::vector<double> in(11);
		for (double x = -1; x <= 1; x += 0.03125*0.5)
		{
			for (double y = -1; y <= 1; y += 0.03125*0.5)
			{
				in[0] = x;
				in[1] = y;
				for (int t = 2; t < numInputs; t++)
					in[t] = in[t-1]*in[t%2];
				double res = *cl->test(in);
				if (threshold)
				{
					if (res > 0.5)
						glColor4f(0, 1, 0, 0.75);
					else
						glColor4f(1, 0, 0, 0.75);
				}
				else {
					glColor3f((1-res)*(1-res), res*res, 0);
				}
				glVertex3f(x, y, 0);
			}
		}
		glEnd();
	}
}


int MyCLHandler(char *argument[], int maxNumArgs)
{
	if (strcmp( argument[0], "-map" ) == 0 )
	{
		return 2;
	}
	else if (strcmp( argument[0], "-size" ) == 0 )
	{
		return 2;
	}
	else if (strcmp(argument[0], "-problems" ) == 0 )
	{
		return 3;
	}
	else if (strcmp(argument[0], "-problems2" ) == 0 )
	{
		return 3;
	}
	return 2; //ignore typos
}

void MyDisplayHandler(unsigned long windowID, tKeyboardModifier mod, char key)
{
	switch (key)
	{
		case 'n':
		{
			NN = !NN;
			printf("Neural Network = %s\n", NN?"true":"false");
			
			delete cl;
			delete l;
			if (!NN)
			{
				cl = new LinearRegression(11, 1, 0.01);
				l = new LinearRegression(10, 1, 0.01);
			}
			else {
				cl = new class NN(11, 20, 1, 0.01);
				l = new class NN(10, 20, 1, 0.01);
//				cl = new LogisticRegression(3, 1, 0.01);
//				l = new LogisticRegression(2, 1, 0.01);
			}
//			l->setLearnRate(0.01/numInputs);
//			cl->setLearnRate(0.01/numInputs);

			break;
		}
		case 'l':
		{
			delete cl;
			delete l;
			cl = new LogisticRegression(11, 1, 0.01);
			l = new LogisticRegression(10, 1, 0.01);
			submitTextToBuffer("Logistic regression");
			break;
		}
		case 'c': CLASSIFIER = !CLASSIFIER;
			printf("Classifier = %s\n", CLASSIFIER?"true":"false");
		case '[':
		case ']': threshold = !threshold; break;
		case 'r':
		{
			runTraining = !runTraining;
			std::string s = "Continual training ";
			s += runTraining?"on":"off";
			submitTextToBuffer(s.c_str());
		}
			break;
		case '0': numInputs = 1; break;
		case '1': numInputs = 2; break;
		case '2': numInputs = 3; break;
		case '3': numInputs = 4; break;
		case '4': numInputs = 5; break;
		case '5': numInputs = 6; break;
		case '6': numInputs = 7; break;
		case '7': numInputs = 8; break;
		case '8': numInputs = 9; break;
		case '9': numInputs = 10; break;
		case 't':
		{
//			l->setLearnRate(0.01/numInputs);
//			cl->setLearnRate(0.01/numInputs);
			//cl.setOutputActivation(kStep);
			int maxVal = 1;
			if (kShiftDown == mod)
			{
				maxVal = 10000;
				printf("10000!\n");
			}
			for (int z = 0; z < maxVal; z++)
			{
				for (unsigned int x = 0; x < points.size(); x++)
				{
					int which = random()%points.size();
					std::vector<double> in(11);
					std::vector<double> out(1);
					in[0] = points[which].loc.x;
					in[1] = points[which].loc.y;
					for (int x = 2; x < numInputs; x++)
						in[x] = in[x-1]*in[x%2];
					//in[2] = in[0]*in[1];//sqrt(in[0]*in[0]+in[1]*in[1]); //
					out[0] = points[which].val?1:0;
					//out[0] = points[x].val?1:-1;
					cl->train(in, out);
				}
			}
			break;
		}
////		case 't':
////			//cl->setOutputActivation(kStep);
////			for (unsigned int x = 0; x < points.size(); x++)
////			{
////				int which = random()%points.size();
////				std::vector<double> in(3);
////				std::vector<double> out(1);
////				in[0] = points[which].loc.x;
////				in[1] = points[which].loc.y;
////				in[2] = in[0]*in[1];//sqrt(in[0]*in[0]+in[1]*in[1]);
////				out[0] = points[which].val?1:0;
////				//out[0] = points[x].val?1:-1;
////				cl->train(in, out);
////			}
//			
//			break;
		case 'p':
			points.resize(0);
			break;
		case 'o':
		{
//			l->setLearnRate(0.01/numInputs);
//			cl->setLearnRate(0.01/numInputs);
			int maxVal = 1;
			if (kShiftDown == mod)
			{
				maxVal = 100000;
				printf("100000!\n");
			}

			for (int z = 0; z < maxVal; z++)
			for (unsigned int x = 0; x < points.size(); x++)
			{
				std::vector<double> in(11);
				std::vector<double> out(1);
				in[0] = points[x].loc.x;
				for (int x = 1; x < numInputs; x++)
					in[x] = in[0]*in[x-1];
				out[0] = points[x].loc.y/scale+0.5;
				//out[0] = points[x].val?1:-1;
				l->train(in, out);
			}
		}
			break;
		default:
			break;
	}
}

void MyRandomUnitKeyHandler(unsigned long, tKeyboardModifier , char)
{

}

void MyPathfindingKeyHandler(unsigned long windowID, tKeyboardModifier , char)
{
}

bool MyClickHandler(unsigned long windowID, int, int, point3d loc, tButtonType button, tMouseEventType mType)
{
//	loc.x/=scale;
//	loc.y/=scale;

	printf("Hit (%f, %f, %f)\n", loc.x, loc.y, loc.z);
	//	mouseTracking = false;
	if (button == kLeftButton && mType == kMouseDown)
	{
		printf("Hit (%f, %f, %f)\n", loc.x, loc.y, loc.z);
		data d;
		d.loc = loc;
		d.val = true;
		points.push_back(d);
		//printf("Mouse down at (%d, %d)\n", px1, py1);
		return true;
	}
	if (button == kRightButton && mType == kMouseDown)
	{
		printf("Hit (%f, %f, %f)\n", loc.x, loc.y, loc.z);
		data d;
		d.loc = loc;
		d.val = false;
		points.push_back(d);
		//printf("Mouse down at (%d, %d)\n", px1, py1);
		return true;
	}
	return false;
}
