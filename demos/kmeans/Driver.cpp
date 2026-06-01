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
#include <vector>

bool recording = false;
bool running = false;
const int numGroups = 8;
const int numElements = 520;

std::vector<recColor> groupColor = {colors::lightgreen, colors::darkred, colors::cyan, colors::yellow, colors::darkblue, colors::orange, colors::lightred, colors::lightgray};

void AssignGroups();

std::pair<double, double> GetData(int which);

struct ds {
	std::pair<double, double> data;
	int group;
};

std::vector<ds> data;
std::vector<std::pair<double, double>> groups(numGroups);

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
		for (int x = 0; x < numElements; x++)
		{
			ds d;
			d.data = GetData(x);
			d.group = -1;
			data.push_back(d);
		}
		for (int x = 0; x < numGroups; x++)
		{
			data[x].group = x;
			groups[x] = data[x].data;
		}
		AssignGroups();
	}
}

double dist(const std::pair<double, double> &p1, const std::pair<double, double> &p2)
{
	return sqrt((p1.first-p2.first)*(p1.first-p2.first)+(p1.second-p2.second)*(p1.second-p2.second));
}

int GetClosestGroup(const std::pair<double, double> &p)
{
	int best = 0;
	for (int x = 1; x < groups.size(); x++)
	{
		if (dist(p, groups[x]) < dist(p, groups[best]))
			best = x;
	}
	return best;
}

void AssignGroups()
{
	for (auto &ds : data)
	{
		ds.group = GetClosestGroup(ds.data);
	}
}

void ComputeMeans()
{
	std::vector<int> count(numGroups);
	for (int x = 0; x < numGroups; x++)
		groups[x] = {0,0};
	for (auto &ds : data)
	{
		count[ds.group]++;
		groups[ds.group].first += ds.data.first;
		groups[ds.group].second += ds.data.second;
	}
	for (int x = 0; x < numGroups; x++)
	{
		groups[x].first /= count[x];
		groups[x].second /= count[x];
	}
}

void MyFrameHandler(unsigned long windowID, unsigned int viewport, void *)
{
	double step = 0.008;
	double pointSize = 0.015;
	for (double y = -1; y <= 1; y += step)
	{
		for (double x = -1; x <= 1; x += step)
		{
			int group = GetClosestGroup({x, y});
			glColor3f(groupColor[group].r, groupColor[group].g, groupColor[group].b);
			DrawBox(x, y, 0, step);
		}
	}

	for (int x = 0; x < data.size(); x++)
	{
		glColor3f(0.5, 0.5, 0.5);
		DrawBox(data[x].data.first, data[x].data.second, -0.010, pointSize);
		int group = data[x].group;
		glColor3f(groupColor[group].r, groupColor[group].g, groupColor[group].b);
		DrawBox(data[x].data.first, data[x].data.second, -0.011-pointSize/2, pointSize/2.0);
	}
	for (int x = 0; x < groups.size(); x++)
	{
		glColor3f(0, 0, 0);
		DrawBox(groups[x].first, groups[x].second, -0.012, pointSize);
		glColor3f(1, 1, 1);
		DrawBox(groups[x].first, groups[x].second, -0.013-pointSize/2, pointSize/2.0);
	}
	if (running)
		MyDisplayHandler(windowID, kNoModifier, 't');
}

int MyCLHandler(char *argument[], int maxNumArgs)
{
	if (maxNumArgs <= 1)
		return 0;
	return 0;
}

void MyDisplayHandler(unsigned long windowID, tKeyboardModifier mod, char key)
{
	static int i = 0;
	switch (key)
	{
		case 't':
		{
			if (i%2)
				AssignGroups();
			else
				ComputeMeans();
			i++;
			break;
		}
		case '{':
		{
			break;
		}
		case ']':
		{
		}
			break;
		case '[':
		{
		}
			break;
		case '|':
		{
		}
			break;
		case 'w':
			break;
		case 'r':
		{
			//recording = !recording;
			for (int x = 0; x < numElements; x++)
				data[x].group = -1;
			for (int x = 0; x < numGroups; x++)
			{
				groups[x] = data[random()%numElements].data;
			}
			AssignGroups();
			i = 0;
		}
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
			running = !running;
			break;
		case 'o':
		{
			for (int x = 0; x < numElements && x < numGroups; x++)
				data[x].data = std::pair<double, double>(((random()%1000)/500.0-1.0),
														 ((random()%1000)/500.0-1.0));
			for (int x = numGroups; x < numElements; x++)
			{
				int parent = random()%numGroups;
				data[x].data.first  = data[parent].data.first+0.2*sin(random()%1000)*sin(random()%1000);
				data[x].data.second = data[parent].data.second+0.2*sin(random()%1000)*sin(random()%1000);
				if (data[x].data.first > 1) data[x].data.first = 1;
				if (data[x].data.first < -1) data[x].data.first = -1;
				if (data[x].data.second > 1) data[x].data.second = 1;
				if (data[x].data.second < -1) data[x].data.second = -1;
			}
			for (int x = 0; x < numElements; x++)
//				data[x].data = std::pair<double, double>(((random()%1000)/500.0-1.0)*sin(random()%1000),
//														 ((random()%1000)/500.0-1.0)*cos(random()%1000));
			for (int x = 0; x < numElements; x++)
				data[x].group = -1;
			for (int x = 0; x < numGroups; x++)
			{
				groups[x] = data[random()%numElements].data;
			}
			AssignGroups();
			i = 0;
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

std::pair<double, double> GetData(int which)
{
	static double max1, max2, min1, min2 = -1;
	double data[numElements][2] = {
		{65.78331,112.9925},
		{71.51521,136.4873},
		{69.39874,153.0269},
		{68.2166,142.3354},
		{67.78781,144.2971},
		{68.69784,123.3024},
		{69.80204,141.4947},
		{70.01472,136.4623},
		{67.90265,112.3723},
		{66.78236,120.6672},
		{66.48769,127.4516},
		{67.62333,114.143},
		{68.30248,125.6107},
		{67.11656,122.4618},
		{68.27967,116.0866},
		{71.0916,139.9975},
		{66.461,129.5023},
		{68.64927,142.9733},
		{71.23033,137.9025},
		{67.13118,124.0449},
		{67.83379,141.2807},
		{68.87881,143.5392},
		{63.48115,97.90191},
		{68.42187,129.5027},
		{67.62804,141.8501},
		{67.20864,129.7244},
		{70.84235,142.4235},
		{67.49434,131.5502},
		{66.53401,108.3324},
		{65.44098,113.8922},
		{69.5233,103.3016},
		{65.8132,120.7536},
		{67.8163,125.7886},
		{70.59505,136.2225},
		{71.80484,140.1015},
		{69.20613,128.7487},
		{66.80368,141.7994},
		{67.65893,121.2319},
		{67.80701,131.3478},
		{64.04535,106.7115},
		{68.57463,124.3598},
		{65.18357,124.8591},
		{69.65814,139.6711},
		{67.96731,137.3696},
		{65.98088,106.4499},
		{68.67249,128.7639},
		{66.88088,145.6837},
		{67.69868,116.819},
		{69.82117,143.6215},
		{69.08817,134.9325},
		{69.91479,147.0219},
		{67.33182,126.3285},
		{70.26939,125.4839},
		{69.10344,115.7084},
		{65.38356,123.4892},
		{70.18447,147.8926},
		{70.40617,155.8987},
		{66.54376,128.0742},
		{66.36418,119.3701},
		{67.537,133.8148},
		{66.50418,128.7325},
		{68.99958,137.5453},
		{68.30355,129.7604},
		{67.01255,128.824},
		{70.80592,135.3165},
		{68.21951,109.6113},
		{69.05914,142.4684},
		{67.73103,132.749},
		{67.21568,103.5275},
		{67.36763,124.7299},
		{65.27033,129.3137},
		{70.84278,134.0175},
		{69.92442,140.3969},
		{64.28508,102.8351},
		{68.2452,128.5214},
		{66.35708,120.2991},
		{68.36275,138.6036},
		{65.4769,132.9574},
		{69.71947,115.6233},
		{67.72554,122.524},
		{68.63941,134.6254},
		{66.78405,121.8986},
		{70.05147,155.3767},
		{66.27848,128.9418},
		{69.20198,129.1013},
		{69.13481,139.4733},
		{67.36436,140.8901},
		{70.09297,131.5916},
		{70.1766,121.1232},
		{68.22556,131.5127},
		{68.12932,136.5479},
		{70.24256,141.4896},
		{71.48752,140.6104},
		{69.20477,112.1413},
		{70.06306,133.457},
		{70.55703,131.8001},
		{66.28644,120.0285},
		{63.42577,123.0972},
		{66.76711,128.1432},
		{68.88741,115.4759},
		{64.87434,102.0927},
		{67.09272,130.353},
		{68.34761,134.1842},
		{65.61073,98.64133},
		{67.75551,114.5599},
		{68.0212,123.4917},
		{67.66193,123.048},
		{66.3146,126.4772},
		{69.43706,128.417},
		{63.83624,127.1941},
		{67.72277,122.0562},
		{70.05098,127.6064},
		{70.18602,131.6423},
		{65.94588,111.8955},
		{70.007,122.039},
		{68.61129,128.5547},
		{68.80817,132.6792},
		{69.76212,136.0632},
		{65.45539,115.9403},
		{68.82534,136.9041},
		{65.8003,119.8804},
		{67.21474,109.0055},
		{69.42021,128.2705},
		{68.94396,135.2913},
		{67.9415,106.8558},
		{65.62506,123.2939},
		{66.49607,109.5143},
		{67.92809,119.3087},
		{68.89415,140.2402},
		{70.241,133.9841},
		{68.26623,132.5807},
		{71.23161,130.6988},
		{69.09747,115.5637},
		{64.39693,123.7941},
		{71.09585,128.1427},
		{68.21868,135.9646},
		{65.91721,116.6273},
		{67.4369,126.8241},
		{73.90107,151.3913},
		{69.98149,130.4022},
		{69.51862,136.2068},
		{65.18437,113.3989},
		{68.00869,125.3287},
		{68.3384,127.5846},
		{65.18417,107.1564},
		{68.26209,116.4588},
		{68.56865,133.8402},
		{64.49675,112.8901},
		{68.71053,130.7568},
		{68.89148,137.7571},
		{69.54011,125.4036},
		{67.39964,138.4659},
		{66.47521,120.8184},
		{66.01217,140.1539},
		{72.44434,136.7397},
		{64.12642,106.1139},
		{70.98112,158.9562},
		{67.50124,108.7868},
		{72.01515,138.7758},
		{65.31143,115.9136},
		{67.07509,146.2922},
		{64.39148,109.8765},
		{69.37003,139.0499},
		{68.37921,119.9001},
		{65.31018,128.3069},
		{67.1369,127.2428},
		{68.39468,115.2306},
		{66.2918,124.7975},
		{67.1866,126.9511},
		{65.99156,111.2711},
		{69.43393,122.6089},
		{67.97463,124.2084},
		{67.76133,124.6453},
		{65.27864,119.5169},
		{73.83364,139.2983},
		{66.81312,104.8265},
		{66.89411,123.0424},
		{65.73568,118.8923},
		{65.98283,121.4939},
		{66.58396,119.2488},
		{67.11294,135.0239},
		{65.87481,116.228},
		{66.78067,109.1731},
		{68.73577,124.2237},
		{66.22666,141.1645},
		{65.95968,129.1501},
		{68.58372,127.8693},
		{66.59347,120.9244},
		{66.96574,127.6466},
		{68.08015,101.4693},
		{70.19025,144.9927},
		{65.52149,110.9523},
		{67.45905,132.8625},
		{67.40985,146.3385},
		{69.66158,145.5894},
		{65.79799,120.8431},
		{66.10558,115.7813},
		{68.23987,128.3019},
		{68.02403,127.4718},
		{71.39044,127.8761},
		{65.7316,121.4997},
		{66.43358,112.7148},
		{70.01309,135.002},
		{69.48146,128.6789},
		{68.62764,124.4062},
		{68.36275,140.026},
		{68.39028,117.519},
		{68.77413,143.8737},
		{69.9236,141.1703},
		{71.55542,155.9414},
		{68.44764,134.0093},
		{66.71398,130.0975},
		{66.68413,106.2265},
		{67.93699,112.0489},
		{68.89855,136.1884},
		{67.29191,131.236},
		{69.57212,131.3231},
		{67.67479,119.5261},
		{69.04155,116.9965},
		{67.96765,138.5255},
		{65.83982,109.6518},
		{65.77288,130.1569},
		{71.14106,137.1114},
		{67.83055,113.759},
		{65.0693,114.9725},
		{69.70745,127.7149},
		{69.92983,121.9972},
		{66.11569,117.9607},
		{68.61364,127.7141},
		{68.9976,117.9619},
		{66.79171,125.1554},
		{68.02363,141.1026},
		{69.67258,145.4822},
		{71.82178,116.065},
		{72.74676,135.7458},
		{67.27951,132.9248},
		{67.41015,115.6622},
		{68.5315,114.3184},
		{68.47126,148.952},
		{68.51867,142.1878},
		{63.72529,134.104},
		{67.70483,141.8926},
		{69.47115,138.7444},
		{66.70198,134.449},
		{65.23126,117.0167},
		{69.89473,115.6752},
		{69.83048,134.7905},
		{65.3979,120.5746},
		{68.32214,120.0835},
		{65.93895,84.3598},
		{70.09805,138.9394},
		{66.05531,143.5245},
		{68.23481,123.1347},
		{65.21758,115.5261},
		{69.16173,120.9844},
		{67.60064,120.6473},
		{67.28621,124.0352},
		{66.84323,117.2238},
		{68.08281,127.9598},
		{66.56991,116.7527},
		{70.16973,121.9957},
		{67.85113,123.9952},
		{66.62309,108.855},
		{63.68816,104.6574},
		{68.0356,132.248},
		{66.941,127.1834},
		{66.26292,125.1209},
		{70.7301,133.322},
		{70.70844,122.3364},
		{73.26872,130.2636},
		{63.79021,116.7431},
		{67.84109,125.8534},
		{67.78058,100.627},
		{67.23901,118.4945},
		{66.75701,137.1255},
		{69.03098,118.6991},
		{68.52652,124.1641},
		{64.65419,124.9442},
		{65.14295,126.3889},
		{66.74124,121.5255},
		{66.99923,133.2951},
		{70.26611,127.4072},
		{67.57666,125.08},
		{66.30582,102.8114},
		{68.90702,134.2212},
		{65.60543,100.9758},
		{68.00189,145.8886},
		{69.93715,155.3046},
		{70.1459,138.7175},
		{66.66364,116.2611},
		{69.40676,120.6087},
		{66.80598,114.4083},
		{67.70048,133.3406},
		{69.13438,92.74955},
		{67.53769,131.7825},
		{65.77912,118.3865},
		{66.25769,131.1466},
		{67.39038,130.2033},
		{64.94327,129.1836},
		{69.25699,143.1445},
		{69.07739,138.7307},
		{69.64403,130.9528},
		{69.07705,124.3032},
		{68.01304,135.3702},
		{68.85664,124.1988},
		{63.19158,111.2389},
		{65.57591,129.4037},
		{69.12101,127.7444},
		{69.78765,121.3311},
		{67.48203,118.2183},
		{69.08809,123.3441},
		{68.5529,151.9118},
		{68.21327,115.3108},
		{67.1012,122.3844},
		{71.23661,129.2323},
		{69.11946,138.2824},
		{65.49848,144.0565},
		{66.58706,120.8482},
		{67.99831,113.3034},
		{72.57747,149.3227},
		{67.97585,130.6373},
		{66.62347,122.387},
		{67.29574,133.8677},
		{65.95951,134.8336},
		{66.54011,137.8363},
		{68.07723,112.6812},
		{65.83291,121.1964},
		{69.23721,114.9262},
		{64.69413,110.3762},
		{69.21372,126.0337},
		{67.16792,118.7008},
		{67.8467,119.3108},
		{69.27389,150.7397},
		{67.90547,121.1142},
		{67.076,118.4065},
		{70.01531,142.759},
		{70.05454,145.6305},
		{70.86714,127.3425},
		{66.0586,115.9678},
		{65.23745,104.4441},
		{68.7413,138.4905},
		{69.51946,145.5049},
		{70.30764,124.4065},
		{67.29982,124.1866},
		{67.60582,126.1659},
		{67.85566,138.8125},
		{66.46536,121.5515},
		{68.92071,114.2555},
		{68.46886,129.4022},
		{69.5193,137.5583},
		{70.23956,134.4996},
		{67.85651,127.7103},
		{68.00435,115.6709},
		{67.13983,131.5906},
		{68.09931,127.5839},
		{68.99077,125.7211},
		{69.32618,119.9885},
		{70.09286,140.8676},
		{68.74304,136.9164},
		{64.18633,118.4759},
		{66.09074,129.5516},
		{69.24418,152.0714},
		{69.66512,120.8646},
		{68.42721,136.1027},
		{68.28613,130.0611},
		{69.37684,131.7743},
		{68.35315,139.38},
		{72.32489,168.229},
		{70.37159,127.6446},
		{65.72927,131.8008},
		{66.49627,122.8561},
		{70.81339,140.2133},
		{67.37579,142.6929},
		{67.79709,137.4299},
		{68.1607,126.0392},
		{68.85782,143.5985},
		{67.71342,122.62},
		{64.66007,108.6044},
		{65.73721,100.6617},
		{68.42441,116.1397},
		{68.88946,140.5293},
		{68.27367,125.6907},
		{69.18045,134.3329},
		{72.60996,136.4649},
		{66.04157,127.8822},
		{69.2288,123.1361},
		{68.87697,116.3375},
		{66.46042,124.5581},
		{69.30444,125.7853},
		{67.58249,107.5905},
		{69.41816,143.0236},
		{67.46867,118.7434},
		{66.44873,118.9856},
		{68.76185,131.8744},
		{63.76881,119.3691},
		{69.03663,126.1223},
		{66.48966,126.6122},
		{66.74224,113.5597},
		{67.69394,115.8413},
		{66.69066,130.2738},
		{67.02877,125.4542},
		{65.91096,117.1655},
		{70.0304,137.3184},
		{69.26978,149.4447},
		{70.07695,151.0695},
		{64.90942,116.9994},
		{69.68744,138.1787},
		{67.56524,127.621},
		{63.85529,120.7514},
		{71.6189,144.2537},
		{64.98993,105.7939},
		{64.60759,120.1567},
		{62.01666,109.0848},
		{65.28516,125.971},
		{67.70163,110.2325},
		{70.85794,134.4706},
		{68.17075,115.5067},
		{67.45006,125.1086},
		{71.04224,116.4113},
		{67.83032,133.3944},
		{67.15488,108.7478},
		{68.30289,134.3986},
		{65.41208,131.2085},
		{68.66755,130.8318},
		{70.06733,127.0838},
		{69.13413,119.4067},
		{67.02078,105.5616},
		{65.52274,134.4733},
		{70.19808,132.6897},
		{67.66866,126.058},
		{69.73257,126.3532},
		{67.61719,142.3603},
		{70.79589,124.5873},
		{69.65804,122.3604},
		{66.73277,123.2298},
		{67.58008,131.4049},
		{65.25317,124.3037},
		{70.71387,128.3836},
		{71.27374,136.853},
		{66.55913,118.0569},
		{68.04247,121.4969},
		{68.11699,119.2811},
		{71.2367,126.5545},
		{66.92387,129.2477},
		{71.10381,134.7804},
		{69.41466,146.0725},
		{69.16807,162.4109},
		{68.57725,130.1885},
		{69.60315,132.1941},
		{69.05087,127.4591},
		{68.28699,125.3466},
		{69.25276,134.7559},
		{68.23862,121.9395},
		{67.6868,125.6688},
		{68.84083,145.956},
		{69.52061,119.9207},
		{66.97668,111.5093},
		{72.0053,138.676},
		{67.44482,135.0151},
		{66.06214,116.509},
		{65.5747,123.9575},
		{68.30171,136.952},
		{71.02603,153.0797},
		{70.76882,126.357},
		{67.50812,145.0614},
		{69.50936,135.1739},
		{66.4455,108.806},
		{68.5689,110.5155},
		{67.01296,138.2685},
		{68.70004,119.5111},
		{68.23013,115.3214},
		{70.40713,132.3684},
		{65.69989,93.99438},
		{65.7031,127.4051},
		{68.48055,134.3024},
		{66.62891,124.9721},
		{66.84285,124.072},
		{69.89734,133.5476},
		{67.6543,134.1509},
		{68.30917,130.2489},
		{69.74848,138.0935},
		{68.57906,126.6064},
		{68.14477,131.8076},
		{67.98093,131.7363},
		{64.82535,117.6261},
		{69.20785,126.1943},
		{69.2656,127.5132},
		{70.9403,142.4785},
		{69.07826,139.444},
		{66.20183,110.3204},
		{71.05779,131.9541},
		{68.99493,129.2037},
		{70.96089,124.3513},
		{66.02496,126.5251},
		{69.8641,149.2303},
		{66.82802,142.1577},
		{69.09424,127.1436},
		{66.81425,122.3353},
		{67.13183,112.6659},
		{64.57428,134.2647},
		{68.68038,120.6936},
		{67.53724,115.783},
		{71.17732,128.6855},
		{70.53514,134.7611},
		{71.53411,118.3419},
		{66.77301,106.1557},
		{66.33636,126.3823},
		{64.83095,114.3716},
		{68.38247,130.2787},
		{68.05038,123.3066},
		{69.09149,122.8571},
		{69.91046,125.6932},
		{68.40737,111.7247},
		{68.32559,125.5516},
		{66.95555,119.9702},
		{70.54816,132.6043},
		{69.37805,132.6738},
		{67.52012,117.4521},
		{64.87142,101.8549},
		{69.13396,128.4418}};
	if (min2 == -1)
	{
		min1 = 1000; min2 = 1000;
		max1 = 0; max2 = 0;
		for (int x = 0; x < numElements; x++)
		{
			max1 = std::max(max1, data[x][0]);
			min1 = std::min(min1, data[x][0]);

			max2 = std::max(max2, data[x][1]);
			min2 = std::min(min2, data[x][1]);
		}
	}
	return std::pair<double, double>(2*(data[which][0]-min1)/(max1-min1)-1,
									 2*(data[which][1]-min2)/(max2-min2)-1);
}