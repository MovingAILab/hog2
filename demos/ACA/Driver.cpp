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
#include "Graphics.h"
#include <string>
#include <random>
#include <cassert>
#include <algorithm>

bool recording = false;
bool running = false;
bool debug = false;

int frame = 0;
float timePerFrame = 1.0f/30.0f;

const float maxF = 10.0f;
const float worldSize = 20.0f;
int numAgents = 2;
bool accurateAgents = true;

struct agentLine {
	Graphics::point start, end;
};

struct agentCircle {
	Graphics::point start;
	float radius;
};

struct agentCollision {
	float collisionTime;
	int c1, c2;
};

struct ACAAgent {
	float speed;
	float rotationSpeed;
	float timeHorizon;
	// target velocity, which is also the "front" of the agent
	// rotations are not explicitly modeled
	Graphics::point goalVelocity;
	// actual current veloicy
	Graphics::point velocity;
	// avoidance force currently being applied
	Graphics::point force;
	// avoidance force from last frame
	Graphics::point lastForce;
	// location of agent - all forces applied to this point as a rigid body
	Graphics::point position;
//	int numLines;
//	agentLine line[3];
	std::vector<agentLine> bounds;
	int numCircles;
	agentCircle circle[3];
	rgbColor color;
};

Graphics::point zero;
std::vector<ACAAgent> agents;
void ResetAgents();
void MoveAgents(float delta);
void UpdateAgents();
void RotateAgent(int which, float radians);

// -------------- MAIN FUNCTION ----------- //
int main(int argc, char* argv[])
{
	InstallHandlers();
	RunHOGGUI(argc, argv, 800, 800);
	return 0;
}


/**
 * Allows you to install any keyboard handlers needed for program interaction.
 */
void InstallHandlers()
{
	InstallKeyboardHandler(MyDisplayHandler, "Add", "Add accurate agent", kAnyModifier, 'a');
	InstallKeyboardHandler(MyDisplayHandler, "Add", "Add inaccurate agent", kAnyModifier, 'b');
	InstallKeyboardHandler(MyDisplayHandler, "Reset", "Reset to start state", kAnyModifier, 'r');
	InstallKeyboardHandler(MyDisplayHandler, "Step", "Step animation", kAnyModifier, 'o');
	InstallKeyboardHandler(MyDisplayHandler, "Pause", "Pause animation", kAnyModifier, 'p');
	InstallKeyboardHandler(MyDisplayHandler, "Slower", "Slower animation", kAnyModifier, '[');
	InstallKeyboardHandler(MyDisplayHandler, "Faster", "Faster animation", kAnyModifier, ']');
	InstallKeyboardHandler(MyDisplayHandler, "Debug", "Toggle Debug", kAnyModifier, '|');

	InstallWindowHandler(MyWindowHandler);
	
	InstallMouseClickHandler(MyClickHandler, static_cast<tMouseEventType>(kMouseDrag|kMouseDown|kMouseUp));
}

void MakeMap(int mapType = -1)
{
	int mapSize = 35;
	int which = mapType;
	if (mapType == -1)
	{
		which = random()%7;
	}
	switch (which)
	{
		case 0:
		case 1:
		case 2:
		{
		}
			break;
		case 3:
		
		case 4:
		case 5:
		case 6:
		
			break;
	}
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
		ReinitViewports(windowID, {-1, -1, 1, 1}, kScaleToSquare);

		ResetAgents();
	}
}

Graphics::point WorldFromHOG(const Graphics::point &p)
{
	Graphics::point res(worldSize*(p.x+1)/2.0f, worldSize*(p.y+1)/2.0f, 0);
	return res;
}

Graphics::point HOGFromWorld(const Graphics::point &p)
{
	Graphics::point res((2*p.x/worldSize)-1, (2*p.y/worldSize)-1, 0);
	return res;
}

float HOGFromWorld(float radius)
{
	return 2*(radius/worldSize);
}

void RotateAgent(int which, float radians)
{
	if (which >= agents.size() || which < 0)
		return;
	float c = cos(radians);
	float s = sin(radians);
	
	// rotate goal velocity and all circles
	agents[which].goalVelocity = {
		agents[which].goalVelocity.x*c - agents[which].goalVelocity.y*s,
		agents[which].goalVelocity.x*s + agents[which].goalVelocity.y*c};
	for (int x = 0; x < agents[which].numCircles; x++)
	{
		agents[which].circle[x].start = {
			agents[which].circle[x].start.x*c - agents[which].circle[x].start.y*s,
			agents[which].circle[x].start.x*s + agents[which].circle[x].start.y*c};
	}
	
	for (int x = 0; x < agents[which].bounds.size(); x++)
	{
		agents[which].bounds[x].start = {
			agents[which].bounds[x].start.x*c - agents[which].bounds[x].start.y*s,
			agents[which].bounds[x].start.x*s + agents[which].bounds[x].start.y*c};
		agents[which].bounds[x].end = {
			agents[which].bounds[x].end.x*c - agents[which].bounds[x].end.y*s,
			agents[which].bounds[x].end.x*s + agents[which].bounds[x].end.y*c};
	}
}

void DrawAgent(Graphics::Display &display, ACAAgent &a)
{
	Graphics::point worldWidth = {worldSize, 0, 0};
	Graphics::point worldHeight = {0, worldSize, 0};
	float lw = HOGFromWorld(1)*0.05f;
//	display.FrameCircle(HOGFromWorld(a.position), HOGFromWorld(a.radius), a.color, lw);
	for (int x = 0; x < a.numCircles; x++)
	{
		agentCircle &c = a.circle[x];

		if (!debug)
		{
			int cnt = 0;
			Graphics::point center;
			for (auto &l : a.bounds)
			{
				center += l.start;
				center += l.end;
				cnt++;
			}
			center /= cnt;
			for (auto &l : a.bounds)
			{
				display.FillTriangle(HOGFromWorld(a.position+l.start), HOGFromWorld(a.position+l.end), HOGFromWorld(a.position+center), a.color);
				display.DrawLine(HOGFromWorld(a.position+l.start),
								 HOGFromWorld(a.position+l.end),
								 lw, Colors::black);
				
			}
		}
//		if (x != 0)
//		{
//			// draw bounding box around circle 0 and all other circles
//			Graphics::point vec = a.circle[x].start-a.circle[0].start;
//			Graphics::point vecNorm = vec;
//			vecNorm.normalise();
//			float r = std::max(a.circle[0].radius, a.circle[x].radius);
//			vecNorm *= r;
//
//			Graphics::point perp1(-vecNorm.y, vecNorm.x);
//			Graphics::point perp2(vecNorm.y, -vecNorm.x);
//			perp1.normalise();
//			perp2.normalise();
//			perp1 *= r;
//			perp2 *= r;
//			display.DrawLine(HOGFromWorld(a.position+a.circle[0].start-perp1-vecNorm),
//							 HOGFromWorld(a.position+a.circle[0].start-perp2-vecNorm),
//							 lw, Colors::black);
//			display.DrawLine(HOGFromWorld(a.position+a.circle[x].start+perp1+vecNorm),
//							 HOGFromWorld(a.position+a.circle[x].start+perp2+vecNorm),
//							 lw, Colors::black);
//			display.DrawLine(HOGFromWorld(a.position+a.circle[0].start-perp1-vecNorm),
//							 HOGFromWorld(a.position+a.circle[x].start+perp2+vecNorm),
//							 lw, Colors::black);
//			display.DrawLine(HOGFromWorld(a.position+a.circle[x].start+perp1+vecNorm),
//							 HOGFromWorld(a.position+a.circle[0].start-perp2-vecNorm),
//							 lw, Colors::black);
//		}

		if (debug)
		{
			display.FillCircle(HOGFromWorld(a.position+c.start), HOGFromWorld(c.radius), a.color);
			display.FrameCircle(HOGFromWorld(a.position+c.start), HOGFromWorld(c.radius), Colors::black, lw);
			// Draw wrapping
			if (a.position.x+c.start.x + c.radius > worldSize)
			{
				display.FillCircle(HOGFromWorld(a.position+c.start-worldWidth), HOGFromWorld(c.radius), a.color);
				display.FrameCircle(HOGFromWorld(a.position+c.start-worldWidth), HOGFromWorld(c.radius), Colors::black, lw);
			}
			if (a.position.x+c.start.x - c.radius < 0)
			{
				display.FillCircle(HOGFromWorld(a.position+c.start+worldWidth), HOGFromWorld(c.radius), a.color);
				display.FrameCircle(HOGFromWorld(a.position+c.start+worldWidth), HOGFromWorld(c.radius), Colors::black, lw);
			}
			if (a.position.y+c.start.y + c.radius > worldSize)
			{
				display.FillCircle(HOGFromWorld(a.position+c.start-worldHeight), HOGFromWorld(c.radius), a.color);
				display.FrameCircle(HOGFromWorld(a.position+c.start-worldHeight), HOGFromWorld(c.radius), Colors::black, lw);
			}
			if (a.position.y+c.start.y - c.radius < 0)
			{
				display.FillCircle(HOGFromWorld(a.position+c.start+worldHeight), HOGFromWorld(c.radius), a.color);
				display.FrameCircle(HOGFromWorld(a.position+c.start+worldHeight), HOGFromWorld(c.radius), Colors::black, lw);
			}
		}
	}
//	for (int x = 0; x < a.numLines; x++)
//	{
//		display.DrawLine(HOGFromWorld(a.position+a.line[x].start),
//						 HOGFromWorld(a.position+a.line[x].end), lw, a.color);
//	}

	display.DrawLine(HOGFromWorld(a.position), HOGFromWorld(a.position+a.velocity), lw, Colors::red);
	if (debug)
	{
		display.DrawLine(HOGFromWorld(a.position), HOGFromWorld(a.position+a.goalVelocity), lw, Colors::green);
		// force
		display.DrawLine(HOGFromWorld(a.position+a.velocity), HOGFromWorld(a.position+a.velocity+a.force), lw, Colors::brown);
	}
}

void MyFrameHandler(unsigned long windowID, unsigned int viewport, void *)
{
	Graphics::Display &display = getCurrentContext()->display;
	display.FillRect({-1, -1, 1, 1}, Colors::white);
	for (auto &a : agents)
	{
		DrawAgent(display, a);
	}
	if (timePerFrame > 0)
	{
		UpdateAgents();
		MoveAgents(timePerFrame);
	}
}

int MyCLHandler(char *argument[], int maxNumArgs)
{
	if (maxNumArgs <= 1)
		return 0;
	strncpy(gDefaultMap, argument[1], 1024);
	return 2;
}


std::random_device rd;
std::mt19937 mt(rd());
std::uniform_real_distribution<float> rand01(0.0, 1.0);

void AddAgent(bool accurate)
{
	ACAAgent a;
	// Note: Not uniformly distributed.
	a.goalVelocity.x = 2*rand01(mt)-1;//random();
	a.goalVelocity.y = 2*rand01(mt)-1;//random();
	a.goalVelocity.z = 0;
	a.goalVelocity.normalise();
	a.position.x = rand01(mt)*worldSize;
	a.position.y = rand01(mt)*worldSize;
	a.position.z = 0;
//	a.radius = 1.0f;
	a.speed = 1.0f+rand01(mt);
	a.rotationSpeed = (rand01(mt)-0.5f)*0.02f;
	a.timeHorizon = 2.0f;

//	// Not implemented fully yet
//	a.numLines = 0;
//	// fixed relative to position
//	a.line[0].start = {0,0};
//	a.line[0].end = {2.0f,0};

	// main circle
	a.numCircles = 1;
	a.circle[0].start = {0,0};
	a.circle[0].radius = 1;

	// shaped creatures
	a.numCircles = 3;
	a.circle[1].start = a.goalVelocity*-1;
	a.circle[1].radius = 0.8f;
	a.circle[2].start = a.goalVelocity*-1.5;
	a.circle[2].radius = 0.6f;

	// build bounding box
	{
		Graphics::point vec = a.circle[2].start-a.circle[0].start;
		Graphics::point vecNorm = vec;
		vecNorm.normalise();
		//float r = std::max(a.circle[0].radius, a.circle[2].radius);
		float r1 = a.circle[0].radius;
		float r2 = a.circle[2].radius;

		Graphics::point perp1(-vecNorm.y, vecNorm.x);
		Graphics::point perp2(vecNorm.y, -vecNorm.x);
		perp1.normalise();
		perp2.normalise();
//		perp1 *= a.circle[0].radius;
//		perp2 *= a.circle[2].radius;
		
		a.bounds.push_back({a.circle[0].start-perp1*r1-vecNorm*r1, a.circle[0].start-perp2*r1-vecNorm*r1});
		a.bounds.push_back({a.circle[2].start+perp1*r2+vecNorm*r2, a.circle[2].start+perp2*r2+vecNorm*r2});
		a.bounds.push_back({a.circle[0].start-perp1*r1-vecNorm*r1, a.circle[2].start+perp2*r2+vecNorm*r2});
		a.bounds.push_back({a.circle[2].start+perp1*r2+vecNorm*r2, a.circle[0].start-perp2*r1-vecNorm*r1});
	}
	if (!accurate)
		a.numCircles = 1;
	// Random creatures
//	a.numCircles = 1+random()%3;
//	for (int x = 1; x < 3; x++)
//	{
//		// secondary circle
//		a.circle[x].start = {(2*rand01(mt)-1),(2*rand01(mt)-1)};
//		a.circle[x].start.normalise();
//		a.circle[x].radius = (1.0f-(rand01(mt)*0.5f));
//		//a.circle[x].start *= 1+a.circle[x].radius*0.5f;
//	}

	a.color = rgbColor::hsl(rand01(mt), 0.5f, 0.5f);
	agents.push_back(a);
}

void ResetAgents()
{
	agents.resize(0);
	for (int x = 0; x < numAgents; x++)
	{
		AddAgent(accurateAgents);
	}
}

agentCollision ttc(const ACAAgent &i, const ACAAgent &j, const Graphics::point &offset)
{
	agentCollision collision = {std::numeric_limits<float>::infinity(), 0, 0};
	for (int x = 0; x < i.numCircles; x++)
	{
		for (int y = 0; y < j.numCircles; y++)
		{
			// check distance from main agent circle
			float r = (i.circle[x].radius + j.circle[y].radius);
			Graphics::point w = (j.position+j.circle[y].start) + offset - (i.position+i.circle[x].start);
			float c = Graphics::point::Dot(w, w) - r * r;
			if (c < 0) // collision
				return {0, x, y};
			Graphics::point v = i.velocity - j.velocity;
			float a = Graphics::point::Dot(v, v);
			float b = Graphics::point::Dot(w, v);
			float discr = b * b - a * c;
			if (discr <= 0)
				continue; //return std::numeric_limits<float>::infinity();
			float tau = (b - sqrt (discr)) / a;
			if (tau < 0)
				continue; //return std::numeric_limits<float>::infinity();
			if (tau < collision.collisionTime)
			{
				collision = {tau, x, y};
			}
		}
	}
	return collision;
}

agentCollision ttc(const ACAAgent &i, const ACAAgent &j, float &t, Graphics::point &offset)
{
	float xOff = worldSize, yOff = worldSize;
	if (j.position.x > worldSize / 2)
		xOff = -xOff;
	if (j.position.y > worldSize / 2)
		yOff = -yOff;
	agentCollision t1, t2, t3, t4;
	t1 = ttc (i, j, zero);
	t2 = ttc (i, j, Graphics::point(0, yOff, 0));
	t3 = ttc (i, j, Graphics::point(xOff, 0, 0));
	t4 = ttc (i, j, Graphics::point(xOff, yOff, 0));
	t = std::min({t1.collisionTime, t2.collisionTime, t3.collisionTime, t4.collisionTime});
	if (t == t2.collisionTime)
	{
		offset = Graphics::point(0, yOff, 0);
		return t2;
	}
	else if (t == t3.collisionTime)
	{
		offset = Graphics::point(xOff, 0, 0);
		return t3;
	}
	else if (t == t4.collisionTime)
	{
		offset = Graphics::point(xOff, yOff, 0);
		return t4;
	}
	else {
		offset = zero;
		return t1;
	}
}


void MoveAgents(float delta)
{
	if (delta == 0)
		return;
	for (int x = 0; x < agents.size(); x++)
	{
		auto &a = agents[x];
		
		a.velocity += a.force * delta;
		a.position += a.velocity * delta;
		a.lastForce = a.force;
		
		if (a.position.x > worldSize)
			a.position -= Graphics::point(worldSize, 0, 0);
		else if (a.position.x < 0)
			a.position += Graphics::point(worldSize, 0, 0);
		if (a.position.y > worldSize)
			a.position -= Graphics::point(0, worldSize, 0);
		else if (a.position.y < 0)
			a.position += Graphics::point(0, worldSize, 0);
	}
}

void UpdateAgents()
{
	//	transform.localScale = new Vector3 (size, 1f, size);
	for (int x = 0; x < agents.size(); x++)
	{
		auto &a = agents[x];
		// Do heading changes - agent rotation
		if (fabs(agents[x].rotationSpeed) < 0.001)
		{
			agents[x].rotationSpeed = (rand01(mt)-0.5f)*0.04f;
			agents[x].speed = 1.0f+rand01(mt);
			//printf("%d New rotation: %f\n", x, agents[x].rotationSpeed);
		}
		agents[x].rotationSpeed *= 0.95f;
		RotateAgent(x, agents[x].rotationSpeed);
		a.goalVelocity.z = 0; // not needed here
		a.goalVelocity.normalise();
		a.goalVelocity *= a.speed;
	}
	
	for (int x = 0; x < agents.size(); x++)
	{
		auto &a = agents[x];
		a.force = (a.goalVelocity - a.velocity)*1.75f;
		for (int y = 0; y < agents.size(); y++)
		{
			if (x != y)
			{
				Graphics::point offset;
				float t;
				agentCollision c = ttc(a, agents[y], t, offset);
				if (t >= a.timeHorizon)
				{
//					std::cout << "far:" << x << ": " << a.position << " " << t << " " << agents[y].position << "\n";
					continue;
				}
				// need to know which circles were colliding to do this correctly
				Graphics::point FAvoid = (a.position+a.circle[c.c1].start) +
				a.velocity * t - (agents[y].position+agents[y].circle[c.c2].start) - offset - agents[y].velocity * t;
				FAvoid.normalise();
				float mag = (a.timeHorizon - t) / (t + 0.001f);
				if (mag > maxF)
					mag = maxF;
				FAvoid *= mag;
//				std::cout << x << ": " << a.position << " " << t << " " << FAvoid << "\n";
				a.force += FAvoid;
			}
			a.force = a.force*0.5f + a.lastForce*0.5;
			if (a.force.length() > maxF)
			{
				a.force.normalise();
				a.force = a.force*maxF;
			}
		}
//		if (agents [0] == this)
//		{
//			if (Input.GetKey (KeyCode.UpArrow))
//			{
//				goalVelocity.z += 0.1f;
//			}
//			if (Input.GetKey (KeyCode.DownArrow))
//			{
//				goalVelocity.z -= 0.1f;
//			}
//			if (Input.GetKey (KeyCode.RightArrow))
//			{
//				goalVelocity.x += 0.1f;
//			}
//			if (Input.GetKey (KeyCode.LeftArrow))
//			{
//				goalVelocity.x -= 0.1f;
//			}
//		}
	}
}

void MyDisplayHandler(unsigned long windowID, tKeyboardModifier mod, char key) // handles keypresses that change display
{
	switch (key)
	{
		case 'r':
			ResetAgents();
			break;
		case kUpArrow:
		case 'w': // y velocity goes up
			break;
		case kDownArrow:
		case 's':
			break;
		case kLeftArrow:
		case 'a':
			accurateAgents = true;
			AddAgent(true);
			break;
		case 'b':
			accurateAgents = false;
			AddAgent(false);
			break;
		case kRightArrow:
		case '|':
			debug = !debug;
			break;
		case '[':
			timePerFrame /= 2;
			break;
		case ']':
			if (timePerFrame == 0)
				timePerFrame = 1.0f/30.0f;
			else
				timePerFrame *= 2;
			break;
		case 'p':
			if (timePerFrame > 0)
				timePerFrame = 0;
			else
				timePerFrame = 1.0f/30.0f;
			break;
		case 'o':
		{
			UpdateAgents();
			MoveAgents(1.0f/30.0f);
			break;
		}
		case '0':
		case '1':
		case '2':
		case '3':
		case '4':
		case '5':
		case '6':
		case 'm':
		default:
			break;
			
	}
	
	
}

void RemoveClosestAgent(Graphics::point p)
{
	assert(agents.size() > 0);
	if (agents.size() == 1)
	{
		agents.resize(0);
		return;
	}
	int closest = 0;
	float dist = (agents[0].position-p).squaredLength();
	for (int x = 1; x < agents.size(); x++)
	{
		float d = (agents[x].position-p).squaredLength();
		if (d < dist)
		{
			closest = x;
			dist = d;
		}
	}
	
	agents[closest] = agents.back();
	agents.pop_back();
}


ACAAgent &GetClosestAgent(Graphics::point p)
{
	assert(agents.size() > 0);
	int closest = 0;
	float dist = (agents[0].position-p).squaredLength();
	for (int x = 1; x < agents.size(); x++)
	{
		float d = (agents[x].position-p).squaredLength();
		if (d < dist)
		{
			closest = x;
			dist = d;
		}
	}
	return agents[closest];
}


/*
 * Code runs when user clicks or moves mouse in window
 *
 * Application does not currently need mouse support
 */
bool MyClickHandler(unsigned long , int windowX, int windowY, point3d loc, tButtonType button, tMouseEventType mType)
{
	Graphics::point p = WorldFromHOG(loc);
	
	switch (mType)
	{
		case kMouseDrag:
		{
			if (button == kLeftButton)
			{
				ACAAgent &a = GetClosestAgent(p);
				a.position = p;
				UpdateAgents();
			}
		}
			break;
		case kMouseDown:
		{
			if (button == kRightButton)
			{
				RemoveClosestAgent(p);
			}
			else if (button == kLeftButton)
			{
				ACAAgent &a = GetClosestAgent(p);
				a.position = p;
				UpdateAgents();
			}
		}
			break;
		default:
		{
		}
			break;
	}
	return true;
}

