/*
 *  $Id: timer.h
 *  hog2
 *
 *  Created by Nathan Sturtevant on 10/30/06.
 *  Modified by Nathan Sturtevant on 02/29/20.
 *
 * This file is part of HOG2. See https://github.com/nathansttt/hog2 for licensing information.
 *
 */

#ifndef TIMER_H
#define TIMER_H

#include <stdint.h>
#include <fstream>
#include <chrono>

class Timer {
public:
	Timer();
	
	void StartTimer();
	double EndTimer();
	double GetElapsedTime();
private:
	std::chrono::high_resolution_clock::time_point startTime;
	double elapsedTime;
	
};

#endif // TIMER_H
