#include "Timer.h"

Timer::Timer()
{
	elapsedTime = 0;
}


void Timer::StartTimer()
{
	startTime = std::chrono::high_resolution_clock::now();
}


double Timer::EndTimer()
{
	auto stopTime =  std::chrono::high_resolution_clock::now();
	auto difference = stopTime - startTime;
	std::chrono::duration<double> time_span = std::chrono::duration_cast<std::chrono::duration<double>>(difference);
	elapsedTime = time_span.count();
	
	return elapsedTime;
}

double Timer::GetElapsedTime()
{
	return elapsedTime;
}
