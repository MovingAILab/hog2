//
//  SharedQueue.h
//  hog2 glut
//
//  Created by Nathan Sturtevant on 11/3/14.
//  Copyright (c) 2014 University of Denver. All rights reserved.
//

#ifndef SHARED_QUEUE_H
#define SHARED_QUEUE_H

#include <iostream>
#include <mutex>
#include <deque>
#include <condition_variable>
/* SharedQueue
 *
 * A producer/consumer queue backed by a deque. Can optionally enforce
 * capacity constraints. TODO: Need to verify that no deadlocks occur if
 * mixing waiting and non-waiting functions.
 */

template <typename T>
class SharedQueue {
public:
	SharedQueue(int maxCapacity = -1);
	~SharedQueue();
	bool IsEmpty() const;
	void Add(T value);
	bool Remove(T &item);
	
	void WaitAdd(const T &value);
	void WaitRemove(T &item);
	void Print();
	size_t size();
private:
	std::deque<T> queue;
	int maxCapacity;
	mutable std::mutex lock;
	mutable std::condition_variable notFull;
	mutable std::condition_variable notEmpty;
};

template <typename T>
SharedQueue<T>::SharedQueue(int capacity)
:maxCapacity(capacity)
{
}

template <typename T>
SharedQueue<T>::~SharedQueue()
{
}

template <typename T>
bool SharedQueue<T>::IsEmpty() const
{
	lock.lock();
	bool result = (queue.empty());
	lock.unlock();
	return result;
}

template <typename T>
size_t SharedQueue<T>::size()
{
	lock.lock();
	size_t result = queue.size();
	lock.unlock();
	return result;
}

/* Adds item to queue, regardless of capacity.  */
template <typename T>
void SharedQueue<T>::Add(T value)
{
	lock.lock();
	queue.push_back(value);
	lock.unlock();
	notEmpty.notify_one();
}

/* Tries to remove, returning true if successful, false otherwise. */
template <typename T>
bool SharedQueue<T>::Remove(T &item)
{
	lock.lock();
	if (queue.empty())
	{
		lock.unlock();
		return false;
	}
	
	item = queue.front();
	queue.pop_front();
	lock.unlock();
	notFull.notify_one();
	return true;
}

/* Adds item to queue, waiting if the queue is full.  */
template <typename T>
void SharedQueue<T>::WaitAdd(const T &value)
{
	std::unique_lock<std::mutex> l(lock);
	
	notFull.wait(l, [this](){return queue.size() < maxCapacity; });
	queue.push_back(value);
	
	notEmpty.notify_one();
}

/* Removes item from queue, waiting until item is available.  */
template <typename T>
void SharedQueue<T>::WaitRemove(T &item)
{
	std::unique_lock<std::mutex> l(lock);

	notEmpty.wait(l, [this](){return queue.size() > 0; });

	item = queue.front();
	queue.pop_front();

	notFull.notify_one();
}

template <typename T>
void SharedQueue<T>::Print()
{
	lock.lock();
	for (auto c : queue)
	{
		std::cout << c << " ";
	}
	std::cout << std::endl;
	lock.unlock();
}


#endif
