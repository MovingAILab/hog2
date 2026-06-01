#pragma once
#include <functional>
#include <iostream>
#include <limits>
#include <unordered_map>
#include <vector>

using namespace std;

template<class T>
class PriorityQueue
{
protected:
    vector<T>                 c;
    unordered_map<T, size_t>  item2index;
    std::function<bool(T, T)> comp;
    double                    capacity;

    static bool lessThan(const T n1, const T n2) { return n1 < n2; }

public:
    PriorityQueue()
    {
        capacity = numeric_limits<double>::infinity();
        comp     = lessThan;
        makeHeap();
    }

    PriorityQueue(double                                       maxCapacity,
                  const std::function<bool(const T, const T)>& comp_)
        : capacity(maxCapacity)
        , comp(comp_)
    {
        makeHeap();
    }

    PriorityQueue(const std::function<bool(const T, const T)>& comp_)
        : comp(comp_)
    {
        capacity = numeric_limits<double>::infinity();
        makeHeap();
    }

    PriorityQueue(double maxCapacity)
        : capacity(maxCapacity)
    {
        comp = lessThan;
        makeHeap();
    }

    PriorityQueue(const PriorityQueue<T>& pq)
    {
        c        = pq.c;
        comp     = pq.comp;
        capacity = pq.capacity;
        item2index = pq.item2index;
    }

    PriorityQueue<T>& operator=(const PriorityQueue<T>& rhs)
    {
        if (&rhs == this)
            return *this;
        c        = rhs.c;
        comp     = rhs.comp;
        capacity = rhs.capacity;
        item2index = rhs.item2index;
        return *this;
    }

    void swapComparator(const std::function<bool(const T, const T)>& comp_)
    {
        comp = comp_;
        makeHeap();
    }

    void update(T item)
    {
        if (item2index.find(item) == item2index.end()) {
            return;
        }

        auto itemIndex = item2index[item];
        if (comp(c[itemIndex], c[parent(itemIndex)]))
            pullUp(itemIndex);
        else
            pushDown(itemIndex);
    }

    bool empty() const { return c.empty(); }

    size_t size() const { return c.size(); }

    size_t getItem2IndexMapSize() const { return item2index.size(); }
    size_t getItem2IndexMapValue(T item) const { return item2index.at(item); }

    const T top() const { return c.front(); }

    void push(const T item)
    {
        // If the size is at maximum capacity, find the worst item in the queue
        if (c.size() == capacity) {
            size_t worstIndex = 0;
            for (size_t i = 1; i < c.size(); i++) {
                if (comp(c[worstIndex], c[i])) {
                    worstIndex = i;
                }
            }

            // Check if the new item is better than the worst...
            if (comp(item, c[worstIndex])) {
                // Erase the worst
                swap2item(worstIndex, last());

                item2index.erase(c[last()]);
                c.erase(c.begin() + static_cast<long int>(last()));

                if (comp(c[worstIndex], c[parent(worstIndex)]))
                    pullUp(worstIndex);
                else
                    pushDown(worstIndex);
            } else {
                // If it isn't, do not add this item
                return;
            }
        }

        c.push_back(item);
        item2index[item] = c.size() - 1;
        pullUp(last());
    }

    void pop()
    {
        if (c.empty()) {
            return;
        }
        swap2item(0, last());

        item2index.erase(c[last()]);
        c.erase(c.begin() + static_cast<long int>(last()));

        pushDown(0);
    }

    void remove(T item)
    {
        if (item2index.find(item) == item2index.end()) {
            return;
        }

        auto itemIndex = item2index[item];
        swap2item(item2index[item], last());
        item2index.erase(c[last()]);
        c.erase(c.begin() + static_cast<long int>(last()));

        if (comp(c[itemIndex], c[parent(itemIndex)]))
            pullUp(itemIndex);
        else
            pushDown(itemIndex);
    }

    void clear()
    {
        while (!c.empty()) {
            c.pop_back();
        }
    }

    typename vector<T>::iterator begin() { return c.begin(); }
    typename vector<T>::iterator end() { return c.end(); }

	typename vector<T>::const_iterator begin() const { return c.begin(); }
	typename vector<T>::const_iterator end() const { return c.end(); }

    typename vector<T>::iterator find(T item)
    {
        if (item2index.find(item) == item2index.end()) {
            return c.end();
        }

        return c.begin() + static_cast<long int>(item2index[item]);
    }

    void makeHeap()
    {
        for (int i = (static_cast<int>(c.size()) / 2) - 1; i >= 0; i--) {
            pushDown(static_cast<size_t>(i));
        }
    }

private:
    size_t last() { return c.size() - 1; }

    size_t parent(size_t i) { return i > 1 ? (i - 1) / 2 : 0; }

    size_t rightChild(size_t i) { return static_cast<size_t>(2 * i + 2); }

    size_t leftChild(size_t i) { return static_cast<size_t>(2 * i + 1); }

    void swap2item(size_t a, size_t b)
    {
        item2index[c[a]] = b;
        item2index[c[b]] = a;
        swap(c[a], c[b]);
    }

    void pullUp(size_t i)
    {
        if (comp(c[i], c[parent(i)])) {
            swap2item(i, parent(i));
            pullUp(parent(i));
        }
    }

    void pushDown(size_t i)
    {
        size_t smallesti = i;

        if (rightChild(i) < c.size() && comp(c[rightChild(i)], c[smallesti])) {
            smallesti = rightChild(i);
        }

        if (leftChild(i) < c.size() && comp(c[leftChild(i)], c[smallesti])) {
            smallesti = leftChild(i);
        }

        if (smallesti != i) {
            swap2item(i, smallesti);
            pushDown(smallesti);
        }
    }
};
