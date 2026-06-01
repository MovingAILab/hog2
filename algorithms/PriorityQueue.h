#include <iostream>
#include <vector>
#include <functional> // for std::less

template<typename T, typename Compare = std::less<T>>
class PriorityQueue {
public:
    PriorityQueue() {}

    bool empty() const {
        return heap.empty();
    }

    size_t size() const {
        return heap.size();
    }

    const T& top() const {
        return heap.front();
    }

    void push(const T& element) {
        heap.push_back(element);
        heapifyUp(heap.size() - 1);
    }

    void pop() {
        if (empty()) {
            return; // Empty heap
        }
        std::swap(heap[0], heap.back());
        heap.pop_back();
        heapifyDown(0);
    }

private:
    std::vector<T> heap;
    Compare compare;

    void heapifyUp(size_t index) {
        while (index > 0) {
            size_t parent = (index - 1) / 2;
            if (compare(heap[index], heap[parent])) {
                std::swap(heap[index], heap[parent]);
                index = parent;
            } else {
                break;
            }
        }
    }

    void heapifyDown(size_t index) {
        size_t size = heap.size();
        while (true) {
            size_t leftChild = index * 2 + 1;
            size_t rightChild = index * 2 + 2;
            size_t smallest = index;

            if (leftChild < size && compare(heap[leftChild], heap[smallest])) {
                smallest = leftChild;
            }
            if (rightChild < size && compare(heap[rightChild], heap[smallest])) {
                smallest = rightChild;
            }

            if (smallest != index) {
                std::swap(heap[index], heap[smallest]);
                index = smallest;
            } else {
                break;
            }
        }
    }
};

