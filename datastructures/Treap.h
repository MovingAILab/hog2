//
//  Treap.h
//  hog2 mac native demos
//
//  Created by Nathan Sturtevant on 12/29/19.
//  Copyright © 2019 NS Software. All rights reserved.
//

#ifndef Treap_h
#define Treap_h
#include <functional>

template <class key>
class Treap
{
	struct TreapNode
	{
		key value;
		long randValue; // TODO: replace with C++11 rand
		size_t left, right, parent;
	};
	typedef size_t TreapNodePtr;
	const size_t nullTreapNode = -1;
	const size_t rootOfTree = -2;
	TreapNodePtr head;
//	TreapNodePtr freeList;
	std::vector<TreapNode> nodes;
	TreapNodePtr GetNode();
	void Recycle(TreapNodePtr n);
	void Insert(TreapNodePtr n, TreapNodePtr head);
	void HeapifyUp(TreapNodePtr n);

	void RotateLeftChildUp(TreapNodePtr n);
	void RotateRightChildUp(TreapNodePtr n);
	void PrintHelper(TreapNodePtr n, int depth);
	void VerifyHelper(TreapNodePtr n);
	void Remove(TreapNodePtr n);
	size_t GetHeightHelper(TreapNodePtr n);
	void IterateHelper(TreapNodePtr n, double, double, const std::function<void (const key &)> &);
public:
	Treap():head(nullTreapNode) {}
	void Reset();
	void Add(key k);
	key RemoveSmallest();
	bool Remove(key &k);
	void Iterate(double, double, const std::function<void (const key &)> &);
	const key &Peek();
	void Print();
	size_t Size();
	size_t GetHeight();
	key GetNode(size_t);
	void Verify();
};


template <class key>
void Treap<key>::Reset()
{
	head = nullTreapNode;
	nodes.resize(0);
}

template <class key>
void Treap<key>::Add(key k)
{
	TreapNodePtr n = GetNode();
	nodes[n].value = k;
	nodes[n].randValue = random();
	assert(nodes[n].right == nullTreapNode);
	assert(nodes[n].left == nullTreapNode);
	if (head == nullTreapNode)
	{
		head = n;
		nodes[n].parent = rootOfTree;
	}
	else {
		Insert(n, head);
	}
}

template <class key>
void Treap<key>::HeapifyUp(TreapNodePtr n)
{
	while (nodes[n].parent != rootOfTree)
	{
		// heap invariant - parents value >= child value
		if (nodes[nodes[n].parent].randValue >= nodes[n].randValue)
			return; // done
		
		if (nodes[nodes[n].parent].left == n)
		{
			RotateLeftChildUp(nodes[n].parent);
		}
		else {
			assert(nodes[nodes[n].parent].right == n);
			RotateRightChildUp(nodes[n].parent);
		}
	}
}

template <class key>
void Treap<key>::RotateLeftChildUp(TreapNodePtr n)
{
//	Print();
//	printf("LRotating %d:\n", nodes[n].value);

	// 1. left child gets parent as right child
	// 2. parent gets left/right child as left child

	TreapNodePtr l = nodes[n].left;
	TreapNodePtr lr = nodes[l].right;
	if (nodes[n].parent != rootOfTree)
	{
		if (nodes[nodes[n].parent].right == n)
			nodes[nodes[n].parent].right = l;
		else {
			assert(nodes[nodes[n].parent].left == n);
			nodes[nodes[n].parent].left = l;
		}
	}
	else {
//		printf("At root\n");
		head = l;
	}

	nodes[l].parent = nodes[n].parent;
	nodes[n].parent = l;
	if (lr != nullTreapNode)
		nodes[lr].parent = n;
	nodes[n].left = lr;
	nodes[l].right = n;
}

template <class key>
void Treap<key>::RotateRightChildUp(TreapNodePtr n)
{
//	Print();
//	printf("RRotating %d:\n", nodes[n].value);
	
	// 1. right child gets parent as left child
	// 2. parent gets right/left child as right child

	TreapNodePtr r = nodes[n].right;
	TreapNodePtr rl = nodes[r].left;
	if (nodes[n].parent != rootOfTree)
	{
		if (nodes[nodes[n].parent].right == n)
			nodes[nodes[n].parent].right = r;
		else {
			assert(nodes[nodes[n].parent].left == n);
			nodes[nodes[n].parent].left = r;
		}
	}
	else {
//		printf("At root\n");
		head = r;
	}
	
	nodes[r].parent = nodes[n].parent;
	nodes[n].parent = r;
	if (rl != nullTreapNode)
		nodes[rl].parent = n;
	nodes[n].right = rl;
	nodes[r].left = n;
}

template <class key>
void Treap<key>::Insert(TreapNodePtr n, TreapNodePtr where)
{
	if (nodes[n].value < nodes[where].value) // insert left
	{
		if (nodes[where].left == nullTreapNode)
		{
			nodes[where].left = n;
			nodes[n].parent = where;
			HeapifyUp(n);
//			Verify();
		}
		else {
			Insert(n, nodes[where].left);
		}
	}
	else { // insert right
		if (nodes[where].right == nullTreapNode)
		{
			nodes[where].right = n;
			nodes[n].parent = where;
			HeapifyUp(n);
//			Verify();
		}
		else {
			Insert(n, nodes[where].right);
		}
	}
}

template <class key>
typename Treap<key>::TreapNodePtr Treap<key>::GetNode()
{
	TreapNodePtr t;
//	if (freeList == nullTreapNode)
//	{
//	}
//	else {
//		t = freeList;
//		freeList = nodes[t].parent;
//	}
	nodes.resize(nodes.size()+1);
	t = nodes.size()-1;
	nodes[t].right = nodes[t].left = nullTreapNode;
	return t;
}

template <class key>
void Treap<key>::Recycle(TreapNodePtr n)
{
	// Lucky us - removing last element
	if (n == nodes.size()-1)
	{
		nodes.resize(nodes.size()-1);
		if (n == 0)
			head = nullTreapNode;
		return;
	}
	// Swap with end and switch pointers
	nodes[n] = nodes[nodes.size()-1];
	nodes.resize(nodes.size()-1);
	if (nodes[n].parent != rootOfTree)
	{
		if (nodes[nodes[n].parent].right == nodes.size())
			nodes[nodes[n].parent].right = n;
		else {
			assert(nodes[nodes[n].parent].left == nodes.size());
			nodes[nodes[n].parent].left = n;
		}
	}
	else {
		head = n;
	}
	
	if (nodes[n].left != nullTreapNode)
		nodes[nodes[n].left].parent = n;

	if (nodes[n].right != nullTreapNode)
		nodes[nodes[n].right].parent = n;
//	Verify();
	//	nodes[n].parent = freeList;
	//	freeList = n;
}


template <class key>
bool Treap<key>::Remove(key &k)
{
	// No elements in treap
	TreapNodePtr n = head;
	while (n != nullTreapNode)
	{
		if (k == nodes[n].value)
		{
			Remove(n);
//			Verify();
			return true;
		}
		if (k < nodes[n].value)
		{
			n = nodes[n].left;
		}
		else {
			n = nodes[n].right;
		}
	}
	return false;
}

template <class key>
key Treap<key>::RemoveSmallest()
{
	// No elements in treap
	assert(head != nullTreapNode);
	TreapNodePtr n = head;
	while (nodes[n].left != nullTreapNode)
		n = nodes[n].left;
	key k = nodes[n].value;
	Remove(n);
	return k;
}

template <class key>
void Treap<key>::Iterate(double low, double high, const std::function<void (const key &)> &f)
{
	IterateHelper(head, low, high, f);
}

template <class key>
void Treap<key>::IterateHelper(TreapNodePtr n, double low, double high, const std::function<void (const key &)> &f)
{
	if (n == nullTreapNode)
		return;
	if (nodes[n].value > low && nodes[n].value <= high)
	{
		f(nodes[n].value);
	}
	if (nodes[n].value <= high)
		IterateHelper(nodes[n].right, low, high, f);
	if (nodes[n].value > low)
		IterateHelper(nodes[n].left, low, high, f);

}


template <class key>
const key &Treap<key>::Peek()
{
	// No elements in treap
	assert(head != nullTreapNode);
	TreapNodePtr n = head;
	while (nodes[n].left != nullTreapNode)
		n = nodes[n].left;
	return nodes[n].value;
}

template <class key>
void Treap<key>::Remove(TreapNodePtr n)
{
	// Case 1: Leaf
	if (nodes[n].left == nullTreapNode && nodes[n].right == nullTreapNode)
	{
		if (nodes[n].parent == rootOfTree)
		{
			head = nullTreapNode;
			Recycle(n);
			return;
		}
		if (nodes[nodes[n].parent].left == n)
		{
			nodes[nodes[n].parent].left = nullTreapNode;
		}
		else {
			assert(nodes[nodes[n].parent].right == n);
			nodes[nodes[n].parent].right = nullTreapNode;
		}
		Recycle(n);
	}
	// Case 2: Single child
	else if (nodes[n].left == nullTreapNode)
	{
		if (nodes[n].parent == rootOfTree)
		{
			head = nodes[n].right;
			nodes[nodes[n].right].parent = rootOfTree;
		}
		else if (nodes[nodes[n].parent].left == n)
		{
			nodes[nodes[n].parent].left = nodes[n].right;
			nodes[nodes[n].right].parent = nodes[n].parent;
		}
		else {
			assert(nodes[nodes[n].parent].right == n);
			nodes[nodes[n].parent].right = nodes[n].right;
			nodes[nodes[n].right].parent = nodes[n].parent;
		}
		Recycle(n);
	}
	else if (nodes[n].right == nullTreapNode)
	{
		if (nodes[n].parent == rootOfTree)
		{
			head = nodes[n].left;
			nodes[nodes[n].left].parent = rootOfTree;
		}
		else if (nodes[nodes[n].parent].left == n)
		{
			nodes[nodes[n].parent].left = nodes[n].left;
			nodes[nodes[n].left].parent = nodes[n].parent;
		}
		else {
			assert(nodes[nodes[n].parent].right == n);
			nodes[nodes[n].parent].right = nodes[n].left;
			nodes[nodes[n].left].parent = nodes[n].parent;
		}
		Recycle(n);
	}
	// Case 3: Find next largest key and move it up
	else {
		// Q: What if we copy *just* the key up? Then the heap property is
		// maintained, as is the binary tree property.
		// Q: Does it violate the distribution of keys?
		// A: I don't know - need to analyze this further.
		
		// 1. Find next key (right/->left)
		TreapNodePtr next = nodes[n].right;
		while (nodes[next].left != nullTreapNode)
			next = nodes[next].left;
		nodes[n].value = nodes[next].value;
		Remove(next);
	}
//	Verify();
}


template <class key>
void Treap<key>::Print()
{
	PrintHelper(head, 0);
	std::cout << "\n";
}

template <class key>
void Treap<key>::PrintHelper(TreapNodePtr n, int depth)
{
	if (n == nullTreapNode)
		return;
	if (nodes[n].right != nullTreapNode)
		PrintHelper(nodes[n].right, depth+1);
	for (int x = 0; x < 4*depth; x++)
		std::cout << " ";
	std::cout << nodes[n].value << " (d" << depth << ", " << nodes[n].randValue << ")\n";
	if (nodes[n].left != nullTreapNode)
		PrintHelper(nodes[n].left, depth+1);
}

template <class key>
void Treap<key>::Verify()
{
//	Print();
	assert(nodes[head].parent == rootOfTree);
	VerifyHelper(head);
}

template <class key>
void Treap<key>::VerifyHelper(TreapNodePtr n)
{
	if (nodes[n].left != nullTreapNode)
	{
		assert(nodes[nodes[n].left].parent == n);
		assert(nodes[n].randValue >= nodes[nodes[n].left].randValue);
		VerifyHelper(nodes[n].left);
	}
	if (nodes[n].right != nullTreapNode)
	{
		assert(nodes[nodes[n].right].parent == n);
		assert(nodes[n].randValue >= nodes[nodes[n].right].randValue);
		VerifyHelper(nodes[n].right);
	}
}

template <class key>
size_t Treap<key>::Size()
{
	return nodes.size();
}

template <class key>
key Treap<key>::GetNode(size_t which)
{
	return nodes[which].value;
}

template <class key>
size_t Treap<key>::GetHeight()
{
	return GetHeightHelper(head);
}

template <class key>
size_t Treap<key>::GetHeightHelper(TreapNodePtr n)
{
	size_t h = 0;
	if (n == nullTreapNode)
		return h;
	if (nodes[n].right != nullTreapNode)
		h = std::max(h, 1+GetHeightHelper(nodes[n].right));
	if (nodes[n].left != nullTreapNode)
		h = std::max(h, 1+GetHeightHelper(nodes[n].left));
	return h;
}


#endif /* Treap_h */
