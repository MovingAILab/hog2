//
//  FixedSizeSet.h
//  hog2 glut
//
//  Created by Nathan Sturtevant on 7/27/16.
//  Copyright © 2016 University of Denver. All rights reserved.
//

#ifndef FixedSizeSet_h
#define FixedSizeSet_h

#include <stdio.h>
#include <string.h>
#include <functional>
#include <cstddef>
template<typename T>
class fssIterator;

template <typename T, class Hash = std::hash<T>>
class FixedSizeSet {
private:
	struct field {
		T item;
		field *next;
		bool valid;
	};
public:
	FixedSizeSet(size_t capacity);
	~FixedSizeSet();
	void resize(size_t capacity);
	void swap(FixedSizeSet<T, Hash> &s);
	void clear();
	size_t size() { return currentMemoryEntry/*-removed*/; }
	typedef fssIterator<field> iterator;
	
	iterator begin(){return iterator(&memory[0]);}
	iterator end() const {return iterator(&memory[currentMemoryEntry]);}
	iterator find(const T &item) const
	{
		size_t hash = h(item);
		for (field* f = hashTable[hash%hashTableSize]; f; f = f->next)
		{
			if (f->item == item && f->valid)
				return iterator(f);
		}
		return end();
	}
	void erase(iterator &i);
	void insert(const T &item);
	void PrintStats();
private:
	field **hashTable;
	field *memory;
	size_t currentMemoryEntry;
	size_t capacity;
	size_t hashTableSize;
	size_t removed;
	Hash h;
};

template <typename T, class Hash>
void FixedSizeSet<T, Hash>::swap(FixedSizeSet<T, Hash> &s)
{
	field **a, *b;
	size_t c, d, e, f;
	a = hashTable;
	b = memory;
	c = currentMemoryEntry;
	d = capacity;
	e = hashTableSize;
	f = removed;
	*this = s;
	s.hashTable = a;
	s.memory = b;
	s.currentMemoryEntry = c;
	s.capacity = d;
	s.hashTableSize = e;
	s.removed = f;
}


template <typename T, class Hash>
FixedSizeSet<T, Hash>::FixedSizeSet(size_t capacity)
{
	hashTableSize = (2*capacity)|1;//capacity*2+3;
	hashTable = new field*[hashTableSize];
	memory = new field[capacity];
	memset(memory, 0x0, capacity*sizeof(field));
	memset(hashTable, 0x0, hashTableSize*sizeof(field*));
	currentMemoryEntry = 0;
	this->capacity = capacity;
	removed = 0;
}

template <typename T, class Hash>
FixedSizeSet<T, Hash>::~FixedSizeSet()
{
	delete [] hashTable;
	delete [] memory;
	memory = 0;
	hashTable = 0;
	currentMemoryEntry = 0;
	capacity = 0;
	removed = 0;
}

template <typename T, class Hash>
void FixedSizeSet<T, Hash>::resize(size_t capacity)
{
//	if (capacity <= this->capacity)
//	{
//		currentMemoryEntry = 0;
//		removed = 0;
//		memset(hashTable, 0x0, hashTableSize*sizeof(field*));
//		memset(memory, 0x0, capacity*sizeof(field));
//	}
//	else {
	this->capacity = capacity;
	delete [] hashTable;
	delete [] memory;
	hashTableSize = (2*capacity)|1;//capacity*2+3;
	hashTable = new field*[hashTableSize];
	memory = new field[capacity];
	memset(memory, 0x0, capacity*sizeof(field));
	memset(hashTable, 0x0, hashTableSize*sizeof(field*));
	currentMemoryEntry = 0;
	removed = 0;
//	}
}

template <typename T, class Hash>
void FixedSizeSet<T, Hash>::clear()
{
	currentMemoryEntry = 0;
	removed = 0;
	memset(hashTable, 0x0, capacity*sizeof(field*));
	memset(memory, 0x0, capacity*sizeof(field));
}

template <typename T, class Hash>
void FixedSizeSet<T, Hash>::PrintStats()
{
	std::vector<int> dist;
	for (int x = 0; x < hashTableSize; x++)
	{
		int len = 0;
		for (field* f = hashTable[x]; f; f = f->next)
		{
			len++;
		}
		if (len >= dist.size())
			dist.resize(len+1);
		dist[len]++;
	}
	for (int x = 0; x < dist.size(); x++)
		printf("%d : %d\n", x, dist[x]);
}

template <typename T, class Hash>
void FixedSizeSet<T, Hash>::erase(iterator &i)
{
//	if ((*i).valid == true)
//	{
		(*i).valid = false;
//		removed++;
//	}
}

template <typename T, class Hash>
void FixedSizeSet<T, Hash>::insert(const T &item)
{
	size_t hash = h(item);
	if (hashTable[hash%hashTableSize] == 0)
	{
		field *f = &memory[currentMemoryEntry];
		currentMemoryEntry++;
		f->item = item;
		f->next = 0;
		f->valid = true;
		hashTable[hash%hashTableSize] = f;
	}
	else {
		for (field* t = hashTable[hash%hashTableSize]; t; t = t->next)
		{
			if (t->item == item)
				return;
			if (t->next == 0)
			{
				field *f = &memory[currentMemoryEntry];
				currentMemoryEntry++;
				f->item = item;
				f->next = 0;
				f->valid = true;
				t->next = f;
			}
		}
	}
}


template<typename T>
class fssIterator : public std::iterator<std::random_access_iterator_tag, T, ptrdiff_t, T*, T&>
{
public:
	fssIterator(T* ptr = nullptr){m_ptr = ptr;}
	fssIterator(const fssIterator<T>& rawIterator) = default;
	~fssIterator(){}
	
	fssIterator<T>& operator=(const fssIterator<T>& rawIterator) = default;
	fssIterator<T>& operator=(T* ptr){m_ptr = ptr;return (*this);}
	
	operator bool()const
	{
		if(m_ptr)
			return true;
		else
			return false;
	}

	bool operator==(const fssIterator<T>& rawIterator)const{return (m_ptr == rawIterator.getConstPtr());}
	bool operator!=(const fssIterator<T>& rawIterator)const{return (m_ptr != rawIterator.getConstPtr());}
	fssIterator<T>& operator+=(const ptrdiff_t& movement){m_ptr += movement;return (*this);}
	fssIterator<T>& operator-=(const ptrdiff_t& movement){m_ptr -= movement;return (*this);}
	fssIterator<T>& operator++(){++m_ptr;return (*this);}
	fssIterator<T>& operator--(){--m_ptr;return (*this);}
	fssIterator<T> operator++(int){auto temp(*this);++m_ptr;return temp;}
	fssIterator<T> operator--(int){auto temp(*this);--m_ptr;return temp;}
	fssIterator<T> operator+(const ptrdiff_t& movement){auto oldPtr = m_ptr;m_ptr+=movement;auto temp(*this);m_ptr = oldPtr;return temp;}
	fssIterator<T> operator-(const ptrdiff_t& movement){auto oldPtr = m_ptr;m_ptr-=movement;auto temp(*this);m_ptr = oldPtr;return temp;}
	
	ptrdiff_t operator-(const fssIterator<T>& rawIterator){return std::distance(rawIterator.getPtr(),this->getPtr());}
	T& operator*(){return *m_ptr;}
	const T& operator*()const{return *m_ptr;}
	T* operator->(){return m_ptr;}
	T* getPtr()const{return m_ptr;}
	const T* getConstPtr()const{return m_ptr;}
	
protected:
	
	T* m_ptr;
};
//-------------------------------------------------------------------

#endif /* FixedSizeSet_h */
