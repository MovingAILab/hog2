#include <set>
#include <vector>
#include <map>
#include <cmath>
#include <algorithm>
#include <iostream>

using namespace std;

enum DistanceType {l2, l1, linf, lmax};


template<int N, class Data>
class QuadTree
{
private:
	std::set<Data> data;
	QuadTree<N, Data>* children[N] = {};
	QuadTree<N, Data>* parent;
	

public:
	int _size;
	vector<int> min, max, center;
	//QuadTree(uint32_t min_x, uint32_t min_y, uint32_t max_x, uint32_t max_y, QuadTree* parent);
	QuadTree(vector<int> _min, vector<int> _max, QuadTree<N, Data>* parent);
	~QuadTree();
	//uint32_t GetPosition(uint32_t x, uint32_t y){return (x << 16) + y;}
	//uint32_t GetX(uint32_t position){return (position >> 16);}
	//uint32_t GetY(uint32_t position){return (position & 0xffff);}
	bool IsLeaf();
	int GetIndex(vector<int> position);
	std::pair<vector<int>, vector<int>> GetChildBound(int index);
	double GetDistance(vector<int> position);
	void Insert(vector<int> pos, Data _data);
	void Remove(vector<int> pos, Data _data);
	void Remove(Data _data);
	std::vector<QuadTree<N, Data>> SortedNeighbors(vector<int> position);
	void _FindNearest(vector<int> target, double &minDist, vector<int>& nearest, int &count, std::set<Data> &_data);
	std::pair<vector<int>, std::set<Data>> FindNearest(vector<int> target);
	std::vector<QuadTree<N, Data>> GetChildren();
	double DistanceToCenter(vector<int> point, DistanceType dtype);
	double DistanceToBounds(vector<int> point, DistanceType dtype);
	void PrintExistingPoints()
	{
		if (IsLeaf())
		{
			for (auto item : center)
			{
				cout << item << " ";
			}
			cout << endl;
		}
		else
		{
			for (auto ch : children)
			{
				if (ch != NULL)
				{
					ch->PrintExistingPoints();
				}
			}
		}
	}

	int size()
	{
		return _size;
	}
};


template<int N, class Data>
struct QTreeComparer {
    QTreeComparer(vector<int> position) { this->position = position; }
    bool operator () (QuadTree<N, Data> x, QuadTree<N, Data> y) {
		return x.GetDistance(position) < y.GetDistance(position) ? -1 : 1;
	}
    vector<int> position;
};


template<int N, class Data>
QuadTree<N, Data>::QuadTree(vector<int> _min, vector<int> _max, QuadTree<N, Data>* parent)
{
	_size = 0;
	this->parent = parent;
	min = _min;
	max = _max;
    center.clear();
    for (size_t i = 0; i < min.size(); i++)
    {
        center.push_back((min[i] + max[i]) / 2);
    }
}

template<int N, class Data>
QuadTree<N, Data>::~QuadTree()
{
}

template<int N, class Data>
bool QuadTree<N, Data>::IsLeaf()
{
	return min == max;
}

template<int N, class Data>
int QuadTree<N, Data>::GetIndex(vector<int> position)
{
    int ind = 0;
    for (size_t i = 0; i < position.size(); i++)
    {
        ind += (position[i] > center[i] ? pow(2, i) : 0);
    }
    return ind;
}

template<int N, class Data>
std::pair<vector<int>, vector<int>> QuadTree<N, Data>::GetChildBound(int index)
{
    vector<int> _min, _max;
    for (size_t i = 0; i < min.size(); i++)
    {
        if (index & 1 == 1)
        {
            _min.push_back(center[i] + 1);
            _max.push_back(max[i]);
			//cout << "1";
        }
        else
        {
            _min.push_back(min[i]);
            _max.push_back(center[i]);
			//cout << "0";
        }
        index /= 2;
    }
	//cout << endl;
    return make_pair(_min, _max);
}


template<int N, class Data>
double QuadTree<N, Data>::GetDistance(vector<int> position)
{
	return DistanceToBounds(position, linf);
}

template<int N, class Data>
void QuadTree<N, Data>::Insert(vector<int> pos, Data _data)
{
	int ind = GetIndex(pos);
	//cout << "index: " << ind << endl;
    if (!IsLeaf())
    {
        if (children[ind] == NULL)
        {
            auto bound = GetChildBound(ind);
            children[ind] = new QuadTree<N, Data>(bound.first, bound.second, this);
        }
        children[ind]->Insert(pos, _data);
    }
	else
	{
		_size++;
		data.insert(_data);
	}
}

template<int N, class Data>
void QuadTree<N, Data>::Remove(vector<int> pos, Data _data)
{
	if (IsLeaf() && pos == center)
    {
        Remove(_data);
    }
    if (!IsLeaf() && children[GetIndex(pos)] != NULL)
	{
		children[GetIndex(pos)]->Remove(pos, _data);
		
	}
        
}

template<int N, class Data>
void QuadTree<N, Data>::Remove(Data _data)
{
	if (IsLeaf())
	{
		if (data.size() > 0 && data.find(_data) != data.end())
		{
			_size--;
			data.erase(_data);
		}
		if (data.size() > 0)
		{
			return;
		}
	}
	if (parent != NULL)
    {
        parent->children[parent->GetIndex(center)] = NULL;
		for (auto item : parent->children)
		{
			if (item != NULL)
                return;
		}
        parent->Remove(_data);
    }
}

template<int N, class Data>
std::vector<QuadTree<N, Data>> QuadTree<N, Data>::SortedNeighbors(vector<int> position)
{
	std::vector<QuadTree<N, Data>> l;
	for (auto item : children)
	{
	    if (item != NULL)
	    {
	        l.push_back(*item);
	    }
	}
	std::sort(l.begin(), l.end(), QTreeComparer<N, Data>(position));
	return l;
}

template<int N, class Data>
void QuadTree<N, Data>::_FindNearest(vector<int> target, double &minDist, vector<int>& nearest, int &count, std::set<Data> &_data)
{
    count++;
    if (IsLeaf())
    {
		double dist = DistanceToCenter(target, linf);//pow(pow((int)GetX(center) - (int)GetX(target), 2) + pow((int)GetY(center) - (int)GetY(target), 2), 0.5);
		//cout << "Dist: " << dist << endl;
        if (dist < minDist)
        {
			//std::cout << GetX(center) << " " << GetY(center) << " " << GetX(target) << " " << GetY(target) << std::endl;
			//std::cout << "Dist before: " << minDist << std::endl;
            minDist = dist;
            nearest = center;
			_data = data;
			//std::cout << "Dist after: " << minDist << std::endl;
        }
        return;
    }
    std::vector<QuadTree<N, Data>> l;
	l = SortedNeighbors(target);
    for (auto item : l)
    {
        if (item.GetDistance(target) < minDist)
            item._FindNearest(target, minDist, nearest, count, _data);
    }
}

template<int N, class Data>
std::pair<vector<int>, std::set<Data>> QuadTree<N, Data>::FindNearest(vector<int> target)
{
	double minDist = INFINITY;
	int count = 0;
	vector<int> res;
	std::set<Data> _data;
	_FindNearest(target, minDist, res, count, _data);
	//std::cout << "\tcount: " << count << endl;
	return make_pair(res, _data);
}


template<int N, class Data>
std::vector<QuadTree<N, Data>> QuadTree<N, Data>::GetChildren()
{
	std::vector<QuadTree<N, Data>> l;
	for (size_t i = 0; i < min.size(); i++)
	{
		if (children[i] != nullptr)
			l.push_back(*children[i]);
	}
	return l;
}


template<int N, class Data>
double QuadTree<N, Data>::DistanceToCenter(vector<int> point, DistanceType dtype)
{
	double res = 0;
	int m1, m2;
	switch (dtype)
	{
	case l1:
        res = 0;
        for (size_t i = 0; i < min.size(); i++)
        {
            res += abs(point[i] - center[i]);
        }
		return res;
	case l2:
        res = 0;
        for (size_t i = 0; i < min.size(); i++)
        {
            res += pow(center[i] - point[i], 2);
        }
		return pow(res, 0.5);
	case linf:
        res = -1;
        for (size_t i = 0; i < min.size(); i++)
        {
            if (abs(point[i] - center[i]) > res)
            {
                res = abs(point[i] - center[i]);
            }
        }
		return res;
	case lmax:
		m1 = *std::max_element(point.begin(), point.end());
		m2 = *std::max_element(center.begin(), center.end());
		return abs(m1-m2);
	default:
		return -1;
		break;
	}
}

template<int N, class Data>
double QuadTree<N, Data>::DistanceToBounds(vector<int> point, DistanceType dtype)
{
	bool b;
	switch (dtype)
	{
	case l1:
		b = true;
		for (size_t i = 0; i < min.size(); i++)
		{
			if (point[i] < min[i] || point[i] > max[i])
			{
				b = false;
			}
		}
		if (b)
		{
			return 0;
		}
		else
		{
			int res;
			for (size_t i = 0; i < min.size(); i++)
			{
				res += std::min(abs(point[i] - min[i]), abs(point[i] - max[i]));
			}
			return res;
		}
	case l2:
		b = true;
		for (size_t i = 0; i < min.size(); i++)
		{
			if (point[i] < min[i] || point[i] > max[i])
			{
				b = false;
			}
		}
		if (b)
		{
			return 0;
		}
		else
		{
			double res;
			for (size_t i = 0; i < min.size(); i++)
			{
				res += pow(std::min(abs(point[i] - min[i]), abs(point[i] - max[i])), 2);
			}
			return pow(res, 0.5);
		}
	case linf:
		{
			double temp = 0;
			double res = 0;
			for (size_t i = 0; i < min.size(); i++)
			{
				if (point[i] <= max[i] && point[i] >= min[i])
				{
					temp = 0;
				}
				else
				{
					temp = std::min(abs(point[i] - min[i]), abs(point[i] - max[i]));
				}
				if (temp > res)
					res = temp;
			}
			return res;
		}
	case lmax:
		b = true;
		for (size_t i = 0; i < min.size(); i++)
		{
			if (point[i] < min[i] || point[i] > max[i])
			{
				b = false;
				break;
			}
		}
		if (b)
		{
			return 0;
		}
		else
		{
			auto v1 = std::max_element(point.begin(), point.end());
			auto vmax = std::max_element(max.begin(), max.end());
			auto vmin = std::max_element(min.begin(), min.end());
			return std::min(abs(v1 - vmax), abs(v1 - vmin));
		}
	default:
		return -1;
		break;
	}
}
