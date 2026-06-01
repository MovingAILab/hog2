/*
 *  $Id: Graph.cpp
 *  hog2
 *
 *  Created by Nathan Sturtevant on 10/24/06.
 *  Modified by Nathan Sturtevant on 02/29/20.
 *
 * This file is part of HOG2. See https://github.com/nathansttt/hog2 for licensing information.
 *
 */ 

// HOG File

#include <cstdlib>
#include "Graph.h"
#include <cstring>

#include <vector>

using namespace std;

void graph_object::Print(ostream& /*out*/) const
{
	//	out << val;
}

ostream& operator <<(ostream & out, const graph_object &_Obj)
{
	_Obj.Print(out);
	return out;
}

Graph::Graph()
:_nodes(), _edges()
{
	//node_index = edge_index = 0;
}

Graph::~Graph()
{
	//	cout << "destructor got called" << endl;
	Reset();
}

void Graph::Reset()
{
	node_iterator ni;
	edge_iterator ei;
	ni = getNodeIter();
	while (1)
	{
		node *n = nodeIterNext(ni);
		if (n)
		{
			delete n;
			n = 0;
		}
		else
			break;
	}
	
	ei = getEdgeIter();
	while (1)
	{
		edge *e = edgeIterNext(ei);
		if (e)
		{
			delete e;
			e = 0;
		} else
			break;
	}
	_nodes.clear();
	_edges.clear();
}

graph_object *Graph::Clone() const
{
	Graph *g = new Graph();
	node_iterator it = getNodeIter();
	while (1)
	{
		node *n = (node *)nodeIterNext(it);
		if (n) 
			g->AddNode((node*)n->Clone());
		else
			break;
	}
	return g;
}

Graph *Graph::cloneAll() const
{
	Graph *g = new Graph();
	node_iterator it = getNodeIter();
	while (1)
	{
		node *n = (node *)nodeIterNext(it);
		if (n)
			g->AddNode((node*)n->Clone());
		else
			break;
	}
	edge_iterator ei = getEdgeIter();
	while (1)
	{
		edge *e = (edge *)edgeIterNext(ei);
		if (e)
			g->AddEdge((edge*)e->Clone());
		else
			break;
	}
	return g;
}

void Graph::Export(const char *fname)
{
	FILE *f = fopen(fname, "w+");
	if (f)
	{
		fprintf(f, "d\n%d %d\n", GetNumNodes(), GetNumEdges());
		edge_iterator ei = getEdgeIter();
		while (1)
		{
			edge *e = (edge *)edgeIterNext(ei);
			if (e)
				fprintf(f, "%d %d %d 1\n", e->getFrom(), e->getTo(), (int)(100.0*e->GetWeight()));
			else
				break;
		}
		fclose(f);
	}
}


int Graph::AddNode(node *n)
{
	if (n)
	{
		_nodes.push_back(n);
		n->nodeNum = _nodes.size()-1;
		//n->uniqueID = uniqueCounter++;
		//node_index;
		return n->nodeNum;
		//node_index++;
		//return node_index-1;
	}
	cerr << "ERRROR ERRROR ERRROR ERRROR ERRROR" << endl;
	return -1;
}

node *Graph::GetNode(unsigned long num)
{
	if (num < _nodes.size()) return _nodes[num];
	return 0;
}

const node *Graph::GetNode(unsigned long num) const
{
	if (num < _nodes.size()) return _nodes[num];
	return 0;
}

edge *Graph::GetEdge(unsigned long num)
{
	if (num < _edges.size()) return _edges[num];
	return 0;
}

void Graph::AddEdge(edge *e)
{
	if (e)
	{
		_edges.push_back(e);
		e->edgeNum = _edges.size()-1;
		//_edges[edge_index] = e;
		if (e->getFrom() < _nodes.size())
			_nodes[e->getFrom()]->AddEdge(e);
		else
			cerr << "Adding edge from illegal index" << endl;
		if (e->getTo() < _nodes.size())
			_nodes[e->getTo()]->AddEdge(e);
		else
			cerr << "Adding edge to illegal index" << endl;
		//edge_index++;
	}
}

edge *Graph::findDirectedEdge(unsigned int from, unsigned int to)
{
	node *n = GetNode(from);
	edge_iterator ei = n->getOutgoingEdgeIter();
	while (1)
	{
		edge *e = n->edgeIterNextOutgoing(ei);
		if (e == 0)
			break;
		if (e->getTo() == to)
			return e;
	}
	return 0;
}

const edge *Graph::FindEdge(unsigned int from, unsigned int to) const
{
	const node *n = GetNode(from);
	const node *t = GetNode(to);
	if (n->_allEdges.size() > t->_allEdges.size())
		n = t;
	if (n)
	{
		edge_iterator ei = n->getEdgeIter();
		while (1)
		{
			edge *e = n->edgeIterNext(ei);
			if (e == 0) break;
			if (((e->getTo() == to) && (e->getFrom() == from)) ||
				((e->getFrom() == to) && (e->getTo() == from)))
				return e;
		}
	}
	return 0;
}

/**
 * Finds an edge between nodes with ids from and to, no matter which direction
 *
 * \return the edge found or null if no edge is found
 */
edge *Graph::FindEdge(unsigned int from, unsigned int to)
{
	node *n = GetNode(from);
	node *t = GetNode(to);
	if (n->_allEdges.size() > t->_allEdges.size())
		n = t;
	if (n)
	{
		edge_iterator ei = n->getEdgeIter();
		while (1)
		{
			edge *e = n->edgeIterNext(ei);
			if (e == 0) break;
			if (((e->getTo() == to) && (e->getFrom() == from)) ||
				((e->getFrom() == to) && (e->getTo() == from)))
				return e;
		}
	}
	return 0;
}

bool Graph::relax(edge *e, int weightIndex)
{
	int from = e->getFrom();
	int to = e->getTo();
	if (e == 0) return false;
	double newCost = GetNode(from)->GetLabelF(weightIndex)+e->GetWeight();
	if (newCost <	GetNode(to)->GetLabelF(weightIndex))
	{
		//cout << "re-weighting" << endl << *GetNode(to) << endl;
		GetNode(to)->SetLabelF(weightIndex, newCost);
		//cout << *GetNode(to) << endl;
		return true;
	}
	return false;
}

bool Graph::relaxReverseEdge(edge *e, int weightIndex)
{
	int from = e->getFrom();
	int to = e->getTo();
	if (e == 0) return false;
	double newCost = GetNode(to)->GetLabelF(weightIndex)+e->GetWeight();
	if (newCost <	GetNode(from)->GetLabelF(weightIndex))
	{
		GetNode(from)->SetLabelF(weightIndex, newCost);
		return true;
	}
	return false;
}

node *Graph::GetRandomNode()
{
	if (_nodes.size() == 0) return 0;
	int rand_val = (int)(((double)random()/RAND_MAX)*_nodes.size());
	return _nodes[rand_val];
}

edge *Graph::GetRandomEdge()
{
	if (_edges.size() == 0) return 0;
	//	int rand_val = random()%edge_index;
	int rand_val = (int)(((double)random()/RAND_MAX)*_edges.size());
	return _edges[rand_val];
	//return 0;
}

// fixme: should be inlined
node_iterator Graph::getNodeIter() const
{
	return _nodes.begin();
}

node *Graph::nodeIterNext(node_iterator &node_iter) const
{
	if (node_iter != _nodes.end())
	{
		node *v = *node_iter;
		node_iter++;
		return v;
	}
	return 0;
}

// fixme: should be inlined
edge_iterator Graph::getEdgeIter() const
{
	return _edges.begin();
}

edge *Graph::edgeIterNext(edge_iterator &edge_iter) const
{
	if (edge_iter != _edges.end())
	{
		edge *v = *edge_iter;
		edge_iter++;
		return v;
	}
	return 0;
}

void Graph::RemoveEdge(edge *e)
{
	//cout << "Removing edge " << e->edgeNum << " from " << e->from << " to " << e->to << endl;
	GetNode(e->from)->RemoveEdge(e);
	GetNode(e->to)->RemoveEdge(e);
	unsigned int oldLoc = e->edgeNum;
	edge *replacement = _edges.back();
	//cout << "Removing edge at " << oldLoc << " and putting " << replacement->edgeNum << " in its place" << endl;
	if (_edges[oldLoc] == e)
	{
		_edges[oldLoc] = replacement;
		replacement->edgeNum = oldLoc;
		_edges.pop_back();
	}
	else {
		cerr << "ERROR removing edge at " << oldLoc << endl;
		//		for (unsigned int x = 0; x < _edges.size(); x++)
		//		{
		//			if (_edges[x] == e)
		//				cout << "We found it at " << x << endl;
		//		}
	}
}

// returns the node that had it's node number changed, if any
node *Graph::RemoveNode(node *n, unsigned int &oldID)
{
	if (!n) return 0;
	
	while (n->_allEdges.size() > 0)
	{
		edge *e = n->_allEdges.back();
		//		cout << "All edges contain " << n->_allEdges.size() << " edges." << endl;
		//		cout << "incoming has " << n->_edgesIncoming.size() << " and outgoing has "
		//			<< n->_edgesOutgoing.size() << " edges" << endl;
		//		cout << "Going to remove " << e << " " << *e << endl;
		if (e) RemoveEdge(e);
		else   break;
	}
	//printf("_nodes size is %u\n", _nodes.size());
	node *tmp = _nodes.back();
	_nodes.pop_back();
	if (_nodes.size() > 0)
	{
		_nodes[n->GetNum()] = tmp;
		oldID = tmp->nodeNum;
		tmp->nodeNum = n->GetNum();
	}
	
	if (n == tmp) return 0;
	
	// repair edges to and from this node...
	edge_iterator ei = tmp->getIncomingEdgeIter();
	for (edge *e = tmp->edgeIterNextIncoming(ei); e; e = tmp->edgeIterNextIncoming(ei))
	{
		e->to = tmp->nodeNum;
	}
	ei = tmp->getOutgoingEdgeIter();
	for (edge *e = tmp->edgeIterNextOutgoing(ei); e; e = tmp->edgeIterNextOutgoing(ei))
	{
		e->from = tmp->nodeNum;
	}
	return tmp;
}

// fixme: should be inlined
int Graph::GetNumEdges()
{
	return _edges.size();
}

// fixme: should be inlined
int Graph::GetNumNodes()
{
	return _nodes.size();
}

// BFS from start node
vector<node*>* Graph::getReachableNodes(node* start)
{
	// Non-const array size trick in new compiler; appears as though
	// an initializer is not allowed
	std::vector<int> visitedList(_nodes.size());
	for (unsigned int i = 0; i < _nodes.size(); i++)
	{
		visitedList[i] = 0;
	}
	// Preallocation estimate
	vector<node*> *nodeList = new vector<node*>();
	nodeList->reserve(_nodes.size() * 3 / 4);
	unsigned int index = 0;
	int neighborNum = 0;
	node *n;
	
	nodeList->push_back(start);
	visitedList[start->GetNum()] = 1;
	while (index < nodeList->size())
	{
		n = (*nodeList)[index];
		neighbor_iterator ni = n->getNeighborIter();
		while ((neighborNum = n->nodeNeighborNext(ni)) >= 0)
		{
			//cout << neighborNum << endl;
			if (visitedList[neighborNum] <= 0)
			{
				visitedList[neighborNum] = 1;
				nodeList->push_back(_nodes[neighborNum]);
			}
		}
		index++;
	}
	
	return nodeList;
}

void Graph::Print(ostream &out) const
{
	node_iterator ni = getNodeIter();
	edge_iterator ei = getEdgeIter();
	
	out << "Nodes:" << endl;
	while (1)
	{
		node *n = nodeIterNext(ni);
		if (!n) break;
		out << *n << endl;
	}
	out << "Edges:" << endl;
	while (1)
	{
		edge *e = edgeIterNext(ei);
		if (!e) break;
		out << *e << endl;
	}
}

void Graph::printStats()
{
	cout << GetNumEdges() << " total edges; " << GetNumNodes() << " total nodes." << endl;
	int minEdges = GetNumEdges();
	int maxEdges[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
	node_iterator ni = getNodeIter();
	while (1)
	{
		node *n = nodeIterNext(ni);
		if (!n) break;
		minEdges = min(minEdges, n->GetNumEdges());
		for (int x = 0; x < 10; x++)
		{
			if (n->GetNumEdges() > maxEdges[x])
			{
				for (int y = x; y < 9; y++)
					maxEdges[y+1] = maxEdges[y];
				maxEdges[x] = n->GetNumEdges();
				break;
			}
		}
		//maxEdges = max(maxEdges, n->GetNumEdges());
	}
	cout << "Min edges: " << minEdges << ", max edges: ";
	for (int x = 0; x < 10; x++)
		cout << maxEdges[x] << ", ";
	cout << endl;
}

bool Graph::verifyGraph() const
{
	bool verified = true;
	int totalEdges1 = 0, totalEdges2 = 0;
	node_iterator ni = getNodeIter();
	edge_iterator ei;
	
	//	if (node_index != _nodes.size())
	//	{
	//		cerr << "Error: our node count doesn't match the size of nodes stored" << endl;
	//		verified = false;
	//	}
	//	if (edge_index != _edges.size())
	//	{
	//		cerr << "Error: our edge count doesn't match the size of edges stored" << endl;
	//		verified = false;
	//	}
	
	for (node *n = nodeIterNext(ni); n; n = nodeIterNext(ni))
	{
		if (!n)
		{
			cerr << "Error; we've stored null node" << endl;
			verified = false;
			continue;
		}
		else {
			//cout << "Testing node " << n->GetNum() << endl;
		}
		if (_nodes[n->nodeNum] != n)
		{
			cerr << "Error; node " << n->nodeNum << "'s node number is off." << endl;
			verified = false;
		}
		
		ei = n->getEdgeIter();
		int supposedCount = n->getNumIncomingEdges()+n->getNumOutgoingEdges();
		int realCount = 0;
		for (edge *e = n->edgeIterNext(ei); e; e = n->edgeIterNext(ei))
		{
			totalEdges1++;
			realCount++;
			if ((e->getFrom() != n->GetNum()) && (e->getTo() != n->GetNum()))
			{
				cerr << "At node " << n->GetNum() << " found edge between " << e->getFrom() << " and " << e->getTo() << endl;
				verified = false;
			}
		}
		if (realCount != supposedCount)
		{
			cerr << "At node " << n->GetNum() << " supposed count is " << supposedCount << " but only found " << realCount << endl;
			verified = false;
		}
	}
	
	ei = getEdgeIter();
	for (edge *e = edgeIterNext(ei); e; e = edgeIterNext(ei))
	{
		if (_edges[e->edgeNum] != e)
		{
			cerr << "Error; edge " << e->edgeNum << "'s edge number is off." << endl;
			verified = false;
		}
		
		totalEdges2++;
	}
	if (totalEdges1/2 != totalEdges2)
	{
		cerr << "Edge counts don't match - from nodes " << totalEdges1 << ", from edges " << totalEdges2 << endl; 
		verified = false;
	}
	return verified;
}

edge::edge(unsigned int f, unsigned int t, double w)
: mark(false), from(f), to(t), label()
{ 
	SetLabelF(kEdgeWeight, w);
	//	if ((from == -1) || (to == -1))
	//		cerr << "Error - " << from << "->" << to << endl;
}

void edge::Print(ostream& out) const
{
	out << from << "->" << to << " (" << GetLabelF(kEdgeWeight) << ")";
}

void edge::SetLabelF(unsigned int index, double val)
{
	if (index < label.size())
		label[index].fval = val;
	else {
		while (index > label.size())
		{
			labelValue v; v.fval = (double)MAXINT;
			label.push_back(v);
		}
		labelValue v; v.fval = val;
		label.push_back(v);
	}
}

void edge::SetLabelL(unsigned int index, long val)
{
	if (index < label.size())
		label[index].lval = val;
	else {
		while (index > label.size())
		{
			labelValue v; v.lval = MAXINT;
			label.push_back(v);
		}
		labelValue v; v.lval = val;
		label.push_back(v);
	}
}


unsigned node::uniqueIDCounter = 0;

node::node(const char *n)
:label(), _edgesOutgoing(), _edgesIncoming(), _allEdges()
{
	// fixme: unsafe, 30?
	strncpy(name, n, 30);
	//	for (int x = 0; x < MAXLABELS; x++)
	//		label[x] = MAXINT;
	keyLabel = 0;
	markedEdge = 0;
	uniqueID = uniqueIDCounter++;
}

graph_object *node::Clone() const
{
	node *n = new node(name);
	for (unsigned int x = 0; x < label.size(); x++) n->label.push_back(label[x]);
	n->keyLabel = keyLabel;
	return n;
}

void node::AddEdge(edge *e)
{
	if (e)
	{
		_allEdges.push_back(e);
		if (e->getFrom() == nodeNum)
			_edgesOutgoing.push_back(e);
		else if (e->getTo() == nodeNum)
			_edgesIncoming.push_back(e);
		else
			cerr << "Added an adge that doesn't belong to this node (" << nodeNum << ")" << endl;
	}
}

void node::RemoveEdge(edge *e)
{
	if (nodeNum == e->getTo())
	{
		for (unsigned int x = 0; x < _edgesIncoming.size(); x++)
		{
			if (_edgesIncoming[x] == e)
			{
				_edgesIncoming[x] = _edgesIncoming.back();
				_edgesIncoming.pop_back();
				break;
			}
		}
	}
	else {
		for (unsigned int x = 0; x < _edgesOutgoing.size(); x++)
		{
			if (_edgesOutgoing[x] == e)
			{
				_edgesOutgoing[x] = _edgesOutgoing.back();
				_edgesOutgoing.pop_back();
				break;
			}
		}
	}
	for (unsigned int x = 0; x < _allEdges.size(); x++)
	{
		if (_allEdges[x] == e)
		{
			_allEdges[x] = _allEdges.back();
			_allEdges.pop_back();
			break;
		}
	}
	if (markedEdge == e) markedEdge = 0;
}

void node::SetLabelF(unsigned int index, double val) const
{
	if (index < label.size())
		label[index].fval = val;
	else {
		while (index > label.size())
		{
			labelValue v; v.fval = (double)MAXINT;
			label.push_back(v);
		}
		labelValue v; v.fval = val;
		label.push_back(v);
	}
}

void node::SetLabelL(unsigned int index, long val) const
{
	if (index < label.size())
		label[index].lval = val;
	else {
		while (index > label.size())
		{
			labelValue v; v.lval = MAXINT;
			label.push_back(v);
		}
		labelValue v; v.lval = val;
		label.push_back(v);
	}
}

edge_iterator node::getIncomingEdgeIter() const
{
	return _edgesIncoming.begin();
}

edge *node::edgeIterNextIncoming(edge_iterator &iterIncoming) const
{
	if (iterIncoming != _edgesIncoming.end())
	{
		edge *v = *iterIncoming;
		iterIncoming++;
		return v;
	}
	return 0;
}

edge_iterator node::getOutgoingEdgeIter() const
{
	return _edgesOutgoing.begin();
}

edge *node::edgeIterNextOutgoing(edge_iterator &iterOutgoing) const
{
	if (iterOutgoing != _edgesOutgoing.end())
	{
		edge *v = *iterOutgoing;
		iterOutgoing++;
		return v;
	}
	return 0;
}

edge_iterator node::getEdgeIter() const
{
	return _allEdges.begin();
}

edge *node::edgeIterNext(edge_iterator &iter) const
{
	if (iter != _allEdges.end())
	{
		edge *v = *iter;
		iter++;
		return v;
	}
	return 0;
}

edge *node::getRandomIncomingEdge()
{
	int rand_val = random()%_edgesIncoming.size();
	return _edgesIncoming[rand_val];
}

edge *node::getRandomOutgoingEdge()
{
	int rand_val = random()%_edgesOutgoing.size();
	return _edgesOutgoing[rand_val];
}

edge *node::GetRandomEdge()
{
	int rand_val = random()%_allEdges.size();
	return _allEdges[rand_val];
}

// fixme: should be inlined
int node::getNumOutgoingEdges()
{
	return _edgesOutgoing.size();
}

int node::getNumIncomingEdges()
{
	return _edgesIncoming.size();
}

edge *node::getEdge(unsigned int which)
{
	if (which > _allEdges.size())
		return 0;
	return _allEdges[which];
}


neighbor_iterator node::getNeighborIter() const
{
	return 0;
}

int node::nodeNeighborNext(neighbor_iterator& ni) const
{
	int result = -1;
	unsigned int os = _edgesOutgoing.size();
	if (ni < os)
		result = _edgesOutgoing[ni]->getTo();
	else if (ni - os < _edgesIncoming.size())
		result = _edgesIncoming[ni-os]->getFrom();
	ni++;
	return result;
}


void node::Print(ostream& out) const
{
	out << "\"" << name << "\"" << " (" << nodeNum << ")";
	for (unsigned int x = 0; x < label.size(); x++)
	{
		out << " - " << label[x].lval;
	}
}

ostream& operator <<(ostream & out, const Graph &_Graph)
{
	_Graph.Print(out);
	return out;
}

ostream& operator <<(ostream & out, const node &_Node)
{
	_Node.Print(out);
	return out;
}

ostream& operator <<(ostream & out, const edge &_Edge)
{
	_Edge.Print(out);
	return out;
}
