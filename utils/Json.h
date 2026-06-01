#pragma once

#include <iostream>
#include <vector>
#include <memory>
#include <cstring>
#include <cassert>


/// Simple class that alows for building a tree of data that can be serialized into Json format
class Json
{
 public:
	class Node;

 private:

	// Union type that abstracts node types into one object
	class NodeValue
	{
	public:
		NodeValue();
		NodeValue(bool);
		NodeValue(int);
		NodeValue(double);
		NodeValue(std::string);
		NodeValue(const char*);
		NodeValue(const NodeValue&);
		NodeValue(std::shared_ptr<Json::Node>);
		NodeValue(std::vector<std::unique_ptr<NodeValue>>);
		NodeValue(std::unique_ptr<std::vector<std::unique_ptr<NodeValue>>>);
		~NodeValue();

		enum{NIL=1, BOOL=2, INT=3, DOUBLE=4, STRING=5, NODE=6, VECTOR=7} tag;
		union Values
		{
			Values() { memset(this, 0, sizeof(*this)); }
		Values(std::unique_ptr<bool> x) : b(std::move(x)) {}
		Values(std::unique_ptr<int> x) : i(std::move(x)) {}
		Values(std::unique_ptr<double> x) : d(std::move(x)) {}
		Values(std::unique_ptr<std::string> x) : s(std::move(x)) {}
		Values(std::shared_ptr<Json::Node> x) : n(std::move(x)) {}
		Values(std::unique_ptr<std::vector<std::unique_ptr<NodeValue>>> x) : v(std::move(x)) {}
			~Values() { }
			void destruct(int tag) {
				switch (tag) {
				case NIL:
					b.~unique_ptr();
					break;

				case BOOL:
					b.~unique_ptr();
					break;

				case INT:
					i.~unique_ptr();
					break;

				case DOUBLE:
					d.~unique_ptr();
					break;

				case STRING:
					s.~unique_ptr();
					break;

				case NODE:
					n.~shared_ptr();
					break;

				case VECTOR:
					v.~unique_ptr();
					break;

				default:
					std::cout << "UNKONW TAG TYPE\n";
					exit(0);
				}

				this->~Values();
			}
			std::unique_ptr<bool> b;
			std::unique_ptr<int> i;
			std::unique_ptr<double> d;
			std::unique_ptr<std::string> s;
			std::shared_ptr<Node> n;
			std::unique_ptr<std::vector<std::unique_ptr<NodeValue>>> v;
		} value;
	};

	std::unique_ptr<std::vector<std::shared_ptr<Node>>> root;

 public:
	Json();
	~Json();

	std::shared_ptr<Node> addChild(std::string);
	template <typename T>
		std::shared_ptr<Node> addChild(std::string, T);


	// Node object for Json format
	class Node : public std::enable_shared_from_this<Node>
	{
	public:
		Node(std::string);
		Node(std::string, std::unique_ptr<NodeValue>);
		~Node();

		std::shared_ptr<Node> getPtr();

		std::shared_ptr<Node> addChild(std::string);
		template <typename T>
			std::shared_ptr<Node> addChild(std::string, T);
		template<typename T>
			void addData(T);

	private:
		std::string name;
		std::unique_ptr<Json::NodeValue> value;

		friend std::ostream &operator<<(std::ostream&, Node const&);
		friend std::ostream &operator<<(std::ostream&, Json::NodeValue const&);
		friend std::ostream &operator<<(std::ostream&, Json const&);
	};


	friend std::ostream &operator<<(std::ostream&, Json const&);
	friend std::ostream &operator<<(std::ostream&, NodeValue const&);
};


/////////////////////// Template Functions ///////////////////////////

// Adds a new child to the Json root
template <typename T>
std::shared_ptr<Json::Node> Json::addChild(std::string name, T val)
{
	std::unique_ptr<Json::NodeValue> value(new Json::NodeValue(val));
	//std::shared_ptr<Json::Node> node(new Json::Node(name, std::move(value)));
	std::shared_ptr<Json::Node> node = std::make_shared<Json::Node>(name, std::move(value));

	root->push_back(node);

	return node;
}

// Adds a child node to the current node
template <typename T>
std::shared_ptr<Json::Node> Json::Node::addChild(std::string name, T val) {
	// If there is no value yet overwrite child
	if (value->tag == Json::NodeValue::NIL)
		{
			std::unique_ptr<Json::NodeValue> newValue(new Json::NodeValue(val));
			std::shared_ptr<Json::Node> newNode(new Json::Node(name, std::move(newValue)));
			std::unique_ptr<Json::NodeValue> nodeValue(new Json::NodeValue(newNode));

			value = std::move(nodeValue);

			return newNode;
		}
	// Else if it is a vector append new child
	else if (value->tag == Json::NodeValue::VECTOR)
		{
			std::unique_ptr<Json::NodeValue> newValue(new Json::NodeValue(val));
			std::shared_ptr<Json::Node> newNode(new Json::Node(name, std::move(newValue)));
			std::unique_ptr<Json::NodeValue> nodeValue(new Json::NodeValue(newNode));
			
			value->value.v->push_back(std::move(nodeValue));

			return newNode;
		}
	// Otherwise make a new vector and add both the old child and the new child to it
	else
		{
			std::unique_ptr<Json::NodeValue> newValue(new Json::NodeValue(val));
			std::shared_ptr<Json::Node> newNode(new Json::Node(name, std::move(newValue)));
			std::unique_ptr<Json::NodeValue> nodeValue(new Json::NodeValue(newNode));
			std::unique_ptr<std::vector<std::unique_ptr<Json::NodeValue>>> vec(new std::vector<std::unique_ptr<Json::NodeValue>>());
			vec->push_back(std::move(value));
			vec->push_back(std::move(nodeValue));
			value->value.~Values();
			std::unique_ptr<Json::NodeValue> joinedValue(new Json::NodeValue(std::move(vec)));
			value = std::move(joinedValue);

			return newNode;
		}

	std::unique_ptr<Json::NodeValue> value(new Json::NodeValue(val));
	std::shared_ptr<Json::Node> node(new Json::Node(name, std::move(value)));

	return node;
}

// Adds or overwrites the data of a node. Note: should probably remove this method
template <typename T>
void Json::Node::addData(T data) {
	// If there is no value yet overwrite child
	if (value->tag == Json::NodeValue::NIL)
		{
			std::unique_ptr<Json::NodeValue> newValue(new Json::NodeValue(data));
			value = std::move(newValue);
		}
	// Else if it is a vector append new child
	else if (value->tag == Json::NodeValue::VECTOR)
		{
			// TODO
		}
	// Otherwise make a new vector and add both the old child and the new child to it
	else
		{
			std::unique_ptr<Json::NodeValue> newValue(new Json::NodeValue(data));
			std::unique_ptr<std::vector<std::unique_ptr<Json::NodeValue>>> vec(new std::vector<std::unique_ptr<Json::NodeValue>>());
			vec->push_back(std::move(value));
			vec->push_back(std::move(newValue));
			value->value.~Values();
			std::unique_ptr<Json::NodeValue> joinedValue(new Json::NodeValue(std::move(vec)));
			value = std::move(joinedValue);
		}
}
