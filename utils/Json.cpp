#include <memory>
#include <cassert>
#include "Json.h"


///////////////// NodeValue /////////////////////

// Constructors
Json::NodeValue::NodeValue()
	: tag(NIL), value(std::unique_ptr<bool>(new bool(false)))
{ //value.b.reset(new bool);
	//*(value.b) = false;
	std::cout << "Constructed\n"; }

Json::NodeValue::NodeValue(bool boolVal)
	: tag(BOOL), value(std::unique_ptr<bool>(new bool(boolVal)))
{ }

Json::NodeValue::NodeValue(int intVal)
	: tag(INT), value(std::unique_ptr<int>(new int(intVal)))
{ }

Json::NodeValue::NodeValue(double doubleVal)
	: tag(DOUBLE), value(std::unique_ptr<double>(new double(doubleVal)))
{ }

Json::NodeValue::NodeValue(std::string stringVal)
	: tag(STRING), value(std::unique_ptr<std::string>(new std::string(stringVal)))
{ }

Json::NodeValue::NodeValue(const char* stringVal)
	: tag(STRING), value(std::unique_ptr<std::string>(new std::string(stringVal)))
{ }

Json::NodeValue::NodeValue(std::shared_ptr<Json::Node> nodeVal)
	: tag(NODE), value(nodeVal)
{ }

Json::NodeValue::NodeValue(const Json::NodeValue &nodeVal) {
	switch (nodeVal.tag) {
	case NIL:
		NodeValue();
		break;

	case BOOL:
		NodeValue(*(nodeVal.value.b));
		break;

	case INT:
		NodeValue(*(nodeVal.value.i));
		break;

	case DOUBLE:
		NodeValue(*(nodeVal.value.d));
		break;

	case STRING:
		NodeValue(*(nodeVal.value.s));
		break;

	case VECTOR:
		// TODO
		//NodeValue(*(nodeVal.value.v));
		break;

	default:
		assert(0);
	}
}

Json::NodeValue::NodeValue(std::vector<std::unique_ptr<Json::NodeValue>> vectorValue)
	: tag(VECTOR), value(std::unique_ptr<std::vector<std::unique_ptr<Json::NodeValue>>>(new std::vector<std::unique_ptr<Json::NodeValue>>()))
{
	for (auto &elem : vectorValue) {
		value.v->push_back(std::unique_ptr<Json::NodeValue>(new NodeValue(*elem)));
	}
}

Json::NodeValue::NodeValue(std::unique_ptr<std::vector<std::unique_ptr<Json::NodeValue>>> vectorValue)
	: tag(VECTOR), value(std::move(vectorValue))
{ }

Json::NodeValue::~NodeValue()	 { value.destruct(tag); }

// Format the value to a stream
std::ostream &operator<<(std::ostream &os, Json::NodeValue const &nodeValue)
{
	bool list = false;
	switch (nodeValue.tag)
		{
		case Json::NodeValue::NIL:
			os << "null";
			break;

		case Json::NodeValue::BOOL:
			os << (*(nodeValue.value.b) ? "true" : "false");
			break;

		case Json::NodeValue::INT:
			os << *(nodeValue.value.i);
			break;

		case Json::NodeValue::DOUBLE:
			os << *(nodeValue.value.d);
			break;

		case Json::NodeValue::STRING:
			os << "\"" << *(nodeValue.value.s) << "\"";
			break;

		case Json::NodeValue::NODE:
			if (nodeValue.value.n->value->tag != Json::NodeValue::NODE && nodeValue.value.n->value->tag != Json::NodeValue::VECTOR) {
				os << *(nodeValue.value.n);
			} else {
				os << "{\n" << *(nodeValue.value.n) << "\n}";
			}
			break;

		case Json::NodeValue::VECTOR:
			for (const auto &elem : *(nodeValue.value.v)) {
				if (elem->tag != Json::NodeValue::NODE) {
					list = true;
					break;
				}
			}

			if (list) {
				os << "[\n";
			} else {
				os << "{\n";
			}
			for (const auto &elem : *(nodeValue.value.v))
				{
					if (elem != *(nodeValue.value.v->begin())) {
						os << ", \n";
					}
					os << *elem;
				}
			if (list) {
				os << "\n]";
			} else {
				os << "\n}";
			}
			break;

		default:
			assert(0);
		}

	return os;
}


///////////////// Node /////////////////////
// Constructors
Json::Node::Node(std::string name)
	: name(name), value(std::unique_ptr<NodeValue>(new NodeValue(false)))	 { }

Json::Node::Node(std::string name, std::unique_ptr<NodeValue> val)
	: name(name), value(std::move(val))	 { }

Json::Node::~Node() { }

// Used to get the "this" smart pointer
std::shared_ptr<Json::Node> Json::Node::getPtr()
{
	return shared_from_this();
}

// Add a default child node to the current node
std::shared_ptr<Json::Node> Json::Node::addChild(std::string name) {
	// If there is no value yet overwrite child
	if (value->tag == Json::NodeValue::NIL)
		{
			std::unique_ptr<Json::NodeValue> newValue(new Json::NodeValue());
			std::shared_ptr<Json::Node> newNode(new Json::Node(name, std::move(newValue)));
			std::unique_ptr<Json::NodeValue> nodeValue(new Json::NodeValue(newNode));

			value = std::move(nodeValue);

			return newNode;
		}
	// Else if it is a vector append new child
	else if (value->tag == Json::NodeValue::VECTOR)
		{
			std::unique_ptr<Json::NodeValue> newValue(new Json::NodeValue());
			std::shared_ptr<Json::Node> newNode(new Json::Node(name, std::move(newValue)));
			std::unique_ptr<Json::NodeValue> nodeValue(new Json::NodeValue(newNode));
			
			value->value.v->push_back(std::move(nodeValue));

			return newNode;
		}
	// Otherwise make a new vector and add both the old child and the new child to it
	else
		{
			std::unique_ptr<Json::NodeValue> newValue(new Json::NodeValue());
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
}

// Format a json node to a stream
std::ostream &operator<<(std::ostream &os, Json::Node const &node)
{
	os << "\"" << node.name << "\" : ";
	os << *(node.value);
	return os;
}


///////////////// Json /////////////////////
Json::Json()
	: root(std::unique_ptr<std::vector<std::shared_ptr<Node>>>(new std::vector<std::shared_ptr<Node>>))	 { }

Json::~Json()	 { }

// Adds a default child node to the json root
std::shared_ptr<Json::Node> Json::addChild(std::string name)
{
	std::unique_ptr<Json::NodeValue> value(new Json::NodeValue());
	std::shared_ptr<Json::Node> node(new Json::Node(name, std::move(value)));

	root->push_back(node);

	return node;
}

// Format the json tree to a stream
std::ostream &operator<<(std::ostream &os, Json const &json)
{
	os << "{\n";
	for (const auto &elem : *(json.root))
		{
			if (elem != *(json.root->begin())) {
				os << ",\n";
			}
			os << *elem; 
		}
	os << "\n}";

	return os;
}

