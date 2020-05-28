#include "node.h"

#include <iostream>

namespace fem {

	Node::Node() {

	}

	void Node::print() {
		std::cout << "Node[" << id << "]";
	}
}