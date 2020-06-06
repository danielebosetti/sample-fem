#include "node.h"

#include <iostream>

namespace fem {

	Node::Node(int id_, double x_, double y_, double z_) : id{ id_ }, x{ x_ }, y{ y_ }, z{ z_ } {
		counter++;
	}
	Node::~Node() {
		counter;
	}

	Eigen::Vector3d Node::getPosition() {
		return Eigen::Vector3d{x, y, z};
	}

	int Node::getId() const {
		return id;
	}
	int Node::dofCount() {
		return 3;
	}
}

