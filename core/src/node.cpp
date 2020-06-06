#include "node.h"

#include <fmt/format.h>
#include <iostream>

namespace fem {

	Node::Node(int id_, double x_, double y_, double z_) : id{ id_ }, x{ x_ }, y{ y_ }, z{ z_ } {
	}

	Eigen::Vector3d Node::getPosition() {
		return Eigen::Vector3d{x, y, z};
	}

	int Node::getId() {
		return id;
	}
	int Node::dofCount() {
		return 3;
	}
}

