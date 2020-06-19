#include "node.h"

#include <iostream>

namespace fem {

	Node::Node(int id_, Eigen::Vector3d position_) :
		id{ id_ }, position{ position_ } {
	}
	Node::~Node() {
	}

	bool Node::operator ==(const Node& that) const {
		return
			(id == that.id) &&
			(position == that.position);
	}

	Eigen::Vector3d Node::getPosition() {
		return position;
	}

	int Node::getId() const {
		return id;
	}
	int Node::dofCount() {
		return 3;
	}
	double Node::getVal(int localCoord) {
		return position(localCoord);
	}
}

