#include "node.h"

#include <iostream>

namespace fem {

	Node::Node(int id_, Eigen::VectorXd position_) :
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
		Eigen::Vector3d pos;
		pos << position.x(), position.y(), position.z();
		return pos;
	}

	int Node::getId() const {
		return id;
	}
	int Node::dofCount() {
		return 6;
	}
	double Node::getVal(int localCoord) {
		return position(localCoord);
	}
}

