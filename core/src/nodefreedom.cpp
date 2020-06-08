#include "nodefreedom.h"

#include <iostream>

namespace fem {

	int NodeFreedom::getId() const {
		return id;
	}
	int NodeFreedom::getNodeId() const {
		return nodeId;
	}
	NodeFreedom::NodeFreedom(int id_, int nodeId_, std::vector<int> constraints_) :
		id{ id_ }, nodeId{ nodeId_ }, constraints{ constraints_}
	{}

}

