#pragma once

#include <Eigen/Dense>
#include <string>
#include <vector>
#include <fmt/ostream.h>

namespace fem {

	/*
	node-freedom, set the constrained (global) coordinates for node-i
	*/
	class NodeFreedom {
	public:
		/*
		local coords indexed by [constraints] are constrained
		*/
		NodeFreedom(int id, int nodeId, std::vector<int> constraints);
		int getId() const;
		int getNodeId() const;
		const std::vector<int> getConstraints() const { return constraints; }
	private:
	    int id, nodeId;
		std::vector<int> constraints;
			
		template<typename ostream>
		friend ostream& operator<<(ostream& os, const NodeFreedom& n)
		{
			return os << fmt::format("NodeFreedom[id={},nodeId={}", id, nodeId );
		}
	};
	class NodeFreedomBuilder {
	public:
		NodeFreedomBuilder() {}
		NodeFreedomBuilder& setId(int id_) {
			id = id_;
		}
		NodeFreedomBuilder& setNodeId(int nodeId_) {
			nodeId = nodeId_;
		}
		NodeFreedomBuilder& addConstraintDimension(int coordId) {
			coordDimensions.push_back(coordId);
		}
		NodeFreedom build() {
			return NodeFreedom(id, nodeId, coordDimensions);
		}
	private:
		int id, nodeId;
		std::vector<int> coordDimensions;
	};
}
