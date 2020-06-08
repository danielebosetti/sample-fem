#pragma once

#include <Eigen/Dense>
#include <string>
#include <vector>
#include <fmt/ostream.h>

namespace fem {
	// TODO find a better name?
	//DX, DY, DZ, RX, RY, RZ
	//class ConstraintDimension {
	//public:
	//	ConstraintDimension(int localCoordId_) : localCoordId{ localCoordId_} {}
	//	int getLocalCoorId() const {
	//		return localCoordId;
	//	}
	//private:
	//	int localCoordId;
	//};

	class NodeFreedom {
	public:
		NodeFreedom(int id, int nodeId, std::vector<int> constraints);
		// return the node id
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
