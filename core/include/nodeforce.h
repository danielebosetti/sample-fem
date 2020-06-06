#pragma once

#include <Eigen/Dense>
#include <string>
#include <atomic>
#include <fmt/ostream.h>

namespace fem {
	class NodeForce {
	public:
		NodeForce(int id, int nodeId, double x, double y, double z);
		int getId();

	private:
		int id;
		int nodeId;
		double x, y, z;
		Eigen::Vector3d position;

		template<typename ostream>
		friend ostream& operator<<(ostream& os, const NodeForce& f)
		{
			return os << fmt::format("NodeForce[id={},nodeId={},force=[{},{},{}]]", f.id, f.nodeId, f.x, f.y, f.z);
		}
	};
}
