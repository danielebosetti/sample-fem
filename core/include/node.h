#pragma once

#include <Eigen/Dense>
#include <string>
#include <fmt/ostream.h>

namespace fem {
	class Node {
	public:
		Node() : id{ 0 }, x{ 0 }, y{ 0 }, z{ 0 } {};
		Node(const Node& other) = default;
		Node(int id, double x, double y, double z);
		Eigen::Vector3d getPosition();
		// return the node id
		int getId();
		// return the number of DOF of this node
		int dofCount();

	private:
		int id;
		double x, y, z;
		Eigen::Vector3d position;


		template<typename ostream>
		friend ostream& operator<<(ostream& os, const Node& n)
		{
			return os << fmt::format("Node[id={},pos=[{},{},{}]]", n.id, n.x, n.y, n.z);
		}
	};
}
