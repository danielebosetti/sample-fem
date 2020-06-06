#pragma once

#include <Eigen/Dense>
#include <string>
#include <atomic>
#include <fmt/ostream.h>

namespace fem {
	class Node {
		std::atomic<int> counter{ 0 };
	public:
		Node() : Node{ 0, 0, 0, 0 } {};
		Node(const Node& other) : Node{ other.id, other.x, other.y, other.z } {};
		Node(int id, double x, double y, double z);
		~Node();

		Eigen::Vector3d getPosition();
		// return the node id
		int getId() const;
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
