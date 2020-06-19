#pragma once

#include <Eigen/Dense>
#include <string>
#include <atomic>
#include <fmt/ostream.h>

namespace fem {
	class Node {
	public:
		Node() : Node{0,0,0,0}{}
		Node(const Node& other) : Node{ other.id, other.position } {};
		Node(int id, double x, double y, double z) : Node{ id, {x,y,z} } {}
		Node(int id, Eigen::Vector3d position);
		~Node();

		Eigen::Vector3d getPosition();
		// return the node id
		int getId() const;
		// return the number of DOF of this node
		int dofCount();
		// return the current value of a dof (either a position or a rotation)
		// expressed in global coordinates
		double getVal(int localCoord);

		bool operator ==(const Node& that) const;
		Node& operator=(const Node& that) = default;

	private:
		int id;
		Eigen::Vector3d position;

		//template<typename ostream>
		using ostream = std::basic_ostream<char, std::char_traits<char>>;
		friend ostream& operator<<(ostream& os, const Node& n)
		{
			return os << fmt::format("Node[id={},pos=[{},{},{}]]", n.id, n.position.x(), n.position.y(), n.position.z());
		}
	};
}
