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
		Node(int id, double x, double y, double z) : Node{ id, x, y, z, 0, 0, 0 } {}
		Node(int id_, double x, double y, double z, double rx, double ry, double rz) : id{ id_ } {
			position.resize(6);
			position << x, y, z, rx, ry, rz;
		}
		Node(int id, Eigen::VectorXd position);
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
		// tx, ty, tz, rx, ry, rz in global coords
		Eigen::VectorXd position;

		//template<typename ostream>
		using ostream = std::basic_ostream<char, std::char_traits<char>>;
		friend ostream& operator<<(ostream& os, const Node& n)
		{
			return os << fmt::format("Node[id={},pos=[{},{},{}]]", n.id, n.position(0), n.position(1), n.position(2));
		}
	};
}
