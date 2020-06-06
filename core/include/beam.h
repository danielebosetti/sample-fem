#pragma once

#include <Eigen/Dense>
#include "node.h"
#include <fmt/ostream.h>

namespace fem {
	// node has 6 degree-of-freedom
	using NodeDofMatrix = Eigen::Matrix<double, 6, 6>;
	/* 
	the local system of reference is composed of 3 versors
	*/
	using LocalSOR = Eigen::Matrix<double, 3, 3>;

	class Beam {
	public:
		Beam(int id_, fem::Node node1, fem::Node node2);
		NodeDofMatrix getLocalStiffness();
		LocalSOR getLocalSOR();
		int getId() const;
		fem::Node getNode1() { return node1; }
		fem::Node getNode2() { return node2; }

		template<typename ostream>
		friend ostream& operator<<(ostream& os, const fem::Beam& b)
		{
			return os << fmt::format("Beam[id={},node1={},node2={}]", b.id, b.node1.getId(), b.node2.getId());
		}

	private:
		int id;
		fem::Node node1, node2;
	};
}
