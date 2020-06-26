#pragma once

#include <Eigen/Dense>
#include "node.h"
#include <fmt/ostream.h>

namespace fem {
	/* 
	the local system of reference is composed of 3 versors
	*/

	class Beam {
	public:
		Beam(int id_, fem::Node node1, fem::Node node2);
		Eigen::MatrixXd getLocalStiffness();
		Eigen::Matrix3d getLocalSOR();
		int getId() const;
		fem::Node& getNode1() { return node1; }
		fem::Node& getNode2() { return node2; }

		/*
		build the local stiffness matrix
		convert the local stiffness matrix to global coordinates (rotate only, don't re-index)
		then, re-index the coordinates to global indexes
		*/
		Eigen::MatrixXd calcStiffnessMatrix();

		template<typename ostream>
		friend ostream& operator<<(ostream& os, const fem::Beam& b)
		{
			return os << fmt::format("Beam[id={},node1={},node2={}]", b.id, b.node1.getId(), b.node2.getId());
		}

		/*
		init length using the nod pos
		*/
		void init() {
			L = (node2.getPosition() - node1.getPosition()).norm();
		}

		void setE(double E_) { E = E_; }
		void setA(double A_) { A = A_; }
		void setG(double G_) { G = G_; }
		void setIx(double Ix_) { Ix = Ix_; }
		void setIy(double Iy_) { Iy = Iy_; }
		void setIz(double Iz_) { Iz = Iz_; }

	private:
		int id;
		double L, E, A, G, Ix, Iy, Iz;
		fem::Node node1, node2;
	};
}
