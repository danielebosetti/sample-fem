#include "beam.h"

#include <iostream>
#include <Eigen/Geometry>
#include <fmt/format.h>

using fem::Node;
using Eigen::Vector3d;
using Eigen::Matrix3d;
using Eigen::MatrixXd;

namespace fem {

	Beam::Beam(int id_, Node node1_, Node node2_) : id{ id_ }, node1{ node1_ }, node2{ node2_ } {

	}

	MatrixXd Beam::getLocalStiffness() {
		MatrixXd res(12,12);
		res.setZero();
		// see souma pag.55/613
		double eal = E * A / L;
		double eizl3 = E * Iz / (L * L * L);
		double eiyl3 = E * Iy / (L * L * L);
		double eiyl2 = E * Iy / (L * L);
		double eizl2 = E * Iz / (L * L);
		double gixl = G * Ix / L;
		double eizl = E * Iz / L;

		res(0, 0) = eal;
		res(0, 6) = -eal;
		res(6, 0) = -eal;
		res(6, 6) = eal;

		res(1, 1) = 12.* eizl3;
		res(1, 7) = -12. * eizl3;
		res(7, 1) = -12. * eizl3;
		res(7, 7) = 12. * eizl3;

		res(1, 5) = 6. * eizl2;
		res(1, 11) = 6. * eizl2;
		res(7, 5) = -6. * eizl2;
		res(7, 11) = -6. * eizl2;

		res(2, 2) = 12. * eiyl3;
		res(2, 8) = -12. * eiyl3;
		res(8, 2) = -12. * eiyl3;
		res(8, 8) = 12. * eiyl3;

		res(2, 4) = -6. * eiyl2;
		res(2, 10) = -6. * eiyl2;
		res(8, 4) = 6. * eiyl2;
		res(8, 10) = 6. * eiyl2;

		res(3, 3) = gixl;
		res(3, 9) = -gixl;
		res(9, 3) = -gixl;
		res(9, 9) = gixl;

		res(4, 2) = -eiyl2;
		res(4, 8) = eiyl2;
		res(10, 2) = -eiyl2;
		res(10, 8) = eiyl2;

		res(5, 1) = eizl2;
		res(5, 7) = -eizl2;
		res(11, 1) = eizl2;
		res(11, 7) = -eizl2;

		res(5, 5) = 4. * eizl;
		res(5, 11) = 2. * eizl;
		res(11, 5) = 2. * eizl;
		res(11, 11) = 4. * eizl;

		return res;
	}

	/*
	return a 3*3 matrix, 
	where each column are the coordinates of the local versors in the global coordinate system
	Eg. versor u_{1} points from node1 to node2.
	u_{1}=(1,0,0) in the local CS, but coordinates are different when expressed in terms of e_{1}, e_{2}, e_{3}
	*/
	Eigen::Matrix3d Beam::getLocalSOR() {
		Eigen::Matrix3d res;
		Vector3d e1 = node2.getPosition() - node1.getPosition();
		// TODO handle the case when the norm is zero
		e1.normalize();

		Vector3d next_versor;
		if ((e1.dot(Vector3d::UnitZ()) > 0.9)) {
			next_versor = Vector3d::UnitX();
		}
		else {
			next_versor = Vector3d::UnitZ();
		}
		Vector3d e3 = next_versor - e1 * (e1.dot(next_versor));
		e3.normalize();
		Vector3d e2 = -e1.cross(e3);
		res << e1, e2, e3;
		return res;
	}

	MatrixXd Beam::calcStiffnessMatrix() {
		// calc k, in local coord indexes, but in global SOR
		MatrixXd k = getLocalStiffness();
		Matrix3d sor = getLocalSOR();
		MatrixXd rotation(6, 6);
		MatrixXd zero(6, 6);
		zero.setZero();
		MatrixXd rotation2(12, 12);

		rotation << sor.transpose(), Matrix3d::Zero(), Matrix3d::Zero(), sor.transpose();
		rotation2 << rotation, zero, zero, rotation;

		return rotation2.transpose() * k * rotation2;
	}


	int Beam::getId() const
	{
		return id;
	}

}
