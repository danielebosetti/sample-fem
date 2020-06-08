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
		MatrixXd res(6,6);
		res.setZero();
		res(0, 0) = 1;
		res(3, 0) = -1;
		res(0, 3) = -1;
		res(3, 3) = 1;
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
		rotation << sor.transpose(), Matrix3d::Zero(), Matrix3d::Zero(), sor.transpose();
		return rotation.transpose() * k * rotation;
	}


	int Beam::getId() const
	{
		return id;
	}

}
