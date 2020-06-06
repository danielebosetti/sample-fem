#include "beam.h"

#include <iostream>
#include <Eigen/Geometry>
#include <fmt/format.h>

using fem::Node;
using Eigen::Vector3d;

namespace fem {

	Beam::Beam(int id_, Node node1_, Node node2_) : id{ id_ }, node1{ node1_ }, node2{ node2_ } {

	}

	NodeDofMatrix Beam::getLocalStiffness() {
		NodeDofMatrix res;
		res << NodeDofMatrix::Zero();
		res(0, 0) = 1;
		res(3, 0) = -1;
		res(0, 3) = -1;
		res(3, 3) = 1;
		return res;
	}

	LocalSOR Beam::getLocalSOR() {
		LocalSOR res;
		Vector3d e1 = node2.getPosition() - node1.getPosition();
		e1.normalize();

		Vector3d z_versor = Vector3d::UnitZ();
		Vector3d e3 = z_versor - e1 * (e1.dot(z_versor));
		e3.normalize();
		Vector3d e2 = -e1.cross(e3);
		res << e1, e2, e3;
		return res;
	}

	int Beam::getId() const
	{
		return id;
	}

}
