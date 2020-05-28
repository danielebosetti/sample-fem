#pragma once

#include <Eigen/Dense>


namespace fem {
	class Node {
	public:
		Node();
		void print();
	private:
		int id;
		Eigen::Vector3d position;
	};
}
