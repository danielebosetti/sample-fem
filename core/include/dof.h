#pragma once

#include <Eigen/Dense>


namespace fem {
  /*
  * degree of freedom
  */
	class DOF {
	public:
		DOF(int local_id, int global_id);
		void print();
	private:
    int local_id, global_id;
	};
}
