#include "dof.h"

#include <iostream>

namespace fem {

	DOF::DOF(int local_id_, int global_id_) : local_id{local_id_}, global_id{global_id_} {
	}

	void DOF::print() {
		std::cout << "DOF[" << global_id << ","<<global_id<<"]";
	}
}