#include "nodeforce.h"

#include <fmt/format.h>
#include <iostream>

namespace fem {
	NodeForce::NodeForce(int id_, int nodeId_, double x_, double y_, double z_) :
		id{ id_ }, nodeId{ nodeId_ }, x{ x_ }, y{ y_ }, z{z_},
		position{x_,y_,z_ }
	{
	}
	int NodeForce::getId()
	{
		return id;
	}
}

