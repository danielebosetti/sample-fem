#include "beamdistload.h"

// #include <fmt/format.h>
#include <iostream>
#include "spdlog/spdlog.h"

using spdlog::info;
using spdlog::warn;

namespace fem {

	BeamDistLoad::BeamDistLoad(int id_, int beamId_, double x_, double y_, double z_) :
		id{ id_ }, beamId{ beamId_ }, direction{x_, y_, z_} {}

}