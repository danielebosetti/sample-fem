#pragma once

#include "beam.h"
#include "beamdistload.h"
#include <unordered_map>
#include <utility>

namespace fem {
	class Model {
	public:
		void add(Node n);
		void add(BeamDistLoad beamDistLoad);
		void listNodes();
		void listBeamDistLoads();
		void listDofs();
	private:
		std::unordered_map<int, Node> nodes;
		std::vector<BeamDistLoad> beamDistLoads;
		// map global dofs to each node, and the index in that node
		// but dofs ids in the node?
		std::unordered_map<int, std::pair<int, int>> dofs;
		int globalDofCount = 0;
	};
}
