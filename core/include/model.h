#pragma once

#include "node.h"
#include "beam.h"
#include "nodeforce.h"
#include "beamdistload.h"
#include <unordered_map>
#include <unordered_set>
#include <utility>

namespace fem {
	class Model {
	public:
		Model() : Model{ 0 } {}
		Model(int id_) : id{ id_ } {}
		template<typename First, typename... Others>
		void add(First elem, Others... list) {
			add(elem);
			add(list...);
		}

		void add(Node n);
		void add(Beam b);
		void add(NodeForce f);
		void add(BeamDistLoad beamDistLoad);

		void solve();

		// debug/check
		void listAll();
	private:
		int id;
		std::vector<Node> nodes;
		std::vector<Beam> beams;
		std::vector<NodeForce> nodeForces;
		std::vector<BeamDistLoad> beamDistLoads;

		std::unordered_set<int> nodeIds;
		std::unordered_set<int> beamIds;
		// map global dofs to each node, and the index in that node
		// but dofs ids in the node?
		// std::unordered_map<int, std::pair<int, int>> dofs;
		//int globalDofCount = 0;

		template<typename ostream>
		friend ostream& operator<<(ostream& os, const fem::Model& m)
		{
			return os << fmt::format("Model[id={}]", m.id);
		}

	};

	/*
	coords for a node (local system) have indexes 0..n
	coords in the global system have different indexes
	map a local coord into a global coord
	*/
	struct CoordMapping {
		int globalId;
		int nodeId;
		int localId;
		template<typename ostream>
		friend ostream& operator<<(ostream& os, const fem::CoordMapping& m)
		{
			return os << fmt::format("CoordMapping[globalId={},nodeId={},localId={}]",
				m.globalId, m.nodeId, m.localId );
		}
	};

}
