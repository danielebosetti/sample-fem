#pragma once

#include "coords.h"
#include "node.h"
#include "beam.h"
#include "nodeforce.h"
#include "beamdistload.h"
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <Eigen/Dense>

namespace fem {
	class Model {
	public:
		Model() : Model{ 0 } {}
		Model(int modelId_) : modelId{ modelId_ } {}
		template<typename First, typename... Others>
		void add(First elem, Others... list) {
			add(elem);
			add(list...);
		}

		void add(Node n);
		void add(Beam b);
		void add(NodeForce f);
		void add(BeamDistLoad beamDistLoad);

		bool hasNode(int nodeId);
		void validate();
		void initCoords();
		void solve();

		/* 
		get all actions, expressed in global coords
		actions can be forces or moments
		*/
		std::unique_ptr<Eigen::VectorXd> getGlobalActions();

		// debug/check
		void listAll();

		Node getNode(int id);
		// retrieve the coord mapping for the node
		//std::vector<CoordMapping> getCoords(int nodeId);

	private:
		int modelId;
		std::vector<Node> nodes;
		std::unordered_map<int, Node> nodesMap;
		std::vector<Beam> beams;
		std::vector<NodeForce> nodeForces;
		std::vector<BeamDistLoad> beamDistLoads;

		std::unordered_set<int> nodeIds;
		std::unordered_set<int> beamIds;
		// map global dofs to each node, and the index in that node
		// but dofs ids in the node?
		// std::unordered_map<int, std::pair<int, int>> dofs;
		//int globalDofCount = 0;

		std::unique_ptr<CoordsHolder> ch;

		template<typename ostream>
		friend ostream& operator<<(ostream& os, const fem::Model& m)
		{
			return os << fmt::format("Model[id={}]", m.id);
		}

	};

}
