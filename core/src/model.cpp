#include "model.h"

#include <iostream>
#include "spdlog/spdlog.h"

using spdlog::info;
using spdlog::warn;

namespace fem {

	template<typename key> 
	void addAndCheckDuplicate(std::unordered_set<key>& ids, key id, std::string type) {
		if (ids.find(id) != ids.end()) {
			warn("found duplicate {} id={}", type, nodeId);
			throw std::exception(fmt::format("duplicate {} id={}", type, id));
		}
	}

	void Model::add(Node n) {
		addAndCheckDuplicate(nodeIds, n.getId(), "node");
		nodes.push_back(n);
	}
	void Model::add(Beam b) {
		addAndCheckDuplicate(beamIds, b.getId(), "beam");
		beams.push_back(b);
	}
	void Model::add(NodeForce f) {
		nodeForces.push_back(f);
	}

	template<class T>
	void listAll(std::string caption, std::vector<T> list) {
		info(caption);
		for (auto& item : list) {
			info("{}", item);
		}
	}

	void Model::listAll()
	{
		info("model={}", *this);
		fem::listAll("Nodes:", nodes);
		fem::listAll("Beams:", beams);
		fem::listAll("NodeForces:", nodeForces);
		fem::listAll("BeamDistLoads:", beamDistLoads);
	}

	void Model::add(BeamDistLoad beamDistLoad) {
		beamDistLoads.push_back(beamDistLoad);
	}
	
	/*
	solve the model using linear static analysis
	build the total forces applied to the model,
	build the global stiffness matrix, and solve for
	K*displacements=external_forces
	
	*/
	void Model::solve() {
		info("solve: called");

		// init the global coords
		info("init the global coords");
		std::vector<CoordMapping> coords;
		int globalCount = 0;
		// for all nodes,
		for (auto& n : nodes) {
			// get x, y, z
			coords.push_back(CoordMapping{ globalCount++,n.getId(),0 });
			coords.push_back(CoordMapping{ globalCount++,n.getId(),1 });
			coords.push_back(CoordMapping{ globalCount++,n.getId(),2 });
		}
		fem::listAll("CoordMappings:", coords);

		// for all elements,
		for (Beam& b : beams) {
			auto k = b.getLocalStiffness();
			auto sor = b.getLocalSOR();

		}
		// build the local stiffness matrix

		// for all forces/loads, retrieve the local force and convert into global force



	}

}