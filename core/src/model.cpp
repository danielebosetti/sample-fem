
#include "model.h"

#include <iostream>
#include "spdlog/spdlog.h"
#include "coords.h"

using spdlog::info;
using spdlog::warn;

namespace fem {

	/*
	adds [id] to the given list, and throws if id is duplicated
	*/
	template<typename key> 
	void addAndCheckDuplicate(std::unordered_set<key>& ids, key id, std::string type) {
		if (ids.find(id) != ids.end()) {
			warn("found duplicate {} id={}", type, id);
			throw std::exception(fmt::format("duplicate {} id={}", type, id).c_str());
		}
		ids.insert(id);
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

	Node Model::getNode(int id) {
		if (hasNode(id)) {
			return nodesMap[id];
		}
		else {
			warn("getNode: node not found, id={}", id);
			throw std::exception(fmt::format("getNode: node not found, id={}", id).c_str());
		}
	}

	bool Model::hasNode(int nodeId) {
		return nodeIds.find(nodeId) != nodeIds.end();
	}

	void Model::validate() {
		info("validate");
		for (NodeForce& f : nodeForces) {
			if (!hasNode(f.getNodeId())) {
				auto msg = fmt::format("validate: missing node, force id={} node-id={}", f.getId(), f.getNodeId());
				warn(msg);
				throw std::exception(msg.c_str());
			}
		}
	}

	void Model::initCoords() {
		// init the global coords
		info("init the global coords");

		ch = std::make_unique<CoordsHolder>();

		// for all nodes,
		for (auto& n : nodes) {
			ch->registerNode(n);
		}
	}

	/*
	solve the model using linear static analysis
	build the total forces applied to the model,
	build the global stiffness matrix, and solve for
	K*displacements=external_forces
	
	*/
	void Model::solve() {
		info("solve: called");
		initCoords();

		// compute the residual force
		// for all forces,
		for (NodeForce& f : nodeForces) {
			// convert the local force to a global force
			// get node id
			int nodeId = f.getNodeId();
			// get node
			Node& node = getNode(nodeId);
			// get coords
			//ch.getCoords(nodeId);
			// map force_x, force_y, force_z
		}


		// for all elements,
		for (Beam& b : beams) {
			auto k = b.getLocalStiffness();
			auto sor = b.getLocalSOR();

		}
		// build the local stiffness matrix

		// for all forces/loads, retrieve the local force and convert into global force


	}
	
	using Eigen::VectorXd;

	std::unique_ptr<VectorXd> Model::getGlobalActions() {
		if (!ch) {
			warn("ch is uninitialized");
			throw std::exception("ch is uninitialized");
		}
		int numNodes = nodes.size();
		// sum up all coords for all nodes
		auto res = std::make_unique<VectorXd>(numNodes*3);
		res->setZero();
		// for each force
		for (NodeForce& f : nodeForces) {

			// convert the local forces into global forces
			// for each force coord, get the respective global coord
			int nodeId = f.getNodeId();
			
			int globalIdX = ch->getGlobalCoord(nodeId, 0);
			int globalIdY = ch->getGlobalCoord(nodeId, 1);
			int globalIdZ = ch->getGlobalCoord(nodeId, 2);
			(*res)[globalIdX] += f.getPosition().x();
			(*res)[globalIdY] += f.getPosition().y();
			(*res)[globalIdZ] += f.getPosition().z();
		}
		return res;
	}


}