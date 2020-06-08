
#include "model.h"

#include <iostream>
#include "spdlog/spdlog.h"
#include "coords.h"

using spdlog::info;
using spdlog::warn;
using Eigen::VectorXd;
using Eigen::MatrixXd;
using Eigen::Matrix3d;

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
		ch.registerNode(n);
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

	void Model::add(NodeFreedom nodeFreedom) {
		addAndCheckDuplicate(nodeFreedomIds, nodeFreedom.getId(), "nodeFreedom");
		nodeFreedoms.push_back(nodeFreedom);
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

	/*
	return the list of all constrained global coords
	*/
	std::vector<int> Model::getConstrainedCoords() {
		std::vector<int> res;
		// for each nodeFreedom
		for (NodeFreedom& nf : nodeFreedoms) {
			int nodeId = nf.getId();
			for (int constrainedCoord : nf.getConstraints()) {
				int globalCoordId=ch.getGlobalCoord(nodeId, constrainedCoord);
				res.push_back(globalCoordId);
			}
		}
		return res;
	}

	/*
	solve the model using linear static analysis
	build the total forces applied to the model,
	build the global stiffness matrix, and solve for
	K*displacements=external_forces
	
	SPLIT free and constrained nodes
	*/
	VectorXd Model::solve() {
		info("solve: called");
		MatrixXd K = computeGlobalStiffnessMatrix();
		MatrixXd F = getGlobalActions();

		std::vector<int> constrainedCoords = getConstrainedCoords();

		info("solve: K=\n{}", K);
		info("solve: F=\n{}", F);
		VectorXd sol = K.colPivHouseholderQr().solve(F);
		info("solve: sol=\n{}", sol);
		return sol;
	}
	
	/*
	build K, apply displacement to K, and return K*disp-r
	displacement is expressed in global coordinates
	*/
	VectorXd Model::computeResidual(VectorXd displacement) {
		// residual, solve for K*x=r
		auto res = getGlobalActions();
		auto K = computeGlobalStiffnessMatrix();
		// build the local stiffness matrix
		// for all forces/loads, retrieve the local force and convert into global force

		return K*displacement-res;
	}

	MatrixXd Model::computeGlobalStiffnessMatrix() {
		// init global_K
		MatrixXd K(numGlobalCoords(), numGlobalCoords());
		K.setZero();

		// for all elements,
		for (Beam& b : beams) {
			auto K_beam = b.calcStiffnessMatrix();
			// K_beam is expressed in the global coord system, but using local indexes (does this make sense?)
			// map coordinates into K_global

			int nodeId1 = b.getNode1().getId();
			int nodeId2 = b.getNode2().getId();

			// k_beam is 6*6, using vars:
			std::vector<int> coordMap;
			coordMap.push_back(ch.getGlobalCoord(nodeId1, 0));
			coordMap.push_back(ch.getGlobalCoord(nodeId1, 1));
			coordMap.push_back(ch.getGlobalCoord(nodeId1, 2));
			coordMap.push_back(ch.getGlobalCoord(nodeId2, 0));
			coordMap.push_back(ch.getGlobalCoord(nodeId2, 1));
			coordMap.push_back(ch.getGlobalCoord(nodeId2, 2));

			// extract from k_beam
			for (int i = 0; i < coordMap.size(); i++) {
				for (int j = 0; j < coordMap.size(); j++) {
					K(coordMap[i], coordMap[j]) += K_beam(i, j);
				}
			}
		}
		return K;
	}

	/*
	return a zero array, with local displacements for all coordinates
	NOTE, is a dof the same as a coordinate? (NO)
	shoudl we distinguish between dof (enters into the stiffness matrix K)
	and a coordinate (could be excluded from the stiffness matrix)
	OR, do we put all coords into K? then if K entries are zero (no reaction on a variable), we have a zero row
	note, K_{i}=(0,..,0) iff the reaction on coordinate j is zero, when x_{i} is displaced. 
	NOTE, the constraints coordinates DO NOT become rows of K.
	that is, the initially are, but then we split free vs. constrained nodes, and build a REDUCED K matrix.
	known coordinates (constrained coordinates are not changes, so we know the solution for these) go on the RHS
	*/
	VectorXd Model::getZeroDisplacement() {
		int modelDimension = nodes.size() * 3;
		VectorXd zeroDisp(modelDimension);
		zeroDisp.setZero();
		return zeroDisp;
	}

	/*
	return the total number of coordinates for this model
	including constrained nodes/dof-s
	*/
	int Model::numGlobalCoords() {
		return nodes.size() * 3;
	}

	VectorXd Model::getGlobalActions() {
		// sum up all coords for all nodes
		VectorXd res(numGlobalCoords());
		res.setZero();
		// for each force
		for (NodeForce& f : nodeForces) {

			// convert the local forces into global forces
			// for each force coord, get the respective global coord
			int nodeId = f.getNodeId();
			
			int globalIdX = ch.getGlobalCoord(nodeId, 0);
			int globalIdY = ch.getGlobalCoord(nodeId, 1);
			int globalIdZ = ch.getGlobalCoord(nodeId, 2);
			res[globalIdX] += f.getPosition().x();
			res[globalIdY] += f.getPosition().y();
			res[globalIdZ] += f.getPosition().z();
		}
		return res;
	}


}