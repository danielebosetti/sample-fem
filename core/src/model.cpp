
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
	build the local stiffness matrix
	convert the local stiffness matrix to global coordinates (rotate only, don't re-index)
	then, re-index the coordinates to global indexes
	*/
	MatrixXd Model::calcStiffnessMatrix(Beam b) {
		// calc k, in local coord indexes, but in global SOR
		MatrixXd k = b.getLocalStiffness();
		Matrix3d sor = b.getLocalSOR();
		MatrixXd rotation(6, 6);
		rotation << sor.transpose(), Matrix3d::Zero(), Matrix3d::Zero(), sor.transpose();
		return rotation.transpose() * k * rotation;
	}

	/*
	solve the model using linear static analysis
	build the total forces applied to the model,
	build the global stiffness matrix, and solve for
	K*displacements=external_forces
	
	*/
	VectorXd Model::solve() {
		info("solve: called");

		
		VectorXd displacement;
		return displacement;
	}
	
	/*
	build K, apply displacement to K, and return K*d-r
	*/
	VectorXd Model::computeResidual(VectorXd displacement) {

		// residual, solve for K*x=r
		auto res = getGlobalActions();

		// for all elements,
		for (Beam& b : beams) {
			calcStiffnessMatrix(b);
		}
		// build the local stiffness matrix
		// for all forces/loads, retrieve the local force and convert into global force

		VectorXd residual;
		return residual;
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
	
	std::unique_ptr<VectorXd> Model::getGlobalActions() {
		int numNodes = nodes.size();
		// sum up all coords for all nodes
		auto res = std::make_unique<VectorXd>(numNodes*3);
		res->setZero();
		// for each force
		for (NodeForce& f : nodeForces) {

			// convert the local forces into global forces
			// for each force coord, get the respective global coord
			int nodeId = f.getNodeId();
			
			int globalIdX = ch.getGlobalCoord(nodeId, 0);
			int globalIdY = ch.getGlobalCoord(nodeId, 1);
			int globalIdZ = ch.getGlobalCoord(nodeId, 2);
			(*res)[globalIdX] += f.getPosition().x();
			(*res)[globalIdY] += f.getPosition().y();
			(*res)[globalIdZ] += f.getPosition().z();
		}
		return res;
	}


}