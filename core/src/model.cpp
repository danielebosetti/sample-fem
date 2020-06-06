#include "model.h"

// #include <fmt/format.h>
#include <iostream>
#include "spdlog/spdlog.h"

using spdlog::info;
using spdlog::warn;

namespace fem {
	
	 void Model::add(Node n) {
		 int nodeId = n.getId();
		 if (nodes.find(nodeId) != nodes.end()) {
			 warn("duplicate node id={}", nodeId);
			 throw std::exception("duplicate node id");
		 }
		 nodes[nodeId] = n;
		 for (int i = 0; i < n.dofCount(); i++) {
			 dofs[globalDofCount++] = std::make_pair(nodeId, i);
		 }
	 }
	 void Model::listNodes() {
		 for (auto& e : nodes) {
			 int id = e.first;
			 Node n = e.second;
			 info("node id={} node={}", id, n);
		 }
	 }

	 void Model::listBeamDistLoads() {
		 for (auto& e : beamDistLoads) {
			 info("beamDistLoad load={}", e.toString());
		 }
	 }


	 void listDofs() {
	 }

	 void Model::add(BeamDistLoad beamDistLoad) {
		 beamDistLoads.push_back(beamDistLoad);
	 }


	// Node::Node(int id_, double x_, double y_, double z_) : id{ id_ }, x{ x_ }, y{ y_ }, z{ z_ } {
	// }

	// std::string Node::toString() {
		// return fmt::format("Node[id={},pos=[{},{},{}]]", id, x, y, z);
	// }

	// Eigen::Vector3d Node::getPosition() {
		// return Eigen::Vector3d{x, y, z};
	// }

	// int Node::getId() {
		// return id;
	// }
	// int Node::dofCount() {
		// return 3;
	// }


}