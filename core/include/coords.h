#pragma once

#include <fmt/ostream.h>
#include <node.h>
#include <vector>
#include <functional>

namespace fem {

	struct LocalCoord {
		int nodeId;
		int localId;
		bool operator==(const LocalCoord& other) const
		{
			return nodeId == other.nodeId && localId == other.localId;
		}
	};

	struct HashLocalCoord
	{
		//		template <>
		std::size_t operator() (const LocalCoord c) const
		{
			std::size_t h1 = std::hash<int>()(c.nodeId);
			std::size_t h2 = std::hash<int>()(c.localId);
			return h1 ^ h2;
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
				m.globalId, m.nodeId, m.localId);
		}
	};

	template<class T>
	void listAll(std::string caption, std::vector<T> list);

	/* 
	store the mapping between local node coords (numbered 0..n) and global model coords (numbered 0..N) 
	*/
	class CoordsHolder {
	public:
		CoordsHolder() {
		}
		/*
		reserves a global coordinate index for the given local coordinate
		*/
		void registerCoord(int nodeId, int localId) {
			int globalId = globalCount++;
			CoordMapping cm{ globalId, nodeId, localId };
			coords.push_back(cm);
			// map (node,index) to globalIndex
			localToGlobal[LocalCoord{ nodeId, localId }] = globalId;
			globalToCoords[globalId] = cm;
		}
		/*
		reserves global coordinate indexes for the given node
		*/
		void registerNode(Node n) {
			int nodeId = n.getId();
			registerCoord(nodeId, 0);
			registerCoord(nodeId, 1);
			registerCoord(nodeId, 2);
			registerCoord(nodeId, 3);
			registerCoord(nodeId, 4);
			registerCoord(nodeId, 5);
		}
		void listAll() {
			fem::listAll("CoordMappings:", coords);
		}
		std::vector<CoordMapping> getCoords(int nodeId) {
			std::vector<CoordMapping> res;
			for (auto& c : coords) {
				if (c.nodeId == nodeId) {
					res.push_back(c);
				}
			}
			return res;
		}
		/*
		given a nodeId and a local index (eg. 0 for local-x),
		return the index of that coordinate in the global coord system
		*/
		int getGlobalCoord(int nodeId, int localCoordIndex) {
			LocalCoord key{ nodeId, localCoordIndex };
			if (localToGlobal.find(key) == localToGlobal.end()) {
				auto msg = fmt::format("missing key={}-{}", nodeId, localCoordIndex);
				throw std::exception(msg.c_str());
			}
			return localToGlobal[key];
		}

		/*
		
		*/
		CoordMapping getGlobalCoord(int globalId) {
			assert(globalToCoords.find(globalId) != globalToCoords.end());
			return globalToCoords[globalId];
		}

	private:
		std::vector<CoordMapping> coords;
		std::unordered_map<LocalCoord, int, HashLocalCoord> localToGlobal;
		// reference from global-id to 
		std::unordered_map<int, CoordMapping> globalToCoords;
		int globalCount = 0;
	};

}
