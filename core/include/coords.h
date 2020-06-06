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

	// "store" all nodes, 
	class CoordsHolder {
	public:
		CoordsHolder() {
		}
		void registerCoord(int nodeId, int localId) {
			int globalId = globalCount++;
			coords.push_back(CoordMapping{ globalId ,nodeId, localId });
			// map (node,index) to globalIndex
			localToGlobal[LocalCoord{ nodeId, localId }] = globalId;
		}
		// registers coordinates for this node
		void registerNode(Node n) {
			int nodeId = n.getId();
			registerCoord(nodeId, 0);
			registerCoord(nodeId, 1);
			registerCoord(nodeId, 2);
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

	private:
		std::vector<CoordMapping> coords;
		std::unordered_map<LocalCoord, int, HashLocalCoord> localToGlobal;
		int globalCount = 0;
	};

}
