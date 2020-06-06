#pragma once

#include <Eigen/Dense>

namespace fem {
	class BeamDistLoad {
	public:
		BeamDistLoad(int id, int beamId, double x, double y, double z);
		std::string BeamDistLoad::toString();
	private:
		int id;
		int beamId;
		Eigen::Vector3d direction;

		template<typename ostream>
		friend ostream& operator<<(ostream& os, const fem::BeamDistLoad& b)
		{
			return os << fmt::format("BeamDistLoad[id={},nodeId={}, direction=[{},{},{}]]",
				b.id, b.beamId, b.direction.x(), b.direction.y(), b.direction.z());
		}

	};
}
