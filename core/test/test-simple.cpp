#include "node.h"
#include "beam.h"
#include "model.h"
#include "nodeforce.h"
#include "beamdistload.h"
#include "gtest/gtest.h"

#include <spdlog/spdlog.h>

using namespace fem;
using std::cout;
using spdlog::info;
using spdlog::warn;

/* single beam, on x axis, with x force*/
TEST(test_solve, single_beam_x)
{
	Node n1{ 0, 0, 0, 0 };
	Node n2{ 0, 1, 0, 0 };
	Beam b1{ 0, n1, n2 };
	NodeForce f1{ 0, 0, 1, 0, 0 };
	Model m;
	m.add(n1, n2, b1, f1);
	m.listAll();
	
	m.solve();
}  

class TestEnvironment : public ::testing::Environment {
public:
    virtual void SetUp() {
		spdlog::set_pattern("[%^%l%$] %v");
		//spdlog::set_pattern("%H:%M:%S.%e [%^%l%$] %v");
	}
};

int main(int argc, char* argv[]) {
  spdlog::set_pattern("[%^%l%$] %v");
	::testing::InitGoogleTest(&argc, argv);
	::testing::AddGlobalTestEnvironment(new TestEnvironment);
	return RUN_ALL_TESTS();
}