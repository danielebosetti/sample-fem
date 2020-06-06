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

int main(int argc, char* argv[]) {
	spdlog::set_pattern("[%^%l%$] %v");
	::testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}

/* single beam, on x axis, with x force*/
TEST(test_solve, single_beam_x)
{
	Model m;
	Node n1{ 0, 0, 0, 0 };
	Node n2{ 1, 1, 0, 0 };
	Beam b1{ 0, n1, n2 };
	NodeForce f1{ 0, 0, 1, 0, 0 };
	m.add(n1, n2, b1, f1);
	m.listAll();

	m.solve();
}

using Eigen::VectorXd;

TEST(core_misc, global_actions_1)
{
	Model m;
	Node n{ 0, 0, 0, 0 };
	m.add(n);
	m.initCoords();
	VectorXd r = *m.getGlobalActions();
	VectorXd expected(3);
	expected.setZero();

	EXPECT_EQ(r.rows(), 3);
	info("r={}", r.transpose());
	info("expected={}", expected.transpose());
	EXPECT_NEAR((r - expected).norm(), 0, 1.e-6);
}
TEST(core_misc, check_throw_on_missing_coord) {
	CoordsHolder ch;
	EXPECT_ANY_THROW(ch.getGlobalCoord(0, 0));
}
TEST(core_misc, check_hash_map) {
	CoordsHolder ch;
	ch.registerCoord(0, 0);
	ch.registerCoord(0, 1);
	int c1 = ch.getGlobalCoord(0, 0);
	EXPECT_EQ(c1, 0);
}
TEST(core_misc, check_unique_ptr) {
	std::unique_ptr<int> p;
	EXPECT_FALSE(p);
	p = std::make_unique<int>(9);
	EXPECT_TRUE(p);
}

TEST(core_misc, basic_1) {
	Model m;
	EXPECT_FALSE(m.hasNode(0));
	EXPECT_FALSE(m.hasNode(1));
	m.add(Node{ 0,0,0,0 });
	EXPECT_TRUE(m.hasNode(0));
	EXPECT_FALSE(m.hasNode(1));
	m.add(Node{ 1,2,0,0 });
	EXPECT_TRUE(m.hasNode(0));
	EXPECT_TRUE(m.hasNode(1));
}

TEST(core_misc, global_actions_2_force)
{
	Model m;
	Node n{ 0, 0, 0, 0 };
	NodeForce f{ 0, 0, 0, 0, -3 };
	m.add(n, f);
	m.initCoords();
	VectorXd r = *m.getGlobalActions();
	Eigen::Vector3d expected(0, 0, -3);

	info("r={}", r.transpose());
	info("expected={}", expected.transpose());
	EXPECT_NEAR((r - expected).norm(), 0, 1.e-6);
}
TEST(core_misc, global_actions_3_force)
{
	Model m;
	Node n1{ 0, 0, 0, 0 };
	Node n2{ 1, 1, 0, 0 };
	Node n3{ 2, 0, 1, 1 };
	NodeForce f1{ 0, 0, 0, 0, -3 };
	NodeForce f2{ 1, 1, 1, 2, -3 };
	NodeForce f3{ 2, 1, 0, -4, -3 };
	NodeForce f4{ 3, 0, 4, 0, -3 };
	NodeForce f5{ 4, 2, 5, 5, 0 };
	m.add(n1, n2, n3, f1, f2, f3, f4, f5);
	m.validate();
	m.initCoords();
	VectorXd r = *m.getGlobalActions();
	Eigen::VectorXd expected(9);
	expected << 4, 0, -6, 1, -2, -6, 5, 5, 0;
	info("r={}", r.transpose());
	info("expected={}", expected.transpose());
	EXPECT_NEAR((r - expected).norm(), 0, 1.e-6);
}
