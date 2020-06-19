#include "model.h"

#include "gtest/gtest.h"
#include <spdlog/spdlog.h>
#include <algorithm>

using namespace fem;
using std::cout;
using std::vector;
using spdlog::info;
using spdlog::warn;

using Eigen::VectorXd;
using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::Matrix3d;

constexpr double EPS = 1.e-6;

class TestModelFactory {
public:
	// single beam on X axis, single force on X axis
	// node1 is constrained
	static Model getTestModel1() {
		Model m;
		Node n1{ 0, 0, 0, 0 };
		Node n2{ 1, 1, 0, 0 };
		Beam b1{ 0, n1, n2 };
		NodeForce f1{ 0, 1, 1, 0, 0 };
		NodeFreedom nf1{ 0, 0, {0,1,2} };
		m.add(n1, n2, b1, f1, nf1);
		return m;
	}

	// single beam on X axis, single force on X axis
	static Model getTestModel2() {
		Model m;
		Node n1{ 0, 0, 0, 0 };
		Node n2{ 1, 1, 0, 0 };
		Node n3{ 2, 2, 0, 0 };
		Beam b1{ 0, n1, n2 };
		Beam b2{ 1, n2, n3 };
		NodeForce f1{ 0, 1, 0.01, 0, 0 };
		m.add(n1, n2, n3, b1, b2, f1);
		auto sol = m.solve();
		info("sol=\n{}", sol);
		return m;
	}
};


/* local stiffness on single beam, beam is on X axis*/
TEST(test_solve, DISABLED_local_k_x)
{
	Model m;
	Node n1{ 0, 0, 0, 0 };
	Node n2{ 1, 1, 0, 0 };
	Beam b1{ 0, n1, n2 };
	info("k=\n{}", b1.getLocalStiffness());
	info("K=\n{}\n", b1.calcStiffnessMatrix());
}
/* local stiffness on single beam, beam is on Y axis*/
TEST(test_solve, DISABLED_local_k_y)
{
	Model m;
	Node n1{ 0, 0, 0, 0 };
	Node n2{ 1, 0, 1, 0 };
	Beam b1{ 0, n1, n2 };
	info("k=\n{}", b1.getLocalStiffness());
	info("K=\n{}\n", b1.calcStiffnessMatrix());
}
/* local stiffness on single beam, beam is on Z axis*/
TEST(test_solve, DISABLED_local_k_z)
{
	Model m;
	Node n1{ 0, 0, 0, 0 };
	Node n2{ 1, 0, 0, 1 };
	Beam b1{ 0, n1, n2 };
	info("k=\n{}", b1.getLocalStiffness());
	info("K=\n{}\n", b1.calcStiffnessMatrix());
}

/* single beam, on x axis, with x force
node 1 is constrained, so solution must be..
*/
TEST(test_solve, solve_simple_1)
{
	Model m = TestModelFactory::getTestModel1();
	auto zeroDisp = m.getZeroDisplacement();
	EXPECT_NEAR(zeroDisp.norm(), 0, EPS);
	EXPECT_EQ(zeroDisp.rows(), 6);
	// how do we check that model is solved?
	VectorXd residual = m.computeResidual(zeroDisp);
	EXPECT_EQ(residual.rows(), 6);

	VectorXd sol = m.solve();
	//info("sol={}", sol);
	//VectorXd solve_residual = m.computeResidual(sol);
	//info("solve_residual=\n{}", solve_residual);
}
