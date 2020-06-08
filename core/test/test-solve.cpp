#include "model.h"

#include "gtest/gtest.h"
#include <spdlog/spdlog.h>
#include <algorithm>

using namespace fem;
using std::cout;
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
		NodeFreedom nf1{ 0, 0, std::vector{0,1,2} };
		m.add(n1, n2, b1, f1);
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

/* local stiffness on single beam, beam is on the xy  plane */
TEST(test_solve, local_k_xy)
{
	Model m;
	Node n1{ 0, 0, 0, 0 };
	Node n2{ 1, 1, 1, 0 };
	Beam b1{ 0, n1, n2 };
	info("k=\n{}", b1.getLocalStiffness());
	info("K=\n{}\n", b1.calcStiffnessMatrix());
	info("local-sor=\n{}\n", b1.getLocalSOR());
}

TEST(test_solve, model_num_coords) {
	Model m;
	EXPECT_EQ(m.numGlobalCoords(), 0);
	m.add(Node{ 0,0,0,0 });
	EXPECT_EQ(m.numGlobalCoords(), 3);
}

/* single beam, on x axis, with x force*/
TEST(test_solve, solve_simple_1)
{
	Model m = TestModelFactory::getTestModel1();
	auto zeroDisp = m.getZeroDisplacement();
	EXPECT_NEAR(zeroDisp.norm(), 0, EPS);
	EXPECT_EQ(zeroDisp.rows(), 6);
	// how do we check that model is solved?
	VectorXd residual = m.computeResidual(zeroDisp);
	info("residual={}", residual);
	EXPECT_EQ(residual.rows(), 6);
	VectorXd sol = m.solve();
	VectorXd solve_residual = m.computeResidual(sol);
	info("solve_residual=\n{}", solve_residual);
}

/* single beam, on x axis, with x force*/
TEST(test_solve, DISABLED_solve_model_2) {
	Model m = TestModelFactory::getTestModel2();
	auto K = m.computeGlobalStiffnessMatrix();
	info("global K=\n{}", K);
	auto F = m.getGlobalActions();
	info("F=\n{}", F);
}


TEST(test_solve, test_constrained_coords) {
	Model m;
	auto v1 = m.getConstrainedCoords();
	EXPECT_EQ(v1.size(), 0);
	//m.add(NodeFreedom{ 0,0,std::vector<ConstraintDimension>{} })
//	std::vector<int> Model::getConstrainedCoords()
}

