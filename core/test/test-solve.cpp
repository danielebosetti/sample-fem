#include "model.h"

#include "gtest/gtest.h"
#include <spdlog/spdlog.h>

using namespace fem;
using std::cout;
using spdlog::info;
using spdlog::warn;

using Eigen::VectorXd;
using Eigen::MatrixXd;
using Eigen::Matrix3d;

constexpr double EPS = 1.e-6;

class TestModelFactory {
public:
	// single beam on X axis, single force on X axis
	static Model getTestModel1() {
		Model m;
		Node n1{ 0, 0, 0, 0 };
		Node n2{ 1, 1, 0, 0 };
		Beam b1{ 0, n1, n2 };
		NodeForce f1{ 0, 0, 1, 0, 0 };
		m.add(n1, n2, b1, f1);
		return m;
	}
};


/* local stiffness on single beam, beam is on X axis*/
TEST(test_solve, local_k_x)
{
	Model m;
	Node n1{ 0, 0, 0, 0 };
	Node n2{ 1, 1, 0, 0 };
	Beam b1{ 0, n1, n2 };
	info("k=\n{}", b1.getLocalStiffness());
	info("K=\n{}\n", m.calcStiffnessMatrix(b1));
}
/* local stiffness on single beam, beam is on Y axis*/
TEST(test_solve, local_k_y)
{
	Model m;
	Node n1{ 0, 0, 0, 0 };
	Node n2{ 1, 0, 1, 0 };
	Beam b1{ 0, n1, n2 };
	info("k=\n{}", b1.getLocalStiffness());
	info("K=\n{}\n", m.calcStiffnessMatrix(b1));
}
/* local stiffness on single beam, beam is on Z axis*/
TEST(test_solve, local_k_z)
{
	Model m;
	Node n1{ 0, 0, 0, 0 };
	Node n2{ 1, 0, 0, 1 };
	Beam b1{ 0, n1, n2 };
	info("k=\n{}", b1.getLocalStiffness());
	info("K=\n{}\n", m.calcStiffnessMatrix(b1));
}

/* local stiffness on single beam, beam is on the xy  plane */
TEST(test_solve, local_k_xy)
{
	Model m;
	Node n1{ 0, 0, 0, 0 };
	Node n2{ 1, 1, 1, 0 };
	Beam b1{ 0, n1, n2 };
	info("k=\n{}", b1.getLocalStiffness());
	info("K=\n{}\n", m.calcStiffnessMatrix(b1));
	info("local-sor=\n{}\n", b1.getLocalSOR());
}


/* single beam, on x axis, with x force*/
TEST(test_solve, DISABLED_solve_simple_1)
{
	Model m = TestModelFactory::getTestModel1();
	//m.listAll();
	auto zeroDisp = m.getZeroDisplacement();
	EXPECT_NEAR(zeroDisp.norm(), 0, EPS);
	EXPECT_EQ(zeroDisp.rows(), 6);
	// how do we check that model is solved?
	VectorXd residual = m.computeResidual(zeroDisp);
	info("residual={}", residual);
	EXPECT_EQ(residual.rows(), 6);
}
