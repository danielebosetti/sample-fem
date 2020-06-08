#include "node.h"
#include "beam.h"
#include "model.h"
#include "beamdistload.h"
#include "gtest/gtest.h"

#include <filesystem>
#include <sstream>
#include "spdlog/spdlog.h"
#include "spdlog/sinks/basic_file_sink.h"

using namespace fem;
using std::cout;
using spdlog::info;
using spdlog::warn;
using Eigen::VectorXd;
using Eigen::MatrixXd;
using Eigen::Matrix3d;

TEST(core_misc_test, test_duplicate_node_id)
{
	Node n1{ 0, 0, 0, 0 };
	Node n2{ 0, 1, 0, 0 };
	Model m;
	EXPECT_ANY_THROW(m.add(n1, n2));
}
TEST(core_misc_test, test_duplicate_beam_id)
{
	Node n{ 0, 0, 0, 0 };
	Beam b1{ 0, n, n} ;
	Model m;
	EXPECT_ANY_THROW(m.add(b1, b1));
}

TEST(core_misc_test, test_missing_node)
{
	Model m;
	EXPECT_ANY_THROW(m.getNode(1));
}

/*
demo using matrix block operations
*/
void test_001()
//TEST(test_solve, use_matrix_block)
{
	MatrixXd a(6, 6);
	Matrix3d r, z, b, k;
	r.setIdentity();
	z.setZero();
	b.setConstant(3);
	k.setConstant(9);
	a << r, z, b, k;
	info("a=\n{}", a);
}

/* single beam, on x axis, with x force*/
TEST(test_solve, DISABLED_solve_qr) {
	MatrixXd A(2, 2);
	A.setIdentity();
	VectorXd r(2);
	r(0) = 1; r(1) = 3;
	VectorXd sol = A.colPivHouseholderQr().solve(r);
	EXPECT_EQ(sol(0), 1);
	EXPECT_EQ(sol(1), 3);
}
/* single beam, on x axis, with x force*/
TEST(test_solve, DISABLED_solve_qr_singular) {
	MatrixXd A(2, 2);
	A.setZero();
	A(0, 0) = 1;
	VectorXd r(2);
	r(0) = 4; r(1) = 0;
	VectorXd sol = A.colPivHouseholderQr().solve(r);
	/*info("A=\n{}", A);
	info("r=\n{}", r);
	info("sol=\n{}", sol);*/
	// this seems to work
	EXPECT_EQ(sol(0), 4);
}


struct SomeFunction {
	int operator()(int x) { return x + 2; }
};
TEST(test_solve, DISABLED_use_stl) {
	std::vector<int> input{ 1, 2, 3, 4 };
	std::vector<int> out;
	std::transform(begin(input), end(input), std::back_inserter(out), SomeFunction{});
	for (auto x : input) { info("input: {}", x); }
	for (auto x : out) { info("out: {}", x); }
}

