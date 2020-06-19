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
using std::vector;
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
	ASSERT_EQ(m.numGlobalCoords(), 3);
	vector expAll{ 0,1,2 };
	ASSERT_EQ(m.getAllCoords(), expAll);
	ASSERT_EQ(m.getFreeCoords(), expAll);
	m.add(Node{ 1,0,0,0 });
	vector expAll2{ 0,1,2,3,4,5 };
	ASSERT_EQ(m.getAllCoords(), expAll2);
	ASSERT_EQ(m.getFreeCoords(), expAll2);
	m.add(NodeFreedom{ 0,1,{0} });
	vector expFree{ 0,1,2,4,5 };
	ASSERT_EQ(m.getAllCoords(), expAll2);
	ASSERT_EQ(m.getConstrainedCoords(), vector{ 3 });
	ASSERT_EQ(m.getFreeCoords(), expFree);
	m.add(NodeFreedom{ 1,0,{1,2} });
	vector expConst2{ 1,2,3 };
	vector expFree2{ 0,4,5 };
	ASSERT_EQ(m.getConstrainedCoords(), expConst2);
	ASSERT_EQ(m.getFreeCoords(), expFree2);
	m.add(NodeFreedom{ 2,0,{0,1,2} });
	m.add(NodeFreedom{ 3,1,{0,1,2} });
	vector<int> expEmpty{};
	ASSERT_EQ(m.getConstrainedCoords(), expAll2);
	ASSERT_EQ(m.getFreeCoords(), expEmpty);
}

TEST(misc, check_add_node) {
	Model m;
	int nodeId = 123;
	Node n1{ nodeId,1,2,3 };
	Node nodeExpected{ 123,1,2,3 };
	m.add(n1);
	Node n3 = m.getNode(nodeId);
	ASSERT_EQ(n1, n3);
}

TEST(misc, global_pos) {
	Model m;
	auto p1 = m.getGlobalPositions();
	ASSERT_EQ(p1.size(), 0);
	m.add(Node{ 0,12,4,5 });
	auto p2 = m.getGlobalPositions();
	vector expected{ 12., 4., 5. };
	ASSERT_EQ(expected, p2);
	m.add(Node{ 1,6,4,2 });
	vector expected2{ 12., 4., 5., 6., 4., 2. };
	auto p3 = m.getGlobalPositions();
	ASSERT_EQ(expected2, p3);
}

TEST(misc, test_global_coord) {
	CoordsHolder ch;
	Model m;
	Node n1{ 0,1,2,3 };
	m.add(n1);
	ch.registerNode(n1);
	CoordMapping c = ch.getGlobalCoord(2);
	Node n = m.getNode(c.nodeId);
	double val = n.getVal(c.localId);
	ASSERT_EQ(val, 3);
}



// WIP


/* single beam, on x axis, with x force*/
TEST(test_solve, DISABLED_solve_model_2) {
	//Model m = TestModelFactory::getTestModel2();
	//auto K = m.computeGlobalStiffnessMatrix();
	//info("global K=\n{}", K);
	//auto F = m.getGlobalActions();
	//info("F=\n{}", F);
}
