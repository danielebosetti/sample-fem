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

int main(int argc, char* argv[]) {
	spdlog::set_pattern("%X.%e [%^%l%$] %v");
	::testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}

TEST(core_misc_test, test_missing_node)
{
	Model m;
	EXPECT_ANY_THROW(m.getNode(1));
}
