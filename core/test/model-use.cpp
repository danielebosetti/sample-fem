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

//TEST(sample_test_case, DISABLED_test_versor)
void go()
{
	std::filesystem::create_directory("logs");
	auto logger = spdlog::basic_logger_mt("test", "logs/test.txt");
	spdlog::set_pattern("%H:%M:%S.%e [%l] %v");
//	spdlog::set_default_logger(logger);

	Node n1{ 0, 0, 0, 0 };
	Node n2{ 0, 1, 0, 0 };
	Beam b1{ 0, n1, n2 };

	auto sor = b1.getLocalSOR();
	info("\n{}", sor);

	info("sor={}", sor);

	auto e1 = sor.col(0);
	auto e2 = sor.col(1);
	auto e3 = sor.col(2);
	EXPECT_FLOAT_EQ(e1.norm(), 1.);
	EXPECT_FLOAT_EQ(e2.norm(), 1.);
	EXPECT_FLOAT_EQ(e3.norm(), 1.);
}

//TEST(sample_test_case, sample_test)
void go2()
{
	Node n1{ 0, 0, 0, 0 };
	Node n2{ 1, 1, 0, 0 };
	Node n3{ 2, 2, 0, 0 };

	Beam b1{ 0, n1, n2 };
	Beam b2{ 0, n2, n3 };
	BeamDistLoad beamDistLoad1{ 0, 1, 0, 0, -10 };

	Model m;
	m.add(n1);
	m.add(n2);
	m.add(n3);
	m.add(b1);
	m.add(b2);
	m.add(beamDistLoad1);
	
	m.listAll();

}

