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

TEST(sample_test_case, test_versor)
{
	std::filesystem::create_directory("logs");
	auto logger = spdlog::basic_logger_mt("test", "logs/test.txt");
	spdlog::set_pattern("%H:%M:%S.%e [%l] %v");
//	spdlog::set_default_logger(logger);

	Node n1{ 0, 0, 0, 0 };
	Node n2{ 0, 1, 0, 0 };
	Beam b1{ 0, n1, n2 };
	cout << "b1=" << b1;

	auto sor = b1.getLocalSOR();
	cout << "sor=\n" << sor << "\n";

	std::stringstream ss;
	ss << sor;
	spdlog::info("sor={}", ss.str());

	auto e1 = sor.col(0);
	auto e2 = sor.col(1);
	auto e3 = sor.col(2);
	EXPECT_FLOAT_EQ(e1.norm(), 1.);
	EXPECT_FLOAT_EQ(e2.norm(), 1.);
	EXPECT_FLOAT_EQ(e3.norm(), 1.);
}

TEST(sample_test_case, sample_test)
{
	Node n1{ 0, 0, 0, 0 };
	Node n2{ 1, 1, 1, 1 };
	Beam b1{ 0, n1, n2 };
	BeamDistLoad beamDistLoad{ 0, 1, 0, 0, -10 };

	spdlog::info("b1={}", b1);
	spdlog::info("beamDistLoad={}", beamDistLoad);

	//auto k = b1.getLocalStiffness();
	//cout << "\nk=\n" << k << "\n";
	auto sor = b1.getLocalSOR();
//	cout << "\nsor=\n" << sor << "\n";
	Model m;
	m.add(n1);
	m.add(n2);
	m.listNodes();
	m.add(beamDistLoad);
	m.listBeamDistLoads();

}

