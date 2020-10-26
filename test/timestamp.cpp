// Bring in my package's API, which is what I'm testing
#include "timestamp.h"
#include <gtest/gtest.h>
#include <string>
#include <cstdio>
#include <stdio.h>
#include <thread>
#include <chrono>

using namespace std;
using namespace humotion;

namespace {

// the fixture for testing class Foo.
class timestamp_Test : public ::testing::Test {
protected:
	timestamp_Test() {
		// set-up work for EACH test here
	}

	virtual ~timestamp_Test() {
		// clean-up work that doesn't throw exceptions here
	}

	// if the constructor and destructor are not enough for setting up
	// and cleaning up each test, you can define the following methods:
	virtual void SetUp() {
		// code here will be called immediately after the constructor (right before each test).
	}

	virtual void TearDown() {
		// code here will be called immediately after each test (right
		// before the destructor).
	}

	// ojects declared here can be used by all tests in the test case for Foo
};

// Tests that the Foo::Bar() method does Abc.
TEST_F(timestamp_Test, init_from_values) {
	// check sec/nsec initialiser
	Timestamp a(1.0, 0.0);
	ASSERT_EQ(a.to_seconds(), 1.0);

	Timestamp b(999, 123);
	ASSERT_EQ(b.to_seconds(), 999.0 + 123 / 1000000000.0);

	Timestamp c(666.777);
	ASSERT_EQ(c.to_seconds(), 666.777);
	ASSERT_EQ(c.sec, 666);
	ASSERT_EQ(c.nsec, 0.777 / 1E-9);
	;

	// test initializer with now()
	struct timespec tp;
	clock_gettime(CLOCK_REALTIME, &tp);

	Timestamp ts_d;
	std::this_thread::sleep_for(std::chrono::seconds(1));
	Timestamp ts_e;

	// they should be at least 1 second apart:
	double diff = ts_e.to_seconds() - ts_d.to_seconds();
	ASSERT_GE(diff, 1.0);
	ASSERT_LE(diff, 2.0);

	// d should also be greater equal that timespec values:
	ASSERT_GE(ts_d.sec, tp.tv_sec);

	SUCCEED();
}

TEST_F(timestamp_Test, comparison) {
	// check sec/nsec initialiser
	Timestamp a(1234567, 89101213);
	Timestamp b(1234567, 89101213);
	ASSERT_TRUE(a == b);
	ASSERT_TRUE(a >= b);
	ASSERT_TRUE(a <= b);
	ASSERT_FALSE(a != b);
	ASSERT_FALSE(a < b);
	ASSERT_FALSE(a > b);

	// check double initializer
	Timestamp e(1.234567);
	Timestamp f(1.234568);
	ASSERT_FALSE(e == f);
	ASSERT_FALSE(e >= f);
	ASSERT_TRUE(e <= f);
	ASSERT_TRUE(e != f);
	ASSERT_TRUE(e < f);
	ASSERT_FALSE(e > f);

	// check now() initializer
	Timestamp c;
	std::this_thread::sleep_for(std::chrono::seconds(1));
	Timestamp d;
	ASSERT_FALSE(c == d);
	ASSERT_FALSE(c >= d);
	ASSERT_TRUE(c <= d);
	ASSERT_TRUE(c != d);
	ASSERT_TRUE(c < d);
	ASSERT_FALSE(c > d);

	// ok now test comparisons based on sec:
	Timestamp g(100, 1234567);
	Timestamp h(101, 1234567);
	ASSERT_FALSE(g == h);
	ASSERT_FALSE(g >= h);
	ASSERT_TRUE(g <= h);
	ASSERT_TRUE(g != h);
	ASSERT_TRUE(g < h);
	ASSERT_FALSE(g > h);

	// ok now test comparisons based on nsec:
	{
		Timestamp i(100, 1234567);
		Timestamp j(100, 1234568);
		ASSERT_FALSE(i == j);
		ASSERT_FALSE(i >= j);
		ASSERT_TRUE(i <= j);
		ASSERT_TRUE(i != j);
		ASSERT_TRUE(i < j);
		ASSERT_FALSE(i > j);
	}

	// ok now test comparisons based on sec + nsec:
	{
		Timestamp i(100, 1234568);
		Timestamp j(101, 1234567);
		ASSERT_FALSE(i == j);
		ASSERT_FALSE(i >= j);
		ASSERT_TRUE(i <= j);
		ASSERT_TRUE(i != j);
		ASSERT_TRUE(i < j);
		ASSERT_FALSE(i > j);
	}

	SUCCEED();
}

// Tests that Foo does Xyz.
// TEST_F(xsc3_client_cpp_Test, DoesXyz) {
// 	CheckConnection(xsc3_client);
// }

} // namespace

// Run all the tests that were declared with TEST()
int main(int argc, char** argv) {
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
