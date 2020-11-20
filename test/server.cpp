// Bring in my package's API, which is what I'm testing
#include "humotion/server/server.h"
#include <gtest/gtest.h>
#include <string>
#include <cstdio>
using namespace std;
using namespace humotion;
using namespace humotion::server;

namespace {

// the fixture for testing class Foo.
class server_Test : public ::testing::Test {
protected:
	server_Test() {
		// set-up work for EACH test here
		s = new Server("test", "ROS", nullptr);
	}

	~server_Test() override {
		// clean-up work that doesn't throw exceptions here
		delete (s);
		s = nullptr;
	}

	// if the constructor and destructor are not enough for setting up
	// and cleaning up each test, you can define the following methods:
	void SetUp() override {
		// code here will be called immediately after the constructor (right before each test).
	}

	void TearDown() override {
		// code here will be called immediately after each test (right
		// before the destructor).
	}

	// ojects declared here can be used by all tests in the test case for Foo
	Server* s;
};

void dump_incoming_data(Server* s) {
	SCOPED_TRACE("dump incoming data");

	// make sure it is not null
	if (s == nullptr) {
		SCOPED_TRACE("server ptr NULL?");
		FAIL();
	}

	// make sure we are connected
	EXPECT_EQ(s->ok(), true);

	for (int i = 0; i < 100000; i++) {
		// s->tick();
	}
}

// Tests that the Foo::Bar() method does Abc.
TEST_F(server_Test, dump_incoming_data) {
	// connected?
	dump_incoming_data(s);
	//   const string input_filepath = "this/package/testdata/myinputfile.dat";
	//   const string output_filepath = "this/package/testdata/myoutputfile.dat";
	//   Foo f;
	//   EXPECT_EQ(0, f.Bar(input_filepath, output_filepath));
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
