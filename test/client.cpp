// Bring in my package's API, which is what I'm testing
#include "humotion/client/client.h"
#include <gtest/gtest.h>
#include <string>
#include <cstdio>
using namespace std;
using namespace humotion;
using namespace humotion::client;

namespace {

// the fixture for testing class Foo.
class client_Test : public ::testing::Test {
protected:
	client_Test() {
		// set-up work for EACH test here
		s = new Client("test", "ROS");
	}

	virtual ~client_Test() {
		// clean-up work that doesn't throw exceptions here
		delete (s);
		s = NULL;
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
	Client* s;
};

// Tests that the Foo::Bar() method does Abc.
TEST_F(client_Test, send_mouth_data) {
	MouthState m;
	for (int i = 0; i < 100; i++) {
		m.opening_left = 12.0;
		m.opening_center = 12.0;
		m.opening_right = 12.0;
		m.position_left = 10 + (20.0 * i) / 100.0;
		m.position_center = 10 + (20.0 * i) / 100.0;
		m.position_right = 10 + (20.0 * i) / 100.0;

		s->update_mouth_target(m, true);
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
