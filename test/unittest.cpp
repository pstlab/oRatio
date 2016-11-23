#include "CppUnitTest.h"

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

#include "../solver/solver.h"

namespace test {
	TEST_CLASS(UnitTest1) {
public:

	TEST_METHOD(TestAC) {
		ac::network n;

		ac::arith_var* x0 = n.new_real();
		Assert::IsNotNull(x0);
		ac::arith_var* x1 = n.new_real();
		Assert::IsNotNull(x1);

		bool added = n.add({ n.geq(x1, x0), n.geq(x0, n.new_real(0)) });
		Assert::IsTrue(added);

		Assert::AreEqual(0, x0->to_interval().lb, 0);
		Assert::AreEqual(std::numeric_limits<double>::infinity(), x0->to_interval().lb, 0);
	}

	TEST_METHOD(TestSolver) {
		oratio::solver s;
	}

	};
}