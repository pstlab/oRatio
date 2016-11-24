#include "CppUnitTest.h"

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

#include "../solver/solver.h"
#include "../ac/or_propagator.h"

namespace test {
	TEST_CLASS(UnitTest1) {
public:

	TEST_METHOD(TestAC) {
		ac::network n;

		ac::arith_var* x0 = n.new_real();
		Assert::IsNotNull(x0);
		Assert::AreEqual(-std::numeric_limits<double>::infinity(), x0->to_interval().lb);
		Assert::AreEqual(std::numeric_limits<double>::infinity(), x0->to_interval().ub);

		ac::arith_var* x1 = n.new_real();
		Assert::IsNotNull(x1);
		Assert::AreEqual(-std::numeric_limits<double>::infinity(), x1->to_interval().lb);
		Assert::AreEqual(std::numeric_limits<double>::infinity(), x1->to_interval().ub);

		bool added = n.add({ n.geq(x1, x0), n.geq(x0, n.new_real(0)) });
		Assert::IsTrue(added);

		Assert::AreEqual(0.0, x0->to_interval().lb);
		Assert::AreEqual(std::numeric_limits<double>::infinity(), x0->to_interval().ub);
		Assert::AreEqual(0.0, x1->to_interval().lb);
		Assert::AreEqual(std::numeric_limits<double>::infinity(), x1->to_interval().ub);
	}

	TEST_METHOD(TestACNoGood) {
		ac::network n;

		ac::bool_var* b0 = n.new_bool();
		ac::bool_var* b1 = n.new_bool();
		ac::bool_var* b2 = n.new_bool();
		ac::bool_var* b3 = n.new_bool();
		ac::bool_var* b4 = n.new_bool();
		ac::bool_var* b5 = n.new_bool();
		ac::bool_var* b6 = n.new_bool();
		ac::bool_var* b7 = n.new_bool();
		ac::bool_var* b8 = n.new_bool();

		bool added;
		added = n.add({ n.disjunction({ b0, b1 }) });
		Assert::IsTrue(added);
		added = n.add({ n.disjunction({ b0, b2, b6 }) });
		Assert::IsTrue(added);
		added = n.add({ n.disjunction({ n.negate(b1), n.negate(b2), b3 }) });
		Assert::IsTrue(added);
		added = n.add({ n.disjunction({ n.negate(b3), b4, b7 }) });
		Assert::IsTrue(added);
		added = n.add({ n.disjunction({ n.negate(b3), b5, b8 }) });
		Assert::IsTrue(added);
		added = n.add({ n.disjunction({ n.negate(b4), n.negate(b5) }) });
		Assert::IsTrue(added);

		bool assigned;
		assigned = n.assign_true(n.negate(b6));
		Assert::IsTrue(assigned);
		assigned = n.assign_true(n.negate(b7));
		Assert::IsTrue(assigned);
		assigned = n.assign_true(n.negate(b8));
		Assert::IsTrue(assigned);

		// this assignment should create an inconsistency..
		assigned = n.assign_true(n.negate(b0));
		Assert::IsFalse(assigned);

		// this is the unsat core..
		std::vector<ac::bool_var*> unsat_core = n.get_unsat_core();
		// we build the no-good from the unsat core..
		std::vector<ac::bool_var*> no_good;
		for (const auto& v : unsat_core) {
			no_good.push_back(n.negate(v));
		}
		// we backjump..
		while (ac::or_propagator::unsatisfiable(no_good)) {
			if (n.root_level()) {
				Assert::IsFalse(n.root_level());
			}
			else {
				n.pop();
			}
		}
	}

	TEST_METHOD(TestSolver) {
		oratio::solver s;
	}

	};
}