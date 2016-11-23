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

		ac::bool_var* x0 = n.new_bool();
		ac::bool_var* x1 = n.new_bool();
		ac::bool_var* x2 = n.new_bool();
		ac::bool_var* x3 = n.new_bool();
		ac::bool_var* x4 = n.new_bool();
		ac::bool_var* x5 = n.new_bool();
		ac::bool_var* x6 = n.new_bool();
		ac::bool_var* x7 = n.new_bool();
		ac::bool_var* x8 = n.new_bool();

		bool added;
		added = n.add({ n.disjunction({ x0, x1 }) });
		Assert::IsTrue(added);
		added = n.add({ n.disjunction({ x0, x2, x6 }) });
		Assert::IsTrue(added);
		added = n.add({ n.disjunction({ n.negate(x1), n.negate(x2), x3 }) });
		Assert::IsTrue(added);
		added = n.add({ n.disjunction({ n.negate(x3), x4, x7 }) });
		Assert::IsTrue(added);
		added = n.add({ n.disjunction({ n.negate(x3), x5, x8 }) });
		Assert::IsTrue(added);
		added = n.add({ n.disjunction({ n.negate(x4), n.negate(x5) }) });
		Assert::IsTrue(added);

		bool assigned;
		assigned = n.assign_true(n.negate(x6));
		Assert::IsTrue(assigned);
		assigned = n.assign_true(n.negate(x7));
		Assert::IsTrue(assigned);
		assigned = n.assign_true(n.negate(x8));
		Assert::IsTrue(assigned);

		// this assignment should create an inconsistency..
		assigned = n.assign_true(n.negate(x1));
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