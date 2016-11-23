/*
* Copyright (C) 2016 Riccardo De Benedictis <riccardo.debenedictis@istc.cnr.it>
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#pragma once

#include <unordered_map>
#include <unordered_set>
#include <stack>
#include <list>
#include <queue>
#include <z3++.h>

namespace ac {

	class domain;
	class var;
	class propagator;
	template<typename T>
	class enum_var;
	typedef enum_var<bool> bool_var;
	class arith_var;
	template<typename T>
	class enum_eq_propagator;

	class network {
		template<typename T>
		friend class enum_var;
		friend class arith_var;
		friend class sum_propagator;
		friend class product_propagator;
		friend class div_propagator;
		friend class lt_propagator;
		friend class leq_propagator;
		friend class arith_eq_propagator;
		friend class geq_propagator;
		friend class gt_propagator;
	public:
		network();
		network(network&&) = delete;
		virtual ~network();

		void store(propagator* const p);
		void forget(propagator* const p);

		bool_var* new_bool();
		bool_var* new_bool(bool value);
		arith_var* new_int();
		arith_var* new_int(long value);
		arith_var* new_real();
		arith_var* new_real(double value);

		template<typename T>
		enum_var<T>* new_enum(const std::unordered_set<T>& allowed_vals) {
			return new enum_var<T>(this, "e" + std::to_string(n_vars++), allowed_vals);
		}

		bool_var* negate(bool_var * const var);
		bool_var* conjunction(const std::vector<bool_var*>& vars);
		bool_var* disjunction(const std::vector<bool_var*>& vars);
		bool_var* exactly_one(const std::vector<bool_var*>& vars);

		bool_var* imply(bool_var * const var0, bool_var * const var1) {
			return disjunction(std::vector<bool_var*>({ negate(var0), var1 }));
		}

		arith_var* minus(arith_var * const var);
		arith_var* sum(const std::vector<arith_var*>& vars);
		arith_var* sub(const std::vector<arith_var*>& vars);
		arith_var* mult(const std::vector<arith_var*>& vars);
		arith_var* div(arith_var * const var0, arith_var * const var1);
		bool_var* lt(arith_var * const var0, arith_var * const var1);
		bool_var* leq(arith_var * const var0, arith_var * const var1);
		bool_var* eq(arith_var * const var0, arith_var * const var1);
		bool_var* geq(arith_var * const var0, arith_var * const var1);
		bool_var* gt(arith_var * const var0, arith_var * const var1);

		template<typename T>
		enum_var<bool>* eq(enum_var<T> * const var0, enum_var<T> * const var1) {
			enum_eq_propagator<T>* prop = new enum_eq_propagator<T>(this, var0, var1);
			return prop->_eq;
		}

		template<typename T>
		enum_var<bool>* eq(enum_var<T> * const var0, const T& val) {
			enum_eq_propagator<T>* prop = new enum_eq_propagator<T>(this, var0, new_enum<T>(std::unordered_set<T>({ val })));
			return prop->_eq;
		}

		void push();
		void pop();
		bool add(const std::vector<bool_var*>& exprs);
		bool assign_true(bool_var* var);

		domain* get_domain(var*const v) {
			return _layers.top()->_domains.at(v);
		}

		bool root_level() const {
			return _layers.empty();
		}

		size_t level() {
			return _layers.size();
		}

		std::vector<bool_var*> get_unsat_core() {
			return _unsat_core;
		}
	private:
		unsigned int n_vars = 0;
		std::unordered_map<var*, std::list<propagator*>> _watches;
		std::queue<var*> _prop_q;
		std::unordered_map<var*, propagator*> _causes;
		std::vector<bool_var*> _unsat_core;

		struct layer {

			layer(bool_var * const cv) : _choice_var(cv) { }

			bool_var * const _choice_var;
			std::unordered_map<var*, domain*> _domains;
			std::unordered_map<var*, std::vector<propagator*>> _impl_graph;
		};
		std::stack<layer*> _layers;

		bool enqueue(var * const v, domain * const d, propagator * const p);

		z3::context * const _context;
		z3::solver * const _solver;
		z3::check_result _state;
		z3::model _model;
		std::unordered_map<z3::expr*, bool_var*> _smt_map;

		bool add(const z3::expr& e);
	};
}

