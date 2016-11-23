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

#include "parser/parser.h"
#include "scope.h"
#include "env.h"
#include "../ac/network.h"

namespace oratio {

	class item;
	class bool_item;
	class arith_item;
	class enum_item;
	class string_item;
	class disjunction;

	class core : public parser, public scope, public env {
		friend class type_declaration_listener;
		friend class type_refinement_listener;
	public:
		core();
		core(core&&) = delete;
		virtual ~core();

		bool_item * new_bool();
		bool_item * new_bool(bool value);
		arith_item * new_int();
		arith_item * new_int(long value);
		arith_item * new_real();
		arith_item * new_real(double value);
		virtual enum_item* new_enum(const type * const t, const std::unordered_set<item*>& allowed_vals);
		string_item * new_string();
		string_item * new_string(std::string& val);

		bool_item* negate(bool_item * const var);
		bool_item* conjunction(const std::vector<bool_item*>& vars);
		bool_item* disjunction(const std::vector<bool_item*>& vars);
		bool_item* exactly_one(const std::vector<bool_item*>& vars);
		arith_item* minus(arith_item * const var);
		arith_item* sum(const std::vector<arith_item*>& vars);
		arith_item* sub(const std::vector<arith_item*>& vars);
		arith_item* mult(const std::vector<arith_item*>& vars);
		arith_item* div(arith_item * const var0, arith_item * const var1);
		bool_item* lt(arith_item * const var0, arith_item * const var1);
		bool_item* leq(arith_item * const var0, arith_item * const var1);
		bool_item* eq(arith_item * const var0, arith_item * const var1);
		bool_item* geq(arith_item * const var0, arith_item * const var1);
		bool_item* gt(arith_item * const var0, arith_item * const var1);

		bool_item * eq(item * i0, item * i1);

		bool assert_facts(const std::vector<bool_item*>& exprs);

		virtual bool new_fact(atom * const a) = 0;
		virtual bool new_goal(atom * const a) = 0;
		virtual bool new_disjunction(env * const e, oratio::disjunction * const d) = 0;

		field * get_field(const std::string& name) const override;
		method * get_method(const std::string& name, const std::vector<const type*>& ts) const override;

		std::vector<method*> get_methods() const override {
			std::vector<method*> methods;
			for (const auto& ms : _methods) {
				methods.insert(methods.begin(), ms.second.begin(), ms.second.end());
			}
			return methods;
		}

		predicate * get_predicate(const std::string& name) const override;

		std::unordered_map<std::string, predicate*> get_predicates() const override {
			return _predicates;
		}

		type * get_type(const std::string& name) const override;

		std::unordered_map<std::string, type*> get_types() const override {
			return _types;
		}
	public:
		ac::network _net;
	protected:
		ac::bool_var* _ctr_var;
		std::unordered_map<std::string, std::vector<method*>> _methods;
		std::unordered_map<std::string, type*> _types;
		std::unordered_map<std::string, predicate*> _predicates;
	};
}

