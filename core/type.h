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

#include "scope.h"
#include <unordered_set>

#define BOOL_KEYWORD "bool"
#define INT_KEYWORD "int"
#define REAL_KEYWORD "real"
#define STRING_KEYWORD "string"

namespace oratio {

	class item;
	class env;
	class constructor;

	class type : public scope {
		friend class item;
		friend class type_declaration_listener;
		friend class type_refinement_listener;
	public:
		type(core * const c, scope * const s, const std::string& name, bool primitive = false);
		type(type&&) = delete;
		virtual ~type();

		std::vector<type*> get_supertypes() const {
			return _supertypes;
		}

		bool is_assignable_from(const type& t) const;

		virtual item * new_instance(env * const e);
		virtual item * new_existential();

		std::unordered_set<item*> get_instances() const {
			return _instances;
		}

		constructor * get_constructor(const std::vector<const type*>& ts) const;

		std::vector<constructor*> get_constructors() const {
			return _constructors;
		}

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
		const std::string _name;
		const bool _primitive;
	protected:
		std::vector<type*> _supertypes;
		std::vector<constructor*> _constructors;
		std::unordered_map<std::string, std::vector<method*>> _methods;
		std::unordered_map<std::string, type*> _types;
		std::unordered_map<std::string, predicate*> _predicates;
		std::unordered_set<item*> _instances;
	protected:

		virtual void predicate_defined(predicate * const p) { }
	};

	class bool_type : public type {
	public:
		bool_type(core * const c);
		bool_type(bool_type&&) = delete;
		virtual ~bool_type();

		item* new_instance(env * const e) override;
	};

	class int_type : public type {
	public:
		int_type(core * const c);
		int_type(int_type&&) = delete;
		virtual ~int_type();

		item* new_instance(env * const e) override;
	};

	class real_type : public type {
	public:
		real_type(core * const c);
		real_type(real_type&&) = delete;
		virtual ~real_type();

		item* new_instance(env * const e) override;
	};

	class string_type : public type {
	public:
		string_type(core * const c);
		string_type(string_type&&) = delete;
		virtual ~string_type();

		item* new_instance(env * const e) override;
	};
}

