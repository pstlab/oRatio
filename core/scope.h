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

#include <string>
#include <unordered_map>
#include <vector>

#define THIS_KEYWORD "this"
#define RETURN_KEYWORD "return"

namespace oratio {

	class core;
	class field;
	class method;
	class type;
	class predicate;
	class atom;

	class scope {
		friend class type_refinement_listener;
	public:
		scope(core * const c, scope * const s);
		scope(scope&&) = delete;
		virtual ~scope();

		virtual field * get_field(const std::string& name) const;
		std::unordered_map<std::string, field*> get_fields() const;

		virtual method * get_method(const std::string& name, const std::vector<const type*>& ts) const {
			return _scope->get_method(name, ts);
		}

		virtual std::vector<method*> get_methods() const {
			return _scope->get_methods();
		}

		virtual type * get_type(const std::string& name) const {
			return _scope->get_type(name);
		}

		virtual std::unordered_map<std::string, type*> get_types() const {
			return _scope->get_types();
		}

		virtual predicate * get_predicate(const std::string& name) const {
			return _scope->get_predicate(name);
		}

		virtual std::unordered_map<std::string, predicate*> get_predicates() const {
			return _scope->get_predicates();
		}

		virtual bool fact_created(atom * const a) {
			return true;
		}

		virtual bool goal_created(atom * const a) {
			return true;
		}
	public:
		core * const _core;
		scope * const _scope;
	protected:
		std::unordered_map<std::string, field*> _fields;
	};
}

