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
#include <string>

namespace oratio {

	class core;
	class item;

	class env {
		friend class default_constructor;
		friend class defined_constructor;
		friend class defined_method;
		friend class defined_predicate;
		friend class defined_conjunction;
		friend class statement_visitor;
	public:
		env(core * const c, env * const e);
		env(env&&) = delete;
		virtual ~env();

		virtual item* get(const std::string& name) const;
		std::unordered_map<std::string, item*> get_items() const;
	public:
		core * const _core;
		env * const _env;
	protected:
		std::unordered_map<std::string, item*> _items;
	};
}

