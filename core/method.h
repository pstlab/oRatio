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

namespace oratio {

	class env;
	class item;

	class method : public scope {
		friend class core;
		friend class type;
	public:
		method(core * const c, scope * const s, const std::string& name, const std::vector<field*>& args, const type * const return_type = nullptr);
		method(method&&) = delete;
		virtual ~method();

		virtual bool invoke(env * const e, const std::vector<item*>& exprs) = 0;
	public:
		const std::string _name;
	protected:
		const std::vector<field*> _args;
	public:
		const type * const _return_type;
	};
}

