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
#include "../ac/network.h"

namespace oratio {

	class conjunction;
	class env;

	class disjunction : public scope {
	public:
		disjunction(core * const c, scope * const s);
		disjunction(disjunction&&) = delete;
		virtual ~disjunction();

		std::vector<conjunction*> get_conjunctions() const {
			return _conjunctions;
		}
	private:
		std::vector<conjunction*> _conjunctions;
	};

	class conjunction : public scope {
	public:
		conjunction(core * const c, scope * const s, ac::arith_var& cst);
		conjunction(conjunction&&) = delete;
		virtual ~conjunction();

		ac::arith_var& get_cost() const {
			return _cost;
		}

		virtual bool apply(env * const e) const = 0;
	private:
		ac::arith_var& _cost;
	};
}

