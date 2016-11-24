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

#include "item.h"
#include "../ac/network.h"

namespace oratio {

	class predicate;

	class atom_state {};

	static atom_state active, inactive, unified;

	class atom : public item {
	public:
		atom(core * const c, env * const e, const predicate * const p);
		atom(atom&&) = delete;
		virtual ~atom();

		ac::enum_var<atom_state*>& get_state() const {
			return _state;
		}
	private:
		ac::enum_var<atom_state*>& _state;
	};
}

