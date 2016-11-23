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

#include "var.h"
#include <vector>

namespace ac {

	class network;

	class propagator {
		friend class network;
	public:
		propagator(propagator&&) = delete;
		virtual ~propagator();

		std::vector<var*> get_vars();
	protected:
		network* const _net;
		std::vector<var*> _vars;

		propagator(network* const n);

		template<typename T>
		bool complement(enum_var<T>* v, const std::unordered_set<T>& vals) {
			return v->complement(vals, this);
		}

		template<typename T>
		bool exclude_val(enum_var<T>* v, const T& val) {
			return v->complement({ val }, this);
		}

		template<typename T>
		bool intersect(enum_var<T>* v, const std::unordered_set<T>& vals) {
			return v->intersect(vals, this);
		}

		template<typename T>
		bool assign_val(enum_var<T>* v, const T& val) {
			return v->intersect({ val }, this);
		}

		bool intersect(arith_var* v, const interval& i) {
			return v->intersect(i, this);
		}
	private:
		virtual bool propagate(const var * const v) = 0;
	};
}
