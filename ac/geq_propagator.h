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

#include "propagator.h"

namespace ac {

	class geq_propagator : public propagator {
		friend class network;
	public:
		geq_propagator(geq_propagator&&) = delete;
		virtual ~geq_propagator();

		static std::unordered_set<bool> evaluate(arith_var * const left, arith_var * const right);
	private:
		arith_var * const _left;
		arith_var * const _right;
		bool_var * const _geq;
		z3::expr _geq_expr;

		geq_propagator(network * const net, arith_var * const left, arith_var * const right);
		bool propagate(const var * const v) override;
		static bool_var * evaluate(network * const net, arith_var * const left, arith_var * const right);
		static std::string to_string(arith_var * const left, arith_var * const right);
	};
}

