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

	class minus_propagator : public propagator {
		friend class network;
	public:
		minus_propagator(minus_propagator&&) = delete;
		virtual ~minus_propagator();

		static interval evaluate(const arith_var * const v);
	private:
		arith_var * const _var;
		arith_var * const _minus;

		minus_propagator(network * const net, arith_var * const v);
		bool propagate(const var * const v) override;
		static arith_var * evaluate(network * const net, const arith_var * const v);
		static std::string to_string(const arith_var * const v);
	};
}

