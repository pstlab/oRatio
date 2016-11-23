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

#include "minus_propagator.h"

using namespace ac;

minus_propagator::minus_propagator(network * const net, arith_var * const v) : propagator(net), _var(v), _minus(evaluate(net, v)) {
	assert(evaluate(_var).consistent());
	_vars.push_back(_var);
	_vars.push_back(_minus);
	if (!_net->root_level()) {
		bool i = intersect(_minus, evaluate(_var));
		assert(i);
	}
}

minus_propagator::~minus_propagator() {
	delete _minus;
}

bool minus_propagator::propagate(const var * const v) {
	if (v == _var) {
		interval val = -_var->to_interval();
		return intersect(_minus, val);
	}
	else {
		interval val = -_minus->to_interval();
		return intersect(_var, val);
	}
}

interval minus_propagator::evaluate(arith_var * const v) {
	return -v->to_interval();
}

arith_var* minus_propagator::evaluate(network * const net, arith_var * const v) {
	if (net->root_level()) {
		return new arith_var(net, "-" + v->_name, evaluate(v), -v->_expr);
	}
	else {
		return new arith_var(net, "-" + v->_name, interval(), -v->_expr);
	}
}