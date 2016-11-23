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

#include "product_propagator.h"

using namespace ac;

product_propagator::product_propagator(network * const net, const std::vector<arith_var*> vars) : propagator(net), _vars(vars), _prod(evaluate(net, vars)) {
	assert(evaluate(_vars).consistent());
	for (const auto& i : _vars) {
		propagator::_vars.push_back(i);
	}
	propagator::_vars.push_back(_prod);
	if (!_net->root_level()) {
		bool i = intersect(_prod, evaluate(_vars));
		assert(i);
	}
}

product_propagator::~product_propagator() {
	delete _prod;
}

bool product_propagator::propagate(const var * const v) {
	if (v != _prod) {
		// we update the current bounds..
		interval s(1);
		for (const auto& i : _vars) {
			s *= i->to_interval();
		}
		if (!intersect(_prod, s)) {
			return false;
		}
	}
	for (const auto& v0 : _vars) {
		if (v != v0) {
			// we update other variables bounds..
			interval c_i(_prod->to_interval());
			for (const auto& v1 : _vars) {
				if (v0 != v1) {
					c_i /= v1->to_interval();
				}
			}
			if (!intersect(v0, c_i)) {
				return false;
			}
		}
	}
	return true;
}

interval product_propagator::evaluate(const std::vector<arith_var*>& vars) {
	interval s(1);
	for (const auto& i : vars) {
		s *= i->to_interval();
	}
	return s;
}

arith_var* product_propagator::evaluate(network * const net, const std::vector<arith_var*>& vars) {
	Z3_ast* args = new Z3_ast[vars.size()];
	for (unsigned int i = 0; i < vars.size(); i++) {
		args[i] = vars.at(i)->_expr;
	}
	if (net->root_level()) {
		return new arith_var(net, to_string(vars), evaluate(vars), z3::expr(*net->_context, Z3_mk_mul(*net->_context, vars.size(), args)));
	}
	else {
		return new arith_var(net, to_string(vars), interval(), z3::expr(*net->_context, Z3_mk_mul(*net->_context, vars.size(), args)));
	}
}

std::string product_propagator::to_string(const std::vector<arith_var*>& vars) {
	std::string s("(");
	for (const auto& var : vars) {
		s.append(var->_name);
		s.append(" * ");
	}
	s.pop_back();
	s.pop_back();
	s.pop_back();
	s.append(")");
	return s;
}