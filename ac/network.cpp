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

#include "network.h"
#include "propagator.h"
#include "not_propagator.h"
#include "and_propagator.h"
#include "or_propagator.h"
#include "exct_one_propagator.h"
#include "minus_propagator.h"
#include "sum_propagator.h"
#include "product_propagator.h"
#include "div_propagator.h"
#include "lt_propagator.h"
#include "leq_propagator.h"
#include "arith_eq_propagator.h"
#include "geq_propagator.h"
#include "gt_propagator.h"
#include "enum_eq_propagator.h"

using namespace ac;

network::network() : _context(new z3::context()), _solver(new z3::solver(*_context)), _state(_solver->check()), _model(_solver->get_model()) {}

network::~network() {}

void network::store(propagator * const p) {
	for (const auto& arg : p->_vars) {
		if (!arg->singleton()) {
			if (_watches.find(arg) == _watches.end()) {
				_watches.insert({ arg, std::list<propagator*>() });
			}
			_watches.at(arg).push_back(p);
		}
	}
}

void network::forget(propagator * const p) {
	for (const auto& arg : p->_vars) {
		if (_watches.find(arg) == _watches.end()) {
			_watches.at(arg).remove(p);
			if (_watches.at(arg).empty()) {
				_watches.erase(arg);
			}
		}
	}
}

bool_var * network::new_bool() {
	std::string name("b" + std::to_string(_vars.size()));
	bool_var* bv = new bool_var(this, name, { true, false });
	_vars.insert({ name,bv });
	return bv;
}

bool_var * network::new_bool(bool value) {
	return new bool_var(this, value ? "true" : "false", { value });
}

arith_var* network::new_int() {
	std::string name("x" + std::to_string(_vars.size()));
	arith_var* av = new arith_var(this, name, interval(), _context->int_const(name.c_str()));
	_vars.insert({ name,av });
	return av;
}

arith_var* network::new_int(long value) {
	return new arith_var(this, value);
}

arith_var* network::new_real() {
	std::string name("x" + std::to_string(_vars.size()));
	arith_var* av = new arith_var(this, name, interval(), _context->real_const(name.c_str()));
	_vars.insert({ name,av });
	return av;
}

arith_var* network::new_real(double value) {
	return new arith_var(this, value);
}

bool_var* network::negate(bool_var * const var) {
	if (_vars.find(not_propagator::to_string(var)) != _vars.end()) {
		return static_cast<bool_var*>(_vars.at(not_propagator::to_string(var)));
	}
	not_propagator* prop = new not_propagator(this, var);
	store(prop);
	_vars.insert({ prop->_not->_name,prop->_not });
	return prop->_not;
}

bool_var* network::conjunction(const std::vector<bool_var*>& vars) {
	if (_vars.find(and_propagator::to_string(vars)) != _vars.end()) {
		return static_cast<bool_var*>(_vars.at(and_propagator::to_string(vars)));
	}
	and_propagator* prop = new and_propagator(this, vars);
	store(prop);
	_vars.insert({ prop->_and->_name,prop->_and });
	return prop->_and;
}

bool_var* network::disjunction(const std::vector<bool_var*>& vars) {
	if (_vars.find(or_propagator::to_string(vars)) != _vars.end()) {
		return static_cast<bool_var*>(_vars.at(or_propagator::to_string(vars)));
	}
	or_propagator* prop = new or_propagator(this, vars);
	store(prop);
	_vars.insert({ prop->_or->_name,prop->_or });
	return prop->_or;
}

bool_var* network::exactly_one(const std::vector<bool_var*>& vars) {
	if (_vars.find(exct_one_propagator::to_string(vars)) != _vars.end()) {
		return static_cast<bool_var*>(_vars.at(exct_one_propagator::to_string(vars)));
	}
	exct_one_propagator* prop = new exct_one_propagator(this, vars);
	store(prop);
	_vars.insert({ prop->_exct_one->_name,prop->_exct_one });
	return prop->_exct_one;
}

arith_var* network::minus(arith_var * const var) {
	if (_vars.find(minus_propagator::to_string(var)) != _vars.end()) {
		return static_cast<arith_var*>(_vars.at(minus_propagator::to_string(var)));
	}
	minus_propagator* prop = new minus_propagator(this, var);
	store(prop);
	_vars.insert({ prop->_minus->_name,prop->_minus });
	return prop->_minus;
}

arith_var* network::sum(const std::vector<arith_var*>& vars) {
	if (_vars.find(sum_propagator::to_string(vars)) != _vars.end()) {
		return static_cast<arith_var*>(_vars.at(sum_propagator::to_string(vars)));
	}
	sum_propagator* prop = new sum_propagator(this, vars);
	store(prop);
	_vars.insert({ prop->_sum->_name,prop->_sum });
	return prop->_sum;
}

arith_var* network::sub(const std::vector<arith_var*>& vars) {
	std::vector<arith_var*> c_vars(vars.size());
	c_vars.push_back(vars.at(0));
	for (unsigned int i = 1; i < vars.size(); i++) {
		c_vars.push_back(minus(vars.at(i)));
	}
	if (_vars.find(sum_propagator::to_string(c_vars)) != _vars.end()) {
		return static_cast<arith_var*>(_vars.at(sum_propagator::to_string(c_vars)));
	}
	sum_propagator* prop = new sum_propagator(this, c_vars);
	store(prop);
	_vars.insert({ prop->_sum->_name,prop->_sum });
	return prop->_sum;
}

arith_var* network::mult(const std::vector<arith_var*>& vars) {
	if (_vars.find(product_propagator::to_string(vars)) != _vars.end()) {
		return static_cast<arith_var*>(_vars.at(product_propagator::to_string(vars)));
	}
	product_propagator* prop = new product_propagator(this, vars);
	store(prop);
	_vars.insert({ prop->_prod->_name,prop->_prod });
	return prop->_prod;
}

arith_var* network::div(arith_var * const var0, arith_var * const var1) {
	if (_vars.find(div_propagator::to_string(var0, var1)) != _vars.end()) {
		return static_cast<arith_var*>(_vars.at(div_propagator::to_string(var0, var1)));
	}
	div_propagator* prop = new div_propagator(this, var0, var1);
	store(prop);
	_vars.insert({ prop->_div->_name,prop->_div });
	return prop->_div;
}

bool_var* network::lt(arith_var * const var0, arith_var * const var1) {
	if (_vars.find(lt_propagator::to_string(var0, var1)) != _vars.end()) {
		return static_cast<bool_var*>(_vars.at(lt_propagator::to_string(var0, var1)));
	}
	lt_propagator* prop = new lt_propagator(this, var0, var1);
	store(prop);
	_vars.insert({ prop->_lt->_name,prop->_lt });
	return prop->_lt;
}

bool_var* network::leq(arith_var * const var0, arith_var * const var1) {
	if (_vars.find(leq_propagator::to_string(var0, var1)) != _vars.end()) {
		return static_cast<bool_var*>(_vars.at(leq_propagator::to_string(var0, var1)));
	}
	leq_propagator* prop = new leq_propagator(this, var0, var1);
	store(prop);
	_vars.insert({ prop->_leq->_name,prop->_leq });
	return prop->_leq;
}

bool_var* network::eq(arith_var * const var0, arith_var * const var1) {
	if (_vars.find(arith_eq_propagator::to_string(var0, var1)) != _vars.end()) {
		return static_cast<bool_var*>(_vars.at(arith_eq_propagator::to_string(var0, var1)));
	}
	arith_eq_propagator* prop = new arith_eq_propagator(this, var0, var1);
	store(prop);
	_vars.insert({ prop->_eq->_name,prop->_eq });
	return prop->_eq;
}

bool_var* network::geq(arith_var * const var0, arith_var * const var1) {
	if (_vars.find(geq_propagator::to_string(var0, var1)) != _vars.end()) {
		return static_cast<bool_var*>(_vars.at(geq_propagator::to_string(var0, var1)));
	}
	geq_propagator* prop = new geq_propagator(this, var0, var1);
	store(prop);
	_vars.insert({ prop->_geq->_name,prop->_geq });
	return prop->_geq;
}

bool_var* network::gt(arith_var * const var0, arith_var * const var1) {
	if (_vars.find(gt_propagator::to_string(var0, var1)) != _vars.end()) {
		return static_cast<bool_var*>(_vars.at(gt_propagator::to_string(var0, var1)));
	}
	gt_propagator* prop = new gt_propagator(this, var0, var1);
	store(prop);
	_vars.insert({ prop->_gt->_name,prop->_gt });
	return prop->_gt;
}

void network::push() {
	_solver->push();
	_layers.push(new layer(nullptr));
}

void network::pop() {
	std::queue<var*>().swap(_prop_q);
	_solver->pop();
	for (const auto& d : _layers.top()->_domains) {
		d.first->restore();
	}
	delete _layers.top();
	_layers.pop();
}

bool network::add(const std::vector<bool_var*>& exprs) {
	for (const auto& var : exprs) {
		if (!var->intersect({ true }, nullptr)) {
			return false;
		}
	}

	while (!_prop_q.empty()) {
		var* v = _prop_q.front();
		_prop_q.pop();
		propagator* cause = _causes.at(v);
		_causes.erase(v);
		for (const auto& p : _watches.at(v)) {
			if (p != cause) {
				if (!p->propagate(v)) {
					// we use the conflict 'p' to generate the no-good..
					assert(_unsat_core.empty());

					std::unordered_set<var*> visited;
					std::queue<var*> q;
					for (const auto& c_v : p->_vars) {
						q.push(c_v);
					}

					while (!q.empty()) {
						var* c_v = q.front();
						if (visited.find(c_v) == visited.end()) {
							visited.insert(c_v);
							if (_layers.top()->_impl_graph.find(c_v) != _layers.top()->_impl_graph.end()) {
								// the variable is in the implication graph..
								for (const auto& c_p : _layers.top()->_impl_graph.at(c_v)) {
									for (const auto& c_c_v : c_p->_vars) {
										if (visited.find(c_c_v) == visited.end()) {
											q.push(c_c_v);
										}
									}
								}
							}
							else if (_reason.find(c_v) != _reason.end()) {
								// the variable has been assigned at a previous level..
								_unsat_core.push_back(_reason.at(c_v));
							}
						}
						q.pop();
					}
					return false;
				}
			}
		}
		if (v->singleton() && root_level()) {
			_watches.erase(v);
		}
	}
	return true;
}

bool network::add(const z3::expr& e) {
	_solver->add(e);
	_state = _solver->check();
	switch (_state) {
		case z3::unsat:
		{
			if (!root_level()) {
				z3::expr_vector ev = _solver->unsat_core();
				for (unsigned int i = 0; i < ev.size(); i++) {
					_unsat_core.push_back(_smt_map.at(&ev[i]));
				}
			}
			return false;
		}
		case z3::sat:
		{
			_model = _solver->get_model();
			return true;
		}
		default:
		{
			std::cerr << "invalid smt solver state.." << std::endl;
			return false;
		}
	}
}

bool network::assign_true(bool_var* choice_var) {
	_solver->push();
	_layers.push(new layer(choice_var));
	_unsat_core.clear();
	bool prop = add({ choice_var });
	return prop;
}

size_t network::relevance(var * const v) {
	if (_watches.find(v) != _watches.end()) {
		assert(!_watches.at(v).empty());
		return _watches.at(v).size();
	}
	else {
		return 0;
	}
}

void network::enqueue(var * const v, domain * const d, propagator * const p) {
	if (!root_level()) {
		if (_layers.top()->_domains.find(v) == _layers.top()->_domains.end()) {
			_layers.top()->_domains.insert({ v, d });
		}
		if (p) {
			if (_layers.top()->_impl_graph.find(v) == _layers.top()->_impl_graph.end()) {
				_layers.top()->_impl_graph.insert({ v, std::vector<propagator*>() });
			}
			_layers.top()->_impl_graph.at(v).push_back(p);
		}
		if (v->singleton()) {
			_reason.insert({ v, _layers.top()->_choice_var });
		}
	}
	if (_watches.find(v) != _watches.end()) {
		_prop_q.push(v);
		_causes.insert({ v, p });
	}
}
