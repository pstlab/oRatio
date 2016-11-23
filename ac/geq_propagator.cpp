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

/* 
 * File:   geq_propagator.cpp
 * Author: Riccardo De Benedictis <riccardo.debenedictis@istc.cnr.it>
 * 
 * Created on November 8, 2016, 3:52 PM
 */

#include "geq_propagator.h"

using namespace ac;

geq_propagator::geq_propagator(network * const net, arith_var * const left, arith_var * const right) : propagator(net), _left(left), _right(right), _geq(evaluate(net, left, right)), _geq_expr(net->_context->bool_const(to_string(left, right).c_str())) {
    assert(!evaluate(_left, _right).empty());
    _net->_solver->add(_geq_expr == (_left->_expr >= _right->_expr));
    _vars.push_back(_left);
    _vars.push_back(_right);
    _vars.push_back(_geq);
    if (!_net->root_level()) {
        bool i = intersect(_geq, evaluate(_left, _right));
        assert(i);
    }
}

geq_propagator::~geq_propagator() {
    delete _geq;
}

bool geq_propagator::propagate(const var * const v) {
    if (v == _geq) {
        if (_geq->allows(true)) {
            if (!_net->add(_geq_expr)) {
                return exclude_val(_geq, true);
            }
            interval c_left(_right->to_interval().lb, std::numeric_limits<double>::infinity());
            interval c_right(-std::numeric_limits<double>::infinity(), _left->to_interval().ub);
            if (!intersect(_left, c_left) || !intersect(_right, c_right)) {
                throw std::logic_error("smt solver is inconsistent with arc-consistency propagators..");
            }
        } else {
            if (!_net->add(!_geq_expr)) {
                return exclude_val(_geq, false);
            }
            interval c_left(-std::numeric_limits<double>::infinity(), _right->to_interval().lb);
            interval c_right(_left->to_interval().lb, std::numeric_limits<double>::infinity());
            if (!intersect(_left, c_left) || !intersect(_right, c_right)) {
                throw std::logic_error("smt solver is inconsistent with arc-consistency propagators..");
            }
        }
    } else if (_left->to_interval() >= _right->to_interval()) {
        // The constraint is already satisfied..
        if (!assign_val(_geq, true)) {
            return false;
        }
    } else if (_left->to_interval() < _right->to_interval()) {
        // The constraint cannot be satisfied..
        if (!assign_val(_geq, false)) {
            return false;
        }
    } else if (_geq->singleton()) {
        if (v == _left) {
            if (_geq->allows(true)) {
                interval c_right(-std::numeric_limits<double>::infinity(), _left->to_interval().ub);
                if (!intersect(_right, c_right)) {
                    throw std::logic_error("smt solver is inconsistent with arc-consistency propagators..");
                }
            } else {
                interval c_right(_left->to_interval().lb, std::numeric_limits<double>::infinity());
                if (!intersect(_right, c_right)) {
                    throw std::logic_error("smt solver is inconsistent with arc-consistency propagators..");
                }
            }
        } else if (v == _right) {
            if (_geq->allows(true)) {
                interval c_left(_right->to_interval().lb, std::numeric_limits<double>::infinity());
                if (!intersect(_left, c_left)) {
                    throw std::logic_error("smt solver is inconsistent with arc-consistency propagators..");
                }
            } else {
                interval c_left(-std::numeric_limits<double>::infinity(), _right->to_interval().lb);
                if (!intersect(_left, c_left)) {
                    throw std::logic_error("smt solver is inconsistent with arc-consistency propagators..");
                }
            }
        }
    }
    return true;
}

std::unordered_set<bool> geq_propagator::evaluate(arith_var * const left, arith_var * const right) {
    if (left->to_interval() >= right->to_interval()) {
        return {true};
    } else if (left->to_interval() < right->to_interval()) {
        return {false};
    } else {
        return {true, false};
    }
}

bool_var* geq_propagator::evaluate(network * const net, arith_var * const left, arith_var * const right) {
    if (net->root_level()) {
        return new bool_var(net, to_string(left, right), evaluate(left, right));
    } else {
        return new bool_var(net, to_string(left, right),{true, false});
    }
}

std::string geq_propagator::to_string(arith_var * const left, arith_var * const right) {
    return left->_name + " >= " + right->_name;
}