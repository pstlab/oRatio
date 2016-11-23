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
 * File:   div_propagator.cpp
 * Author: Riccardo De Benedictis <riccardo.debenedictis@istc.cnr.it>
 * 
 * Created on November 8, 2016, 3:27 PM
 */

#include "div_propagator.h"

using namespace ac;

div_propagator::div_propagator(network * const net, arith_var * const left, arith_var * const right) : propagator(net), _left(left), _right(right), _div(evaluate(net, left, right)) {
    assert(evaluate(_left, _right).consistent());
    _vars.push_back(_left);
    _vars.push_back(_right);
    _vars.push_back(_div);
    if (!_net->root_level()) {
        bool i = intersect(_div, evaluate(_left, _right));
        assert(i);
    }
}

div_propagator::~div_propagator() {
    delete _div;
}

bool div_propagator::propagate(const var * const v) {
    if (v == _left) {
        interval c_div = _left->to_interval() / _right->to_interval();
        interval c_right = _left->to_interval() / _div->to_interval();
        if (!intersect(_div, c_div) || !intersect(_right, c_right)) {
            return false;
        }
    } else if (v == _right) {
        interval c_div = _left->to_interval() / _right->to_interval();
        interval c_left = _right->to_interval() * _div->to_interval();
        if (!intersect(_div, c_div) || !intersect(_left, c_left)) {
            return false;
        }
    } else if (v == _div) {
        interval c_left = _right->to_interval() * _div->to_interval();
        interval c_right = _left->to_interval() / _div->to_interval();
        if (!intersect(_left, c_left) || !intersect(_right, c_right)) {
            return false;
        }
    }
    return true;
}

interval div_propagator::evaluate(arith_var * const left, arith_var * const right) {
    return left->to_interval() / right->to_interval();
}

arith_var* div_propagator::evaluate(network * const net, arith_var * const left, arith_var * const right) {
    if (net->root_level()) {
        return new arith_var(net, to_string(left, right), evaluate(left, right), left->_expr / right->_expr);
    } else {
        return new arith_var(net, to_string(left, right), interval(), left->_expr / right->_expr);
    }
}

std::string div_propagator::to_string(arith_var * const left, arith_var * const right) {
    return +"(" + left->_name + " / " + right->_name + ")";
}