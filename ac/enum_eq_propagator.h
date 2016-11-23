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
 * File:   enum_eq_propagator.h
 * Author: Riccardo De Benedictis <riccardo.debenedictis@istc.cnr.it>
 *
 * Created on November 8, 2016, 3:30 PM
 */

#ifndef ENUM_EQ_PROPAGATOR_H
#define ENUM_EQ_PROPAGATOR_H

#include "propagator.h"

namespace ac {

    template<typename T>
    class enum_eq_propagator : public propagator {
        friend class network;
    public:

        enum_eq_propagator(network * const net, enum_var<T> * const left, enum_var<T> * const right) : propagator(net), _left(left), _right(right), _eq(evaluate(net, left, right)) {
            assert(!evaluate(_left, _right).empty());
            _vars.push_back(_left);
            _vars.push_back(_right);
            _vars.push_back(_eq);
            if (!_net->root_level()) {
                bool i = intersect(_eq, evaluate(_left, _right));
                assert(i);
            }
        }

        enum_eq_propagator(enum_eq_propagator&&) = delete;

        virtual ~enum_eq_propagator() {
            delete _eq;
        }

        bool propagate(const var * const v) override {
            if (_eq->singleton()) {
                if (_eq->allows(true)) {
                    // The constraint must be satisfied..
                    if (!intersect(_left, _right->to_vals())) {
                        return false;
                    }
                    if (!intersect(_right, _left->to_vals())) {
                        return false;
                    }
                } else {
                    // The constraint must be not satisfied..
                    if (_right->to_vals().size() == 1 && !complement(_left, _right->to_vals())) {
                        return false;
                    }
                    if (_left->to_vals().size() == 1 && !complement(_right, _left->to_vals())) {
                        return false;
                    }
                }
            } else {
                bool intersecting = false;
                for (const auto& i : _left->to_vals()) {
                    if (_right->to_vals().find(i) != _right->to_vals().end()) {
                        intersecting = true;
                        break;
                    }
                }
                if (!intersecting) {
                    // The constraint cannot be satisfied..
                    if (!assign_val(_eq, false)) {
                        return false;
                    }
                } else if (_left->singleton() && _right->singleton()) {
                    // The constraint is already satisfied..
                    if (!assign_val(_eq, true)) {
                        return false;
                    }
                }
            }
            return true;
        }
    private:
        enum_var<T> * const _left;
        enum_var<T> * const _right;
        bool_var * const _eq;

        static std::unordered_set<bool> evaluate(enum_var<T> * const left, enum_var<T> * const right) {
            bool intersecting = false;
            for (const auto& i : left->to_vals()) {
                if (right->to_vals().find(i) != right->to_vals().end()) {
                    intersecting = true;
                    break;
                }
            }
            if (!intersecting) {
                return {false};
            } else if (left->singleton() && right->singleton()) {
                return {true};
            } else {
                return {true, false};
            }
        }

        static bool_var * evaluate(network * const net, enum_var<T> * const left, enum_var<T> * const right) {
            if (net->root_level()) {
                return new bool_var(net, to_string(left, right), evaluate(left, right));
            } else {
                return new bool_var(net, to_string(left, right),{true, false});
            }
        }

        static std::string to_string(enum_var<T> * const left, enum_var<T> * const right) {
            return left->_name + " == " + right->_name;
        }
    };
}

#endif /* ENUM_EQ_PROPAGATOR_H */

