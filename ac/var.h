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

#include "network.h"
#include "interval.h"
#include <string>
#include <unordered_set>

namespace ac {

    class domain {
    public:

        domain() {}
        domain(domain&&) = delete;
        virtual ~domain() {}

        virtual bool empty() const = 0;
        virtual bool singleton() const = 0;
    };

    class var {
        friend class network;
    public:
        var(network* const n, const std::string& name, domain* const d) :_net(n), _name(name), _domain(d) {}
        var(var&&) = delete;
        virtual ~var() {}

        bool empty() const {
            return _domain->empty();
        }

        bool singleton() const {
            return _domain->singleton();
        }
    private:
        virtual void restore() = 0;
    protected:
        network * const _net;
        domain * _domain;
    public:
        const std::string _name;
    };

    template<typename T>
    class enum_domain : public domain {
        template<typename Q>
        friend class enum_var;
    public:
        bool empty() const override {
            return _vals.empty();
        }

        bool singleton() const override {
            return _vals.size() == 1;
        }
    private:
        std::unordered_set<T> _vals;

        enum_domain(const std::unordered_set<T>& allowed_vals) : _vals(allowed_vals) {}
        enum_domain(enum_domain&&) = delete;
        virtual ~enum_domain() {}

        std::unordered_set<T> intersection(const std::unordered_set<T>& vals) const {
            std::unordered_set<T> intersection;
            for (const auto& i : _vals) {
                if (vals.find(i) != vals.end()) {
                    intersection.insert(i);
                }
            }
            return intersection;
        }

        bool intersects(const std::unordered_set<T>& vals) const {
            for (const auto& i : _vals) {
                if (vals.find(i) != vals.end()) {
                    return true;
                }
            }
            return false;
        }

        std::unordered_set<T> difference(const std::unordered_set<T>& vals) const {
            std::unordered_set<T> difference;
            for (const auto& i : _vals) {
                if (vals.find(i) == vals.end()) {
                    difference.insert(i);
                }
            }
            return difference;
        }
    };

    template<typename T>
    class enum_var : public var {
        friend class network;
        friend class propagator;
    public:
        enum_var(network * const net, const std::string& name, const std::unordered_set<T>& vals) : var(net, name, new enum_domain<T>(vals)) {}
        enum_var(enum_var&&) = delete;

        virtual ~enum_var() {
            delete _domain;
        }

        bool allows(const T & val) const {
            return static_cast<enum_domain<T>*>(_domain)->_vals.find(val) != static_cast<enum_domain<T>*>(_domain)->_vals.end();
        }

        std::unordered_set<T> to_vals() const {
            return static_cast<enum_domain<T>*>(_domain)->_vals;
        }

        bool intersects(const enum_var<T>& var) const {
            return static_cast<enum_domain<T>*>(_domain)->intersects(static_cast<enum_domain<T>*>(var._domain)->_vals);
        }
    private:
        void restore() override {
            delete _domain;
            _domain = static_cast<enum_domain<T>*> (_net->get_domain(this));
        }

        bool complement(const std::unordered_set<T>& vals, propagator * prop) {
            std::unordered_set<T> intersection = static_cast<enum_domain<T>*>(_domain)->intersection(vals);
            if (!intersection.empty()) {
                enum_domain<T>* c_domain = new enum_domain<T>(static_cast<enum_domain<T>*>(_domain)->_vals);
                for (const auto& i : intersection) {
                    static_cast<enum_domain<T>*>(_domain)->_vals.erase(i);
                }
                return _net->enqueue(this, c_domain, prop);
            }
            assert(!static_cast<enum_domain<T>*>(_domain)->empty());
            return true;
        }

        bool intersect(const std::unordered_set<T>& vals, propagator * prop) {
            std::unordered_set<T> difference = static_cast<enum_domain<T>*>(_domain)->difference(vals);
            if (!difference.empty()) {
                enum_domain<T>* c_domain = new enum_domain<T>(static_cast<enum_domain<T>*>(_domain)->_vals);
                for (const auto& i : difference) {
                    static_cast<enum_domain<T>*>(_domain)->_vals.erase(i);
                }
                return _net->enqueue(this, c_domain, prop);
            }
            assert(!static_cast<enum_domain<T>*>(_domain)->empty());
            return true;
        }
    };

    typedef enum_var<bool> bool_var;

    class arith_domain : public domain {
        friend class arith_var;
    public:

        arith_domain(interval i) : _interval(i) { }

        ~arith_domain() { }

        bool empty() const override {
            return !_interval.consistent();
        }

        bool singleton() const override {
            return _interval.constant();
        }

        bool intersects(const interval& i) const {
            return _interval.intersecting(i);
        }
    private:
        interval _interval;
    };

    class arith_var : public var {
        friend class network;
        friend class propagator;
        friend class minus_propagator;
        friend class sum_propagator;
        friend class product_propagator;
        friend class div_propagator;
        friend class lt_propagator;
        friend class leq_propagator;
        friend class arith_eq_propagator;
        friend class geq_propagator;
        friend class gt_propagator;
    public:
        arith_var(network * const net, double value) : var(net, std::to_string(value), new arith_domain(value)), _expr(net->_context->real_val(std::to_string(value).c_str())) { }
        arith_var(network * const net, const std::string name, interval i, z3::expr e) : var(net, name, new arith_domain(i)), _expr(e) { }
        arith_var(arith_var&&) = delete;

        virtual ~arith_var() {
            delete _domain;
        }

        double eval() const {
            return std::stod(_net->_model.eval(_expr, true).get_decimal_string(5));
        }

        interval to_interval() const {
            return static_cast<arith_domain*>(_domain)->_interval;
        }

        bool intersects(const arith_var& var) const {
            return static_cast<arith_domain*>(_domain)->intersects(static_cast<arith_domain*>(var._domain)->_interval);
        }
    private:
        arith_domain* _domain;
        z3::expr _expr;

        void restore() override {
            delete _domain;
            _domain = static_cast<arith_domain*> (_net->get_domain(this));
        }

        bool intersect(const interval& i, propagator* prop) {
            if (i.contains(_domain->_interval)) {
                return true;
            }
            else {
                arith_domain* c_domain = new arith_domain(_domain->_interval);
                _domain->_interval = _domain->_interval && i;
                return _net->enqueue(this, c_domain, prop);
            }
        }
    };
}

