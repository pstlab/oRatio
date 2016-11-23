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

#include "core.h"
#include "type.h"
#include "item.h"
#include "field.h"
#include "method.h"

using namespace oratio;

core::core() : parser(this), scope(this, nullptr), env(this, nullptr) {
    _types.insert({ BOOL_KEYWORD, new bool_type(this) });
    _types.insert({ INT_KEYWORD, new int_type(this) });
    _types.insert({ REAL_KEYWORD, new real_type(this) });
    _types.insert({ STRING_KEYWORD, new string_type(this) });
}

core::~core() { }

bool_item* core::new_bool() {
    return new bool_item(this, *_net.new_bool());
}

bool_item* core::new_bool(bool value) {
    return new bool_item(this, *_net.new_bool(value));
}

arith_item* core::new_int() {
    return new arith_item(this, _types.at(INT_KEYWORD), *_net.new_int());
}

arith_item* core::new_int(long value) {
    return new arith_item(this, _types.at(INT_KEYWORD), *_net.new_int(value));
}

arith_item* core::new_real() {
    return new arith_item(this, _types.at(REAL_KEYWORD), *_net.new_real());
}

arith_item* core::new_real(double value) {
    return new arith_item(this, _types.at(REAL_KEYWORD), *_net.new_real(value));
}

string_item* core::new_string() {
    std::string init_val;
    return new string_item(this, init_val);
}

string_item* core::new_string(std::string& val) {
    return new string_item(this, val);
}

enum_item* core::new_enum(const type * const t, const std::unordered_set<item*>& allowed_vals) {
    if (t->_name.compare(BOOL_KEYWORD) == 0) {
        return new bool_enum(this, *_net.new_bool(), *_net.new_enum(allowed_vals));
    }
    else if (t->_name.compare(INT_KEYWORD) == 0) {
        return new arith_enum(this, _types.at(INT_KEYWORD), *_net.new_int(), *_net.new_enum(allowed_vals));
    }
    else if (t->_name.compare(REAL_KEYWORD) == 0) {
        return new arith_enum(this, _types.at(REAL_KEYWORD), *_net.new_real(), *_net.new_enum(allowed_vals));
    }
    else {
        return new enum_item(this, t, *_net.new_enum(allowed_vals));
    }
}

bool_item* core::negate(bool_item * const var) {
    return new bool_item(this, *_net.negate(&var->get_bool_var()));
}

bool_item* core::conjunction(const std::vector<bool_item*>& vars) {
    std::vector<ac::bool_var*> c_vars(vars.size());
    for (unsigned int i = 0; i < vars.size(); i++) {
        c_vars[i] = &vars[i]->get_bool_var();
    }
    return new bool_item(this, *_net.conjunction(c_vars));
}

bool_item* core::disjunction(const std::vector<bool_item*>& vars) {
    std::vector<ac::bool_var*> c_vars(vars.size());
    for (unsigned int i = 0; i < vars.size(); i++) {
        c_vars[i] = &vars[i]->get_bool_var();
    }
    return new bool_item(this, *_net.disjunction(c_vars));
}

bool_item* core::exactly_one(const std::vector<bool_item*>& vars) {
    std::vector<ac::bool_var*> c_vars(vars.size());
    for (unsigned int i = 0; i < vars.size(); i++) {
        c_vars[i] = &vars[i]->get_bool_var();
    }
    return new bool_item(this, *_net.exactly_one(c_vars));
}

arith_item* core::minus(arith_item * const var) {
    bool contains_reals = var->_type->_name.compare(REAL_KEYWORD) == 0;
    return new arith_item(this, contains_reals ? _types.at(REAL_KEYWORD) : _types.at(INT_KEYWORD), *_net.minus(&var->get_arith_var()));
}

arith_item* core::sum(const std::vector<arith_item*>& vars) {
    std::vector<ac::arith_var*> c_vars(vars.size());
    bool contains_reals = false;
    for (unsigned int i = 0; i < vars.size(); i++) {
        c_vars[i] = &vars[i]->get_arith_var();
        if (vars[i]->_type->_name.compare(REAL_KEYWORD) == 0) {
            contains_reals = true;
        }
    }
    return new arith_item(this, contains_reals ? _types.at(REAL_KEYWORD) : _types.at(INT_KEYWORD), *_net.sum(c_vars));
}

arith_item* core::sub(const std::vector<arith_item*>& vars) {
    std::vector<ac::arith_var*> c_vars(vars.size());
    bool contains_reals = false;
    for (unsigned int i = 0; i < vars.size(); i++) {
        c_vars[i] = &vars[i]->get_arith_var();
        if (vars[i]->_type->_name.compare(REAL_KEYWORD) == 0) {
            contains_reals = true;
        }
    }
    return new arith_item(this, contains_reals ? _types.at(REAL_KEYWORD) : _types.at(INT_KEYWORD), *_net.sub(c_vars));
}

arith_item* core::mult(const std::vector<arith_item*>& vars) {
    std::vector<ac::arith_var*> c_vars(vars.size());
    bool contains_reals = false;
    for (unsigned int i = 0; i < vars.size(); i++) {
        c_vars[i] = &vars[i]->get_arith_var();
        if (vars[i]->_type->_name.compare(REAL_KEYWORD) == 0) {
            contains_reals = true;
        }
    }
    return new arith_item(this, contains_reals ? _types.at(REAL_KEYWORD) : _types.at(INT_KEYWORD), *_net.mult(c_vars));
}

arith_item* core::div(arith_item * const var0, arith_item * const var1) {
    bool contains_reals = var0->_type->_name.compare(REAL_KEYWORD) == 0 || var1->_type->_name.compare(REAL_KEYWORD) == 0;
    return new arith_item(this, contains_reals ? _types.at(REAL_KEYWORD) : _types.at(INT_KEYWORD), *_net.div(&var0->get_arith_var(), &var1->get_arith_var()));
}

bool_item* core::lt(arith_item * const var0, arith_item * const var1) {
    return new bool_item(this, *_net.lt(&var0->get_arith_var(), &var1->get_arith_var()));
}

bool_item* core::leq(arith_item * const var0, arith_item * const var1) {
    return new bool_item(this, *_net.leq(&var0->get_arith_var(), &var1->get_arith_var()));
}

bool_item* core::eq(arith_item * const var0, arith_item * const var1) {
    return new bool_item(this, *_net.eq(&var0->get_arith_var(), &var1->get_arith_var()));
}

bool_item* core::geq(arith_item * const var0, arith_item * const var1) {
    return new bool_item(this, *_net.geq(&var0->get_arith_var(), &var1->get_arith_var()));
}

bool_item* core::gt(arith_item * const var0, arith_item * const var1) {
    return new bool_item(this, *_net.gt(&var0->get_arith_var(), &var1->get_arith_var()));
}

bool_item* core::eq(item * const i0, item * const i1) {
    return new bool_item(this, *i0->eq(i1));
}

bool core::assert_facts(const std::vector<bool_item*>& exprs) {
    for (unsigned int i = 0; i < exprs.size(); i++) {
        if (!_net.add({ _net.imply(_ctr_var, &exprs[i]->get_bool_var()) })) {
            return false;
        }
    }
    return true;
}

field * core::get_field(const std::string& name) const {
    const auto& fi = _fields.find(name);
    if (fi != _fields.end()) {
        return (*fi).second;
    }

    // not found
    return nullptr;
}

method * core::get_method(const std::string& name, const std::vector<const type*>& ts) const {
    if (_methods.find(name) != _methods.end()) {
        bool found = false;
        for (const auto& m : _methods.at(name)) {
            if (m->_args.size() == ts.size()) {
                found = true;
                for (unsigned int i = 0; i < ts.size(); i++) {
                    if (!m->_args[i]->_type->is_assignable_from(*ts[i])) {
                        found = false;
                        break;
                    }
                }
                if (found) {
                    return m;
                }
            }
        }
    }

    // not found
    return nullptr;
}

predicate * core::get_predicate(const std::string& name) const {
    const auto& p_it = _predicates.find(name);
    if (p_it != _predicates.end()) {
        return (*p_it).second;
    }

    // not found
    return nullptr;
}

type * core::get_type(const std::string& name) const {
    const auto& t_it = _types.find(name);
    if (t_it != _types.end()) {
        return (*t_it).second;
    }

    // not found
    return nullptr;
}