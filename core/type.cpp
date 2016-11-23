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

#include "type.h"
#include "core.h"
#include "item.h"
#include "field.h"
#include "constructor.h"
#include "method.h"
#include <set>

using namespace oratio;

type::type(core * const c, scope * const s, const std::string& name, bool primitive) : scope(c, s), _name(name), _primitive(primitive) { }

type::~type() { }

bool type::is_assignable_from(const type& t) const {
    std::queue<const type*> q;
    q.push(&t);
    while (!q.empty()) {
        if (q.front() == this) {
            return true;
        }
        else {
            for (const auto& st : q.front()->_supertypes) {
                q.push(st);
            }
            q.pop();
        }
    }
    return false;
}

item * type::new_instance(env * const e) {
    item * i = new item(_core, e, this);
    std::set<type*> ts;
    std::queue<type*> q;
    q.push(this);
    while (!q.empty()) {
        ts.insert(q.front());
        for (const auto& st : q.front()->_supertypes) {
            q.push(st);
        }
        q.pop();
    }

    for (const auto& t : ts) {
        t->_instances.insert(i);
    }
    return i;
}

item * type::new_existential() {
    if (_instances.size() == 1) {
        return *_instances.begin();
    }
    else {
        return _core->new_enum(this, _instances);
    }
}

constructor * type::get_constructor(const std::vector<const type*>& ts) const {
    bool found = false;
    for (const auto& c : _constructors) {
        if (c->_args.size() == ts.size()) {
            found = true;
            for (unsigned int i = 0; i < ts.size(); i++) {
                if (!c->_args[i]->_type->is_assignable_from(*ts[i])) {
                    found = false;
                    break;
                }
            }
            if (found) {
                return c;
            }
        }
    }

    // not found
    return nullptr;
}

field * type::get_field(const std::string& name) const {
    const auto& f_it = _fields.find(name);
    if (f_it != _fields.end()) {
        return (*f_it).second;
    }

    // if not here, check any enclosing scope
    if (_scope) {
        field * f = _scope->get_field(name);
        if (f) {
            return f;
        }
    }

    // if not in any enclosing scope, check any superclass
    for (const auto& st : _supertypes) {
        field * f = st->get_field(name);
        if (f) {
            return f;
        }
    }

    // not found
    return nullptr;
}

method * type::get_method(const std::string& name, const std::vector<const type*>& ts) const {
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

    // if not here, check any enclosing scope
    if (_scope) {
        method * m = _scope->get_method(name, ts);
        if (m) {
            return m;
        }
    }

    // if not in any enclosing scope, check any superclass
    for (const auto& st : _supertypes) {
        method * m = st->get_method(name, ts);
        if (m) {
            return m;
        }
    }

    // not found
    return nullptr;
}

predicate * type::get_predicate(const std::string& name) const {
    const auto& p_it = _predicates.find(name);
    if (p_it != _predicates.end()) {
        return (*p_it).second;
    }

    // if not here, check any enclosing scope
    if (_scope) {
        predicate * p = _scope->get_predicate(name);
        if (p) {
            return p;
        }
    }

    // if not in any enclosing scope, check any superclass
    for (const auto& st : _supertypes) {
        predicate * p = st->get_predicate(name);
        if (p) {
            return p;
        }
    }

    // not found
    return nullptr;
}

type * type::get_type(const std::string& name) const {
    const auto& t_it = _types.find(name);
    if (t_it != _types.end()) {
        return (*t_it).second;
    }

    // if not here, check any enclosing scope
    if (_scope) {
        type * t = _scope->get_type(name);
        if (t) {
            return t;
        }
    }

    // if not in any enclosing scope, check any superclass
    for (const auto& st : _supertypes) {
        type * t = st->get_type(name);
        if (t) {
            return t;
        }
    }

    // not found
    return nullptr;
}

bool_type::bool_type(core * const c) : type(c, nullptr, BOOL_KEYWORD, true) { }

bool_type::~bool_type() { }

item * bool_type::new_instance(env * const e) {
    return _core->new_bool();
}

int_type::int_type(core * const c) : type(c, nullptr, INT_KEYWORD, true) { }

int_type::~int_type() { }

item * int_type::new_instance(env * const e) {
    return _core->new_int();
}

real_type::real_type(core * const c) : type(c, nullptr, REAL_KEYWORD, true) { }

real_type::~real_type() { }

item * real_type::new_instance(env * const e) {
    return _core->new_real();
}

string_type::string_type(core * const c) : type(c, nullptr, STRING_KEYWORD, true) { }

string_type::~string_type() { }

item* string_type::new_instance(env * const e) {
    return _core->new_string();
}
