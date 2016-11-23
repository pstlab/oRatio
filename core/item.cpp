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
#include "item.h"
#include "scope.h"
#include "type.h"
#include "field.h"
#include "../ac/enum_eq_propagator.h"
#include <set>

using namespace oratio;

item::item(core * const c, env * const e, const type * const t) : env(c, e), _type(t) { }

item::~item() { }

ac::bool_var* item::eq(item * const i) {
    if (this == i) {
        return _core->_net.new_bool(true);
    }
    else if (enum_item * ei = dynamic_cast<enum_item*> (i)) {
        return ei->eq(this);
    }
    else {
        std::set<const type*> ts;
        std::queue<const type*> q;
        q.push(_type);
        while (!q.empty()) {
            ts.insert(q.front());
            for (const auto& st : q.front()->_supertypes) {
                q.push(st);
            }
            q.pop();
        }

        std::vector<ac::bool_var*> eqs;
        for (const auto& t : ts) {
            for (const auto& f : t->_fields) {
                if (!f.second->_synthetic) {
                    eqs.push_back(_items.at(f.first)->eq(i->_items.at(f.first)));
                }
            }
        }
        if (eqs.empty()) {
            return _core->_net.new_bool(true);
        }
        else if (eqs.size() == 1) {
            return *eqs.begin();
        }
        else {
            return _core->_net.conjunction(eqs);
        }
    }
}

bool item::equates(item * const i) {
    if (this == i) {
        return true;
    }
    else if (enum_item * ei = dynamic_cast<enum_item*> (i)) {
        return ei->equates(this);
    }
    else {
        std::set<const type*> ts;
        std::queue<const type*> q;
        q.push(_type);
        while (!q.empty()) {
            ts.insert(q.front());
            for (const auto& st : q.front()->_supertypes) {
                q.push(st);
            }
            q.pop();
        }

        for (const auto& t : ts) {
            for (const auto& f : t->_fields) {
                if (!f.second->_synthetic) {
                    if (!_items.at(f.first)->equates(i->_items.at(f.first))) {
                        return false;
                    }
                }
            }
        }
        return true;
    }
}

bool_item::bool_item(core * const c, ac::bool_var& v) : env(c, nullptr), item(c, nullptr, c->get_type(BOOL_KEYWORD)), _var(v) { }

ac::bool_var* bool_item::eq(item * const i) {
    if (this == i) {
        return _core->_net.new_bool(true);
    }
    else if (bool_item * bi = dynamic_cast<bool_item*> (i)) {
        return _core->_net.eq(&get_bool_var(), &bi->get_bool_var());
    }
    else {
        return _core->_net.new_bool(false);
    }
}

bool bool_item::equates(item * const i) {
    if (this == i) {
        return true;
    }
    else if (bool_item * bi = dynamic_cast<bool_item*> (i)) {
        return _var.intersects(bi->_var);
    }
    else {
        return false;
    }
}

arith_item::arith_item(core * const c, const type * const t, ac::arith_var& v) : env(c, nullptr), item(c, nullptr, t), _var(v) { }

ac::bool_var* arith_item::eq(item * const i) {
    if (this == i) {
        return _core->_net.new_bool(true);
    }
    else if (arith_item * ai = dynamic_cast<arith_item*> (i)) {
        return _core->_net.eq(&_var, &ai->_var);
    }
    else {
        return _core->_net.new_bool(false);
    }
}

bool arith_item::equates(item * const i) {
    if (this == i) {
        return true;
    }
    else if (arith_item * ai = dynamic_cast<arith_item*> (i)) {
        return _var.intersects(ai->_var);
    }
    else {
        return false;
    }
}

string_item::string_item(core * const c, std::string& v) : env(c, nullptr), item(c, nullptr, c->get_type(STRING_KEYWORD)), _val(v) { }

ac::bool_var* string_item::eq(item * const i) {
    if (this == i) {
        return _core->_net.new_bool(true);
    }
    else if (string_item * si = dynamic_cast<string_item*> (i)) {
        return _core->_net.new_bool(_val.compare(si->_val) == 0);
    }
    else {
        return _core->_net.new_bool(false);
    }
}

bool string_item::equates(item * const i) {
    if (this == i) {
        return true;
    }
    else if (string_item * si = dynamic_cast<string_item*> (i)) {
        return _val.compare(si->_val) == 0;
    }
    else {
        return false;
    }
}

enum_item::enum_item(core * const c, const type * const t, ac::enum_var<item*>& v) : env(c, nullptr), item(c, nullptr, t), _var(v) {
    if (!t->_primitive) {
        for (const auto& val : v.to_vals()) {
            _eqs.insert({ val, _core->_net.eq(&v, val) });
        }
    }
}

item* enum_item::get(const std::string& name) const {
    if (_items.find(name) == _items.end()) {
        if (_var.singleton()) {
            return (*_var.to_vals().begin())->get(name);
        }
        else {
            std::vector<item*> c_vals;
            std::vector<item*> f_vals;
            for (const auto& val : _var.to_vals()) {
                c_vals.push_back(val);
                f_vals.push_back(val->get(name));
                if (dynamic_cast<enum_item*> (val->get(name))) {
                    std::cerr << "invalid use: enum of enums.." << std::endl;
                }
            }

            std::unordered_set<item*> vals;
            for (unsigned int i = 0; i < c_vals.size(); i++) {
                vals.insert(f_vals[i]);
            }
            enum_item* e = _core->new_enum(_type->get_field(name)->_type, vals);

            for (unsigned int i = 0; i < c_vals.size(); i++) {
                bool af = _core->_net.add({ _core->_net.eq(allows(c_vals[i]), e->allows(f_vals[i])) });
                assert(af);
            }

            const_cast<enum_item*> (this)->_items.insert({ name, e });
        }
    }

    return _items.at(name);
}

ac::bool_var* enum_item::eq(item * const i) {
    if (this == i) {
        return _core->_net.new_bool(true);
    }
    else if (enum_item * ei = dynamic_cast<enum_item*> (i)) {
        return _core->_net.eq(&_var, &ei->_var);
    }
    else if (_eqs.find(i) != _eqs.end()) {
        return _eqs.at(i);
    }
    else {
        return _core->_net.new_bool(true);
    }
}

bool enum_item::equates(item * const i) {
    if (this == i) {
        return true;
    }
    else if (enum_item * ei = dynamic_cast<enum_item*> (i)) {
        return _var.intersects(ei->_var);
    }
    else if (_eqs.find(i) != _eqs.end()) {
        return true;
    }
    else {
        return false;
    }
}

bool_enum::bool_enum(core * const c, ac::bool_var& v, ac::enum_var<item*>& vs) : env(c, nullptr), item(c, nullptr, c->get_type(BOOL_KEYWORD)), bool_item(c, v), enum_item(c, c->get_type(BOOL_KEYWORD), vs) {
    for (const auto& val : vs.to_vals()) {
        _eqs.insert({ val, _core->_net.eq(&v, &dynamic_cast<bool_item*> (val)->get_bool_var()) });
    }
}

ac::bool_var* bool_enum::eq(item * const i) {
    return bool_item::eq(i);
}

bool bool_enum::equates(item * const i) {
    return bool_item::equates(i);
}

arith_enum::arith_enum(core * const c, const type * const t, ac::arith_var& v, ac::enum_var<item*>& vs) : env(c, nullptr), item(c, nullptr, t), arith_item(c, t, v), enum_item(c, t, vs) {
    for (const auto& val : vs.to_vals()) {
        _eqs.insert({ val, _core->_net.eq(&v, &dynamic_cast<arith_item*> (val)->get_arith_var()) });
    }
}

ac::bool_var* arith_enum::eq(item * const i) {
    return arith_item::eq(i);
}

bool arith_enum::equates(item * const i) {
    return arith_item::equates(i);
}
