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

#include "env.h"
#include "../ac/network.h"
#include "../ac/var.h"

namespace oratio {

    class type;

    class item : public virtual env {
    public:
        item(core * const c, env * const e, const type * const t);
        item(item&&) = delete;
        virtual ~item();

        const type * const get_type() const {
            return _type;
        }

        virtual ac::bool_var * eq(item * const i);
        virtual bool equates(item * const i);
    public:
        const type * const _type;
    };

    class bool_item : public virtual item {
    public:
        bool_item(core * const c, ac::bool_var& v);
        bool_item(bool_item&&) = delete;

        ac::bool_var& get_bool_var() const {
            return _var;
        }

        ac::bool_var* eq(item * const i) override;
        bool equates(item * const i) override;
    protected:
        ac::bool_var& _var;
    };

    class arith_item : public virtual item {
    public:
        arith_item(core * const c, const type * const t, ac::arith_var& v);
        arith_item(arith_item&&) = delete;

        ac::arith_var& get_arith_var() const {
            return _var;
        }

        ac::bool_var* eq(item * const i) override;
        bool equates(item * const i) override;
    protected:
        ac::arith_var& _var;
    };

    class string_item : public item {
    public:
        string_item(core * const c, std::string& v);
        string_item(string_item&&) = delete;

        std::string& get_value() const {
            return _val;
        }

        void set_value(const std::string& _val) {
            this->_val = _val;
        }

        ac::bool_var* eq(item * const i) override;
        bool equates(item * const i) override;
    protected:
        std::string& _val;
    };

    class enum_item : public virtual item {
        friend class core;
    public:
        enum_item(core * const c, const type * const t, ac::enum_var<item*>& v);
        enum_item(enum_item&&) = delete;

        item* get(const std::string& name) const override;

        ac::enum_var<item*>& get_enum_var() const {
            return _var;
        }

        ac::bool_var* allows(item * const i) const {
            return _eqs.at(i);
        }

        ac::bool_var* eq(item * const i) override;
        bool equates(item * const i) override;
    protected:
        ac::enum_var<item*>& _var;
        std::unordered_map<item*, ac::bool_var*> _eqs;
    };

    class bool_enum : public bool_item, public enum_item {
    public:
        bool_enum(core * const c, ac::bool_var& v, ac::enum_var<item*>& vs);
        bool_enum(bool_enum&&) = delete;

        ac::bool_var* eq(item * const i) override;
        bool equates(item * const i) override;
    };

    class arith_enum : public arith_item, public enum_item {
    public:
        arith_enum(core * const c, const type * const t, ac::arith_var& v, ac::enum_var<item*>& vs);
        arith_enum(arith_enum&&) = delete;

        ac::bool_var* eq(item * const i) override;
        bool equates(item * const i) override;
    };
}

