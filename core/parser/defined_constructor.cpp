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
 * File:   defined_constructor.cpp
 * Author: Riccardo De Benedictis <riccardo.debenedictis@istc.cnr.it>
 * 
 * Created on November 9, 2016, 6:15 PM
 */

#include "defined_constructor.h"
#include "instantiated_field.h"
#include "expression_visitor.h"
#include "../item.h"
#include "../core.h"
#include "../type.h"
#include "statement_visitor.h"

using namespace oratio;

defined_constructor::defined_constructor(core * const c, scope * const s, const std::vector<field*>& args, std::vector<oRatioParser::Initializer_elementContext*> init_els, oRatioParser::BlockContext * const b) : constructor(c, s, args), _init_els(init_els), _block(b) { }

defined_constructor::~defined_constructor() { }

bool defined_constructor::invoke(item * const i, const std::vector<item*>& exprs) {
    std::unordered_map<std::string, field*> fields = _scope->get_fields();
    for (const auto& f : fields) {
        if (instantiated_field * inst_f = dynamic_cast<instantiated_field*> (f.second)) {
            i->_items.insert({f.second->_name, expression_visitor(_core, i).visit(inst_f->_expr)});
        }
    }
    env* c_env = new env(_core, i);
    c_env->_items.insert({THIS_KEYWORD, i});

    for (unsigned int j = 0; j < _args.size(); j++) {
        c_env->_items.insert({_args[j]->_name, exprs[j]});
    }
    for (const auto& el : _init_els) {
        if (fields.find(el->name->getText()) != fields.end()) {
            i->_items.insert({el->name->getText(), expression_visitor(_core, c_env).visit(el->expr_list()->expr(0))});
        } else {
            std::vector<item*> exprs;
            std::vector<const type*> par_types;
            if (el->expr_list()) {
                for (const auto& expr : el->expr_list()->expr()) {
                    item * i = expression_visitor(_core, c_env).visit(expr).as<item*>();
                    exprs.push_back(i);
                    par_types.push_back(i->get_type());
                }
            }
            get_type(el->name->getText())->get_constructor(par_types)->invoke(i, exprs);
        }
    }

    for (const auto& f : _scope->get_fields()) {
        if (!f.second->_synthetic) {
            if (instantiated_field * inst_f = dynamic_cast<instantiated_field*> (f.second)) {
                i->_items.insert({f.second->_name, expression_visitor(_core, i).visit(inst_f->_expr)});
            } else if (f.second->_type->_primitive) {
                i->_items.insert({f.second->_name, const_cast<type*> (f.second->_type)->new_instance(i)});
            } else {
                i->_items.insert({f.second->_name, const_cast<type*> (f.second->_type)->new_existential()});
            }
        }
    }

    return statement_visitor(_core, c_env).visit(_block);
}
