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

#include "parser.h"
#include "type_declaration_listener.h"
#include "type_refinement_listener.h"
#include "statement_visitor.h"
#include "../core.h"

using namespace oratio;

parser::parser(core * const c) : _core(c) { }

parser::~parser() { }

bool parser::read(const std::string& script) {
    _parser = new oRatioParser(new antlr4::CommonTokenStream(new oRatioLexer(new antlr4::ANTLRInputStream(script))));
    oRatioParser::Compilation_unitContext* cu = _parser->compilation_unit();
    type_declaration_listener td(this);
    type_refinement_listener tr(this);
    antlr4::tree::ParseTreeWalker walker;
    walker.walk(&td, cu);
    walker.walk(&tr, cu);
    return statement_visitor(this, _core).visit(cu).as<bool>();
}

bool parser::read(const std::vector<std::string>& files) {
    std::vector<snippet*> snippets;
    for (const auto& f : files) {
        _parser = new oRatioParser(new antlr4::CommonTokenStream(new oRatioLexer(new antlr4::ANTLRFileStream(f))));
        snippet* s = new snippet(f, _parser, _parser->compilation_unit());
        snippets.push_back(s);
    }
    antlr4::tree::ParseTreeWalker walker;
    type_declaration_listener td(this);
    for (const auto& s : snippets) {
        _parser = s->_parser;
        walker.walk(&td, s->_cu);
    }
    type_refinement_listener tr(this);
    for (const auto& s : snippets) {
        _parser = s->_parser;
        walker.walk(&tr, s->_cu);
    }
    statement_visitor sv(this, _core);
    for (const auto& s : snippets) {
        _parser = s->_parser;
        if (!sv.visit(s->_cu).as<bool>()) {
            return false;
        }
    }
    return true;
}

parser::snippet::snippet(const std::string& file, oRatioParser * const p, oRatioParser::Compilation_unitContext * const cu) : _file(file), _parser(p), _cu(cu) { }

parser::snippet::~snippet() { }
