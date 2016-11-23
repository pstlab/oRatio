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

#include "oRatioBaseVisitor.h"

namespace oratio {

    class parser;
    class env;

    class statement_visitor : public oRatioBaseVisitor {
    public:
        statement_visitor(parser * const p, env * const e);
        statement_visitor(statement_visitor&&) = delete;
        virtual ~statement_visitor();
    private:
        parser * const _parser;
        env * const _env;

        antlrcpp::Any visitCompilation_unit(oRatioParser::Compilation_unitContext* ctx) override;
        antlrcpp::Any visitBlock(oRatioParser::BlockContext* ctx) override;
        antlrcpp::Any visitConjunction(oRatioParser::ConjunctionContext* ctx) override;
        antlrcpp::Any visitAssignment_statement(oRatioParser::Assignment_statementContext* ctx) override;
        antlrcpp::Any visitLocal_variable_statement(oRatioParser::Local_variable_statementContext* ctx) override;
        antlrcpp::Any visitExpression_statement(oRatioParser::Expression_statementContext* ctx) override;
        antlrcpp::Any visitFormula_statement(oRatioParser::Formula_statementContext* ctx) override;
        antlrcpp::Any visitReturn_statement(oRatioParser::Return_statementContext* ctx) override;
        antlrcpp::Any visitDisjunction_statement(oRatioParser::Disjunction_statementContext* ctx) override;
    };
}

