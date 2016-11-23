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

#include "oRatioBaseListener.h"

namespace oratio {

    class parser;
    class scope;

    class type_refinement_listener : public oRatioBaseListener {
    public:
        type_refinement_listener(parser * const p);
        type_refinement_listener(type_refinement_listener&&) = delete;
        virtual ~type_refinement_listener();
    private:
        parser * const _parser;
        scope * _scope;

        void enterCompilation_unit(oRatioParser::Compilation_unitContext* ctx) override;

        void enterEnum_declaration(oRatioParser::Enum_declarationContext* ctx) override;

        void enterClass_declaration(oRatioParser::Class_declarationContext* ctx) override;
        void exitClass_declaration(oRatioParser::Class_declarationContext* ctx) override;

        void enterField_declaration(oRatioParser::Field_declarationContext* ctx) override;

        void enterConstructor_declaration(oRatioParser::Constructor_declarationContext* ctx) override;
        void exitConstructor_declaration(oRatioParser::Constructor_declarationContext* ctx) override;

        void enterVoid_method_declaration(oRatioParser::Void_method_declarationContext* ctx) override;
        void exitVoid_method_declaration(oRatioParser::Void_method_declarationContext* ctx) override;
        void enterType_method_declaration(oRatioParser::Type_method_declarationContext* ctx) override;
        void exitType_method_declaration(oRatioParser::Type_method_declarationContext* ctx) override;

        void enterPredicate_declaration(oRatioParser::Predicate_declarationContext* ctx) override;
        void exitPredicate_declaration(oRatioParser::Predicate_declarationContext* ctx) override;

        void enterDisjunction_statement(oRatioParser::Disjunction_statementContext* ctx) override;
        void exitDisjunction_statement(oRatioParser::Disjunction_statementContext* ctx) override;

        void enterConjunction(oRatioParser::ConjunctionContext* ctx) override;
        void exitConjunction(oRatioParser::ConjunctionContext* ctx) override;

        void enterLocal_variable_statement(oRatioParser::Local_variable_statementContext* ctx) override;
        void enterAssignment_statement(oRatioParser::Assignment_statementContext* ctx) override;
        void enterExpression_statement(oRatioParser::Expression_statementContext* ctx) override;
        void enterFormula_statement(oRatioParser::Formula_statementContext* ctx) override;
        void enterReturn_statement(oRatioParser::Return_statementContext* ctx) override;

        void enterQualified_id_expression(oRatioParser::Qualified_id_expressionContext* ctx) override;
        void enterFunction_expression(oRatioParser::Function_expressionContext* ctx) override;
    };
}

