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
 * File:   expression_visitor.h
 * Author: Riccardo De Benedictis <riccardo.debenedictis@istc.cnr.it>
 *
 * Created on 9 novembre 2016, 13.54
 */

#ifndef EXPRESSION_VISITOR_H
#define EXPRESSION_VISITOR_H

#include "oRatioBaseVisitor.h"

namespace oratio {

    class parser;
    class env;

    class expression_visitor : public oRatioBaseVisitor {
    public:
        expression_visitor(parser * const p, env * const e);
        expression_visitor(expression_visitor&&) = delete;
        virtual ~expression_visitor();
    private:
        parser * const _parser;
        env * const _env;

        antlrcpp::Any visitLiteral_expression(oRatioParser::Literal_expressionContext* ctx) override;
        antlrcpp::Any visitParentheses_expression(oRatioParser::Parentheses_expressionContext* ctx) override;
        antlrcpp::Any visitMultiplication_expression(oRatioParser::Multiplication_expressionContext* ctx) override;
        antlrcpp::Any visitDivision_expression(oRatioParser::Division_expressionContext* ctx) override;
        antlrcpp::Any visitAddition_expression(oRatioParser::Addition_expressionContext* ctx) override;
        antlrcpp::Any visitSubtraction_expression(oRatioParser::Subtraction_expressionContext* ctx) override;
        antlrcpp::Any visitMinus_expression(oRatioParser::Minus_expressionContext* ctx) override;
        antlrcpp::Any visitNot_expression(oRatioParser::Not_expressionContext* ctx) override;
        antlrcpp::Any visitQualified_id_expression(oRatioParser::Qualified_id_expressionContext* ctx) override;
        antlrcpp::Any visitFunction_expression(oRatioParser::Function_expressionContext* ctx) override;
        antlrcpp::Any visitRange_expression(oRatioParser::Range_expressionContext* ctx) override;
        antlrcpp::Any visitConstructor_expression(oRatioParser::Constructor_expressionContext* ctx) override;
        antlrcpp::Any visitEq_expression(oRatioParser::Eq_expressionContext* ctx) override;
        antlrcpp::Any visitLt_expression(oRatioParser::Lt_expressionContext* ctx) override;
        antlrcpp::Any visitLeq_expression(oRatioParser::Leq_expressionContext* ctx) override;
        antlrcpp::Any visitGeq_expression(oRatioParser::Geq_expressionContext* ctx) override;
        antlrcpp::Any visitGt_expression(oRatioParser::Gt_expressionContext* ctx) override;
        antlrcpp::Any visitNeq_expression(oRatioParser::Neq_expressionContext* ctx) override;
        antlrcpp::Any visitImplication_expression(oRatioParser::Implication_expressionContext* ctx) override;
        antlrcpp::Any visitConjunction_expression(oRatioParser::Conjunction_expressionContext* ctx) override;
        antlrcpp::Any visitDisjunction_expression(oRatioParser::Disjunction_expressionContext* ctx) override;
        antlrcpp::Any visitExtc_one_expression(oRatioParser::Extc_one_expressionContext* ctx) override;
    };
}

#endif /* EXPRESSION_VISITOR_H */

