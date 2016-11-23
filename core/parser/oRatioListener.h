
// Generated from oRatio.g4 by ANTLR 4.5.3

#pragma once


#include "antlr4-runtime.h"
#include "oRatioParser.h"


namespace oratio {

    /**
     * This interface defines an abstract listener for a parse tree produced by oRatioParser.
     */
    class oRatioListener : public antlr4::tree::ParseTreeListener {
    public:

        virtual void enterCompilation_unit(oRatioParser::Compilation_unitContext *ctx) = 0;
        virtual void exitCompilation_unit(oRatioParser::Compilation_unitContext *ctx) = 0;

        virtual void enterType_declaration(oRatioParser::Type_declarationContext *ctx) = 0;
        virtual void exitType_declaration(oRatioParser::Type_declarationContext *ctx) = 0;

        virtual void enterTypedef_declaration(oRatioParser::Typedef_declarationContext *ctx) = 0;
        virtual void exitTypedef_declaration(oRatioParser::Typedef_declarationContext *ctx) = 0;

        virtual void enterEnum_declaration(oRatioParser::Enum_declarationContext *ctx) = 0;
        virtual void exitEnum_declaration(oRatioParser::Enum_declarationContext *ctx) = 0;

        virtual void enterEnum_constants(oRatioParser::Enum_constantsContext *ctx) = 0;
        virtual void exitEnum_constants(oRatioParser::Enum_constantsContext *ctx) = 0;

        virtual void enterClass_declaration(oRatioParser::Class_declarationContext *ctx) = 0;
        virtual void exitClass_declaration(oRatioParser::Class_declarationContext *ctx) = 0;

        virtual void enterMember(oRatioParser::MemberContext *ctx) = 0;
        virtual void exitMember(oRatioParser::MemberContext *ctx) = 0;

        virtual void enterField_declaration(oRatioParser::Field_declarationContext *ctx) = 0;
        virtual void exitField_declaration(oRatioParser::Field_declarationContext *ctx) = 0;

        virtual void enterVariable_dec(oRatioParser::Variable_decContext *ctx) = 0;
        virtual void exitVariable_dec(oRatioParser::Variable_decContext *ctx) = 0;

        virtual void enterVoid_method_declaration(oRatioParser::Void_method_declarationContext *ctx) = 0;
        virtual void exitVoid_method_declaration(oRatioParser::Void_method_declarationContext *ctx) = 0;

        virtual void enterType_method_declaration(oRatioParser::Type_method_declarationContext *ctx) = 0;
        virtual void exitType_method_declaration(oRatioParser::Type_method_declarationContext *ctx) = 0;

        virtual void enterConstructor_declaration(oRatioParser::Constructor_declarationContext *ctx) = 0;
        virtual void exitConstructor_declaration(oRatioParser::Constructor_declarationContext *ctx) = 0;

        virtual void enterInitializer_element(oRatioParser::Initializer_elementContext *ctx) = 0;
        virtual void exitInitializer_element(oRatioParser::Initializer_elementContext *ctx) = 0;

        virtual void enterPredicate_declaration(oRatioParser::Predicate_declarationContext *ctx) = 0;
        virtual void exitPredicate_declaration(oRatioParser::Predicate_declarationContext *ctx) = 0;

        virtual void enterStatement(oRatioParser::StatementContext *ctx) = 0;
        virtual void exitStatement(oRatioParser::StatementContext *ctx) = 0;

        virtual void enterBlock(oRatioParser::BlockContext *ctx) = 0;
        virtual void exitBlock(oRatioParser::BlockContext *ctx) = 0;

        virtual void enterAssignment_statement(oRatioParser::Assignment_statementContext *ctx) = 0;
        virtual void exitAssignment_statement(oRatioParser::Assignment_statementContext *ctx) = 0;

        virtual void enterLocal_variable_statement(oRatioParser::Local_variable_statementContext *ctx) = 0;
        virtual void exitLocal_variable_statement(oRatioParser::Local_variable_statementContext *ctx) = 0;

        virtual void enterExpression_statement(oRatioParser::Expression_statementContext *ctx) = 0;
        virtual void exitExpression_statement(oRatioParser::Expression_statementContext *ctx) = 0;

        virtual void enterDisjunction_statement(oRatioParser::Disjunction_statementContext *ctx) = 0;
        virtual void exitDisjunction_statement(oRatioParser::Disjunction_statementContext *ctx) = 0;

        virtual void enterConjunction(oRatioParser::ConjunctionContext *ctx) = 0;
        virtual void exitConjunction(oRatioParser::ConjunctionContext *ctx) = 0;

        virtual void enterFormula_statement(oRatioParser::Formula_statementContext *ctx) = 0;
        virtual void exitFormula_statement(oRatioParser::Formula_statementContext *ctx) = 0;

        virtual void enterReturn_statement(oRatioParser::Return_statementContext *ctx) = 0;
        virtual void exitReturn_statement(oRatioParser::Return_statementContext *ctx) = 0;

        virtual void enterAssignment_list(oRatioParser::Assignment_listContext *ctx) = 0;
        virtual void exitAssignment_list(oRatioParser::Assignment_listContext *ctx) = 0;

        virtual void enterAssignment(oRatioParser::AssignmentContext *ctx) = 0;
        virtual void exitAssignment(oRatioParser::AssignmentContext *ctx) = 0;

        virtual void enterCast_expression(oRatioParser::Cast_expressionContext *ctx) = 0;
        virtual void exitCast_expression(oRatioParser::Cast_expressionContext *ctx) = 0;

        virtual void enterQualified_id_expression(oRatioParser::Qualified_id_expressionContext *ctx) = 0;
        virtual void exitQualified_id_expression(oRatioParser::Qualified_id_expressionContext *ctx) = 0;

        virtual void enterDivision_expression(oRatioParser::Division_expressionContext *ctx) = 0;
        virtual void exitDivision_expression(oRatioParser::Division_expressionContext *ctx) = 0;

        virtual void enterSubtraction_expression(oRatioParser::Subtraction_expressionContext *ctx) = 0;
        virtual void exitSubtraction_expression(oRatioParser::Subtraction_expressionContext *ctx) = 0;

        virtual void enterExtc_one_expression(oRatioParser::Extc_one_expressionContext *ctx) = 0;
        virtual void exitExtc_one_expression(oRatioParser::Extc_one_expressionContext *ctx) = 0;

        virtual void enterPlus_expression(oRatioParser::Plus_expressionContext *ctx) = 0;
        virtual void exitPlus_expression(oRatioParser::Plus_expressionContext *ctx) = 0;

        virtual void enterFunction_expression(oRatioParser::Function_expressionContext *ctx) = 0;
        virtual void exitFunction_expression(oRatioParser::Function_expressionContext *ctx) = 0;

        virtual void enterAddition_expression(oRatioParser::Addition_expressionContext *ctx) = 0;
        virtual void exitAddition_expression(oRatioParser::Addition_expressionContext *ctx) = 0;

        virtual void enterParentheses_expression(oRatioParser::Parentheses_expressionContext *ctx) = 0;
        virtual void exitParentheses_expression(oRatioParser::Parentheses_expressionContext *ctx) = 0;

        virtual void enterMinus_expression(oRatioParser::Minus_expressionContext *ctx) = 0;
        virtual void exitMinus_expression(oRatioParser::Minus_expressionContext *ctx) = 0;

        virtual void enterImplication_expression(oRatioParser::Implication_expressionContext *ctx) = 0;
        virtual void exitImplication_expression(oRatioParser::Implication_expressionContext *ctx) = 0;

        virtual void enterLt_expression(oRatioParser::Lt_expressionContext *ctx) = 0;
        virtual void exitLt_expression(oRatioParser::Lt_expressionContext *ctx) = 0;

        virtual void enterNot_expression(oRatioParser::Not_expressionContext *ctx) = 0;
        virtual void exitNot_expression(oRatioParser::Not_expressionContext *ctx) = 0;

        virtual void enterConjunction_expression(oRatioParser::Conjunction_expressionContext *ctx) = 0;
        virtual void exitConjunction_expression(oRatioParser::Conjunction_expressionContext *ctx) = 0;

        virtual void enterGeq_expression(oRatioParser::Geq_expressionContext *ctx) = 0;
        virtual void exitGeq_expression(oRatioParser::Geq_expressionContext *ctx) = 0;

        virtual void enterRange_expression(oRatioParser::Range_expressionContext *ctx) = 0;
        virtual void exitRange_expression(oRatioParser::Range_expressionContext *ctx) = 0;

        virtual void enterMultiplication_expression(oRatioParser::Multiplication_expressionContext *ctx) = 0;
        virtual void exitMultiplication_expression(oRatioParser::Multiplication_expressionContext *ctx) = 0;

        virtual void enterLeq_expression(oRatioParser::Leq_expressionContext *ctx) = 0;
        virtual void exitLeq_expression(oRatioParser::Leq_expressionContext *ctx) = 0;

        virtual void enterGt_expression(oRatioParser::Gt_expressionContext *ctx) = 0;
        virtual void exitGt_expression(oRatioParser::Gt_expressionContext *ctx) = 0;

        virtual void enterConstructor_expression(oRatioParser::Constructor_expressionContext *ctx) = 0;
        virtual void exitConstructor_expression(oRatioParser::Constructor_expressionContext *ctx) = 0;

        virtual void enterDisjunction_expression(oRatioParser::Disjunction_expressionContext *ctx) = 0;
        virtual void exitDisjunction_expression(oRatioParser::Disjunction_expressionContext *ctx) = 0;

        virtual void enterLiteral_expression(oRatioParser::Literal_expressionContext *ctx) = 0;
        virtual void exitLiteral_expression(oRatioParser::Literal_expressionContext *ctx) = 0;

        virtual void enterEq_expression(oRatioParser::Eq_expressionContext *ctx) = 0;
        virtual void exitEq_expression(oRatioParser::Eq_expressionContext *ctx) = 0;

        virtual void enterNeq_expression(oRatioParser::Neq_expressionContext *ctx) = 0;
        virtual void exitNeq_expression(oRatioParser::Neq_expressionContext *ctx) = 0;

        virtual void enterExpr_list(oRatioParser::Expr_listContext *ctx) = 0;
        virtual void exitExpr_list(oRatioParser::Expr_listContext *ctx) = 0;

        virtual void enterLiteral(oRatioParser::LiteralContext *ctx) = 0;
        virtual void exitLiteral(oRatioParser::LiteralContext *ctx) = 0;

        virtual void enterQualified_id(oRatioParser::Qualified_idContext *ctx) = 0;
        virtual void exitQualified_id(oRatioParser::Qualified_idContext *ctx) = 0;

        virtual void enterType(oRatioParser::TypeContext *ctx) = 0;
        virtual void exitType(oRatioParser::TypeContext *ctx) = 0;

        virtual void enterClass_type(oRatioParser::Class_typeContext *ctx) = 0;
        virtual void exitClass_type(oRatioParser::Class_typeContext *ctx) = 0;

        virtual void enterPrimitive_type(oRatioParser::Primitive_typeContext *ctx) = 0;
        virtual void exitPrimitive_type(oRatioParser::Primitive_typeContext *ctx) = 0;

        virtual void enterType_list(oRatioParser::Type_listContext *ctx) = 0;
        virtual void exitType_list(oRatioParser::Type_listContext *ctx) = 0;

        virtual void enterTyped_list(oRatioParser::Typed_listContext *ctx) = 0;
        virtual void exitTyped_list(oRatioParser::Typed_listContext *ctx) = 0;


    };

} // namespace oratio
