
// Generated from oRatio.g4 by ANTLR 4.5.3

#pragma once


#include "antlr4-runtime.h"
#include "oRatioParser.h"


namespace oratio {

	/**
	 * This class defines an abstract visitor for a parse tree
	 * produced by oRatioParser.
	 */
	class oRatioVisitor : public antlr4::tree::AbstractParseTreeVisitor {
	public:

		/**
		 * Visit parse trees produced by oRatioParser.
		 */
		virtual antlrcpp::Any visitCompilation_unit(oRatioParser::Compilation_unitContext *context) = 0;

		virtual antlrcpp::Any visitType_declaration(oRatioParser::Type_declarationContext *context) = 0;

		virtual antlrcpp::Any visitTypedef_declaration(oRatioParser::Typedef_declarationContext *context) = 0;

		virtual antlrcpp::Any visitEnum_declaration(oRatioParser::Enum_declarationContext *context) = 0;

		virtual antlrcpp::Any visitEnum_constants(oRatioParser::Enum_constantsContext *context) = 0;

		virtual antlrcpp::Any visitClass_declaration(oRatioParser::Class_declarationContext *context) = 0;

		virtual antlrcpp::Any visitMember(oRatioParser::MemberContext *context) = 0;

		virtual antlrcpp::Any visitField_declaration(oRatioParser::Field_declarationContext *context) = 0;

		virtual antlrcpp::Any visitVariable_dec(oRatioParser::Variable_decContext *context) = 0;

		virtual antlrcpp::Any visitVoid_method_declaration(oRatioParser::Void_method_declarationContext *context) = 0;

		virtual antlrcpp::Any visitType_method_declaration(oRatioParser::Type_method_declarationContext *context) = 0;

		virtual antlrcpp::Any visitConstructor_declaration(oRatioParser::Constructor_declarationContext *context) = 0;

		virtual antlrcpp::Any visitInitializer_element(oRatioParser::Initializer_elementContext *context) = 0;

		virtual antlrcpp::Any visitPredicate_declaration(oRatioParser::Predicate_declarationContext *context) = 0;

		virtual antlrcpp::Any visitStatement(oRatioParser::StatementContext *context) = 0;

		virtual antlrcpp::Any visitBlock(oRatioParser::BlockContext *context) = 0;

		virtual antlrcpp::Any visitAssignment_statement(oRatioParser::Assignment_statementContext *context) = 0;

		virtual antlrcpp::Any visitLocal_variable_statement(oRatioParser::Local_variable_statementContext *context) = 0;

		virtual antlrcpp::Any visitExpression_statement(oRatioParser::Expression_statementContext *context) = 0;

		virtual antlrcpp::Any visitDisjunction_statement(oRatioParser::Disjunction_statementContext *context) = 0;

		virtual antlrcpp::Any visitConjunction(oRatioParser::ConjunctionContext *context) = 0;

		virtual antlrcpp::Any visitFormula_statement(oRatioParser::Formula_statementContext *context) = 0;

		virtual antlrcpp::Any visitReturn_statement(oRatioParser::Return_statementContext *context) = 0;

		virtual antlrcpp::Any visitAssignment_list(oRatioParser::Assignment_listContext *context) = 0;

		virtual antlrcpp::Any visitAssignment(oRatioParser::AssignmentContext *context) = 0;

		virtual antlrcpp::Any visitCast_expression(oRatioParser::Cast_expressionContext *context) = 0;

		virtual antlrcpp::Any visitQualified_id_expression(oRatioParser::Qualified_id_expressionContext *context) = 0;

		virtual antlrcpp::Any visitDivision_expression(oRatioParser::Division_expressionContext *context) = 0;

		virtual antlrcpp::Any visitSubtraction_expression(oRatioParser::Subtraction_expressionContext *context) = 0;

		virtual antlrcpp::Any visitExtc_one_expression(oRatioParser::Extc_one_expressionContext *context) = 0;

		virtual antlrcpp::Any visitPlus_expression(oRatioParser::Plus_expressionContext *context) = 0;

		virtual antlrcpp::Any visitFunction_expression(oRatioParser::Function_expressionContext *context) = 0;

		virtual antlrcpp::Any visitAddition_expression(oRatioParser::Addition_expressionContext *context) = 0;

		virtual antlrcpp::Any visitParentheses_expression(oRatioParser::Parentheses_expressionContext *context) = 0;

		virtual antlrcpp::Any visitMinus_expression(oRatioParser::Minus_expressionContext *context) = 0;

		virtual antlrcpp::Any visitImplication_expression(oRatioParser::Implication_expressionContext *context) = 0;

		virtual antlrcpp::Any visitLt_expression(oRatioParser::Lt_expressionContext *context) = 0;

		virtual antlrcpp::Any visitNot_expression(oRatioParser::Not_expressionContext *context) = 0;

		virtual antlrcpp::Any visitConjunction_expression(oRatioParser::Conjunction_expressionContext *context) = 0;

		virtual antlrcpp::Any visitGeq_expression(oRatioParser::Geq_expressionContext *context) = 0;

		virtual antlrcpp::Any visitRange_expression(oRatioParser::Range_expressionContext *context) = 0;

		virtual antlrcpp::Any visitMultiplication_expression(oRatioParser::Multiplication_expressionContext *context) = 0;

		virtual antlrcpp::Any visitLeq_expression(oRatioParser::Leq_expressionContext *context) = 0;

		virtual antlrcpp::Any visitGt_expression(oRatioParser::Gt_expressionContext *context) = 0;

		virtual antlrcpp::Any visitConstructor_expression(oRatioParser::Constructor_expressionContext *context) = 0;

		virtual antlrcpp::Any visitDisjunction_expression(oRatioParser::Disjunction_expressionContext *context) = 0;

		virtual antlrcpp::Any visitLiteral_expression(oRatioParser::Literal_expressionContext *context) = 0;

		virtual antlrcpp::Any visitEq_expression(oRatioParser::Eq_expressionContext *context) = 0;

		virtual antlrcpp::Any visitNeq_expression(oRatioParser::Neq_expressionContext *context) = 0;

		virtual antlrcpp::Any visitExpr_list(oRatioParser::Expr_listContext *context) = 0;

		virtual antlrcpp::Any visitLiteral(oRatioParser::LiteralContext *context) = 0;

		virtual antlrcpp::Any visitQualified_id(oRatioParser::Qualified_idContext *context) = 0;

		virtual antlrcpp::Any visitType(oRatioParser::TypeContext *context) = 0;

		virtual antlrcpp::Any visitClass_type(oRatioParser::Class_typeContext *context) = 0;

		virtual antlrcpp::Any visitPrimitive_type(oRatioParser::Primitive_typeContext *context) = 0;

		virtual antlrcpp::Any visitType_list(oRatioParser::Type_listContext *context) = 0;

		virtual antlrcpp::Any visitTyped_list(oRatioParser::Typed_listContext *context) = 0;


	};

} // namespace oratio
