grammar oRatio;

compilation_unit
    :   (type_declaration | method_declaration | predicate_declaration | statement)* EOF;

type_declaration
    :   typedef_declaration
    |   enum_declaration
    |   class_declaration;

typedef_declaration
    :   'typedef' primitive_type expr name=ID ';';

enum_declaration
    :   'enum' name=ID enum_constants ('|' enum_constants)* ';';

enum_constants
    :   '{' StringLiteral (',' StringLiteral)* '}'
    |   type;

class_declaration
    :   'class' name=ID (':' type_list)? '{' member* '}';

member
    :   field_declaration
    |   method_declaration
    |   constructor_declaration
    |   predicate_declaration
    |   type_declaration;

field_declaration
    :   type variable_dec (',' variable_dec)* ';';

variable_dec
    :   name=ID ('=' expr)?;

method_declaration
    :   'void' name=ID '(' typed_list? ')' '{' block '}'                #void_method_declaration
    |   type name=ID '(' typed_list? ')' '{' block '}'                  #type_method_declaration;

constructor_declaration
    :   name=ID '(' typed_list? ')' (':' initializer_element (',' initializer_element)*)? '{' block '}';

initializer_element
    :   name=ID '(' expr_list? ')';

predicate_declaration
    :   'predicate' name=ID '(' typed_list? ')' (':' type_list)? '{' block '}';

statement
    :   assignment_statement
    |   local_variable_statement
    |   expression_statement
    |   disjunction_statement
    |   formula_statement
    |   return_statement
    |   '{' block '}';

block
    :   statement*;

assignment_statement
    :   (object=qualified_id '.')? field=ID '=' value=expr ';';

local_variable_statement
    :   type variable_dec (',' variable_dec)* ';';

expression_statement
    :   expr ';';

disjunction_statement
    :	conjunction ('or' conjunction)+;

conjunction
    :   '{' block '}' ('[' cost=expr ']')?;

formula_statement
    :   (goal='goal' | fact='fact') name=ID '=' 'new' (object=qualified_id '.')? predicate=ID '(' assignment_list? ')' ';';

return_statement
    :   'return' expr ';';

assignment_list
    :   assignment (',' assignment)*;

assignment
    :   field=ID ':' value=expr;

expr
    :   literal                                                         # literal_expression
    |   '(' expr ')'                                                    # parentheses_expression
    |   expr ('*' expr)+                                                # multiplication_expression
    |   expr '/' expr                                                   # division_expression
    |   expr ('+' expr)+                                                # addition_expression
    |   expr ('-' expr)+                                                # subtraction_expression
    |   '+' expr                                                        # plus_expression
    |   '-' expr                                                        # minus_expression
    |   '!' expr                                                        # not_expression
    |   qualified_id                                                    # qualified_id_expression
    |   (object=qualified_id '.')? function_name=ID '(' expr_list? ')'  # function_expression
    |   '(' type ')' expr                                               # cast_expression
    |   '[' min=expr ',' max=expr ']'                                   # range_expression
    |   'new' type '(' expr_list? ')'                                   # constructor_expression
    |   expr '==' expr                                                  # eq_expression
    |   expr '>=' expr                                                  # geq_expression
    |   expr '<=' expr                                                  # leq_expression
    |   expr '>' expr                                                   # gt_expression
    |   expr '<' expr                                                   # lt_expression
    |   expr '!=' expr                                                  # neq_expression
    |   expr '->' expr                                                  # implication_expression
    |   expr ('|' expr)+                                                # disjunction_expression
    |   expr ('&' expr)+                                                # conjunction_expression
    |   expr ('^' expr)+                                                # extc_one_expression;

expr_list
    :   expr (',' expr)*;

literal
    :   numeric=NumericLiteral
    |   string=StringLiteral
    |   t='true'
    |   f='false';

qualified_id
    :   (t='this' | ID) ('.' ID)*;

type
    :	class_type
    |   primitive_type;

class_type
    :   ID ('.' ID)*;

primitive_type
    :   'real'
    |   'bool'
    |   'string';

type_list
    :   type (',' type)*;

typed_list
    :   type ID (',' type ID)*;

TYPE_DEF
    :   'typedef';

REAL
    :	'real';

BOOL
    :	'bool';

STRING
    :	'string';

ENUM
    :   'enum';

CLASS
    :   'class';

GOAL
    :   'goal';

FACT
    :   'fact';

PREDICATE
    :   'predicate';

NEW
    :	'new';

OR
    :   'or';

THIS
    :	'this';

VOID
    :   'void';

TRUE
    :   'true';

FALSE
    :   'false';

RETURN
    :   'return';

DOT
    :	'.';

COMMA
    :	',';

COLON
    :	':';

SEMICOLON
    :	';';

LPAREN
    :	'(';

RPAREN
    :	')';

LBRACKET
    :	'[';

RBRACKET
    :	']';

LBRACE
    :	'{';

RBRACE
    :	'}';

PLUS
    :	'+';

MINUS
    :	'-';

STAR
    :	'*';

SLASH
    :	'/';

AMP
    :   '&';

BAR
    :   '|';

EQUAL
    :   '=';

GT
    :   '>';

LT
    :   '<';

BANG
    :   '!';

EQEQ
    :   '==';

LTEQ
    :   '<=';

GTEQ
    :   '>=';

BANGEQ
    :   '!=';

IMPLICATION
    :   '->';

CARET
    :   '^';

ID
    :    ('a'..'z'|'A'..'Z'|'_') ('a'..'z'|'A'..'Z'|'0'..'9'|'_')*;

NumericLiteral
    :   [0-9]+ ('.' [0-9]+)?
    |   '.' [0-9]+;

StringLiteral
    :   '"' (ESC|.)*? '"' ;

fragment
ESC
    : '\\"' | '\\\\' ;

LINE_COMMENT
    : '//' .*? '\r'? '\n' -> skip ;

COMMENT
    : '/*' .*? '*/' -> skip ;

WS
    :  [ \r\t\u000C\n]+ -> skip;