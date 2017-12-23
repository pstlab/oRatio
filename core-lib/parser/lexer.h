#pragma once

namespace ratio
{

enum symbol
{
  BOOL_ID,          // 'bool'
  INT_ID,           // 'int'
  REAL_ID,          // 'real'
  STRING_ID,        // 'string'
  TYPEDEF_ID,       // 'typedef'
  ENUM_ID,          // 'enum'
  CLASS_ID,         // 'class'
  GOAL_ID,          // 'goal'
  FACT_ID,          // 'fact'
  PREDICATE_ID,     // 'predicate'
  NEW_ID,           // 'new'
  OR_ID,            // 'or'
  THIS_ID,          // 'this'
  VOID_ID,          // 'void'
  TRUE_ID,          // 'true'
  FALSE_ID,         // 'false'
  RETURN_ID,        // 'return'
  DOT_ID,           // '.'
  COMMA_ID,         // ','
  COLON_ID,         // ':'
  SEMICOLON_ID,     // ';'
  LPAREN_ID,        // '('
  RPAREN_ID,        // ')'
  LBRACKET_ID,      // '['
  RBRACKET_ID,      // ']'
  LBRACE_ID,        // '{'
  RBRACE_ID,        // '}'
  PLUS_ID,          // '+'
  MINUS_ID,         // '-'
  STAR_ID,          // '*'
  SLASH_ID,         // '/'
  AMP_ID,           // '&'
  BAR_ID,           // '|'
  EQ_ID,            // '='
  GT_ID,            // '>'
  LT_ID,            // '<'
  BANG_ID,          // '!'
  EQEQ_ID,          // '=='
  LTEQ_ID,          // '<='
  GTEQ_ID,          // '>='
  BANGEQ_ID,        // '!='
  IMPLICATION_ID,   // '->'
  CARET_ID,         // '^'
  ID_ID,            // ('a'..'z'|'A'..'Z'|'_') ('a'..'z'|'A'..'Z'|'0'..'9'|'_')*
  IntLiteral_ID,    // [0-9]+
  RealLiteral_ID,   // [0-9]+ '.' [0-9]+)? | '.' [0-9]+
  StringLiteral_ID, // '" . . ."'
  EOF_ID
};
}