# Extended Backus-Naur Form

```
<compilation_unit> ::= ( <type_declaration> | <method_declaration> | <predicate_declaration> | <statement>)∗

<type_declaration> ::= <typedef_declaration> | <enum_declaration> | <class_declaration>

<typedef_declaration> ::= 'typedef' <primitive_type> <expr> <ID> ';'

<enum_declaration> ::= 'enum' <ID> <enum_consts> ('|' <enum_consts>)∗ ';'

<enum_consts> ::= '{' StringLit (',' StringLit)∗ '}' | <type>

<class_declaration> ::= 'class' <ID> (':'<type_list>)? '{' <member>∗ '}'

<member> ::= <field_declaration> | <method_declaration> | <constructor_declaration> | <predicate_declaration> | <type_declaration>

<field_declaration> ::= <type> <variable_declaration> (',' <variable_declaration>)∗ ';'

<variable_declaration> ::= <ID> ('='<expr>)?

<method_declaration> ::= 'void' <ID> '(' <typed_list>? ')' '{' <block> '}'
 | <type> <ID> '(' <typed_list>? ')' '{' <block> '}'

<constructor_declaration> ::= <ID> '(' <typed_list>? ')' (':' <init_el> (',' <init_el>)*)? '{' <block> '}'

<init_el> ::= <ID> '(' <expr_list>? ')'

<predicate_declaration> ::= 'predicate' <ID> '(' <typed_list>? ')' (':' <type_list>)? '{' <block> '}'

<stmnt> ::= <assignment_stmnt> | <local_var_stmnt> | <expression_stmnt> | <disjunction_stmnt> | <formula_stmnt> | <return_stmnt> | '{' block '}'

<block> ::= <stmnt>*

<assignment_stmnt> ::= (<q_id> '.')? <ID> '=' <expr> ';'

<local_var_stmnt> ::= <type> <variable_dec> (',' <variable_dec>)* ';'

<expression_stmnt> ::= <expr> ';'

<disjunction_stmnt> ::= <conjunction> ('or' <conjunction>)+

<conjunction> ::= '{' <block> '}' ('[' <expr> ']')?

<formula_stmnt> ::= ('goal' | 'fact') <ID> '=' 'new' (<q_id> '.')? <ID> '(' <assignment_list>? ')' ';'

<return_stmnt> ::= 'return' <expr> ';'

<assignment_list> ::= <assignment> (',' <assignment>)*

<assignment> ::= <ID> ':' <expr>

<expr> ::= <lit> | '(' <expr> ')' | <expr> ('*' <expr>)+ | <expr> ('/' <expr>)+ | <expr> ('+' <expr>)+ | <expr> ('-' <expr>)+ | '+' <expr> | '-' <expr> | '!' <expr> | (<q_id> '.')? <ID> '(' <expr_list?> ')' | '(' <type> ')' <expr> | '[' <expr> ',' <expr> ']' | 'new' <type> '(' <expr_list>? ')' | <expr> '==' <expr> | <expr> '>=' <expr> | <expr> '<=' <expr> | <expr> '>' <expr> | <expr> '<' <expr> | <expr> '!=' <expr> | <expr> '->' <expr> | <expr> ('|' <expr>)+ | <expr> ('&' <expr>)+ | <expr> ('^' <expr>)+

<expr_list> ::= <expr> (',' <expr>)*

<lit> ::= <NumericLit> | <StringLit> | 'true' | 'false'

<q_id> ::= ('this' | <ID>) ('.' <ID>)*;

<type> ::= <class_type> | <primitive_type>

<class_type> ::= <ID> ('.' <ID>)*

<primitive_type> ::= 'int' | 'real' | 'bool' | 'string'

<type_list> ::= <type> (',' <type>)*

<typed_list> ::= <type> <ID> (',' <type> <ID>)*

<ID> ::= ('a'..'z'|'A'..'Z'|'_') ('a'..'z'|'A'..'Z'|'0'..'9'|'_')*

<NumericLit> ::= [0-9]+ ('.' [0-9]+)? | '.' [0-9]+

<StringLit> ::= '"' (ESC|.)*? '"'

<ESC> ::= '\\"' | '\\\\'
```