# Backus-Naur Form

```
<compilation_unit> ::= (<statement>|<rule>)*;

<statement> ::= <assert_statement> |
                <retract_statement> |
                <assignment_statement> |
                <function_statement>;

<assert_statement> ::= 'assert' <fact_expression> ';';

<retract_statement> ::= 'retract' ID (',' ID)* ';';

<assignment_statement> ::= ID ('.' ID)? '=' <expression> (',' ID ('.' ID)? '=' <expression>)* ';';

<function_statement> ::= ID '(' (<expression> (',' <expression>)*)? ')' ';';

<expression> ::= <expression> '&' <expression> |
                 <expression> '|' <expression> |
                 '!' <expression> |
                 <expression> '<' <expression> |
                 <expression> '<=' <expression> |
                 <expression> '==' <expression> |
                 <expression> '!=' <expression> |
                 <expression> '>=' <expression> |
                 <expression> '>' <expression> |
                 <expression> '+' <expression> |
                 <expression> '-' <expression> |
                 <expression> '*' <expression> |
                 <expression> '/' <expression> |
                 '(' <expression> ')' |
                 <fact_expression> |
                 <ID> ('.' ID)? |
                 IntLiteral |
                 RealLiteral |
                 StringLiteral |
                 BoolLiteral;

<fact_expression> ::= (ID ':')? ID '(' (ID ':' <expression> (',' ID ':' <expression>)*)? ')';

<rule> ::= '(' (ID ':')? <expression> '=>' <statement>* ')';
```