# SeMiTONE

Satisfiability Modulo Theories (SMT) concerns the satisfiability of formulas with respect to some background theory.
SeMiTONE is a Satisfiability Modulo TheOries NEtwork, allowing the creation of variables and constraints in different underlying theories. Although new theories can be easily integrated, SeMiTONE currently manages a a linear real arithmetic theory and an object variable theory and an integer and real difference logic theory.

SeMiTONE maintains backtrackable data structures, allows the creation of variables and constraints, performs constraint propagation and, whenever conflicts arise, performs conflict analysis, learns a no-good and backjumps to the highest level. It is worth noting that SeMiTONE is not an SMT solver. SeMiTONE is, on the contrary, a network on top of which SMT solvers can be built. In this regard, SeMiTONE deliberately neglects all the aspects related to 'search' as, for example, search algorithms and resolution heuristics, demanding to external modules solving SMT problems.

## Usage

At the core of SeMiTONE there is the `sat_core` module which allows the creation of propositional variables and constraints. Propositional variables are identified through integers. The clause creation procedure introduces a new clause, represented by an array of (direct or negated) literals, into the network, returning `false` if some trivial inconsistency is recognized. It is worth noting that in case the clause creation procedure returns `true` there is no guarantee that the network is still consistent since identifying inconsistencies might occur only after a search process.

```cpp
sat_core sat;

// we create two propositional variables
var b0 = sat.new_var();
var b1 = sat.new_var();

// we create a propositional constraint (i.e. the (¬b0 ∨ b1) clause)
bool nc = sat.new_clause({lit(b0, false), b1});

// the current value of 'b0' (i.e. Undefined)
lbool b0_val = sat.value(b0);
```

Once propositional variables and constraints are created, it is possible to assume values for the variables and verify the consequences through propagation. Assuming a value for a propositional variable stores the context of the network allowing subsequent backtracking (i.e. restoring the context prior of the assignment). While the variable assumption procedure returns `false` if some trivial inconsistency is introduced by the assumption, the propagation procedure might recognize an inconsistency, generate a no-good and backtrack at the highest possible level, returning `false` only in case the network becomes definitely inconsistent, independently from possible subsequent assignments.

```cpp
// we store the context and assume b0
bool assm = core.assume(lit(b0));

// the current value of 'b0' is now True as a consequence of the assignment
b0_val = sat.value(b0);
// the current value of 'b1' is now True as a consequence of the propagation
lbool b1_val = sat.value(b1);
```

Finally, it is possible to restore the context prior of the assignment through the `pop()` procedure.

```cpp
sat.pop();

// the current values of the 'b0' and 'b1' variables is now back to Undefined
b0_val = sat.value(b0);
b1_val = sat.value(b1);
```