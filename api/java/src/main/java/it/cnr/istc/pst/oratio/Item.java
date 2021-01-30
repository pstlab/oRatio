package it.cnr.istc.pst.oratio;

import java.util.Collections;
import java.util.HashMap;
import java.util.Map;

public class Item implements Env {

    final Solver solver;
    final Type type;
    final Map<String, Item> exprs = new HashMap<>();

    Item(final Solver solver, final Type type) {
        this.solver = solver;
        this.type = type;
    }

    /**
     * @return the solver.
     */
    @Override
    public Solver getSolver() {
        return solver;
    }

    /**
     * @return the type.
     */
    public Type getType() {
        return type;
    }

    @Override
    public Item get(String name) throws NoSuchFieldException {
        Item item = exprs.get(name);
        if (item != null)
            return item;

        // not found
        throw new NoSuchFieldException(name);
    }

    /**
     * @return the items.
     */
    @Override
    public Map<String, Item> getExprs() {
        return Collections.unmodifiableMap(exprs);
    }

    public static class BoolItem extends Item {

        private final String lit;
        private final LBool val;

        BoolItem(final Solver solver, final String lit, final LBool val) {
            super(solver, solver.types.get(Solver.BOOL));
            this.lit = lit;
            this.val = val;
        }

        public String getLit() {
            return lit;
        }

        public LBool getValue() {
            return val;
        }

        public enum LBool {
            False, True, Undefined
        }
    }

    public static class ArithItem extends Item {

        private final String lin;
        private final InfRational lb, ub, val;

        ArithItem(final Solver solver, Type type, final String lin, final InfRational lb, final InfRational ub,
                final InfRational val) {
            super(solver, type);
            this.lin = lin;
            this.lb = lb;
            this.ub = ub;
            this.val = val;
        }

        public String getLin() {
            return lin;
        }

        public InfRational getLb() {
            return lb;
        }

        public InfRational getUb() {
            return ub;
        }

        public InfRational getValue() {
            return val;
        }
    }

    public static class StringItem extends Item {

        private final String val;

        StringItem(final Solver solver, final String val) {
            super(solver, solver.types.get(Solver.STRING));
            this.val = val;
        }

        public String getValue() {
            return val;
        }
    }

    public static class EnumItem extends Item {

        private final String var;
        private final Item[] vals;

        EnumItem(final Solver solver, Type type, final String var, final Item[] vals) {
            super(solver, type);
            this.var = var;
            this.vals = vals;
        }

        public String getVar() {
            return var;
        }

        public Item[] getVals() {
            return vals;
        }
    }
}
