package it.cnr.istc.pst.oratio;

import java.util.Collections;
import java.util.HashMap;
import java.util.Map;

public class Item implements Env {

    final Solver solver;
    final Type type;
    final Map<String, Item> exprs = new HashMap<>();
    private String name; // a mnemonic name..

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

    /**
     * @return the mnemonic name of the item.
     */
    public String getName() {
        return name;
    }

    protected void setName(String name) {
        this.name = name;
    }

    @Override
    public Item get(final String name) throws NoSuchFieldException {
        final Item item = exprs.get(name);
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

    @SuppressWarnings("unused")
    private void set(final String id, final Item itm) {
        exprs.put(id, itm);
    }

    public static class BoolItem extends Item {

        private final String lit;
        private LBool val;

        @SuppressWarnings("unused")
        private BoolItem(final Solver solver, final String lit, final byte val) {
            this(solver, lit, LBool.values()[val]);
        }

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

        @SuppressWarnings("unused")
        private void setValue(final byte val) {
            this.val = LBool.values()[val];
        }

        public enum LBool {
            False {
                @Override
                public Boolean booleanValue() {
                    return false;
                }
            },
            True {
                @Override
                public Boolean booleanValue() {
                    return true;
                }
            },
            Undefined;

            public Boolean booleanValue() {
                return null;
            }
        }
    }

    public static class ArithItem extends Item {

        private final String lin;
        private final InfRational lb, ub, val;

        ArithItem(final Solver solver, final Type type, final String lin, final InfRational lb, final InfRational ub,
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

        @SuppressWarnings("unused")
        private void setLb(final long lb_rat_num, final long lb_rat_den, final long lb_inf_num, final long lb_inf_den) {
            this.lb.rat.num = lb_rat_num;
            this.lb.rat.den = lb_rat_den;
            this.lb.inf.num = lb_inf_num;
            this.lb.inf.den = lb_inf_den;
        }

        @SuppressWarnings("unused")
        private void setUb(final long ub_rat_num, final long ub_rat_den, final long ub_inf_num, final long ub_inf_den) {
            this.ub.rat.num = ub_rat_num;
            this.ub.rat.den = ub_rat_den;
            this.ub.inf.num = ub_inf_num;
            this.ub.inf.den = ub_inf_den;
        }

        @SuppressWarnings("unused")
        private void setVal(final long val_rat_num, final long val_rat_den, final long val_inf_num,
                final long val_inf_den) {
            this.val.rat.num = val_rat_num;
            this.val.rat.den = val_rat_den;
            this.val.inf.num = val_inf_num;
            this.val.inf.den = val_inf_den;
        }
    }

    public static class EnumItem extends Item {

        private final String var;
        private Item[] vals;

        EnumItem(final Solver solver, final Type type, final String var, final Item[] vals) {
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

        @SuppressWarnings("unused")
        private void setVals(final Item[] vals) {
            this.vals = vals;
        }
    }

    public static class StringItem extends Item {

        private String val;

        StringItem(final Solver solver, final String val) {
            super(solver, solver.types.get(Solver.STRING));
            this.val = val;
        }

        public String getValue() {
            return val;
        }

        @SuppressWarnings("unused")
        private void setValue(final String val) {
            this.val = val;
        }
    }
}
