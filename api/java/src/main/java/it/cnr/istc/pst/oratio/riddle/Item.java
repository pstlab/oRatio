package it.cnr.istc.pst.oratio.riddle;

import java.util.Collections;
import java.util.HashMap;
import java.util.Map;

public class Item {

    final Core core;
    final Type type;
    final Map<String, Item> exprs = new HashMap<>();

    Item(final Core core, final Type type) {
        this.core = core;
        this.type = type;
    }

    /**
     * @return the core.
     */
    public Core getCore() {
        return core;
    }

    /**
     * @return the type.
     */
    public Type getType() {
        return type;
    }

    /**
     * @return the items.
     */
    public Map<String, Item> getExprs() {
        return Collections.unmodifiableMap(exprs);
    }

    /**
     * @param name the name of the field identifying the desired item.
     * @return the item having the given name
     */
    public Item getExpr(final String name) {
        return exprs.get(name);
    }

    public static class BoolItem extends Item {

        private final String lit;
        private final LBool val;

        BoolItem(final Core core, final String lit, final LBool val) {
            super(core, core.types.get(Core.BOOL));
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

        ArithItem(Core core, Type type, final String lin, final InfRational lb, final InfRational ub,
                final InfRational val) {
            super(core, type);
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

        StringItem(Core core, final String val) {
            super(core, core.types.get(Core.STRING));
            this.val = val;
        }

        public String getValue() {
            return val;
        }
    }

    public static class EnumItem extends Item {

        private final String var;
        private final Item[] vals;

        EnumItem(Core core, Type type, final String var, final Item[] vals) {
            super(core, type);
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
