package it.cnr.istc.pst.oratio.riddle;

import java.util.Map;
import java.util.stream.Collectors;
import java.util.stream.Stream;

public class Atom extends Item {

    private final long sigma;
    private final AtomState state;

    Atom(Core core, Predicate predicate, final long sigma, final AtomState state, final Map<String, Item> pars) {
        super(core, predicate);
        exprs.putAll(pars);
        this.sigma = sigma;
        this.state = state;
    }

    /**
     * @return the sigma
     */
    public long getSigma() {
        return sigma;
    }

    @Override
    public Predicate getType() {
        return (Predicate) super.getType();
    }

    public AtomState getState() {
        return state;
    }

    public Item getTau() {
        return exprs.get("tau");
    }

    public enum AtomState {
        Active, Inactive, Unified
    }

    @Override
    public String toString() {
        return "σ" + sigma + " " + type.getName() + "(" + exprs.entrySet().stream().map(expr -> {
            switch (expr.getValue().getType().getName()) {
                case Core.BOOL:
                    return expr.getKey() + " = " + ((Item.BoolItem) expr.getValue()).getValue();
                case Core.INT:
                case Core.REAL:
                case Core.TP:
                    return expr.getKey() + " = " + ((Item.ArithItem) expr.getValue()).getValue();
                case Core.STRING:
                    return expr.getKey() + " = " + ((Item.StringItem) expr.getValue()).getValue();
                default:
                    String val = expr.getKey() + " = ";
                    if (expr.getValue() instanceof EnumItem) {
                        if (((EnumItem) expr.getValue()).getVals().length == 1)
                            val += core.guessName(((EnumItem) expr.getValue()).getVals()[0]);
                        else
                            val += Stream.of(((EnumItem) expr.getValue()).getVals()).map(itm -> core.guessName(itm))
                                    .collect(Collectors.joining(", "));
                    } else
                        val += core.guessName(expr.getValue());
                    return val;
            }
        }).collect(Collectors.joining(", ")) + ")";
    }
}
