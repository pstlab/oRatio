package it.cnr.istc.pst.oratio;

import java.io.IOException;
import java.util.Map;
import java.util.stream.Collectors;
import java.util.stream.Stream;

import com.fasterxml.jackson.core.JsonGenerator;
import com.fasterxml.jackson.databind.SerializerProvider;
import com.fasterxml.jackson.databind.annotation.JsonSerialize;
import com.fasterxml.jackson.databind.ser.std.StdSerializer;

@JsonSerialize(using = Atom.AtomSerializer.class)
public class Atom extends Item {

    private final long sigma;
    private AtomState state;

    @SuppressWarnings("unused")
    private Atom(final Solver solver, final Predicate predicate, final long sigma, final byte state) {
        this(solver, predicate, sigma, AtomState.values()[state]);
    }

    Atom(final Solver solver, final Predicate predicate, final long sigma, final AtomState state) {
        super(solver, predicate);
        this.sigma = sigma;
        this.state = state;
        setName("σ" + sigma);
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

    @SuppressWarnings("unused")
    private void setState(final byte state) {
        this.state = AtomState.values()[state];
    }

    public Item getTau() {
        return exprs.get("tau");
    }

    public enum AtomState {
        Unified, Active, Inactive
    }

    @Override
    public String toString() {
        return "σ" + sigma + " " + type.getName() + "(" + exprs.entrySet().stream().map(expr -> {
            switch (expr.getValue().getType().getName()) {
                case Solver.BOOL:
                    return expr.getKey() + " = " + ((Item.BoolItem) expr.getValue()).getValue();
                case Solver.INT:
                case Solver.REAL:
                case Solver.TP:
                    return expr.getKey() + " = " + ((Item.ArithItem) expr.getValue()).getValue();
                case Solver.STRING:
                    return expr.getKey() + " = " + ((Item.StringItem) expr.getValue()).getValue();
                default:
                    String val = expr.getKey() + " = ";
                    if (expr.getValue() instanceof EnumItem) {
                        if (((EnumItem) expr.getValue()).getVals().length == 1)
                            val += ((EnumItem) expr.getValue()).getVals()[0].getName();
                        else
                            val += Stream.of(((EnumItem) expr.getValue()).getVals()).map(itm -> itm.getName())
                                    .collect(Collectors.joining(", "));
                    } else
                        val += expr.getValue().getName();
                    return val;
            }
        }).collect(Collectors.joining(", ")) + ")";
    }

    static class AtomSerializer extends StdSerializer<Atom> {

        private AtomSerializer() {
            super(Atom.class);
        }

        @Override
        public void serialize(final Atom value, final JsonGenerator gen, final SerializerProvider provider)
                throws IOException {
            gen.writeStartObject();
            gen.writeNumberField("sigma", value.sigma);
            gen.writeStringField("predicate", value.type.name);
            for (final Map.Entry<String, Item> expr : value.exprs.entrySet()) {
                switch (expr.getValue().getType().getName()) {
                    case Solver.BOOL:
                        gen.writeBooleanField(expr.getKey(),
                                ((Item.BoolItem) expr.getValue()).getValue().booleanValue());
                        break;
                    case Solver.INT:
                    case Solver.REAL:
                    case Solver.TP:
                        gen.writeNumberField(expr.getKey(),
                                ((Item.ArithItem) expr.getValue()).getValue().doubleValue());
                        break;
                    case Solver.STRING:
                        gen.writeStringField(expr.getKey(), ((Item.StringItem) expr.getValue()).getValue());
                        break;
                    default:
                        if (expr.getValue() instanceof EnumItem) {
                            if (((EnumItem) expr.getValue()).getVals().length == 1)
                                gen.writeStringField(expr.getKey(),
                                        ((EnumItem) expr.getValue()).getVals()[0].getName());
                            else {
                                gen.writeArrayFieldStart(expr.getKey());
                                for (final Item val : ((EnumItem) expr.getValue()).getVals()) {
                                    gen.writeString(val.getName());
                                }
                                gen.writeEndArray();
                            }
                        } else
                            gen.writeStringField(expr.getKey(), expr.getValue().getName());
                }
            }
            gen.writeEndObject();
        }
    }
}
