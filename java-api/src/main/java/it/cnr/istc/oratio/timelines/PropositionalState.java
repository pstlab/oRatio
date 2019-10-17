package it.cnr.istc.oratio.timelines;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.stream.Collectors;
import java.util.stream.Stream;

import it.cnr.istc.oratio.riddle.Atom;
import it.cnr.istc.oratio.riddle.Core;
import it.cnr.istc.oratio.riddle.Field;
import it.cnr.istc.oratio.riddle.InfRational;
import it.cnr.istc.oratio.riddle.Item;
import it.cnr.istc.oratio.riddle.Predicate;
import it.cnr.istc.oratio.riddle.Item.EnumItem;
import it.cnr.istc.oratio.riddle.Item.BoolItem.LBool;
import it.cnr.istc.utils.CartesianProductGenerator;

/**
 * PropositionalState
 */
public class PropositionalState implements Timeline<PropositionalState.Fluent> {

    public static TimelineBuilder BUILDER = new PropositionalStateBuilder();

    private final InfRational origin, horizon;
    private final Map<String, Fluent> fluents = new LinkedHashMap<>();

    private PropositionalState(final InfRational origin, final InfRational horizon) {
        this.origin = origin;
        this.horizon = horizon;
    }

    /**
     * @return the origin
     */
    @Override
    public InfRational getOrigin() {
        return origin;
    }

    /**
     * @return the horizon
     */
    @Override
    public InfRational getHorizon() {
        return horizon;
    }

    private void addFluent(String name) {
        fluents.put(name, new Fluent(name));
    }

    private Fluent getFluent(String name) {
        return fluents.get(name);
    }

    @Override
    public List<Fluent> getValues() {
        return new ArrayList<>(fluents.values());
    }

    public class Fluent implements TimelineValue, Timeline<PropositionalState.Literal> {

        private final String name;
        private final List<Literal> literals = new ArrayList<>();

        private Fluent(final String name) {
            this.name = name;
        }

        /**
         * @return the name
         */
        public String getName() {
            return name;
        }

        @Override
        public InfRational getOrigin() {
            return origin;
        }

        @Override
        public InfRational getHorizon() {
            return horizon;
        }

        private void addLiteral(final InfRational from, final InfRational to, final Atom atom) {
            literals.add(new Literal(from, to, atom));
        }

        @Override
        public List<Literal> getValues() {
            return Collections.unmodifiableList(literals);
        }

        @Override
        public InfRational getFrom() {
            return origin;
        }

        @Override
        public InfRational getTo() {
            return horizon;
        }
    }

    public class Literal implements TimelineValue {

        private final InfRational from, to;
        private final Atom atom;

        private Literal(final InfRational from, final InfRational to, final Atom atom) {
            this.from = from;
            this.to = to;
            this.atom = atom;
        }

        @Override
        public InfRational getFrom() {
            return from;
        }

        @Override
        public InfRational getTo() {
            return to;
        }

        /**
         * @return the atoms
         */
        public Collection<Atom> getAtoms() {
            return Arrays.asList(atom);
        }

        public LBool getPolarity() {
            return ((Item.BoolItem) atom.getExpr("polarity")).getValue();
        }
    }

    private static class PropositionalStateBuilder implements TimelineBuilder {

        @Override
        public PropositionalState build(Item itm, Collection<Atom> atoms) {
            Core core = itm.getCore();
            PropositionalState ps = new PropositionalState(
                    ((Item.ArithItem) itm.getCore().getExpr("origin")).getValue(),
                    ((Item.ArithItem) itm.getCore().getExpr("horizon")).getValue());

            for (Predicate p : itm.getType().getPredicates().values()) {
                Item[][] itms = new Item[p.getFields().size()][];
                int i = 0;
                for (Field fld : p.getFields().values())
                    itms[i++] = fld.getType().getInstances().toArray(new Item[fld.getType().getInstances().size()]);

                if (itms.length > 0)
                    for (Item[] c_itms : new CartesianProductGenerator<>(itms))
                        ps.addFluent(p.getName() + "(" + Stream.of(c_itms).map(c_itm -> core.guessName(c_itm))
                                .collect(Collectors.joining(", ")) + ")");
                else
                    ps.addFluent(p.getName() + "()");
            }

            for (Atom atm : atoms) {
                String[][] par_vals = atm.getType().getFields().values().stream()
                        .filter(fld -> !fld.getName().equals("tau")).map(fld -> atm.getExpr(fld.getName()))
                        .map(c_itm -> {
                            if (c_itm instanceof EnumItem) {
                                if (((EnumItem) c_itm).getVals().length == 1)
                                    return new String[] { core.guessName(((EnumItem) c_itm).getVals()[0]) };
                                else
                                    return Stream.of(((EnumItem) c_itm).getVals()).map(i -> core.guessName(i))
                                            .toArray(String[]::new);
                            } else
                                return new String[] { core.guessName(c_itm) };
                        }).toArray(String[][]::new);
                if (par_vals.length == 0)
                    ps.getFluent(atm.getType().getName() + "()").addLiteral(
                            ((Item.ArithItem) atm.getExpr("start")).getValue(),
                            ((Item.ArithItem) atm.getExpr("end")).getValue(), atm);
                else
                    for (String[] vals : new CartesianProductGenerator<>(par_vals)) {
                        ps.getFluent(
                                atm.getType().getName() + "(" + Stream.of(vals).collect(Collectors.joining(", ")) + ")")
                                .addLiteral(((Item.ArithItem) atm.getExpr("start")).getValue(),
                                        ((Item.ArithItem) atm.getExpr("end")).getValue(), atm);
                    }
            }

            return ps;
        }
    }
}