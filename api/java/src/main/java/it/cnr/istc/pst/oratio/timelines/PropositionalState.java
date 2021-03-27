package it.cnr.istc.pst.oratio.timelines;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.stream.Collectors;
import java.util.stream.Stream;

import it.cnr.istc.pst.oratio.Atom;
import it.cnr.istc.pst.oratio.Field;
import it.cnr.istc.pst.oratio.InfRational;
import it.cnr.istc.pst.oratio.Item;
import it.cnr.istc.pst.oratio.Predicate;
import it.cnr.istc.pst.oratio.Solver;
import it.cnr.istc.pst.oratio.Item.ArithItem;
import it.cnr.istc.pst.oratio.Item.BoolItem;
import it.cnr.istc.pst.oratio.Item.BoolItem.LBool;
import it.cnr.istc.pst.oratio.Item.EnumItem;
import it.cnr.istc.pst.utils.CartesianProductGenerator;

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

    private void addFluent(final String name) {
        fluents.put(name, new Fluent(name));
    }

    private Fluent getFluent(final String name) {
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
            try {
                return ((BoolItem) atom.get("polarity")).getValue();
            } catch (final NoSuchFieldException e) {
                e.printStackTrace();
                return null;
            }
        }
    }

    private static class PropositionalStateBuilder implements TimelineBuilder {

        @Override
        public PropositionalState build(final Item itm, final Collection<Atom> atoms) {
            try {
                final PropositionalState ps = new PropositionalState(
                        ((ArithItem) itm.getSolver().get(Solver.ORIGIN)).getValue(),
                        ((ArithItem) itm.getSolver().get(Solver.HORIZON)).getValue());

                for (final Predicate p : itm.getType().getPredicates().values()) {
                    final Item[][] itms = new Item[p.getFields().size()][];
                    int i = 0;
                    for (final Field fld : p.getFields().values())
                        itms[i++] = fld.getType().getInstances().toArray(new Item[fld.getType().getInstances().size()]);

                    if (itms.length > 0)
                        for (final Item[] c_itms : new CartesianProductGenerator<>(itms))
                            ps.addFluent(p.getName() + "("
                                    + Stream.of(c_itms).map(c_itm -> c_itm.getName()).collect(Collectors.joining(", "))
                                    + ")");
                    else
                        ps.addFluent(p.getName() + "()");
                }

                for (final Atom atm : atoms) {
                    final String[][] par_vals = atm.getType().getFields().values().stream()
                            .filter(fld -> !fld.getName().equals("tau")).map(fld -> {
                                try {
                                    return atm.get(fld.getName());
                                } catch (final NoSuchFieldException e) {
                                    e.printStackTrace();
                                    return null;
                                }
                            }).map(c_itm -> {
                                if (c_itm instanceof EnumItem) {
                                    if (((EnumItem) c_itm).getVals().length == 1)
                                        return new String[] { ((EnumItem) c_itm).getVals()[0].getName() };
                                    else
                                        return Stream.of(((EnumItem) c_itm).getVals()).map(i -> i.getName())
                                                .toArray(String[]::new);
                                } else
                                    return new String[] { c_itm.getName() };
                            }).toArray(String[][]::new);
                    if (par_vals.length == 0)
                        ps.getFluent(atm.getType().getName() + "()").addLiteral(
                                ((ArithItem) atm.get(Solver.START)).getValue(),
                                ((ArithItem) atm.get(Solver.END)).getValue(), atm);
                    else
                        for (final String[] vals : new CartesianProductGenerator<>(par_vals)) {
                            ps.getFluent(atm.getType().getName() + "("
                                    + Stream.of(vals).collect(Collectors.joining(", ")) + ")")
                                    .addLiteral(((ArithItem) atm.get(Solver.START)).getValue(),
                                            ((ArithItem) atm.get(Solver.END)).getValue(), atm);
                        }
                }

                return ps;
            } catch (final NoSuchFieldException e) {
                e.printStackTrace();
                return null;
            }
        }
    }
}
