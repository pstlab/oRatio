package it.cnr.istc.pst.oratio.timelines;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.List;

import it.cnr.istc.pst.oratio.Atom;
import it.cnr.istc.pst.oratio.InfRational;
import it.cnr.istc.pst.oratio.Item;
import it.cnr.istc.pst.oratio.Solver;
import it.cnr.istc.pst.oratio.Item.ArithItem;

public class PropositionalAgent implements Timeline<PropositionalAgent.Action> {

    public static TimelineBuilder BUILDER = new PropositionalAgentBuilder();

    private final String name;
    private final InfRational origin, horizon;
    private final List<Action> values = new ArrayList<>();

    private PropositionalAgent(final String name, final InfRational origin, final InfRational horizon) {
        this.name = name;
        this.origin = origin;
        this.horizon = horizon;
    }

    /**
     * @return the name
     */
    public String getName() {
        return name;
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

    private void addValue(final InfRational from, final InfRational to, final Atom atom) {
        values.add(new Action(from, to, atom));
    }

    @Override
    public List<Action> getValues() {
        return Collections.unmodifiableList(values);
    }

    public class Action implements TimelineValue {

        private final InfRational from, to;
        private final Atom atom;

        private Action(final InfRational from, final InfRational to, final Atom atom) {
            this.from = from;
            this.to = to;
            this.atom = atom;
        }

        public boolean isImpulsive() {
            return from == to;
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
        public Atom getAtom() {
            return atom;
        }
    }

    private static class PropositionalAgentBuilder implements TimelineBuilder {

        @Override
        public PropositionalAgent build(Item itm, Collection<Atom> atoms) {
            try {
                PropositionalAgent pa = new PropositionalAgent(itm.getSolver().guessName(itm),
                        ((ArithItem) itm.getSolver().get(Solver.ORIGIN)).getValue(),
                        ((ArithItem) itm.getSolver().get(Solver.HORIZON)).getValue());

                atoms.stream()
                        .filter(atm -> atm.getType().getSuperclasses().stream().anyMatch(
                                t -> t.getName().equals(Solver.IMPULSE) || t.getName().equals(Solver.INTERVAL)))
                        .sorted((Atom a0, Atom a1) -> {
                            try {
                                ArithItem a0_start = (ArithItem) a0.get(a0.getType().getSuperclasses().stream()
                                        .anyMatch(t -> t.getName().equals(Solver.IMPULSE)) ? Solver.AT : Solver.START);
                                ArithItem a1_start = (ArithItem) a1.get(a1.getType().getSuperclasses().stream()
                                        .anyMatch(t -> t.getName().equals(Solver.IMPULSE)) ? Solver.AT : Solver.START);
                                return a0_start.getValue().compareTo(a1_start.getValue());
                            } catch (NoSuchFieldException e) {
                                e.printStackTrace();
                                return 0;
                            }
                        }).forEach(atm -> {
                            try {
                                if (atm.getType().getSuperclasses().stream()
                                        .anyMatch(t -> t.getName().equals(Solver.IMPULSE)))
                                    pa.addValue(((ArithItem) atm.get(Solver.AT)).getValue(),
                                            ((ArithItem) atm.get(Solver.AT)).getValue(), atm);
                                else
                                    pa.addValue(((ArithItem) atm.get(Solver.START)).getValue(),
                                            ((ArithItem) atm.get(Solver.END)).getValue(), atm);
                            } catch (NoSuchFieldException e) {
                                e.printStackTrace();
                            }
                        });

                return pa;
            } catch (NoSuchFieldException e) {
                e.printStackTrace();
                return null;
            }
        }
    }
}
