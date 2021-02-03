package it.cnr.istc.pst.oratio.timelines;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Collection;
import java.util.HashSet;
import java.util.IdentityHashMap;
import java.util.Map;
import java.util.Queue;
import java.util.Set;

import it.cnr.istc.pst.oratio.Atom;
import it.cnr.istc.pst.oratio.Item;
import it.cnr.istc.pst.oratio.Predicate;
import it.cnr.istc.pst.oratio.Solver;
import it.cnr.istc.pst.oratio.StateListener;
import it.cnr.istc.pst.oratio.Type;
import it.cnr.istc.pst.oratio.Item.EnumItem;

public class TimelinesList extends ArrayList<Timeline<?>> implements StateListener {

    private static final long serialVersionUID = 1L;
    private static final Map<Type, TimelineBuilder> BUILDERS = new IdentityHashMap<>();
    private final Solver solver;

    public TimelinesList(final Solver solver) {
        this.solver = solver;
        try {
            BUILDERS.put(solver.getType("StateVariable"), StateVariable.BUILDER);
            BUILDERS.put(solver.getType("ReusableResource"), ReusableResource.BUILDER);
            BUILDERS.put(solver.getType("PropositionalAgent"), PropositionalAgent.BUILDER);
            BUILDERS.put(solver.getType("PropositionalState"), PropositionalState.BUILDER);
        } catch (ClassNotFoundException e) {
            e.printStackTrace();
        }
    }

    @Override
    public void log(String log) {
        // nothing to do here..
    }

    @Override
    public void init() {
        clear();
    }

    @Override
    public void stateChanged() {
        clear();

        Map<Item, Collection<Atom>> atoms = new IdentityHashMap<>();
        for (Type t : solver.getTypes().values()) {
            if (getTimelineType(t) != null) {
                t.getInstances().forEach(i -> atoms.put(i, new ArrayList<>()));
                for (Predicate p : t.getPredicates().values())
                    p.getInstances().stream().map(atm -> (Atom) atm)
                            .filter(atm -> (atm.getState() == Atom.AtomState.Active)).forEach(atm -> {
                                Item tau = atm.getTau();
                                if (tau instanceof Item.EnumItem)
                                    for (Item val : ((Item.EnumItem) tau).getVals())
                                        atoms.get(val).add(atm);
                                else
                                    atoms.get(tau).add(atm);
                            });
            }
        }

        Set<Item> seen = new HashSet<>();
        Queue<Map.Entry<String, Item>> q = new ArrayDeque<>();
        q.addAll(solver.getExprs().entrySet());
        while (!q.isEmpty()) {
            Map.Entry<String, Item> entry = q.poll();
            if (!seen.contains(entry.getValue()) && !(entry.getValue() instanceof EnumItem)) {
                seen.add(entry.getValue());
                Type type = getTimelineType(entry.getValue().getType());
                if (type != null)
                    add(BUILDERS.get(type).build(entry.getValue(), atoms.get(entry.getValue())));
                q.addAll(entry.getValue().getExprs().entrySet());
            }
        }
    }

    private Type getTimelineType(Type t) {
        try {
            Queue<Type> q = new ArrayDeque<>();
            q.add(t);
            while (!q.isEmpty()) {
                Type c_type = q.poll();
                if (BUILDERS.containsKey(c_type))
                    return c_type;
                if (c_type.getPredicates().values().stream().anyMatch(p -> p.getSuperclasses().stream()
                        .anyMatch(sp -> sp.getName().equals("Impulse") || sp.getName().equals("Interval"))))
                    return t.getSolver().getType("PropositionalAgent");
                q.addAll(c_type.getSuperclasses());
            }
        } catch (ClassNotFoundException e) {
            e.printStackTrace();
        }
        return null;
    }
}
