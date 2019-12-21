package it.cnr.istc.oratio.timelines;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Collection;
import java.util.HashSet;
import java.util.IdentityHashMap;
import java.util.Map;
import java.util.Queue;
import java.util.Set;
import java.util.logging.Level;
import java.util.logging.Logger;

import it.cnr.istc.oratio.StateListener;
import it.cnr.istc.oratio.riddle.Atom;
import it.cnr.istc.oratio.riddle.Core;
import it.cnr.istc.oratio.riddle.Item;
import it.cnr.istc.oratio.riddle.Predicate;
import it.cnr.istc.oratio.riddle.Type;
import it.cnr.istc.oratio.riddle.Item.EnumItem;

/**
 * TimelinesList
 */
public class TimelinesList extends ArrayList<Timeline<?>> implements StateListener {

    private static final long serialVersionUID = 1L;
    private static final Logger LOG = Logger.getLogger(TimelinesList.class.getName());
    private static final Map<Type, TimelineBuilder> BUILDERS = new IdentityHashMap<>();

    public TimelinesList(final Core core) {
        try {
            BUILDERS.put(core.getType("StateVariable"), StateVariable.BUILDER);
            BUILDERS.put(core.getType("ReusableResource"), ReusableResource.BUILDER);
            BUILDERS.put(core.getType("PropositionalAgent"), PropositionalAgent.BUILDER);
            BUILDERS.put(core.getType("PropositionalState"), PropositionalState.BUILDER);
        } catch (ClassNotFoundException ex) {
            LOG.log(Level.SEVERE, null, ex);
        }
    }

    @Override
    public void log(String log) {
    }

    @Override
    public void stateChanged(Core core) {
        clear();

        Map<Item, Collection<Atom>> atoms = new IdentityHashMap<>();
        for (Type t : core.getTypes().values()) {
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
        q.addAll(core.getExprs().entrySet());
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
        Queue<Type> q = new ArrayDeque<>();
        q.add(t);
        while (!q.isEmpty()) {
            Type c_type = q.poll();
            if (BUILDERS.containsKey(c_type))
                return c_type;
            if (c_type.getPredicates().values().stream().anyMatch(p -> p.getSuperclasses().stream().anyMatch(
                    sp -> sp.getName().equals("Impulse") || sp.getName().equals("Interval"))))
                try {
                    return t.getCore().getType("PropositionalAgent");
                } catch (ClassNotFoundException e) {
                    e.printStackTrace();
                }
            q.addAll(c_type.getSuperclasses());
        }
        return null;
    }
}