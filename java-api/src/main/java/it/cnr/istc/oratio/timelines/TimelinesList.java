package it.cnr.istc.oratio.timelines;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Collection;
import java.util.IdentityHashMap;
import java.util.Map;
import java.util.Queue;
import java.util.logging.Level;
import java.util.logging.Logger;

import it.cnr.istc.oratio.StateListener;
import it.cnr.istc.oratio.riddle.Atom;
import it.cnr.istc.oratio.riddle.Core;
import it.cnr.istc.oratio.riddle.Item;
import it.cnr.istc.oratio.riddle.Predicate;
import it.cnr.istc.oratio.riddle.Type;

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
            // timeline_builders.put(core.getType("PropositionalState"), new
            // PropositionalStateBuilder(core));
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

        for (Map.Entry<String, Item> entry : core.getExprs().entrySet()) {
            Type type = getTimelineType(entry.getValue().getType());
            if (type != null)
                add(BUILDERS.get(type).build(entry.getValue(), atoms.get(entry.getValue())));
        }
    }

    private Type getTimelineType(Type t) {
        Queue<Type> q = new ArrayDeque<>();
        q.add(t);
        while (!q.isEmpty()) {
            Type c_type = q.poll();
            if (BUILDERS.containsKey(c_type))
                return c_type;
            q.addAll(c_type.getSuperclasses());
        }
        return null;
    }
}