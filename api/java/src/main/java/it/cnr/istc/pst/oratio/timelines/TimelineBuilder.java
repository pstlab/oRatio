package it.cnr.istc.pst.oratio.timelines;

import java.util.Collection;

import it.cnr.istc.pst.oratio.Atom;
import it.cnr.istc.pst.oratio.Item;

/**
 * Classes implementind this interface are intended to build the timelines
 * starting from the atoms which are associated to them.
 */
public interface TimelineBuilder {

    /**
     * Starting from the atoms associated to a timeline, builds the {@code Timeline}
     * as a sequence of {@code TimelineValue}s.
     * 
     * @param itm   the item representing the timeline.
     * @param atoms the collection of atoms associated to the timeline.
     * @return a {@code Timeline} as a sequence of {@code TimelineValue}s
     */
    public Timeline<?> build(Item itm, Collection<Atom> atoms);
}
