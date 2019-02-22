package it.cnr.istc.oratio.timelines;

import java.util.Collection;

import it.cnr.istc.oratio.riddle.Atom;
import it.cnr.istc.oratio.riddle.Item;

/**
 * TimelineBuilder
 */
public interface TimelineBuilder {
    public Timeline<?> build(Item itm, Collection<Atom> atoms);
}