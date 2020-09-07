package it.cnr.istc.pst.oratio.timelines;

import java.util.Collection;

import it.cnr.istc.pst.oratio.riddle.Atom;
import it.cnr.istc.pst.oratio.riddle.Item;

public interface TimelineBuilder {

    public Timeline<?> build(Item itm, Collection<Atom> atoms);
}
