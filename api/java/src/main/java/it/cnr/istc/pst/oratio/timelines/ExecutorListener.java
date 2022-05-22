package it.cnr.istc.pst.oratio.timelines;

import it.cnr.istc.pst.oratio.Rational;

public interface ExecutorListener {

    public void tick(final Rational current_time);

    public void startingAtoms(final long[] atoms);

    public void startAtoms(final long[] atoms);

    public void endingAtoms(final long[] atoms);

    public void endAtoms(final long[] atoms);
}
