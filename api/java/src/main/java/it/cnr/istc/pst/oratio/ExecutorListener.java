package it.cnr.istc.pst.oratio;

public interface ExecutorListener {

    public void tick(final Rational current_time);

    public void startingAtoms(final long[] atoms);

    public void endingAtoms(final long[] atoms);
}
