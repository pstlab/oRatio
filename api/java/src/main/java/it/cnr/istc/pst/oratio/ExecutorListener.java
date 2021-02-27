package it.cnr.istc.pst.oratio;

public interface ExecutorListener {

    public void startedSolving();

    public void solutionFound();

    public void inconsistentProblem();

    public void tick(final Rational current_time);

    public void startingAtoms(final long[] atoms);

    public void endingAtoms(final long[] atoms);
}
