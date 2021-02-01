package it.cnr.istc.pst.oratio;

public interface GraphListener {

    public void flawCreated(final long id, final long[] causes, final String label, final State state,
            final Bound position);

    public void flawStateChanged(final long id, final State state);

    public void flawCostChanged(final long id, final Rational cost);

    public void flawPositionChanged(final long id, final Bound position);

    public void currentFlaw(final long id);

    public void resolverCreated(final long id, final long effect, final Rational cost, final String label,
            final State state);

    public void resolverStateChanged(final long id, final State state);

    public void currentResolver(final long id);

    public void causalLinkAdded(final long flaw, final long resolver);

    enum State {
        Active, Forbidden, Inactive
    }
}
