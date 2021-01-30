package it.cnr.istc.pst.oratio;

public interface GraphListener {

    public void flawCreated(final String id, final String[] causes, final String label, final State state,
            final Bound position);

    public void flawStateChanged(final String id, final State state);

    public void flawCostChanged(final String id, final Rational cost);

    public void flawPositionChanged(final String id, final Bound position);

    public void currentFlaw(final String id);

    public void resolverCreated(final String id, final String effect, final Rational cost, final String label,
            final State state);

    public void resolverStateChanged(final String id, final State state);

    public void currentResolver(final String id);

    public void causalLinkAdded(final String flaw, final String resolver);

    enum State {
        Active, Forbidden, Inactive
    }
}
