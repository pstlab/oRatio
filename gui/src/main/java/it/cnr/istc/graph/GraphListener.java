package it.cnr.istc.graph;

/**
 * GraphListener
 */
public interface GraphListener {

    public void flaw_created(final FlawCreated flaw);

    public void flaw_state_changed(final FlawStateChanged flaw);

    public void flaw_cost_changed(final FlawCostChanged flaw);

    public void current_flaw(final CurrentFlaw flaw);

    public void resolver_created(final ResolverCreated resolver);

    public void resolver_state_changed(final ResolverStateChanged resolver);

    public void resolver_cost_changed(final ResolverCostChanged resolver);

    public void current_resolver(final CurrentResolver resolver);

    public void causal_link_added(final CausalLinkAdded causal_link);
}