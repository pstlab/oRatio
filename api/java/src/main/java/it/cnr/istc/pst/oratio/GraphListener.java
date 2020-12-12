package it.cnr.istc.pst.oratio;

import it.cnr.istc.pst.oratio.Context.Message.CausalLinkAdded;
import it.cnr.istc.pst.oratio.Context.Message.CurrentFlaw;
import it.cnr.istc.pst.oratio.Context.Message.CurrentResolver;
import it.cnr.istc.pst.oratio.Context.Message.FlawCostChanged;
import it.cnr.istc.pst.oratio.Context.Message.FlawCreated;
import it.cnr.istc.pst.oratio.Context.Message.FlawPositionChanged;
import it.cnr.istc.pst.oratio.Context.Message.FlawStateChanged;
import it.cnr.istc.pst.oratio.Context.Message.ResolverCreated;
import it.cnr.istc.pst.oratio.Context.Message.ResolverStateChanged;

public interface GraphListener {

    public void flawCreated(final FlawCreated flaw);

    public void flawStateChanged(final FlawStateChanged flaw);

    public void flawCostChanged(final FlawCostChanged flaw);

    public void flawPositionChanged(final FlawPositionChanged fpc);

    public void currentFlaw(final CurrentFlaw flaw);

    public void resolverCreated(final ResolverCreated resolver);

    public void resolverStateChanged(final ResolverStateChanged resolver);

    public void currentResolver(final CurrentResolver resolver);

    public void causalLinkAdded(final CausalLinkAdded causal_link);
}
