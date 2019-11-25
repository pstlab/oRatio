package it.cnr.istc.oratio;

import it.cnr.istc.oratio.riddle.Rational;

/**
 * GraphListener
 */
public interface GraphListener {

    public void flawCreated(final FlawCreated flaw);

    public void flawStateChanged(final FlawStateChanged flaw);

    public void flawCostChanged(final FlawCostChanged flaw);

    public void currentFlaw(final CurrentFlaw flaw);

    public void resolverCreated(final ResolverCreated resolver);

    public void resolverStateChanged(final ResolverStateChanged resolver);

    public void currentResolver(final CurrentResolver resolver);

    public void causalLinkAdded(final CausalLinkAdded causal_link);

    public void poke();

    public static class FlawCreated {
        public String flaw;
        public String[] causes;
        public String label;
        public int state;
    }

    public static class FlawStateChanged {
        public String flaw;
        public int state;
    }

    public static class FlawCostChanged {
        public String flaw;
        public Rational cost;
    }

    public static class CurrentFlaw {
        public String flaw;
    }

    public static class ResolverCreated {
        public String resolver;
        public String effect;
        public Rational cost;
        public String label;
        public int state;
    }

    public static class ResolverStateChanged {
        public String resolver;
        public int state;
    }

    public static class CurrentResolver {
        public String resolver;
    }

    public static class CausalLinkAdded {
        public String flaw;
        public String resolver;
    }
}