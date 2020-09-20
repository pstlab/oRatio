package it.cnr.istc.pst.oratio;

import it.cnr.istc.pst.oratio.riddle.Rational;

public interface GraphListener {

    public static final int INF = Integer.MAX_VALUE / 2 - 1;

    public void flawCreated(final FlawCreated flaw);

    public void flawStateChanged(final FlawStateChanged flaw);

    public void flawCostChanged(final FlawCostChanged flaw);

    public void flawPositionChanged(FlawPositionChanged fpc);

    public void currentFlaw(final CurrentFlaw flaw);

    public void resolverCreated(final ResolverCreated resolver);

    public void resolverStateChanged(final ResolverStateChanged resolver);

    public void currentResolver(final CurrentResolver resolver);

    public void causalLinkAdded(final CausalLinkAdded causal_link);

    public static class Bound {

        public int min = -INF, max = INF;

        @Override
        public String toString() {
            if (min == max)
                return Integer.toString(min);

            String c_min = min == -INF ? "-inf" : Integer.toString(min);
            String c_max = max == INF ? "+inf" : Integer.toString(max);
            return "[" + c_min + ", " + c_max + "]";
        }
    }

    public static class FlawCreated {
        public String id;
        public String[] causes;
        public String label;
        public int state;
        public Bound position;
    }

    public static class FlawStateChanged {
        public String id;
        public int state;
    }

    public static class FlawCostChanged {
        public String id;
        public Rational cost;
    }

    public static class FlawPositionChanged {
        public String id;
        public Bound position;
    }

    public static class CurrentFlaw {
        public String id;
    }

    public static class ResolverCreated {
        public String id;
        public String effect;
        public Rational intrinsic_cost;
        public String label;
        public int state;
    }

    public static class ResolverStateChanged {
        public String id;
        public int state;
    }

    public static class CurrentResolver {
        public String id;
    }

    public static class CausalLinkAdded {
        public String flaw_id;
        public String resolver_id;
    }
}
