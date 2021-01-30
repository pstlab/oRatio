package it.cnr.istc.pst.oratio;

public class Bound {

    public static final int INF = Integer.MAX_VALUE / 2 - 1;
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
