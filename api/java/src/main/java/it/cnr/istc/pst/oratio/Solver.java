package it.cnr.istc.pst.oratio;

public class Solver {

    static {
        System.loadLibrary("oRatio-lib");
    }

    private final long native_handle;
    private final long native_listener_handle = 0;

    public Solver() {
        native_handle = new_instance();
    }

    private native long new_instance();

    public native void dispose();

    public native void read(String script);

    public native void read(String[] files);

    public native void solve();

    public native String getState();
}
