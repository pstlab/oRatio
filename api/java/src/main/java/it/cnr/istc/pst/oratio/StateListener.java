package it.cnr.istc.pst.oratio;

public interface StateListener {

    public void log(String log);

    public void read(String script);

    public void read(String[] files);

    public void stateChanged();
}
