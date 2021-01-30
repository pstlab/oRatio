package it.cnr.istc.pst.oratio;

public interface StateListener {

    public void log(String log);

    public void init();

    public void stateChanged();
}
