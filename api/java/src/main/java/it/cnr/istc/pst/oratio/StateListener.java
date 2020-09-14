package it.cnr.istc.pst.oratio;

import it.cnr.istc.pst.oratio.riddle.Core;

public interface StateListener {

    public void log(String log);

    public void stateChanged(Core core);

    public void init();
}
