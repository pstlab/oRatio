package it.cnr.istc.oratio;

import it.cnr.istc.oratio.riddle.Core;

/**
 * StateListener
 */
public interface StateListener {

    public void log(String log);

    public void stateChanged(Core core);
}