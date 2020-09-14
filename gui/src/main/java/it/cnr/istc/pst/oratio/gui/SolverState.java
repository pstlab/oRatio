package it.cnr.istc.pst.oratio.gui;

import it.cnr.istc.pst.oratio.StateListener;
import it.cnr.istc.pst.oratio.riddle.Core;

public class SolverState implements StateListener {

    @Override
    public void log(String log) {
        // TODO Auto-generated method stub
    }

    @Override
    public void stateChanged(Core core) {
        // TODO Auto-generated method stub
    }

    @Override
    public void init() {
        App.GRAPH.clear();
    }
}
