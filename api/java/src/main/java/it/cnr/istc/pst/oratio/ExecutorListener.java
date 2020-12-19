package it.cnr.istc.pst.oratio;

import it.cnr.istc.pst.oratio.Context.Message.EndingAtoms;
import it.cnr.istc.pst.oratio.Context.Message.StartingAtoms;
import it.cnr.istc.pst.oratio.Context.Message.Tick;

public interface ExecutorListener {

    public void tick(final Tick tick);

    public void startingAtoms(final StartingAtoms starting_atoms);

    public void endingAtoms(final EndingAtoms ending_atoms);
}
