package it.cnr.istc.pst.oratio.gui;

import com.fasterxml.jackson.core.JsonProcessingException;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import it.cnr.istc.pst.oratio.ExecutorListener;
import it.cnr.istc.pst.oratio.Rational;
import it.cnr.istc.pst.oratio.timelines.TimelinesExecutor;

public class PlanExecutor implements ExecutorListener {

    private static final Logger LOG = LoggerFactory.getLogger(PlanExecutor.class);
    private final TimelinesExecutor solver;
    private Rational current_time = new Rational();

    public PlanExecutor(TimelinesExecutor solver) {
        this.solver = solver;
    }

    public TimelinesExecutor getExecutor() {
        return solver;
    }

    public Rational getCurrentTime() {
        return current_time;
    }

    @Override
    public void tick(final Rational current_time) {
        this.current_time = current_time;
        try {
            App.broadcast(App.MAPPER.writeValueAsString(new App.Message.Tick(current_time)));
        } catch (JsonProcessingException e) {
            LOG.error("Cannot serialize", e);
        }
    }

    @Override
    public void startingAtoms(final long[] atoms) {
        try {
            App.broadcast(App.MAPPER.writeValueAsString(new App.Message.StartingAtoms(atoms)));
        } catch (JsonProcessingException e) {
            LOG.error("Cannot serialize", e);
        }
    }

    @Override
    public void endingAtoms(final long[] atoms) {
        try {
            App.broadcast(App.MAPPER.writeValueAsString(new App.Message.EndingAtoms(atoms)));
        } catch (JsonProcessingException e) {
            LOG.error("Cannot serialize", e);
        }
    }
}
