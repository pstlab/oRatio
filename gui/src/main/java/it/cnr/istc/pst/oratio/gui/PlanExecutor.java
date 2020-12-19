package it.cnr.istc.pst.oratio.gui;

import com.fasterxml.jackson.core.JsonProcessingException;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import it.cnr.istc.pst.oratio.ExecutorListener;
import it.cnr.istc.pst.oratio.Context.Message.EndingAtoms;
import it.cnr.istc.pst.oratio.Context.Message.StartingAtoms;
import it.cnr.istc.pst.oratio.Context.Message.Tick;
import it.cnr.istc.pst.oratio.riddle.Rational;

public class PlanExecutor implements ExecutorListener {

    private static final Logger LOG = LoggerFactory.getLogger(PlanExecutor.class);
    private Rational current_time = new Rational();

    public Rational getCurrentTime() {
        return current_time;
    }

    @Override
    public void tick(Tick tick) {
        current_time = tick.current_time;
        try {
            App.broadcast(App.MAPPER.writeValueAsString(tick));
        } catch (JsonProcessingException e) {
            LOG.error("Cannot serialize", e);
        }
    }

    @Override
    public void startingAtoms(StartingAtoms starting_atoms) {
        try {
            App.broadcast(App.MAPPER.writeValueAsString(starting_atoms));
        } catch (JsonProcessingException e) {
            LOG.error("Cannot serialize", e);
        }
    }

    @Override
    public void endingAtoms(EndingAtoms ending_atoms) {
        try {
            App.broadcast(App.MAPPER.writeValueAsString(ending_atoms));
        } catch (JsonProcessingException e) {
            LOG.error("Cannot serialize", e);
        }
    }
}
