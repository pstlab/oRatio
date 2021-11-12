package it.cnr.istc.pst.oratio.timelines;

import java.util.ArrayList;
import java.util.Collection;

import it.cnr.istc.pst.oratio.Rational;
import it.cnr.istc.pst.oratio.Solver;

public class TimelinesExecutor {

    final Solver solver;
    private final long native_handle;
    private final Collection<ExecutorListener> executor_listeners = new ArrayList<>();

    public TimelinesExecutor(final Solver solver) {
        this(solver, new String[] {}, new Rational(1));
    }

    public TimelinesExecutor(final Solver solver, final String[] rel_preds, final Rational units_per_tick) {
        this.solver = solver;
        this.native_handle = new_instance(rel_preds, units_per_tick.getNumerator(), units_per_tick.getDenominator());
    }

    private native long new_instance(String[] rel_preds, long units_per_tick_num, long units_per_tick_den);

    public native void dispose();

    public synchronized native void tick() throws ExecutorException;

    public synchronized native void dont_start_yet(long[] atoms) throws ExecutorException;

    public synchronized native void dont_end_yet(long[] atoms) throws ExecutorException;

    public synchronized native void failure(long[] atoms) throws ExecutorException;

    private void fireTick(final long current_time_num, final long current_time_den) {
        final Rational current_time = new Rational(current_time_num, current_time_den);
        executor_listeners.stream().forEach(l -> l.tick(current_time));
    }

    private void fireStartingAtoms(final long[] atoms) {
        executor_listeners.stream().forEach(l -> l.startingAtoms(atoms));
    }

    private void fireStartAtoms(final long[] atoms) {
        executor_listeners.stream().forEach(l -> l.startAtoms(atoms));
    }

    private void fireEndingAtoms(final long[] atoms) {
        executor_listeners.stream().forEach(l -> l.endingAtoms(atoms));
    }

    private void fireEndAtoms(final long[] atoms) {
        executor_listeners.stream().forEach(l -> l.endAtoms(atoms));
    }

    public void addExecutorListener(final ExecutorListener l) {
        executor_listeners.add(l);
    }

    public void removeExecutorListener(final ExecutorListener l) {
        executor_listeners.remove(l);
    }
}
