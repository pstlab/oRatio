package it.cnr.istc.pst.oratio.timelines;

import java.util.ArrayList;
import java.util.Collection;

import it.cnr.istc.pst.oratio.ExecutorListener;
import it.cnr.istc.pst.oratio.Rational;
import it.cnr.istc.pst.oratio.Solver;

public class TimelinesExecutor {

    static {
        System.loadLibrary("executor-api");
    }
    final Solver solver;
    private final long native_handle;
    private final Collection<ExecutorListener> executor_listeners = new ArrayList<>();

    public TimelinesExecutor(final Solver solver, final Rational units_per_tick) {
        this.solver = solver;
        this.native_handle = new_instance(units_per_tick.getNumerator(), units_per_tick.getDenominator());
    }

    private native long new_instance(long units_per_tick_num, long units_per_tick_den);

    public native void dispose();

    public native void tick();

    public native void dont_start_yet(long[] atoms);

    public native void dont_end_yet(long[] atoms);

    public native void failure(long[] atoms);

    private void fireTick(final long current_time_num, final long current_time_den) {
        Rational current_time = new Rational(current_time_num, current_time_den);
        executor_listeners.stream().forEach(l -> l.tick(current_time));
    }

    private void fireStartingAtoms(final long[] atoms) {
        executor_listeners.stream().forEach(l -> l.startingAtoms(atoms));
    }

    private void fireEndingAtoms(final long[] atoms) {
        executor_listeners.stream().forEach(l -> l.endingAtoms(atoms));
    }

    public void addExecutorListener(ExecutorListener l) {
        executor_listeners.add(l);
    }

    public void removeExecutorListener(ExecutorListener l) {
        executor_listeners.remove(l);
    }
}
