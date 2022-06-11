package it.cnr.istc.pst.oratio.timelines;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Map;
import java.util.Map.Entry;
import java.util.Set;
import java.util.function.Function;
import java.util.stream.Collectors;

import it.cnr.istc.pst.oratio.Atom;
import it.cnr.istc.pst.oratio.Rational;
import it.cnr.istc.pst.oratio.Solver;

public class TimelinesExecutor {

    final Solver solver;
    private final long native_handle;
    private final Collection<ExecutorListener> executor_listeners = new ArrayList<>();

    public TimelinesExecutor(final Solver solver) {
        this(solver, new Rational(1));
    }

    public TimelinesExecutor(final Solver solver, final Rational units_per_tick) {
        this.solver = solver;
        this.native_handle = new_instance(units_per_tick.getNumerator(), units_per_tick.getDenominator());
    }

    private native long new_instance(long units_per_tick_num, long units_per_tick_den);

    public native void dispose();

    public synchronized native void tick() throws ExecutorException;

    public void dontStartYet(Set<Atom> atoms) throws ExecutorException {
        dontStartYet(atoms.stream().collect(Collectors.toMap(Function.identity(), atm -> new Rational(1))));
    }

    public void dontStartYet(Map<Atom, Rational> atoms) throws ExecutorException {
        long[] ids = new long[atoms.size()];
        long[] nums = new long[atoms.size()];
        long[] dens = new long[atoms.size()];
        int i = 0;
        for (Entry<Atom, Rational> atm : atoms.entrySet()) {
            ids[i] = atm.getKey().getId();
            nums[i] = atm.getValue().getNumerator();
            dens[i] = atm.getValue().getDenominator();
            i++;
        }
        dont_start_yet(ids, nums, dens);
    }

    private synchronized native void dont_start_yet(long[] atoms, long[] nums, long[] dens) throws ExecutorException;

    public void dontEndYet(Set<Atom> atoms) throws ExecutorException {
        dontEndYet(atoms.stream().collect(Collectors.toMap(Function.identity(), atm -> new Rational(1))));
    }

    public void dontEndYet(Map<Atom, Rational> atoms) throws ExecutorException {
        long[] ids = new long[atoms.size()];
        long[] nums = new long[atoms.size()];
        long[] dens = new long[atoms.size()];
        int i = 0;
        for (Entry<Atom, Rational> atm : atoms.entrySet()) {
            ids[i] = atm.getKey().getId();
            nums[i] = atm.getValue().getNumerator();
            dens[i] = atm.getValue().getDenominator();
            i++;
        }
        dont_end_yet(ids, nums, dens);
    }

    private synchronized native void dont_end_yet(long[] atoms, long[] nums, long[] dens) throws ExecutorException;

    public void failure(Set<Atom> atoms) throws ExecutorException {
        long[] ids = new long[atoms.size()];
        int i = 0;
        for (Atom atm : atoms)
            ids[i++] = atm.getId();
        failure(ids);
    }

    private synchronized native void failure(long[] atoms) throws ExecutorException;

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
