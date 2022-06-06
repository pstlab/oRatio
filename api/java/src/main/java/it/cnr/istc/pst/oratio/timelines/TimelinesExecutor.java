package it.cnr.istc.pst.oratio.timelines;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Map;
import java.util.Optional;
import java.util.Set;

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
        long[] atom_ids = new long[atoms.size()];
        long[] delay_nums = new long[atoms.size()];
        long[] delay_dens = new long[atoms.size()];
        int i = 0;
        for (Atom atm : atoms) {
            atom_ids[i] = atm.getId();
            delay_nums[i] = 1;
            delay_dens[i] = 1;
            i++;
        }
        dont_start_yet(atom_ids, delay_nums, delay_dens);
    }

    public void dontStartYet(Map<Atom, Optional<Rational>> atoms) throws ExecutorException {
        long[] atom_ids = new long[atoms.size()];
        long[] delay_nums = new long[atoms.size()];
        long[] delay_dens = new long[atoms.size()];
        int i = 0;
        for (Map.Entry<Atom, Optional<Rational>> atm : atoms.entrySet()) {
            atom_ids[i] = atm.getKey().getId();
            if (atm.getValue().isPresent()) {
                delay_nums[i] = atm.getValue().get().getNumerator();
                delay_dens[i] = atm.getValue().get().getDenominator();
            } else {
                delay_nums[i] = 1;
                delay_dens[i] = 1;
            }
            i++;
        }
        dont_start_yet(atom_ids, delay_nums, delay_dens);
    }

    private synchronized native void dont_start_yet(long[] atoms, long[] delay_nums, long[] delay_dens)
            throws ExecutorException;

    public void dontEndYet(Set<Atom> atoms) throws ExecutorException {
        long[] atom_ids = new long[atoms.size()];
        long[] delay_nums = new long[atoms.size()];
        long[] delay_dens = new long[atoms.size()];
        int i = 0;
        for (Atom atm : atoms) {
            atom_ids[i] = atm.getId();
            delay_nums[i] = 1;
            delay_dens[i] = 1;
            i++;
        }
        dont_end_yet(atom_ids, delay_nums, delay_dens);
    }

    public void dontEndYet(Map<Atom, Optional<Rational>> atoms) throws ExecutorException {
        long[] atom_ids = new long[atoms.size()];
        long[] delay_nums = new long[atoms.size()];
        long[] delay_dens = new long[atoms.size()];
        int i = 0;
        for (Map.Entry<Atom, Optional<Rational>> atm : atoms.entrySet()) {
            atom_ids[i] = atm.getKey().getId();
            if (atm.getValue().isPresent()) {
                delay_nums[i] = atm.getValue().get().getNumerator();
                delay_dens[i] = atm.getValue().get().getDenominator();
            } else {
                delay_nums[i] = 1;
                delay_dens[i] = 1;
            }
            i++;
        }
        dont_end_yet(atom_ids, delay_nums, delay_dens);
    }

    private synchronized native void dont_end_yet(long[] atoms, long[] delay_nums, long[] delay_dens)
            throws ExecutorException;

    public void failure(Set<Atom> atoms) throws ExecutorException {
        long[] atom_ids = new long[atoms.size()];
        int i = 0;
        for (Atom atm : atoms)
            atom_ids[i++] = atm.getId();
        failure(atom_ids);
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
