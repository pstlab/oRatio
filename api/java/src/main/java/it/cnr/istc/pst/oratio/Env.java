package it.cnr.istc.pst.oratio;

import java.util.Map;

public interface Env {

    /**
     * Returns the solver enclosing this environment.
     *
     * @return the solver enclosing this environment.
     */
    public Solver getSolver();

    /**
     * Returns an {@code Item} object that reflects the specified public member
     * field of this {@code Env} object.The {@code name} parameter is a
     * {@code String} specifying the simple name of the desired field.
     *
     * @param name the field name.
     * @return the {@code Item} object of this class specified by {@code name}.
     * @throws java.lang.NoSuchFieldException if the given field cannot be found.
     */
    public Item get(final String name) throws NoSuchFieldException;

    /**
     * Returns a {@code Map} containing all the fields defined in the current
     * environment. The map has the field name as key.
     *
     * @return a {@code Map} containing all the fields defined in the current
     *         environment.
     */
    public Map<String, Item> getExprs();
}
