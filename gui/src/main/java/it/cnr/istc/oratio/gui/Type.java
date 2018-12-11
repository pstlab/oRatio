package it.cnr.istc.oratio.gui;

import java.util.Collections;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;

/**
 * Type
 */
public class Type {

    private final String name;
    Set<Type> supertypes = new HashSet<>();
    Map<String, Type> fields = new HashMap<>();

    public Type(String name) {
        this.name = name;
    }

    /**
     * @return the name
     */
    public String getName() {
        return name;
    }

    /**
     * @return the supertypes
     */
    public Set<Type> getSupertypes() {
        return Collections.unmodifiableSet(supertypes);
    }

    /**
     * @return the fields
     */
    public Map<String, Type> getFields() {
        return Collections.unmodifiableMap(fields);
    }
}