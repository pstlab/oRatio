package it.cnr.istc.oratio.gui;

import java.util.Collections;
import java.util.HashMap;
import java.util.Map;

/**
 * State
 */
public class State {

    Map<String, Item> items = new HashMap<>();
    Map<String, Type> types = new HashMap<>();

    public State() {
    }

    /**
     * @return the items
     */
    public Map<String, Item> getItems() {
        return Collections.unmodifiableMap(items);
    }

    /**
     * @return the item having the given name
     */
    public Item getItem(String name) {
        return items.get(name);
    }

    /**
     * @return the types
     */
    public Map<String, Type> getTypes() {
        return Collections.unmodifiableMap(types);
    }
}