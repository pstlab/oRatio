package it.cnr.istc.oratio.gui;

import java.util.Collections;
import java.util.HashMap;
import java.util.Map;

/**
 * Item
 */
public class Item {

    private final Type type;
    Map<String, Item> items = new HashMap<>();

    public Item(Type type) {
        this.type = type;
    }

    /**
     * @return the type
     */
    public Type getType() {
        return type;
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
}