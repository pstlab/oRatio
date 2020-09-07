package it.cnr.istc.pst.oratio.riddle;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.HashSet;
import java.util.Set;

public class EnumType extends Type {

    Collection<EnumType> enums = new ArrayList<>();
    Collection<String> vals = new ArrayList<>();

    EnumType(final Core core, final Scope scope, final String name) {
        super(core, scope, name);
    }

    public Collection<String> getAllowedValues() {
        Set<String> items = new HashSet<>(vals);
        for (EnumType c_enum : enums)
            items.addAll(c_enum.getAllowedValues());
        return Collections.unmodifiableCollection(items);
    }
}
