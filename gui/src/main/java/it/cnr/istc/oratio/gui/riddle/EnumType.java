/*
 * Copyright (C) 2018 Riccardo De Benedictis <riccardo.debenedictis@istc.cnr.it>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
package it.cnr.istc.oratio.gui.riddle;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.HashSet;
import java.util.Set;

/**
 *
 * @author Riccardo De Benedictis <riccardo.debenedictis@istc.cnr.it>
 */
public class EnumType extends Type {

    Collection<EnumType> enums = new ArrayList<>();
    Collection<String> vals = new ArrayList<>();

    EnumType(final Core core, final Scope scope, final String name) {
        super(core, scope, name);
    }

    public Collection<String> getAllowedValues() {
        Set<String> items = new HashSet<>(vals);
        for (EnumType c_enum : enums) {
            items.addAll(c_enum.getAllowedValues());
        }
        return Collections.unmodifiableCollection(items);
    }
}
