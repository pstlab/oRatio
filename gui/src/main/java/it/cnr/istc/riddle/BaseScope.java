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
package it.cnr.istc.riddle;

import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.Map;

/**
 *
 * @author Riccardo De Benedictis <riccardo.debenedictis@istc.cnr.it>
 */
public class BaseScope implements Scope {

    final Core core;
    final Scope scope;
    final Map<String, Field> fields = new LinkedHashMap<>();

    BaseScope(final Core core, final Scope scope) {
        assert core != null;
        assert scope != null;
        this.core = core;
        this.scope = scope;
    }

    @Override
    public Core getCore() {
        return core;
    }

    @Override
    public Scope getScope() {
        return scope;
    }

    /**
     * Returns the field of this scope having the given name. Checks the
     * enclosing scope if the field is not found within this scope.
     *
     * @param name a string representing the name of the field.
     * @return a field having the given name.
     */
    @Override
    public Field getField(final String name) throws NoSuchFieldException {
        Field field = fields.get(name);
        if (field != null) {
            return field;
        } else {
            return scope.getField(name);
        }
    }

    /**
     * Returns all the field defined within this scope.
     *
     * @return a map containing all the field defined within this scope indexed
     * by the field's name.
     */
    @Override
    public Map<String, Field> getFields() {
        return Collections.unmodifiableMap(fields);
    }
}
