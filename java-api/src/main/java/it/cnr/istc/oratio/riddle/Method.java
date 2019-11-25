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
package it.cnr.istc.oratio.riddle;

/**
 *
 * @author Riccardo De Benedictis <riccardo.debenedictis@istc.cnr.it>
 */
public class Method extends BaseScope {

    final String name;
    final Type return_type;
    final Field[] pars;

    Method(final Core core, final Scope scope, final String name, final Type return_type, final Field... parameters) {
        super(core, scope);
        this.name = name;
        this.return_type = return_type;
        this.pars = parameters;

        for (Field par : parameters) {
            fields.put(par.name, par);
        }
    }

    public String getName() {
        return name;
    }

    public Type getReturnType() {
        return return_type;
    }

    public Field[] getParemeters() {
        return pars;
    }
}
