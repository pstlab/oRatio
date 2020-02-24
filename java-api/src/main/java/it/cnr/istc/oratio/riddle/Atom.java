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

import java.util.Map;
import java.util.stream.Collectors;
import java.util.stream.Stream;

/**
 *
 * @author Riccardo De Benedictis <riccardo.debenedictis@istc.cnr.it>
 */
public class Atom extends Item {

    private final long sigma;
    private final AtomState state;

    Atom(Core core, Predicate predicate, final long sigma, final AtomState state, final Map<String, Item> pars) {
        super(core, predicate);
        exprs.putAll(pars);
        this.sigma = sigma;
        this.state = state;
    }

    /**
     * @return the sigma
     */
    public long getSigma() {
        return sigma;
    }

    @Override
    public Predicate getType() {
        return (Predicate) super.getType();
    }

    public AtomState getState() {
        return state;
    }

    public Item getTau() {
        return exprs.get("tau");
    }

    public enum AtomState {
        Active, Inactive, Unified
    }

    @Override
    public String toString() {
        return "σ" + sigma + " " + type.getName() + "(" + exprs.entrySet().stream().map(expr -> {
            switch (expr.getValue().getType().getName()) {
                case Core.BOOL:
                    return expr.getKey() + " = " + ((Item.BoolItem) expr.getValue()).getValue();
                case Core.INT:
                case Core.REAL:
                    return expr.getKey() + " = " + ((Item.ArithItem) expr.getValue()).getValue();
                case Core.STRING:
                    return expr.getKey() + " = " + ((Item.StringItem) expr.getValue()).getValue();
                default:
                    String val = expr.getKey() + " = ";
                    if (expr.getValue() instanceof EnumItem) {
                        if (((EnumItem) expr.getValue()).getVals().length == 1)
                            val += core.guessName(((EnumItem) expr.getValue()).getVals()[0]);
                        else
                            val += Stream.of(((EnumItem) expr.getValue()).getVals()).map(itm -> core.guessName(itm))
                                    .collect(Collectors.joining(", "));
                    } else
                        val += core.guessName(expr.getValue());
                    return val;
            }
        }).collect(Collectors.joining(", ")) + ")";
    }
}
