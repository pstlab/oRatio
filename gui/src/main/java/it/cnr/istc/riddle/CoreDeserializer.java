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

import com.google.gson.JsonArray;
import com.google.gson.JsonDeserializationContext;
import com.google.gson.JsonDeserializer;
import com.google.gson.JsonElement;
import com.google.gson.JsonObject;
import com.google.gson.JsonParseException;
import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Collection;
import java.util.HashMap;
import java.util.Map;
import java.util.Queue;

/**
 *
 * @author Riccardo De Benedictis <riccardo.debenedictis@istc.cnr.it>
 */
public class CoreDeserializer implements JsonDeserializer<Core> {

    private Core core;

    public void setCore(Core core) {
        this.core = core;

        // we clear all the instances..
        core.exprs.clear();
        Queue<Type> q = new ArrayDeque<>();
        q.addAll(core.types.values());
        q.addAll(core.predicates.values());
        while (!q.isEmpty()) {
            Type t = q.poll();
            t.instances.clear();
            q.addAll(t.types.values());
            q.addAll(t.predicates.values());
        }
    }

    @Override
    public Core deserialize(JsonElement json, java.lang.reflect.Type typeOfT, JsonDeserializationContext context)
            throws JsonParseException {
        final JsonObject object = json.getAsJsonObject();

        final Map<String, Item> items = new HashMap<>();
        final Map<String, Atom> atoms = new HashMap<>();

        JsonArray itms_array = null;
        if (object.has("items")) {
            itms_array = object.getAsJsonArray("items");
            for (JsonElement itm_el : itms_array) {
                JsonObject itm_obj = itm_el.getAsJsonObject();
                String[] c_type = itm_obj.getAsJsonPrimitive("type").getAsString().split(":");
                Type t = core.types.get(c_type[0]);
                for (int i = 1; i < c_type.length; i++)
                    t = t.types.get(c_type[i]);

                final Item item = new Item(core, t);
                t.instances.add(item);

                Queue<Type> q = new ArrayDeque<>();
                q.addAll(t.superclasses);
                while (!q.isEmpty()) {
                    Type st = q.poll();
                    st.instances.add(item);
                    q.addAll(st.superclasses);
                }
                items.put(itm_obj.getAsJsonPrimitive("id").getAsString(), item);
            }
        }

        if (object.has("atoms")) {
            JsonArray atms_array = object.getAsJsonArray("atoms");
            for (JsonElement atm_el : atms_array) {
                JsonObject atm_obj = atm_el.getAsJsonObject();
                String[] c_predicate = atm_obj.getAsJsonPrimitive("predicate").getAsString().split(":");
                Predicate p = null;
                if (c_predicate.length == 1)
                    p = core.predicates.get(c_predicate[0]);
                else {
                    Type t = core.types.get(c_predicate[0]);
                    for (int i = 1; i < c_predicate.length - 1; i++)
                        t = t.types.get(c_predicate[i]);
                    p = t.predicates.get(c_predicate[c_predicate.length - 1]);
                }

                final Map<String, Item> pars = new HashMap<>();
                if (atm_obj.has("pars"))
                    for (JsonElement par_el : atm_obj.getAsJsonArray("pars")) {
                        JsonObject par_obj = par_el.getAsJsonObject();
                        pars.put(par_obj.getAsJsonPrimitive("name").getAsString(), toItem(items, atoms, par_obj));
                    }
                final Atom atom = new Atom(core, p, Atom.AtomState.valueOf(atm_obj.get("state").getAsString()), pars);
                p.instances.add(atom);

                Queue<Type> q = new ArrayDeque<>();
                q.addAll(p.superclasses);
                while (!q.isEmpty()) {
                    Type st = q.poll();
                    st.instances.add(atom);
                    q.addAll(st.superclasses);
                }
                atoms.put(atm_obj.getAsJsonPrimitive("id").getAsString(), atom);
            }
        }

        if (object.has("items"))
            for (JsonElement itm_el : itms_array) {
                JsonObject itm_obj = itm_el.getAsJsonObject();
                if (itm_obj.has("exprs")) {
                    Item item = items.get(itm_obj.getAsJsonPrimitive("id").getAsString());
                    for (JsonElement xpr_el : itm_obj.getAsJsonArray("exprs")) {
                        JsonObject xpr_obj = xpr_el.getAsJsonObject();
                        item.exprs.put(xpr_obj.getAsJsonPrimitive("name").getAsString(), toItem(items, atoms, xpr_obj));
                    }
                }
            }

        if (object.has("exprs"))
            for (JsonElement xpr_el : object.getAsJsonArray("exprs")) {
                JsonObject xpr_obj = xpr_el.getAsJsonObject();
                core.exprs.put(xpr_obj.getAsJsonPrimitive("name").getAsString(), toItem(items, atoms, xpr_obj));
            }
        return core;
    }

    private Item toItem(final Map<String, Item> items, final Map<String, Atom> atoms, final JsonObject obj) {
        JsonElement value = obj.get("value");
        if (value.isJsonPrimitive()) {
            String val = value.getAsString();
            Item item = items.get(val);
            if (item != null)
                return item;
            Atom atom = atoms.get(val);
            if (atom != null)
                return atom;
            return new Item.StringItem(core, val);
        } else {
            JsonObject val = value.getAsJsonObject();
            String[] c_type = obj.getAsJsonPrimitive("type").getAsString().split(":");
            Type t = core.types.get(c_type[0]);
            for (int i = 1; i < c_type.length; i++)
                t = t.types.get(c_type[i]);
            switch (t.name) {
            case Core.BOOL:
                return new Item.BoolItem(core, val.getAsJsonPrimitive("lit").getAsString(),
                        Item.BoolItem.LBool.valueOf(val.getAsJsonPrimitive("val").getAsString()));
            case Core.INT:
            case Core.REAL:
                return new Item.ArithItem(core, t, val.getAsJsonPrimitive("lin").getAsString(),
                        val.has("lb") ? toInfRational(val.getAsJsonObject("lb"))
                                : new InfRational(Rational.NEGATIVE_INFINITY),
                        val.has("ub") ? toInfRational(val.getAsJsonObject("ub"))
                                : new InfRational(Rational.POSITIVE_INFINITY),
                        toInfRational(val.getAsJsonObject("val")));
            default:
                JsonArray vals_array = val.getAsJsonArray("vals");
                Collection<Item> vals = new ArrayList<>();
                for (JsonElement val_el : vals_array)
                    vals.add(items.get(val_el.getAsString()));
                return new Item.EnumItem(core, t, val.getAsJsonPrimitive("var").getAsString(),
                        vals.toArray(new Item[vals.size()]));
            }
        }
    }

    private InfRational toInfRational(JsonObject obj) {
        return obj.has("inf") ? new InfRational(toRational(obj), toRational(obj.getAsJsonObject("inf")))
                : new InfRational(toRational(obj));
    }

    private Rational toRational(JsonObject obj) {
        return new Rational(Long.parseLong(obj.getAsJsonPrimitive("num").getAsString()),
                Long.parseLong(obj.getAsJsonPrimitive("den").getAsString()));
    }
}
