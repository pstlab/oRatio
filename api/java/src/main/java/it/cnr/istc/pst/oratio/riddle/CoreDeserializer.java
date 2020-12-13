package it.cnr.istc.pst.oratio.riddle;

import java.io.IOException;
import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Collection;
import java.util.HashMap;
import java.util.Map;
import java.util.Queue;

import com.fasterxml.jackson.core.JsonParser;
import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.DeserializationContext;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.deser.std.StdDeserializer;

import it.cnr.istc.pst.oratio.Context;

public class CoreDeserializer extends StdDeserializer<Core> {

    private static final long serialVersionUID = 1L;

    public CoreDeserializer() {
        super(Core.class);
    }

    @Override
    public Core deserialize(JsonParser p, DeserializationContext ctxt) throws IOException, JsonProcessingException {
        final Core core = Context.getContext().getCore();
        // we clear all the instances..
        core.clear();

        final Map<String, Item> items = new HashMap<>(); // all the items, indexed by their id..
        final Map<String, Atom> atoms = new HashMap<>(); // all the atoms, indexed by their id..

        final JsonNode tree = p.getCodec().readTree(p);

        JsonNode items_array = tree.get("items");
        if (items_array != null)
            // we add all the items: we need all of them before setting their nested
            // expressions and atoms' parameters..
            for (int i = 0; i < items_array.size(); i++) {
                JsonNode itm_obj = items_array.get(i);
                String[] c_type = itm_obj.get("type").asText().split(":");
                Type t = core.types.get(c_type[0]);
                for (int j = 1; j < c_type.length; j++)
                    t = t.types.get(c_type[j]);

                final Item item = new Item(core, t);
                t.instances.add(item);

                Queue<Type> q = new ArrayDeque<>();
                q.addAll(t.superclasses);
                while (!q.isEmpty()) {
                    Type st = q.poll();
                    st.instances.add(item);
                    q.addAll(st.superclasses);
                }
                items.put(itm_obj.get("id").asText(), item);
            }

        JsonNode atoms_array = tree.get("atoms");
        if (atoms_array != null)
            // we add all the atoms..
            for (int i = 0; i < atoms_array.size(); i++) {
                JsonNode atm_obj = atoms_array.get(i);
                String[] c_predicate = atm_obj.get("predicate").asText().split(":");
                Predicate pred = null;
                if (c_predicate.length == 1)
                    pred = core.predicates.get(c_predicate[0]);
                else {
                    Type t = core.types.get(c_predicate[0]);
                    for (int j = 1; j < c_predicate.length - 1; j++)
                        t = t.types.get(c_predicate[j]);
                    pred = t.predicates.get(c_predicate[c_predicate.length - 1]);
                }

                // we refine the atom's parameters..
                final Map<String, Item> c_pars = new HashMap<>();
                JsonNode pars = atm_obj.get("pars");
                if (pars != null)
                    for (int j = 0; j < pars.size(); j++)
                        c_pars.put(pars.get(j).get("name").asText(), toItem(core, items, atoms, pars.get(j)));
                final Atom atom = new Atom(core, pred, atm_obj.get("sigma").asLong(),
                        Atom.AtomState.valueOf(atm_obj.get("state").asText()), c_pars);
                pred.instances.add(atom);

                Queue<Type> q = new ArrayDeque<>();
                q.addAll(pred.superclasses);
                while (!q.isEmpty()) {
                    Type st = q.poll();
                    st.instances.add(atom);
                    q.addAll(st.superclasses);
                }
                atoms.put(atm_obj.get("id").asText(), atom);
            }

        if (items_array != null)
            // we refine the items' parameters..
            for (int i = 0; i < items_array.size(); i++) {
                Item item = items.get(items_array.get(i).get("id").asText());
                JsonNode exprs = items_array.get(i).get("exprs");
                if (exprs != null)
                    for (int j = 0; j < exprs.size(); j++) {
                        String name = exprs.get(j).get("name").asText();
                        Item itm = toItem(core, items, atoms, exprs.get(j));
                        item.exprs.put(name, itm);
                        core.expr_names.putIfAbsent(itm, name);
                    }
            }

        JsonNode exprs = tree.get("exprs");
        if (exprs != null)
            // we set the core's expressions..
            for (int i = 0; i < exprs.size(); i++) {
                String name = exprs.get(i).get("name").asText();
                Item itm = toItem(core, items, atoms, exprs.get(i));
                core.exprs.put(name, itm);
                core.expr_names.put(itm, name);
            }

        return core;
    }

    private Item toItem(final Core core, final Map<String, Item> items, final Map<String, Atom> atoms,
            final JsonNode obj) {
        JsonNode value = obj.get("value");
        if (value.isObject()) {
            String[] c_type = obj.get("type").asText().split(":");
            Type t = core.types.get(c_type[0]);
            for (int i = 1; i < c_type.length; i++)
                t = t.types.get(c_type[i]);
            switch (t.name) {
                case Core.BOOL:
                    return new Item.BoolItem(core, value.get("lit").asText(),
                            Item.BoolItem.LBool.valueOf(value.get("val").asText()));
                case Core.INT:
                case Core.REAL:
                case Core.TP:
                    return new Item.ArithItem(core, t, value.get("lin").asText(),
                            (value.get("lb") != null) ? toInfRational(value.get("lb"))
                                    : new InfRational(Rational.NEGATIVE_INFINITY),
                            (value.get("ub") != null) ? toInfRational(value.get("ub"))
                                    : new InfRational(Rational.POSITIVE_INFINITY),
                            toInfRational(value.get("val")));
                default:
                    JsonNode vals_array = value.get("vals");
                    Collection<Item> c_vals = new ArrayList<>();
                    for (int j = 0; j < vals_array.size(); j++)
                        c_vals.add(items.get(vals_array.get(j).asText()));
                    return new Item.EnumItem(core, t, value.get("var").asText(),
                            c_vals.toArray(new Item[c_vals.size()]));
            }
        } else {
            String val = value.asText();
            Item item = items.get(val);
            if (item != null)
                return item;
            Atom atom = atoms.get(val);
            if (atom != null)
                return atom;
            return new Item.StringItem(core, val);
        }
    }

    private InfRational toInfRational(final JsonNode obj) {
        return (obj.get("inf") != null) ? new InfRational(toRational(obj), toRational(obj.get("inf")))
                : new InfRational(toRational(obj));
    }

    private Rational toRational(final JsonNode obj) {
        return new Rational(obj.get("num").asLong(), obj.get("den").asLong());
    }
}
