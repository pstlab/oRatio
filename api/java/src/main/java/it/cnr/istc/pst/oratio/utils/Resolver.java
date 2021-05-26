package it.cnr.istc.pst.oratio.utils;

import java.io.IOException;
import java.util.HashSet;
import java.util.Set;

import com.fasterxml.jackson.core.JsonGenerator;
import com.fasterxml.jackson.databind.SerializerProvider;
import com.fasterxml.jackson.databind.annotation.JsonSerialize;
import com.fasterxml.jackson.databind.ser.std.StdSerializer;

import it.cnr.istc.pst.oratio.GraphListener.State;
import it.cnr.istc.pst.oratio.Rational;

@JsonSerialize(using = Resolver.ResolverSerializer.class)
public class Resolver {

    public final long id;
    public final Flaw effect;
    public final String label;
    public State state;
    public final Rational cost;
    public final Set<Flaw> preconditions = new HashSet<>();
    public boolean current = false;

    public Resolver(final long id, final Flaw effect, final String label, final State state, final Rational cost) {
        this.id = id;
        this.effect = effect;
        this.label = label;
        this.state = state;
        this.cost = cost;
    }

    static class ResolverSerializer extends StdSerializer<Resolver> {

        private ResolverSerializer() {
            super(Resolver.class);
        }

        /**
         *
         */
        private static final long serialVersionUID = 1L;

        @Override
        public void serialize(final Resolver resolver, final JsonGenerator gen, final SerializerProvider provider)
                throws IOException {
            gen.writeStartObject();
            gen.writeNumberField("id", resolver.id);
            gen.writeNumberField("effect", resolver.effect.id);
            gen.writeStringField("label", resolver.label);
            gen.writeNumberField("state", resolver.state.ordinal());

            gen.writeObjectField("cost", resolver.cost);

            gen.writeArrayFieldStart("preconditions");
            for (Flaw pre : resolver.preconditions)
                gen.writeNumber(pre.id);
            gen.writeEndArray();

            gen.writeBooleanField("current", resolver.current);

            gen.writeEndObject();
        }
    }
}
