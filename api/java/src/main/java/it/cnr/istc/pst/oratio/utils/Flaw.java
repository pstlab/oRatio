package it.cnr.istc.pst.oratio.utils;

import java.io.IOException;

import com.fasterxml.jackson.core.JsonGenerator;
import com.fasterxml.jackson.databind.SerializerProvider;
import com.fasterxml.jackson.databind.annotation.JsonSerialize;
import com.fasterxml.jackson.databind.ser.std.StdSerializer;

import it.cnr.istc.pst.oratio.Bound;
import it.cnr.istc.pst.oratio.GraphListener.State;
import it.cnr.istc.pst.oratio.Rational;

@JsonSerialize(using = Flaw.FlawSerializer.class)
public class Flaw {

    public final long id;
    public final Resolver[] causes;
    public final String label;
    public State state;
    public Bound position;
    public Rational cost = Rational.POSITIVE_INFINITY;
    public boolean current = false;

    public Flaw(final long id, final Resolver[] causes, final String label, final State state, final Bound position) {
        this.id = id;
        this.causes = causes;
        this.label = label;
        this.state = state;
        this.position = position;
    }

    static class FlawSerializer extends StdSerializer<Flaw> {

        private static final long serialVersionUID = 1L;

        private FlawSerializer() {
            super(Flaw.class);
        }

        @Override
        public void serialize(final Flaw flaw, final JsonGenerator gen, final SerializerProvider provider)
                throws IOException {
            gen.writeStartObject();
            gen.writeNumberField("id", flaw.id);
            gen.writeArrayFieldStart("causes");
            for (Resolver cause : flaw.causes)
                gen.writeNumber(cause.id);
            gen.writeEndArray();
            gen.writeStringField("label", flaw.label);
            gen.writeNumberField("state", flaw.state.ordinal());

            if (flaw.position.min != -Bound.INF || flaw.position.max != Bound.INF)
                gen.writeObjectField("position", flaw.position);

            gen.writeObjectField("cost", flaw.cost);

            gen.writeBooleanField("current", flaw.current);

            gen.writeEndObject();
        }
    }
}
