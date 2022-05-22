package it.cnr.istc.pst.oratio;

import java.io.IOException;

import com.fasterxml.jackson.core.JsonGenerator;
import com.fasterxml.jackson.databind.SerializerProvider;
import com.fasterxml.jackson.databind.annotation.JsonSerialize;
import com.fasterxml.jackson.databind.ser.std.StdSerializer;

@JsonSerialize(using = Bound.BoundSerializer.class)
public class Bound {

    public static final int INF = Integer.MAX_VALUE / 2 - 1;
    public final int min, max;

    public Bound(final int min, final int max) {
        this.min = min;
        this.max = max;
    }

    @Override
    public String toString() {
        if (min == max)
            return Integer.toString(min);

        final String c_min = min == -INF ? "-inf" : Integer.toString(min);
        final String c_max = max == INF ? "+inf" : Integer.toString(max);
        return "[" + c_min + ", " + c_max + "]";
    }

    static class BoundSerializer extends StdSerializer<Bound> {

        private BoundSerializer() {
            super(Bound.class);
        }

        @Override
        public void serialize(Bound value, JsonGenerator gen, SerializerProvider provider) throws IOException {
            gen.writeStartObject();
            if (value.min != -Bound.INF)
                gen.writeNumberField("min", value.min);
            if (value.max != Bound.INF)
                gen.writeNumberField("max", value.max);
            gen.writeEndObject();
        }
    }
}
