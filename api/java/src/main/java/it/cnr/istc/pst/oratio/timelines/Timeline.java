package it.cnr.istc.pst.oratio.timelines;

import java.util.List;

import it.cnr.istc.pst.oratio.InfRational;

public interface Timeline<TV extends TimelineValue> {

    default String getType() {
        return this.getClass().getSimpleName();
    }

    InfRational getOrigin();

    InfRational getHorizon();

    List<TV> getValues();
}