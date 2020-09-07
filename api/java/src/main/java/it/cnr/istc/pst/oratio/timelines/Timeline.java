package it.cnr.istc.pst.oratio.timelines;

import java.util.List;

import it.cnr.istc.pst.oratio.riddle.InfRational;

public interface Timeline<TV extends TimelineValue> {

    InfRational getOrigin();

    InfRational getHorizon();

    List<TV> getValues();
}
