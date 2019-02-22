package it.cnr.istc.oratio.timelines;

import java.util.List;

import it.cnr.istc.oratio.riddle.InfRational;

/**
 * Timeline
 */
public interface Timeline<TV extends TimelineValue> {

    InfRational getOrigin();

    InfRational getHorizon();

    List<TV> getValues();
}