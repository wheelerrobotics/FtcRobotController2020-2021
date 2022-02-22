package org.firstinspires.ftc.teamcode.comp.test.auto.Pathfinding;

import static java.lang.Math.atan;

public class DistanceVector2 extends Line {
    private double magnitude, slope;

    public DistanceVector2(Point start, Point end, double magnitude) {
        super(start, end);
        this.magnitude = magnitude;
    }
    public SlopeVector2d sVector2d(){
        return new SlopeVector2d(slope, magnitude);
    }
    public SlopeVector2d aVector2d(){
        return new SlopeVector2d(atan(slope), magnitude);
    }
    public double magnitude(){
        return this.magnitude;
    }


}
