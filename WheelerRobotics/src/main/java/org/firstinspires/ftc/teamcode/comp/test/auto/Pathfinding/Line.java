package org.firstinspires.ftc.teamcode.comp.test.auto.Pathfinding;


import static java.lang.Math.atan;

public class Line {
    private Point start, end;
    private double distance, slope, angle; // distance in encoder ticks, angle in deg/rad, havent decided :/

    public Line(Point start, Point end){
        this.start = start;
        this.end = end;
        this.distance = MathUtil.distanceForumula(start, end);
        this.slope = MathUtil.getSlope(start, end);
        this.angle = atan(slope);
    }
    public double distance(){
        return this.distance;
    }
    public double angle(){
        return this.angle;
    }
    public double slope(){
        return this.slope;
    }
    public Point start() {
        return this.start;
    }
    public Point end() {
        return this.end;
    }

}
