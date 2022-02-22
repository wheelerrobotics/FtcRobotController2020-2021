package org.firstinspires.ftc.teamcode.comp.test.auto.Pathfinding;

public class Point {
    private double x, y;
    public Point(double x, double y){
        this.x = x;
        this.y = y;
    }
    public double x(){
        return this.x;
    }
    public double y(){
        return this.y;
    }
    public Point minus(Point point){
        return new Point(this.x - point.x(), this.y - point.y());
    }
    public static Point minus(Point point1, Point point2){
        return point1.minus(point2);
    }

}
