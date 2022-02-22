package org.firstinspires.ftc.teamcode.comp.test.auto.Pathfinding;


import static java.lang.Math.pow;
import static java.lang.Math.sqrt;


public class MathUtil {

    public static double distanceForumula(Point point1, Point point2){
        return sqrt( pow( (point1.x() - point2.x()), 2 ) + pow( (point1.y() - point2.y()), 2 ) );
    }
    public static double getSlope(Point point1, Point point2){
        return (point1.y()-point2.y())/(point1.x()/point2.x());
    }

}
