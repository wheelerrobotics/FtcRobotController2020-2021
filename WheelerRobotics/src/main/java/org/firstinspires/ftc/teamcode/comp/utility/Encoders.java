package org.firstinspires.ftc.teamcode.comp.utility;

public class Encoders {
    int left, right, center;

    Encoders(int left, int right, int center){
        this.left = left;
        this.right = right;
        this.center = center;
    }
    public void setEncoders(int left, int right, int center){
        this.left = left;
        this.right = right;
        this.center = center;
    }

    // TODO: Make these accurate later :)
    public static double ticksToInches(double ticks){
        return ticks / 537.6 * 4 * Math.PI;
    }
    public static double inchesToTicks(double inches){
        return inches / (4 * Math.PI) * 537.6;
    }
}
