package org.firstinspires.ftc.teamcode.comp.utility;

public class Pose {

    // question: should I have the delta vars in here?
    public double x = 0;
    public double y = 0;
    public double theta = 0;
    public Pose(double x, double y, double theta){
        this.x = x;
        this.y = y;
        this.theta = theta;
    }
    public void setPose(double x, double y, double theta){
        this.x = x;
        this.y = y;
        this.theta = theta;
    }
}
