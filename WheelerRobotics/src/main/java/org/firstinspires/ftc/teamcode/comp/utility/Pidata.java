package org.firstinspires.ftc.teamcode.comp.utility;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public class Pidata {
    double integral = 0;
    double derivative = 0;
    double proportion = 0;
    double lastError = 0;
    double currentMeasurement = 0;
    double sinceLastMeasurement = 0;
    double sinceStart = 0;
    double error = 0;
    double target = 0;

    boolean paused = false;

    public static double ki = 0;
    public static double kp = 0;
    public static double kd = 0;

    ElapsedTime et = new ElapsedTime();

    public Pidata(double kp, double ki, double kd){
        this.ki = ki;
        this.kd = kd;
        this.kp = kp;
    }
    public Pidata(){}

    public void init(double data){
        lastError = target - data;
        et.reset();
    }
    public void setTarget(double data){
        target = data;
    }
    public double tick(double data){
        if (paused) return 0;
        currentMeasurement = data;
        double timeNow = et.milliseconds();
        sinceLastMeasurement = timeNow - sinceStart;
        sinceStart = timeNow;

        error = target - currentMeasurement;
        integral += sinceLastMeasurement * error;
        FtcDashboard.getInstance().getTelemetry().addData("i", integral);
        FtcDashboard.getInstance().getTelemetry().addData("e", error);
        derivative = (lastError - error)/sinceLastMeasurement;

        lastError = error;

        double out = kp * error + kd * derivative + ki * integral;
        return out;
    }
    public void setPauseState(boolean p){
        paused = p;
        if(!p) et.reset();
    }

}
