package org.firstinspires.ftc.teamcode.comp.helpers;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public class PID {
    double integral = 0;
    double derivative = 1;
    double proportion = 0;
    double lastError = 0;
    double currentMeasurement = 0;
    double sinceLastMeasurement = 0;
    double sinceStart = 0;
    double error = 0;
    double target = 0;

    boolean paused = false;

    double ki = 0;
    double kp = 0;
    double kd = 0;

    // time break
    // rotation shit
    // threshold and derivative brake
    // tune value better (get freshman to do this)
    // make a thing that does drive froward while distance side is a and roattion is b
    // drive at an angle using same math/logic

    // check edge cases
    // sensor out of bounds
    // gyro flip over 0 or 180
    // gyro oscillation
    // absolute distance left, perpendicular line, div by sin CAREFUL OF CORNER EDGE CASES

    ElapsedTime et = new ElapsedTime();

    public PID(double p, double i, double d){
        ki = i;
        kd = d;
        kp = p;
    }

    public void init(double data){
        lastError = target - data;
        et.reset();
    }
    public void setTarget(double data){
        target = data;
    }
    public double getDerivative(){
        return derivative;
    }
    public double tick(double data){
        if (paused) return 0;
        if (data > 800) return 0;
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
