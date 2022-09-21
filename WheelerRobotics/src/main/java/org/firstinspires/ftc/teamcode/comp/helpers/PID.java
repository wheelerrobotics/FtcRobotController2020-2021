package org.firstinspires.ftc.teamcode.comp.helpers;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Arrays;
import java.util.HashMap;

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

    boolean correctJitter = false;
    double[] pastJitter = {0, 0, 0, 0, 0};

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

    public PID(double p, double i, double d, boolean correctJitter){
        ki = i;
        kd = d;
        kp = p;
        this.correctJitter = correctJitter;
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
    public double averageArr(double[] arr){
        double sum = 0;
        for (double i : arr) sum += i;
        return sum/arr.length;
    }
    public double tick(double data){
        if (paused) return 0;
        if (data > 800) return 0;
        currentMeasurement = data;
        double timeNow = et.milliseconds();
        sinceLastMeasurement = timeNow - sinceStart;
        sinceStart = timeNow;
        if (this.correctJitter) {
            for (int i = 1; i < pastJitter.length; i++) pastJitter[i-1] = pastJitter[i];
            pastJitter[pastJitter.length - 1] = currentMeasurement;
        }

        double pastJitterAvg = averageArr(pastJitter);

        error = target - currentMeasurement;
        integral += sinceLastMeasurement * error;
        FtcDashboard.getInstance().getTelemetry().addData("i", integral);
        FtcDashboard.getInstance().getTelemetry().addData("e", error);
        derivative = ((correctJitter ? pastJitterAvg : lastError) - error)/sinceLastMeasurement;

        lastError = error;

        double out = kp * error + kd * derivative + ki * integral;
        return out;
    }
    public void setPauseState(boolean p){
        paused = p;
        if(!p) et.reset();
    }

}
