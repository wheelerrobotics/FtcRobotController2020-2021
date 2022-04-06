package org.firstinspires.ftc.teamcode.comp.chassis;

import static java.lang.Math.abs;

import android.view.Display;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.Arrays;

public class nav extends Meccanum{
    double[] left = {
            1, -1,
            -1, 1
    };
    double[] back = {
            1, -1,
            -1, 1
    };
    double[] clock = {
            1, -1,
            1, -1
    };
    public void doTheThing(double l, double b, double r) {
        double thresh = 10;
        double rthresh = 1;
        double dl = distanceLeft.getDistance(DistanceUnit.MM);
        double db = distanceLeft.getDistance(DistanceUnit.MM);
        double dr = getAngles().firstAngle;
        double kd = 0.5; //distance constant
        double kr = 1; //rot constant
        while (abs(dl-l) < thresh && abs(db-b) < thresh && abs(dr-r) < rthresh){
            double[] out = {0, 0, 0, 0};
            double el = (dl-l);
            double eb = (db-b);
            double er = (dr-r);
            for (int i = 0; i<out.length; i++) {
                out[i] = el * (kd) * this.left[i] + eb * (kd) * this.back[i] + er * (kr) * this.clock[i];
            }
            double abc = absmac(out);
            for (int i = 0; i<out.length; i++){
                out[i] *= 1/abs(abc);
            }
            driveVector(out);

            dl = distanceLeft.getDistance(DistanceUnit.MM);
            db = distanceLeft.getDistance(DistanceUnit.MM);
            dr = getAngles().firstAngle;
        }
        motorStop();


    }
    void driveVector(double[] arr){
        motorFrontLeft.setPower(arr[0]);
        motorFrontRight.setPower(arr[1]);
        motorBackLeft.setPower(arr[2]);
        motorBackRight.setPower(arr[3]);
    }
    double absmac(double[] arr){
        int outi = 0; // index of max
        for (int i = 0; i<arr.length; i++){
            if( abs(arr[i]) > arr[outi]){
                outi = i;
            }
        }
        return arr[outi];

    }
}
