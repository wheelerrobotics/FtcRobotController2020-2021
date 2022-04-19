package org.firstinspires.ftc.teamcode.comp.chassis;

import static java.lang.Math.abs;

import android.view.Display;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.comp.test.congi;
import org.firstinspires.ftc.teamcode.comp.utility.Pidata;

import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.Arrays;

@Config
public class nav extends Meccanum{
    double[] left = {
            1,  -1,
            -1,  1
    };
    double[] back = {
            -1, -1,
            -1, -1
    };
    double[] clock = {
            -1,  1,
            -1,  1
    };
    public void doTheThing(double l, double b, double r, double breakTime) {
        ElapsedTime et = new ElapsedTime();
        et.reset();
        Telemetry tele = FtcDashboard.getInstance().getTelemetry();
        double thresh = 0.1;
        double rthresh = 0.1;
        double dl = distanceLeft.getDistance(DistanceUnit.CM);
        double db = distanceBack.getDistance(DistanceUnit.CM);
        double dri = distanceRight.getDistance(DistanceUnit.CM);
        double dr = getAngles().firstAngle;
        Pidata pb = new Pidata(-0.025, 0 ,0); // -0.025, -0.00008, -0.2
        pb.init(db);
        pb.setTarget(b);

        Pidata pl = new Pidata(-0.025, 0 ,0);
        pl.init(dl);
        pl.setTarget(l);

        /*
        Pidata pri = new Pidata(-0.025, 0 ,0);
        pri.init(dri);
        pri.setTarget(ri);
        */


        Pidata pr = new Pidata(0.005, 0, 0);
        pr.init(dr);
        pr.setTarget(r);
        double dthresh = 0.05;



        ElapsedTime start = new ElapsedTime();
        start.reset();
        while (true){
            //motorDriveForwardEncoded(0.5, 100);
            if (dl < 800) tele.addData("dl", dl);
            if (db < 800) tele.addData("db", db);
            if (dr < 800) tele.addData("dr", dr);
            //if (dri < 800) tele.addData("dri", dri);

             tele.addData("sl", pl.getDerivative());
             tele.addData("sb", pb.getDerivative());
             tele.addData("sr", pr.getDerivative());
    //            tele.addData("sri", pri.getDerivative());

            double[] out = {0, 0, 0, 0};
            //if(Math.random() > 0.6) camServo.setPosition(Math.random()); // really does nothing, but DONT DELETE
            double el = -pl.tick(dl);
            double er = pr.tick(dr); // error rotation
            double eb = pb.tick(db);
            //double eri = (ri == -1) ? pri.tick(dri) : 0;

            tele.addData("el", el);
            tele.addData("eb", eb);
            tele.addData("er", er);
            //tele.addData("eri", eri);

            for (int i = 0; i<out.length; i++) { // add the individual vectors
                out[i] = el * this.left[i] + eb * this.back[i] + er * this.clock[i]; //+ eri * -this.left[i];
            }

            double abc = absmac(out); // get max value for scaling
                if (abc > 1){
                    for (int i = 0; i<out.length; i++){ // normalize based on greatest value
                        out[i] /= abs(abc);
                    }
            }

            /*for (int i = 0; i<out.length; i++){ // scale down
                out[i] *= c.speed;
                //if (out[i] < 0.05) out[i] = 0;
            }**/
            driveVector(out);

            // update vals
            dl = distanceLeft.getDistance(DistanceUnit.CM);
            db = distanceBack.getDistance(DistanceUnit.CM);
            //dri = distanceRight.getDistance(DistanceUnit.CM);
            dr = getAngles().firstAngle;

            tele.addData("o", out);
            tele.update();

            if(
                    //abs(eri) < thresh &&
                    abs(eb) < thresh &&
                    abs(el) < thresh &&
                    abs(er) < rthresh &&
                    abs(pb.getDerivative()) > dthresh &&
                    //abs(pri.getDerivative()) > dthresh &&
                    abs(pl.getDerivative()) > dthresh &&
                    abs(pr.getDerivative()) > dthresh
            ) break;

            if(start.milliseconds() > breakTime) break;
        }

        motorStop();

    }
    public void doTheThingy(double ri, double b, double r, double breakTime) {
        ElapsedTime et = new ElapsedTime();
        et.reset();
        Telemetry tele = FtcDashboard.getInstance().getTelemetry();
        double thresh = 0.1;
        double rthresh = 0.1;
        double dri = distanceRight.getDistance(DistanceUnit.CM);
        double db = distanceBack.getDistance(DistanceUnit.CM);
        double dr = getAngles().firstAngle;
        Pidata pb = new Pidata(-0.025, 0 ,0); // -0.025, -0.00008, -0.2
        pb.init(db);
        pb.setTarget(b);

        Pidata pri = new Pidata(-0.025, 0 ,0);
        pri.init(dri);
        pri.setTarget(ri);

        /*
        Pidata pri = new Pidata(-0.025, 0 ,0);
        pri.init(dri);
        pri.setTarget(ri);
        */


        Pidata pr = new Pidata(0.005, 0, 0);
        pr.init(dr);
        pr.setTarget(r);
        double dthresh = 0.05;



        ElapsedTime start = new ElapsedTime();
        start.reset();
        while (true){
            //motorDriveForwardEncoded(0.5, 100);
            if (dri < 800) tele.addData("dri", dri);
            if (db < 800) tele.addData("db", db);
            if (dr < 800) tele.addData("dr", dr);
            //if (dri < 800) tele.addData("dri", dri);

            tele.addData("sri", pri.getDerivative());
            tele.addData("sb", pb.getDerivative());
            tele.addData("sr", pr.getDerivative());
            //            tele.addData("sri", pri.getDerivative());

            double[] out = {0, 0, 0, 0};
            //if(Math.random() > 0.6) camServo.setPosition(Math.random()); // really does nothing, but DONT DELETE
            double eri = -pri.tick(dri);
            double er = pr.tick(dr); // error rotation
            double eb = pb.tick(db);
            //double eri = (ri == -1) ? pri.tick(dri) : 0;

            tele.addData("el", eri);
            tele.addData("eb", eb);
            tele.addData("er", er);
            //tele.addData("eri", eri);

            for (int i = 0; i<out.length; i++) { // add the individual vectors
                out[i] = eri * -this.left[i] + eb * this.back[i] + er * this.clock[i]; //+ eri * -this.left[i];
            }

            double abc = absmac(out); // get max value for scaling
            if (abc > 1){
                for (int i = 0; i<out.length; i++){ // normalize based on greatest value
                    out[i] /= abs(abc);
                }
            }

            /*for (int i = 0; i<out.length; i++){ // scale down
                out[i] *= c.speed;
                //if (out[i] < 0.05) out[i] = 0;
            }**/
            driveVector(out);

            // update vals
            dri = distanceRight.getDistance(DistanceUnit.CM);
            db = distanceBack.getDistance(DistanceUnit.CM);
            //dri = distanceRight.getDistance(DistanceUnit.CM);
            dr = getAngles().firstAngle;

            tele.addData("o", out);
            tele.update();

            if(
                //abs(eri) < thresh &&
                    abs(eb) < thresh &&
                            abs(eri) < thresh &&
                            abs(er) < rthresh &&
                            abs(pb.getDerivative()) > dthresh &&
                            //abs(pri.getDerivative()) > dthresh &&
                            abs(pri.getDerivative()) > dthresh &&
                            abs(pr.getDerivative()) > dthresh
            ) break;

            if(start.milliseconds() > breakTime) break;
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
            if( abs(arr[i]) > abs(arr[outi])){
                outi = i;
            }
        }
        return arr[outi];

    }
}
