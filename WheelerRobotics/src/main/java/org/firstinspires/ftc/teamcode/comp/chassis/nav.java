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
            1, -1,
            -1, 1
    };
    double[] back = {
            -1, -1,
            -1, -1
    };
    double[] clock = {
            -1, 1,
            -1, 1
    };
    public void doTheThing(double l, double b, double r) {
        ElapsedTime et = new ElapsedTime();
        et.reset();
        Telemetry tele = FtcDashboard.getInstance().getTelemetry();
        congi c = new congi();
        double thresh = 1;
        double rthresh = 1;
        double dl = distanceLeft.getDistance(DistanceUnit.CM);
        double db = distanceBack.getDistance(DistanceUnit.CM);
        double dr = getAngles().firstAngle;
        delay(2000);
        Pidata pb = new Pidata();
        pb.init(db);
        pb.setTarget(b);

        Pidata pl = new Pidata();
        pl.init(dl);
        pl.setTarget(b);


        while (/*abs(dl-l) > thresh || abs(db-b) > thresh || abs(dr-r) > rthresh*/ true){
            //motorDriveForwardEncoded(0.5, 100);
            tele.addData("dl", dl);
            tele.addData("db", db);
            tele.addData("dr", dr);

            double[] out = {0, 0, 0, 0};

            double el = (dl-l);
            double er = (r-dr); // error rotation
            double eb = pb.tick(db);



            tele.addData("el", el);
            tele.addData("eb", eb);
            tele.addData("er", er);

            for (int i = 0; i<out.length; i++) { // add the individual vectors
                out[i] = el * (c.kd) * this.left[i] + eb * this.back[i]; //+ er * (c.kr) * this.clock[i];
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
            dr = getAngles().firstAngle;

            tele.addData("o", out);
            tele.update();
        }

        //motorStop();

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
