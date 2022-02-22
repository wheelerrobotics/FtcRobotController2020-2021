package org.firstinspires.ftc.teamcode.comp.test.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.comp.chassis.Meccanum;
import org.firstinspires.ftc.teamcode.comp.vision.BotVision;

import java.util.HashMap;

@Config
@TeleOp(name="thats crazy")
public class PIDauto extends LinearOpMode {
    // for non next to caurousel
    public static double kd = 0.0;
    public static double kp = 0.0;
    Meccanum meccanum = new Meccanum();

    public void runOpMode() {
        meccanum.init(hardwareMap);

        waitForStart();
        boolean cool = false;
        while(opModeIsActive()){
            if(gamepad1.a && !cool){
                cool = true;
                pidfunc(1800, FtcDashboard.getInstance().getTelemetry(), meccanum);
            }

        }

    }
    public void pidfunc(double degrees, Telemetry telemetry, Meccanum meccanum){
        // self correcting method to turn to an angle relative to current angle
        // used in auto (SUPER COOL)

        double startPos = meccanum.spinner.getCurrentPosition();
        ElapsedTime time = new ElapsedTime();
        time.reset();
        //double integration = 0.0;
        double derivative = 0.0;
        double proportion = 0.0;

        HashMap<Integer, Double> et = new HashMap<Integer, Double>(); // error: rt-yt
        HashMap<Integer, Double> yt = new HashMap<Integer, Double>(); // measured output
        HashMap<Integer, Double> rt = new HashMap<Integer, Double>(); // setpoint/target NOTE: this will not change, but using a hashmap instead of static var makes it future proof
        HashMap<Integer, Double> dt = new HashMap<Integer, Double>(); // change in time over previous interval in millis

        ElapsedTime changeTime = new ElapsedTime();

        double crt = 0.0;
        double cyt = 0.0;
        double cet = 0.0;

        while(true){ // start degrees + degrees is the actual target postition

            changeTime.reset();

            double writeVal = 0;
            //current vals
            crt = startPos + degrees;
            cyt = startPos - meccanum.spinner.getCurrentPosition();
            cet = cyt - crt;


            telemetry.addData("siz", dt.size());

            int ct = dt.size();
            rt.put(rt.size(), crt);
            yt.put(yt.size(), cyt);
            et.put(et.size(), cet);
            dt.put(dt.size(), changeTime.milliseconds());



            //HashMap<Integer, Double> ut = new HashMap<>(); // input
            // maybe change from traditional pid to a pid with an integral that only reacts to change if there is notable difference between yt and rt
            //integration += et.get(ct) * dt.get(ct); // integrates over time
            if(et.containsKey(ct-1)) derivative = ( cet - et.get(ct-1) ) / dt.get(ct); // slope between now and last time step
            else derivative = 0;
            proportion = cet; // const

            writeVal = proportion * kp + derivative * kd;

            meccanum.spinnySpin(writeVal);

            telemetry.addData("derivative", derivative);
            telemetry.addData("proportion", proportion);

            telemetry.addData("yt", cyt);
            telemetry.addData("rt", crt);
            telemetry.addData("et", cet);

            telemetry.update();



        }
    }

}
