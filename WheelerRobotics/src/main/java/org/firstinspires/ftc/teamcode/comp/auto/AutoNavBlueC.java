package org.firstinspires.ftc.teamcode.comp.auto;

import static java.lang.Math.floor;
import static java.lang.Math.sqrt;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.comp.chassis.Meccanum;
import org.firstinspires.ftc.teamcode.comp.chassis.nav;
import org.firstinspires.ftc.teamcode.comp.vision.BotVision;

@Autonomous( name = "Caurousel Blue Nav")
public class AutoNavBlueC extends LinearOpMode {
    // for non next to caurousel
    Meccanum meccanum = new Meccanum();
    BotVision bv = new BotVision();
    Telemetry tele = FtcDashboard.getInstance().getTelemetry();
    int FOOT = 333;
    double SIDEWAYST = 2 / sqrt(2);

    public static double firstL = 5;
    public static double firstB = 5;
    public static double secondL = 72;
    public static double secondB = 5;
    public static double firstR = 0;
    public static double secondR = 0;


    nav navi = new nav();
    public void runOpMode() {
        meccanum.init(hardwareMap);
        bv.init(hardwareMap);
        waitForStart();
        executeAutomaticSequence1();
    }

    private void executeAutomaticSequence1() {
        // auto for near carousel

        int DRIVE_AFTERARM = 350;
        int MARKER_ARM = 1500; // how high the arm should go to place block
        int MARKER_AFTERARM = 300; // how much the arm should raise after as not to catch
        meccanum.alignCamera();
        delay(1000);
        int pos = bv.getConePosition();
        tele.addData("pos", pos);
        tele.update();
        if(pos == 1){
            MARKER_ARM = 500;
            DRIVE_AFTERARM = 100;
            MARKER_AFTERARM = 30;
        }else if(pos == 2){
            MARKER_ARM = 850;
            DRIVE_AFTERARM = 250;
            MARKER_AFTERARM = 70;
        }else if(pos == 3){
            MARKER_ARM = 1300;
            DRIVE_AFTERARM = 350;
            MARKER_AFTERARM = 50;
        }

        meccanum.closeServoFull();
        // ()
        delay(1000);
        meccanum.motorDriveForwardEncoded(meccanum.NORMAL_SPEED, 820);
        // /\
        meccanum.turnDeg(65, meccanum.SPIN_MOTORS_SPEED, telemetry);
        // ~>
        meccanum.moveArmEncoded(meccanum.ARM_MAX_SPEED, MARKER_ARM);
        // |\
        meccanum.motorDriveForwardEncoded(meccanum.NORMAL_SPEED, DRIVE_AFTERARM);
        // /\
        meccanum.openServoFull();
        delay(1000);
        meccanum.moveArmEncoded(meccanum.ARM_MAX_SPEED, MARKER_AFTERARM);
        // (_
        meccanum.motorDriveBackwardEncoded(meccanum.NORMAL_SPEED, 100);
        // \/
        meccanum.turnDeg(25, meccanum.SPIN_MOTORS_SPEED, telemetry); // first spin + 90
        // <~
        delay(100);
        //meccanum.motorDriveEncoded(meccanum.NORMAL_SPEED,200);
        navi.init(hardwareMap);
        navi.doTheThing(firstL, firstB, firstR, 3000);
        delay(100);
        navi.spinnySpinTime(navi.OPTIMAL_SPINNER_POWER, 5000);
        navi.doTheThing(secondL, secondB, secondR, 4000);
    }

    public void delay(double time) {
        if (time <= 100) meccanum.delay(time);
        else if (opModeIsActive()) {
            meccanum.delay(100);
            delay(time - 100);
        } else return;
    }


}
