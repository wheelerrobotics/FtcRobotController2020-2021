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

@Autonomous( name = "Caurousel Red Nav")
public class AutoNavRedC extends LinearOpMode {
    // for non next to caurousel

    Meccanum meccanum = new Meccanum();
    BotVision bv = new BotVision();
    Telemetry tele = FtcDashboard.getInstance().getTelemetry();
    int FOOT = 333;
    double SIDEWAYST = 2 / sqrt(2);

    public static double firstL = 5;
    public static double secondL = 72;
    public static double secondB = 5;
    public static double firstR = 0;
    public static double secondR = 0;
    public static double firstB = 15;

    nav navi = new nav();



    @Override
    public void runOpMode() throws InterruptedException {
        meccanum.init(hardwareMap);
        bv.init(hardwareMap);
        navi.init(hardwareMap);
        waitForStart();
        executeAutomaticSequence1();

    }

    private void executeAutomaticSequence1() {
        // should get 22
        // auto for near carousel

        int DRIVE_AFTERARM = 350 ;
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
            MARKER_ARM = 900;
            DRIVE_AFTERARM = 150;
            MARKER_AFTERARM = 70;
        }else if(pos == 3){
            MARKER_ARM = 1300;
            DRIVE_AFTERARM = 240 ;
            MARKER_AFTERARM = 300;
        }

        meccanum.closeServoFull();
        // ()
        meccanum.motorDriveForwardEncoded(meccanum.NORMAL_SPEED, 820);
        // /\
        meccanum.turnDeg(-65, meccanum.SPIN_MOTORS_SPEED, telemetry);
        // ~>
        delay(1000);
        meccanum.moveArmEncoded(meccanum.ARM_MAX_SPEED, MARKER_ARM);
        // |\
        meccanum.motorDriveForwardEncoded(meccanum.NORMAL_SPEED, DRIVE_AFTERARM);
        // /\
        meccanum.openServoFull();
        delay(100);
        meccanum.moveArmEncoded(meccanum.ARM_MAX_SPEED, MARKER_AFTERARM);
        // (_
        meccanum.motorDriveBackwardEncoded(meccanum.NORMAL_SPEED, 100);
        // \/
        meccanum.turnDeg(-25, meccanum.SPIN_MOTORS_SPEED, telemetry); // first spin + 90
        // <~
        delay(100);
        //meccanum.motorDriveEncoded(meccanum.NORMAL_SPEED,200);
        // <-
        navi.init(hardwareMap);
        navi.doTheThingy(firstL, firstB, firstR, 3000, 0.5);
        delay(100);
        navi.spinnySpinTime(-0.3, 5000);
        navi.doTheThingy(secondL, secondB, secondR, 4000, 1);
        navi.moveArmTime(-meccanum.ARM_MAX_SPEED, 1000);

    }

    public void delay(double time) {
        if (time <= 100) meccanum.delay(time);
        else if (opModeIsActive()) {
            meccanum.delay(100);
            delay(time - 100);
        } else return;
    }


}
