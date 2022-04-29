package org.firstinspires.ftc.teamcode.comp.auto;


import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.usb.serial.SerialPort;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.comp.chassis.Meccanum;
import org.firstinspires.ftc.teamcode.comp.vision.BotVision;

@Autonomous( name = "Warehouse Blue Nav")
public class AutoNavBlue extends LinearOpMode {
    // for non next to caurousel
    Meccanum meccanum = new Meccanum();
    BotVision bv = new BotVision();
    Telemetry tele = FtcDashboard.getInstance().getTelemetry();
    public void runOpMode() {
        meccanum.init(hardwareMap);
        bv.init(hardwareMap);

        waitForStart();
        executeAutomaticSequence1();

    }

    private void executeAutomaticSequence1() {
        // should get 26
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
        meccanum.motorDriveForwardEncoded(meccanum.NORMAL_SPEED, 775);
        // /\
        meccanum.turnDeg(-65, meccanum.SPIN_MOTORS_SPEED, telemetry);
        // ~>
        meccanum.moveArmEncoded(meccanum.ARM_MAX_SPEED, MARKER_ARM);
        // |\
        meccanum.motorDriveForwardEncoded(meccanum.NORMAL_SPEED, DRIVE_AFTERARM);
        // /\
        meccanum.openServoFull();
        delay(1000);
        meccanum.moveArmEncoded(meccanum.ARM_MAX_SPEED, MARKER_AFTERARM);
        // (_
        meccanum.motorDriveBackwardEncoded(meccanum.NORMAL_SPEED, 30);
        // \/
        meccanum.turnDeg(-25, meccanum.SPIN_MOTORS_SPEED, telemetry); // first spin + 90
        // <~
        meccanum.motorDriveRightEncoded(meccanum.NORMAL_SPEED, 350);
        // <-
        meccanum.motorDriveBackwardEncoded(1, 1600);
        // /\
        meccanum.turnDeg(180, meccanum.SPIN_MOTORS_SPEED, telemetry);

        delay(2000);

    }

    public void delay(double time) {
        if (time <= 100) meccanum.delay(time);
        else if (opModeIsActive()) {
            meccanum.delay(100);
            delay(time - 100);
        }
    }



}
