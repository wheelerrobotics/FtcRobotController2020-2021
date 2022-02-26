package org.firstinspires.ftc.teamcode.comp.auto;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.comp.chassis.Meccanum;

@Autonomous( name = "Warehouse Red Nav")
public class AutoNavRed extends LinearOpMode {
    // for non next to caurousel
    Meccanum meccanum = new Meccanum();

    public void runOpMode() {
        meccanum.init2(hardwareMap);
        waitForStart();
        executeAutomaticSequence1();

    }

    private void executeAutomaticSequence1() {
        // auto for near carousel

        meccanum.closeServoFull();
        // ()
        delay(1000);
        meccanum.motorDriveForwardEncoded(meccanum.NORMAL_SPEED, 775);
        // /\
        meccanum.turnDeg(65, meccanum.SPIN_MOTORS_SPEED, telemetry);
        // ~>
        meccanum.moveArmTime(meccanum.ARM_MAX_SPEED, 1500);
        // |\
        meccanum.motorDriveForwardEncoded(meccanum.NORMAL_SPEED, 350);
        // /\
        meccanum.openServoFull();
        delay(1000);
        meccanum.moveArmTime(meccanum.ARM_MAX_SPEED, 400);
        // (_
        meccanum.motorDriveBackwardEncoded(meccanum.NORMAL_SPEED, 30);
        // \/
        meccanum.turnDeg(25, meccanum.SPIN_MOTORS_SPEED, telemetry); // first spin + 90
        // <~
        meccanum.motorDriveLeftEncoded(meccanum.NORMAL_SPEED, 350);
        // <-
        meccanum.motorDriveBackwardEncoded(1, 1400);
        // /\
        meccanum.turnDeg(180, meccanum.SPIN_MOTORS_SPEED, telemetry);

        delay(2000);


        //meccanum.motorDriveRelativeFieldAngleEncoded(90, meccanum.NORMAL_SPEED, 700);
    }

    public void delay(double time) {
        if (time <= 100) meccanum.delay(time);
        else if (opModeIsActive()) {
            meccanum.delay(100);
            delay(time - 100);
        } else return;
    }


}
