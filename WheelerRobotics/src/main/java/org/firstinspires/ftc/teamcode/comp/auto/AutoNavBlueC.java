package org.firstinspires.ftc.teamcode.comp.auto;

import static java.lang.Math.floor;
import static java.lang.Math.sqrt;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.comp.chassis.Meccanum;

@Autonomous( name = "Caurousel Blue Nav")
public class AutoNavBlueC extends LinearOpMode {
    // for non next to caurousel
    Meccanum meccanum = new Meccanum();
    int FOOT = 333;
    double SIDEWAYST = 2 / sqrt(2);

    public void runOpMode() {
        meccanum.init(hardwareMap);
        waitForStart();
        executeAutomaticSequence1();

    }

    private void executeAutomaticSequence1() {
        // auto for near carousel

        meccanum.closeServoFull();
        // ()
        delay(1000);
        meccanum.motorDriveEncodedReg(-meccanum.NORMAL_SPEED,
                -meccanum.NORMAL_SPEED,
                -meccanum.NORMAL_SPEED,
                -meccanum.NORMAL_SPEED,
                775,
                telemetry);
        // /\
        meccanum.turnDeg(65, meccanum.SPIN_MOTORS_SPEED, telemetry);
        // ~>
        meccanum.moveArmTime(meccanum.ARM_MAX_SPEED, 1500);
        // |\
        meccanum.motorDriveForwardEncoded(meccanum.NORMAL_SPEED, 300);
        // /\
        meccanum.openServoFull();
        delay(1000);
        meccanum.moveArmTime(meccanum.ARM_MAX_SPEED, 300);
        // (_
        meccanum.motorDriveBackwardEncoded(meccanum.NORMAL_SPEED, 30);
        // \/
        meccanum.turnDeg(25, meccanum.SPIN_MOTORS_SPEED, telemetry); // first spin + 90
        // <~
        delay(100);
        //meccanum.motorDriveEncoded(meccanum.NORMAL_SPEED,200);
        // <-
        meccanum.motorDriveBackwardEncoded(0.5, 2 * FOOT + 140);
        // /\
        meccanum.delay(2000);
        //
        // here you are facing the warehouse
        meccanum.motorDriveLeftEncoded(meccanum.NORMAL_SPEED, (int) floor(2 * FOOT * SIDEWAYST + 110));
        // ->
        meccanum.spinnySpinTime(meccanum.OPTIMAL_SPINNER_POWER, 2000);
        // *
        // /\
        meccanum.motorDriveRightEncoded(meccanum.NORMAL_SPEED, (int) floor(2 * FOOT * SIDEWAYST + 40));
        delay(1000);
        meccanum.motorDriveBackwardEncoded(meccanum.NORMAL_SPEED, (int) floor(100));
        // ->


    }

    public void delay(double time) {
        if (time <= 100) meccanum.delay(time);
        else if (opModeIsActive()) {
            meccanum.delay(100);
            delay(time - 100);
        } else return;
    }


}
