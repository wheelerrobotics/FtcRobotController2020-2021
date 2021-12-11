package org.firstinspires.ftc.teamcode.comp.auto;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.sun.tools.javac.comp.Todo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.comp.chassis.Meccanum;

@Autonomous
public class AutoNavBlueC extends LinearOpMode {
    // for non next to caurousel
    Meccanum meccanum = new Meccanum();

    public void runOpMode() {
        meccanum.init(hardwareMap);
        waitForStart();
        executeAutomaticSequence1();

    }
    private void executeAutomaticSequence1(){
        // should get 22

        // auto for near carousel
        //FILL IN THE NON NEAR CAROUSEL HERE WITH FLIPPED VALS
        meccanum.closeServoFull();
        // ()
        meccanum.delay(1000);
        meccanum.motorDriveForwardEncoded(meccanum.NORMAL_SPEED, 775);
        // /\
        meccanum.turnDeg(65, meccanum.SPIN_MOTORS_SPEED, telemetry);
        // ~>
        meccanum.moveArmTime(meccanum.ARM_MAX_SPEED, 1000);
        // |\
        meccanum.motorDriveForwardEncoded(meccanum.NORMAL_SPEED, 300);
        // /\
        meccanum.openServoFull();
        meccanum.delay(1000);
        // (_
        meccanum.motorDriveBackEncoded(meccanum.NORMAL_SPEED, 30);
        // \/
        meccanum.turnDeg(25, meccanum.SPIN_MOTORS_SPEED, telemetry); // first spin + 90
        // <~
        meccanum.motorDriveLeftEncoded(meccanum.NORMAL_SPEED,200);
        // <-
        meccanum.motorDriveBackEncoded(1, 1300);
        // /\
        // here you are facing the warehouse
        meccanum.motorDriveRightEncoded(meccanum.NORMAL_SPEED, 800);
        // ->
        meccanum.spinnySpinTime(meccanum.OPTIMAL_SPINNER_POWER, 1000);
        // *
        meccanum.motorDriveForwardEncoded(meccanum.NORMAL_SPEED, 775);
        meccanum.delay(1000);
        // /\
        meccanum.motorDriveRightEncoded(meccanum.NORMAL_SPEED, 775);
        meccanum.delay(1000);
        // ->



    }



}
