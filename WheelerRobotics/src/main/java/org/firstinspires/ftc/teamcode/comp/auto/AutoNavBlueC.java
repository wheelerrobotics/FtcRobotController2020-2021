package org.firstinspires.ftc.teamcode.comp.auto;

import static java.lang.Math.floor;
import static java.lang.Math.sqrt;

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
    int FOOT = 333;
    double SIDEWAYST = 2/sqrt(2);
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
        meccanum.moveArmTime(meccanum.ARM_MAX_SPEED, 1000);
        // |\
        meccanum.motorDriveForwardEncoded(meccanum.NORMAL_SPEED, 300);
        // /\
        meccanum.openServoFull();
        delay(100);
        // (_
        meccanum.motorDriveBackEncoded(meccanum.NORMAL_SPEED, 30);
        // \/
        meccanum.turnDeg(25, meccanum.SPIN_MOTORS_SPEED, telemetry); // first spin + 90
        // <~
        delay(100);
        //meccanum.motorDriveEncoded(meccanum.NORMAL_SPEED,200);
        // <-
        meccanum.motorDriveBackEncoded(0.5, 2*FOOT+140);
        // /\
        meccanum.delay(2000);
        // /\
        // here you are facing the warehouse
        meccanum.motorDriveLeftEncoded(meccanum.NORMAL_SPEED, (int) floor(2*FOOT*SIDEWAYST + 50));
        // ->
        meccanum.spinnySpinTime(meccanum.OPTIMAL_SPINNER_POWER, 2000);
        // *
        // /\
        meccanum.motorDriveRightEncoded(meccanum.NORMAL_SPEED, (int) floor(2*FOOT*SIDEWAYST + 50));
        delay(1000);
        meccanum.motorDriveBackEncoded(meccanum.NORMAL_SPEED, (int) floor(100));
        // ->



    }
    public void delay(double time){
        if(time <= 100) meccanum.delay(time);
        else if(opModeIsActive()){
            meccanum.delay(100);
            delay(time - 100);
        }
        else return;
    }


}
