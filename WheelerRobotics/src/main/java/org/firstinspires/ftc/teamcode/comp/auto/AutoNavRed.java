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
public class AutoNavRed extends LinearOpMode {
    // for non next to caurousel
    Meccanum meccanum = new Meccanum();

    public void runOpMode() {
        meccanum.init(hardwareMap);
        waitForStart();
        executeAutomaticSequence1();

    }
    private void executeAutomaticSequence1() {
        // should get 26
        // auto for near carousel
        // gotta replace 0 with tested vals
        //meccanum.motorDriveRelativeFieldAngleEncoded(0, meccanum.NORMAL_SPEED, 775);
        // meccanum.turnDeg(90, 90, telemetry);
        /*
        JsonParser jp = new JsonParser();
        Object obj = jp.parse(new FileReader("./jsons/gameplanB.json"));
        JSONObject gameplan = (JSONObject) obj;
        for (Iterator<String> it = gameplan.keys(); it.hasNext(); ) {
            String k = it.next();
            String key = it.next();
            // this approach will be synchronous, but it could be async with a "completed" tag in the json move

            parseMove(gameplan.get(key));

        }*/


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
        meccanum.motorDriveForwardEncoded(meccanum.NORMAL_SPEED, 350);
        // /\
        meccanum.openServoFull();
        delay(1000);
        meccanum.moveArmTime(meccanum.ARM_MAX_SPEED, 400);
        // (_
        meccanum.motorDriveBackEncoded(meccanum.NORMAL_SPEED, 30);
        // \/
        meccanum.turnDeg(25, meccanum.SPIN_MOTORS_SPEED, telemetry); // first spin + 90
        // <~
        meccanum.motorDriveLeftEncoded(meccanum.NORMAL_SPEED,350);
        // <-
        meccanum.motorDriveBackEncoded(1, 1400);
        // /\
        meccanum.turnDeg(180, meccanum.SPIN_MOTORS_SPEED, telemetry);

        delay(2000);


        //meccanum.motorDriveRelativeFieldAngleEncoded(90, meccanum.NORMAL_SPEED, 700);
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
