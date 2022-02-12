package org.firstinspires.ftc.teamcode.comp.auto;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.comp.chassis.Meccanum;

@Autonomous( name = "Warehouse Blue Nav")
public class AutoNavBlue extends LinearOpMode {
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
        meccanum.turnDeg(-65, meccanum.SPIN_MOTORS_SPEED, telemetry);
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
        meccanum.turnDeg(-25, meccanum.SPIN_MOTORS_SPEED, telemetry); // first spin + 90
        // <~
        meccanum.motorDriveRightEncoded(meccanum.NORMAL_SPEED, 350);
        // <-
        meccanum.motorDriveBackEncoded(1, 1400);
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
    /*public void parseMove(JSONObject move) throws JSONException {
        switch (move.get("type")){
            case "LINE":
                // do stuff
                if(move.get("relative").equals(1)){
                    //meccanum.motorDriveLineRobotAngle();
                }else{
                    //meccanum.motorDriveLineFieldAngle();
                }
            case "TURN":

            case "SPINNER":

            case "CLAW":

            case "ARM":

            default:
                throw new IllegalStateException("Unexpected value: " + move.get("type"));
        }

    }*/


}
