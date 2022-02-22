package org.firstinspires.ftc.teamcode.comp.test.auto;

//THIS CLASS IS UNSTABLE, (not tested and testing for auto layer detection)

// top -> 1500, 300
// middle -> 500, 50 (prob) TEST
// bottom -> 100, 30 (prob) TEST

// default to top level

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.comp.chassis.Meccanum;
import org.firstinspires.ftc.teamcode.comp.vision.BotVision;

@Autonomous
public class DetectingBlueNav extends LinearOpMode {
    // for non next to caurousel
    Meccanum meccanum = new Meccanum();
    private BotVision bv = new BotVision();

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
        int pos = bv.getConePosition();

        if(pos == 1){
            MARKER_ARM = 306;
            DRIVE_AFTERARM = 150;
            MARKER_AFTERARM = 90;
        }else if(pos == 2){
            MARKER_ARM = 666;
            DRIVE_AFTERARM = 250;
            MARKER_AFTERARM = 50;
        }else if(pos == 3){
            MARKER_ARM = 1500;
            DRIVE_AFTERARM = 350;
            MARKER_AFTERARM = 300;
        }

        meccanum.closeServoFull();
        // ()
        delay(1000);
        meccanum.motorDriveEncodedReg(
                -meccanum.NORMAL_SPEED,
                -meccanum.NORMAL_SPEED,
                -meccanum.NORMAL_SPEED,
                -meccanum.NORMAL_SPEED,
                775,
                telemetry);
        // /\
        meccanum.turnDeg(-65, meccanum.SPIN_MOTORS_SPEED, telemetry);
        // ~>
        meccanum.moveArmTime(meccanum.ARM_MAX_SPEED, MARKER_ARM);
        // |\
        meccanum.motorDriveForwardEncoded(meccanum.NORMAL_SPEED, DRIVE_AFTERARM);
        // /\
        meccanum.openServoFull();
        delay(1000);
        meccanum.moveArmTime(meccanum.ARM_MAX_SPEED, MARKER_AFTERARM);
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
