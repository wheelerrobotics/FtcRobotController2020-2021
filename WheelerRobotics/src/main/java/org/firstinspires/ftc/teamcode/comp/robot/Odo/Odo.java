package org.firstinspires.ftc.teamcode.comp.robot.Odo;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import static java.lang.Math.abs;

import android.content.Context;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.comp.chassis.Meccanum.Meccanum;
import org.firstinspires.ftc.teamcode.comp.helpers.PID;
import org.firstinspires.ftc.teamcode.comp.robot.Robot;
import org.firstinspires.ftc.teamcode.comp.utility.Encoders;
import org.firstinspires.ftc.teamcode.comp.utility.Pose;
import org.firstinspires.ftc.teamcode.comp.vision.BotVision;
import org.firstinspires.ftc.teamcode.comp.vision.pipelines.AprilTagDetectionPipeline;
import org.openftc.apriltag.AprilTagDetection;

import java.util.ArrayList;

public class Odo extends Meccanum implements Robot {

    protected Servo servo = null;
    protected Telemetry tele = tele = FtcDashboard.getInstance().getTelemetry();
    protected HardwareMap hw = null;

    public boolean detecting = true;
    public BotVision bv = new BotVision();
    public AprilTagDetectionPipeline atdp =  new AprilTagDetectionPipeline(0.166, 578.272, 578.272, 402.145, 221.506);

    private Encoders encoders;
    private Pose pose;
    private double dx, dy, dtheta;
    public ArrayList<AprilTagDetection> detections = new ArrayList<>();

    @Override
    public void init(HardwareMap hardwareMap) {
        super.init(hardwareMap);
        // init the class, declare all the sensors and motors and stuff
        // should be called before using class ALWAYS

        // internal IMU setup (copied and pasted, idk what it really does, but it works)
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        // angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);

        //distace sensors (unused for now)

        distanceBack = hardwareMap.get(DistanceSensor.class, "distanceBack");
        distanceRight = hardwareMap.get(DistanceSensor.class, "distanceRight");
        distanceLeft = hardwareMap.get(DistanceSensor.class, "distanceLeft");
        distanceFront = hardwareMap.get(DistanceSensor.class, "distanceFront");

        // Meccanum Motors Definition and setting prefs

        motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft");
        motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft");
        motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight");
        motorBackRight = hardwareMap.dcMotor.get("motorBackRight");

        // Reverse the left side motors and set behaviors to stop instead of coast

        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //define arm and servo objects and also spinner
        servo = hardwareMap.get(Servo.class, "servo");

        //set prefs for arm and servo
        servo.setDirection(Servo.Direction.FORWARD);

        // define hw as the hardware map for possible access later in this class
        hw = hardwareMap;

        bv.init(hardwareMap, atdp);
        AprilThread at = new AprilThread();
        at.start();

        pose = new Pose(0, 0, 0);
        PoseThread pt = new PoseThread();
        pt.start();

        runtime.reset();
    }

    public Pose getPose() {
        return this.pose;
    }
    /*
    public sampleSensorsForPose() {
        this.dx =
    }
    public updatePose(){

    }
*/


    public void alignCamera(){
        servo.setPosition(0.5);
    }
    public int getPrincipalTag() {
        return ((detections != null) ? detections.get(0).id : 0);
    }
    public void playSound(String filename){
        // play a sound
        // doesnt work but would be really fun :(

        int startupID = hw.appContext.getResources().getIdentifier(filename, "raw", hw.appContext.getPackageName());
        Context appContext = hw.appContext;
        SoundPlayer.getInstance().startPlaying(appContext, startupID);
    }
    public Orientation getAngularOrientation(){
        return imu.getAngularOrientation();
    }
    public Encoders getEncoders(){
        updateEncoders();
        return encoders;
    }
    public void updateEncoders(){
        try {
            encoders.right = motorBackLeft.getCurrentPosition();
            encoders.left = motorBackRight.getCurrentPosition();
            encoders.center = motorFrontLeft.getCurrentPosition();
        }
        catch (Exception e) {

        }

    }
    // Late night thoughts so I can continue them tmrw:
    /*
    - ideally, run the positioning stuff on a seperate thread
        - have a method to talk to that thread and get position while we do other stuff on main.
        - this is convinient because it means we dont have to worry about delay on the position resulting in more accurate measurements.
    - Look at the gm0 thing i have open to figure out how to use the encoders.
    - Would be sick if we actually use tensorflow models. Could be useful for object detection of the junctions.

     */
    private class AprilThread extends Thread {
        public AprilThread()
        {
            this.setName("AprilThread");


        }
        @Override
        public void run() {
            int numFramesWithoutDetection = 0;
            final int DECIMATION_LOW = 2;
            final int DECIMATION_HIGH = 3;
            final float THRESHOLD_HIGH_DECIMATION_RANGE_METERS = 1.0f;
            final int THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION = 4;

            while (detecting) {
                detections = atdp.getDetectionsUpdate();
                if (detections != null) {

                    // If we don't see any tags
                    if (detections.size() == 0) {
                        numFramesWithoutDetection++;

                        // If we haven't seen a tag for a few frames, lower the decimation
                        // so we can hopefully pick one up if we're e.g. far back
                        if (numFramesWithoutDetection >= THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION) {
                            atdp.setDecimation(DECIMATION_LOW);
                        }
                    }
                    // We do see tags!
                    else {
                        numFramesWithoutDetection = 0;

                        // If the target is within 1 meter, turn on high decimation to
                        // increase the frame rate
                        if (detections.get(0).pose.z < THRESHOLD_HIGH_DECIMATION_RANGE_METERS) {
                            atdp.setDecimation(DECIMATION_HIGH);
                        }
                    }

                }
            }
        }
    }

    private class PoseThread extends Thread
    {
        public PoseThread()
        {
            this.setName("PoseThread");


        }

        // called when tread.start is called. thread stays in loop to do what it does until exit is
        // signaled by main code calling thread.interrupt.
        @Override
        public void run() {
            // we record the Y values in the main class to make showing them in telemetry
            // easier.

            while (!isInterrupted()) {
                try {
                    //updateEncoders();
                    int center = motorFrontLeft.getCurrentPosition();
                    int right = motorBackLeft.getCurrentPosition();
                    int left = motorBackRight.getCurrentPosition();

                    pose.setPose(center, (left + right) / 2f, left - right);

                    //tele.addData("r", pose.x);
                    //tele.addData("l", pose.y);
                    //tele.addData("c", pose.theta);

                    //tele.update();
                    // telemetry.update();
                }
                catch (Exception e){
                    e.printStackTrace();
                }

            }
        }
    }
}
