package org.firstinspires.ftc.teamcode.comp.chassis;


import static java.lang.Math.abs;
import static java.lang.Math.cos;
import static java.lang.Math.min;
import static java.lang.Math.pow;
import static java.lang.Math.sin;

import android.content.Context;

import androidx.annotation.NonNull;
import androidx.core.graphics.ColorUtils;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorREV2mDistance;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import javax.lang.model.type.NoType;

// robot driving and motion class

public class Meccanum {

    private final ElapsedTime runtime = new ElapsedTime(); // getting a warning to make it final

    private Servo servo0;
    private DcMotor arm;

    public double rx;


    public final String SINGLEPLAYER_CONTROL = "SINGLEPLAYER";
    public final String MULTIPLAYER_CONTROL = "MULTIPLAYER";

    // static variables
    public final double NORMAL_SPEED = 0.2;
    public final double SERVO_FULLY_CLOSED = 0.0;
    public final double SERVO_FULLY_OPENED = 0.5;
    public final double HALF_SERVO_ANGLE = 0.5;
    public final double BACK_SERVO_ANGLE = Math.PI;
    public final double ARM_MAX_SPEED = -   0.5;
    public final double HIGH_SPINNER_POWER = 1;
    public final double OPTIMAL_SPINNER_POWER = 0.5;
    public final double MOTOR_STOP = 0;
    public final double SPIN_MOTORS_SPEED = 0.3;

    public boolean opModeActive = false;

    private DcMotor spinner;

    private BNO055IMU imu;

    private DcMotor motorFrontRight;
    private DcMotor motorBackRight;
    private DcMotor motorFrontLeft;
    private DcMotor motorBackLeft;

    private Orientation angles;

    public HardwareMap hw;

    private float INITIAL_ANGLE;

    public FtcDashboard dash = FtcDashboard.getInstance();

    private SensorREV2mDistance distanceBack;
    private SensorREV2mDistance distanceRight;
    private SensorREV2mDistance distanceLeft;


    private int startupID;
    private Context appContext;


    public void init(@NonNull HardwareMap hardwareMap){
        // init the class, declare all the sensors and motors and stuff
        // should be called before using class ALWAYS

        // internal IMU setup (copied and pasted, idk what it really does, but it works)
        opModeActive = true;
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);

        INITIAL_ANGLE = getAngles().firstAngle;

        //distace sensors (unused for now)
        /*
        distanceBack = hw.get(SensorREV2mDistance.class, "distanceBack");
        distanceRight = hw.get(SensorREV2mDistance.class, "distanceRight");
        distanceLeft = hw.get(SensorREV2mDistance.class, "distanceLeft");
        */

        // Meccanum Motors Definition and setting prefs

        motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft");
        motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft");
        motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight");
        motorBackRight = hardwareMap.dcMotor.get("motorBackRight");

        // Reverse the left side motors and set behaviors to stop instead of coast
        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //define arm and servo objects and also spinner
        servo0 = hardwareMap.get(Servo.class, "servo-0");
        arm = hardwareMap.get(DcMotor.class, "arm");
        spinner = hardwareMap.get(DcMotor.class, "spinner");

        //set prefs for arm and servo
        servo0.setDirection(Servo.Direction.FORWARD);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // define hw as the hardware map for possible access later in this class
        hw = hardwareMap;

        runtime.reset();
    }

    // INDEV METHODS
    public void regulateArm(double scale){
        // METHOD IN DEVELOPMENT
        // eliminates the problem of the arm falling when not moving up
        // useful everywhere if done right
        arm.setTargetPosition(arm.getCurrentPosition());
    }
    public void motorDriveRelativeAngleEncoded(double radians, double speed, double ticks){
        // METHOD STILL IN DEVELOPMENT
        // this method aims to drive the robot's meccanum configuration an encoded distance in a direction
        // this would be extremely useful especially if direction correction is added. It would make auto much easier and more precise

        // im not sure how to acurately do this using encoders, because some wheels are going to spin at different powers (I think)
        // this will cause the ticks to be difficult to calculate, and I dont really want to deal with that rn

        float startAngle = getAngles().firstAngle;
        double maxSpeed = 0.8;
        double spinvec = 0; // not spinning
        double yvec = min(speed, maxSpeed) * sin(radians); // ima be honest, i did this math for a js project 6 months ago and am just hopin it actually works in this context
        double xvec = min(speed, maxSpeed) * cos(radians)+1;

        double y = pow(-yvec,3); // Remember, this is reversed!
        double x = pow(xvec * 1.1,3); // Counteract imperfect strafing
        double rx = pow(spinvec,3);

        double CORRECTION_FACTOR = 0.01;

        //denominator is the largest motor power (absolute value) or 1
        //this ensures all the powers maintain the same ratio, but only when
        //at least one is out of the range [-1, 1]

        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        int blip = motorBackLeft.getCurrentPosition(); // back left initial position :)
        int brip = motorBackRight.getCurrentPosition();
        int flip = motorFrontLeft.getCurrentPosition();
        int frip = motorFrontRight.getCurrentPosition();

        double denominator = Math.max(abs(y) + abs(x) + abs(rx), maxSpeed);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;


        while (abs(motorBackLeft.getCurrentPosition() - blip) < ticks ||
                abs(motorFrontRight.getCurrentPosition() - frip) < ticks ||
                abs(motorFrontLeft.getCurrentPosition() - flip) < ticks ||
                abs(motorBackRight.getCurrentPosition() - brip) < ticks) {

            rx = (INITIAL_ANGLE-getAngles().firstAngle * CORRECTION_FACTOR);
            denominator = Math.max(abs(y) + abs(x) + abs(rx), maxSpeed);
            frontLeftPower = (y + x + rx) / denominator;
            backLeftPower = (y - x + rx) / denominator;
            frontRightPower = (y - x - rx) / denominator;
            backRightPower = (y + x - rx) / denominator;
            motorDrive(frontLeftPower, backLeftPower, frontRightPower, backRightPower);

        }
    }

    // SUPPORT METHODS
    public void motorDrive(double motorFrontLeftPower, double motorBackLeftPower, double motorFrontRightPower, double motorBackRightPower){
        // drive the motors at custom powers for each
        // used for every other drive class

        motorBackLeft.setPower(motorBackLeftPower);
        motorFrontLeft.setPower(motorFrontLeftPower);
        motorBackRight.setPower(motorBackRightPower);
        motorFrontRight.setPower(motorFrontRightPower);
    }
    private void motorDriveEncoded(double motorFrontLeftPower, double motorBackLeftPower, double motorFrontRightPower, double motorBackRightPower, int ticks){
        // basic class for driving motors but using their encoded values. NO PID FUNCTIONALITY
        // used by every encoded drive class

        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // NOTE: not using DcMotor.setTarget() method despite it's built in pid functionality, maybe implement in future, but I dont know if it will stall the program or not like the while loop
        // idk if the blp type vars will be static or change :/

        int blp = motorBackLeft.getCurrentPosition(); // back left initial position :)
        int brp = motorBackRight.getCurrentPosition();
        int flp = motorFrontLeft.getCurrentPosition();
        int frp = motorFrontRight.getCurrentPosition();

        while(abs(motorBackLeft.getCurrentPosition() - blp) < ticks ||
                abs(motorFrontRight.getCurrentPosition() - brp) < ticks ||
                abs(motorFrontLeft.getCurrentPosition() - flp) < ticks ||
                abs(motorBackRight.getCurrentPosition() - frp) < ticks) { // hopefully checks that it is within the positive or negative threshold of target ticks
            motorDrive(motorFrontLeftPower, motorBackLeftPower, motorFrontRightPower, motorBackRightPower);
        }
        motorStop();
    }
    public void motorDriveEncodedReg(double motorFrontLeftPower, double motorBackLeftPower, double motorFrontRightPower, double motorBackRightPower, double ticks, Telemetry telemetry){
        // METHOD STILL IN DEVELOPMENT?
        // this thing is trying to drive in a straight line, idk if it works
        // if it does work, then that would be sick

        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        /*
        final int blp = motorBackLeft.getCurrentPosition(); // idk if this will stay a static value or if it will change with the motor pos, hmm...
        final int brp = motorBackRight.getCurrentPosition();
        final int frp = motorFrontRight.getCurrentPosition();
        final int flp = motorFrontLeft.getCurrentPosition();
        */
        // only checking one motor for ticks rotated rn, should probably check all of them

        // NOTE: not using DcMotor.setTarget() method despite it's built in pid functionality, maybe implement in future, but I dont know if it will stall the program or not like the while loop
        // idk if the blp type vars will be static or change :/

        int blp = motorBackLeft.getCurrentPosition(); // back left initial position :)
        int brp = motorBackRight.getCurrentPosition();
        int flp = motorFrontLeft.getCurrentPosition();
        int frp = motorFrontRight.getCurrentPosition();

        double motorFrontLeftPowerCorrected = motorFrontLeftPower;
        double motorBackLeftPowerCorrected = motorBackLeftPower;
        double motorBackRightPowerCorrected = motorBackRightPower;
        double motorFrontRightPowerCorrected = motorFrontRightPower;
        float startDegrees = getAngles().firstAngle;
        double LOCAL_PROPORTION = 0.04 ;

        while((abs(motorBackLeft.getCurrentPosition() - blp) + abs(motorFrontRight.getCurrentPosition() - frp) + abs(motorFrontLeft.getCurrentPosition() - flp) + abs(motorBackRight.getCurrentPosition() - brp))/4  < ticks) { // hopefully checks that it is within the positive or negative threshold of target ticks
            double difference = AngleUnit.normalizeDegrees(getAngles().firstAngle - startDegrees);
            motorDrive(motorFrontLeftPower + (difference * LOCAL_PROPORTION), motorBackLeftPower + (difference * LOCAL_PROPORTION), motorFrontRightPower - (difference * LOCAL_PROPORTION), motorBackRightPower - (difference * LOCAL_PROPORTION));


            //motorFrontLeftPowerCorrected = motorFrontLeftPower - (difference * LOCAL_PROPORTION);
            //motorBackLeftPowerCorrected = motorBackLeftPower - (difference * LOCAL_PROPORTION);
            //motorBackRightPowerCorrected = motorBackRightPower + (difference * LOCAL_PROPORTION);
            //motorFrontLeftPower - (difference * LOCAL_PROPORTION)
            //motorFrontRightPowerCorrected = motorFrontRightPower + (difference * LOCAL_PROPORTION);

            telemetry.addData("fl", motorFrontLeftPower + (difference * LOCAL_PROPORTION));
            telemetry.addData("bl", motorBackLeftPower + (difference * LOCAL_PROPORTION));
            telemetry.addData("fr", motorFrontRightPower - (difference * LOCAL_PROPORTION));
            telemetry.addData("br", motorBackRightPower - (difference * LOCAL_PROPORTION));
            telemetry.update();

        }
        motorStop();
    }
    public void motorDriveTime(double motorFrontLeftPower, double motorBackLeftPower, double motorFrontRightPower, double motorBackRightPower, double time){
        // drives the motors for an allotted amount of time in a blocking manner
        // used by all time based movement functions

        motorDrive(motorFrontLeftPower, motorBackLeftPower, motorFrontRightPower, motorBackRightPower);
        delay(time);
        motorStop();
    }

    public void motorDriveForward(double speed){
        // drives robot forward
        // used in auto
        motorDrive(speed, speed, speed, speed);
    }
    public void motorDriveLeft(double speed){
        // drives robot left
        // used in auto
        motorDrive(speed, -speed, speed, -speed);
    }
    public void motorDriveRight(double speed){
        // drives robot right
        // used in auto
        motorDrive(-speed, speed, -speed, speed);
    }
    public void motorDriveBack(double speed){
        // drives robot back
        // used in auto
        motorDrive(-speed, -speed, -speed, -speed);
    }

    public void motorSpinLeft(double speed){
        // spins robot left(counter clockwise relative to above it) a specified number of millis
        // used in auto
        // OBSOLETE
        motorDrive(-speed, -speed, speed, speed);
    }
    public void motorSpinRight(double speed){
        // spins robot right(clockwise relative to above it) a specified number of millis
        // used in auto
        // OBSOLETE
        motorDrive(speed, speed, -speed, -speed);
    }

    public void spinnySpinEncoded(double speed, double target, int start){
        // spins spinner for target ticks
        // used in auto
        while (abs(spinner.getCurrentPosition()-start) < target){
            spinnySpin(speed);
        }
        spinnyStop();
    }


    // USEFUL METHODS
        // drive
            // auto
    public void motorDriveForwardEncoded(double speed, int distance){
        // drives robot forward an encoded distance
        // used in auto
        motorDriveEncoded(-speed, -speed, -speed, -speed, distance);
    }
    public void motorDriveLeftEncoded(double speed, int distance){
        // drives robot left an encoded distance
        // used in auto
        motorDriveEncoded(-speed, speed, speed, -speed, distance);
    }
    public void motorDriveRightEncoded(double speed, int distance){
        // drives robot right an encoded distance
        // used in auto
        motorDriveEncoded(speed, -speed, -speed, speed, distance);
    }
    public void motorDriveBackEncoded(double speed, int distance){
        // drives robot back an encoded distance
        // used in auto
        motorDriveEncoded(speed, speed, speed, speed, distance);
    }
    public void motorDriveForwardTime(double speed, double time){
        // drives robot forward a specified number of millis
        // used in auto
        motorDriveTime(speed, speed, speed, speed, time);
    }
    public void motorDriveLeftTime(double speed, double time){
        // drives robot left a specified number of millis
        // used in auto
        motorDriveTime(speed, -speed, speed, -speed, time);
    }
    public void motorDriveRightTime(double speed, double time){
        // drives robot right a specified number of millis
        // used in auto
        motorDriveTime(-speed, speed, -speed, speed, time);
    }
    public void motorDriveBackTime(double speed, double time){
        // drives robot back a specified number of millis
        // used in auto
        motorDriveTime(-speed, -speed, -speed, -speed, time);
    }
            // tele
    public void playSound(String filename){
        // play a sound
        // doesnt work but would be really fun :(

        startupID = hw.appContext.getResources().getIdentifier(filename, "raw", hw.appContext.getPackageName());
        appContext = hw.appContext;
        SoundPlayer.getInstance().startPlaying(appContext, startupID);
    }
    public void motorDriveXYVectors(double xvec, double yvec, double spinvec){
        // this class drives the robot in the direction of vectors from a joystick and a spin value
        // used for teleop mode driving wheels with joysticks


        double y = pow(yvec,1); // Remember, this is reversed!
        double x = pow(xvec * 1.1,1); // Counteract imperfect strafing
        rx = pow(spinvec,1);


        //denominator is the largest motor power (absolute value) or 1
        //this ensures all the powers maintain the same ratio, but only when
        //at least one is out of the range [-1, 1]
        double denominator = Math.max(abs(y) + abs(x) + abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        motorDrive(frontLeftPower,backLeftPower, frontRightPower, backRightPower);

    }
    public void motorStop(){
        // stop all the motors
        // used at the end of all movement functions

        motorBackLeft.setPower(MOTOR_STOP);
        motorFrontLeft.setPower(MOTOR_STOP);
        motorBackRight.setPower(MOTOR_STOP);
        motorFrontRight.setPower(MOTOR_STOP);
    }
    public void moveArm(double power){
        // sets arm power to power
        // used in tele

        arm.setPower(power);
    }
    public void spinnySpin(double speed){
        spinner.setPower(speed);
    }
            // both
    public void setServo(double angle){
        // sets claw  to an angle
        // used to change claw openess

        servo0.setPosition(angle);
    }

        // spin
    public void turnDeg(double degrees, double speed, Telemetry telemetry) {
            // self correcting method to turn to an angle relative to current angle
            // used in auto (SUPER COOL)
            double threshold = .1;
            double PROP_CONST = 0.02;
            double startDegrees = getAngles().firstAngle;
            while(Math.abs(getAngles().firstAngle - AngleUnit.normalizeDegrees(startDegrees + degrees)) > threshold){ // start degrees + degrees is the actual target postition

                double ns = Math.max(0.06, speed * Math.abs(getAngles().firstAngle - AngleUnit.normalizeDegrees(startDegrees + degrees))*PROP_CONST);

                if (getAngles().firstAngle - AngleUnit.normalizeDegrees(startDegrees + degrees) >= 0){
                    motorSpinRight(ns);
                    telemetry.addData("direct", "right");
                } else {
                    motorSpinLeft(ns);
                    telemetry.addData("direct", "left");
                }
                telemetry.addData("togo", Math.abs(getAngles().firstAngle - AngleUnit.normalizeDegrees(startDegrees + degrees)));
                telemetry.addData("fif", getAngles().firstAngle - AngleUnit.normalizeDegrees(startDegrees + degrees));
                telemetry.addData("fang", getAngles().firstAngle );
                telemetry.update();
            }
            motorStop();

        }
        // misc
    public void delay(double time){
        // used to stall program
        // used in many different applications and cloned to the auto classes

        ElapsedTime e = new ElapsedTime();
        e.reset();
        while(e.milliseconds() < time){
            // stal program


        }
    }
    public void setOpModeOn(boolean b){
    // used to set the opmodeon variable
    // used in class refering to meccanum to avoid being stuck in loop at program stop

    opModeActive = b;
}
        // spinner
    public void spinnySpinEncoded(double speed, double target){
        spinnySpinEncoded(speed, target, spinner.getCurrentPosition());
    }
    public void spinnyStop() {
        // stops spinner spinning
        // used in auto and tele
        spinner.setPower(0);
    }
    public void spinnySpinTime(double speed, double time){
        // spins spinner for time millis
        // used in auto
        spinnySpin(speed);
        delay(time);
        spinnyStop();
    }
        // arm
    public void moveArmTime(double power, double time){
        // moves arm for millis time
        // used in auto

        moveArm(power);
        delay(time);
        moveArm(0);

    }
        // claw
    public void openServoFull(){
        // opens claw servo
        // useful everywhere to open claw

        servo0.setPosition(SERVO_FULLY_OPENED);
    }
    public void closeServoFull(){
        // closes claw, might do it a bit too much, because of delay for opening
        // useful everywhere

        servo0.setPosition(SERVO_FULLY_CLOSED);
    }
        // angles
    public Orientation getAngles() {
        // gets current angle on field [-180, 180]               i just used interval notation :)
        // useful for angle methods
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    }
    public double getAngle360() {
        // gets normalized angle on field [0, 360]              i just used interval notation :)
        // useful for angle methods

        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle + 360;
    }


    // OBSOLETE METHODS

    public void openServoHalf(){
        // OBSOLETE
        servo0.setPosition(HALF_SERVO_ANGLE);
    }
    public double turnRadians(double radians, double speed) {
        // turn to angle relative to the field in some direction determined by speed(i think)
        // used in auto
        // KINDA OBSOLERE
        float startRadians = angles.firstAngle;
        double target = startRadians + radians;
        double minSpeed = 0.1;
        while(angles.firstAngle < target){
            motorSpinRight(Math.max(target - angles.firstAngle, minSpeed));
            angles = getAngles();
        }
        motorStop();
        return target-startRadians;


    }
    public void turnToDeg(double degrees, double speed) {
        // turn to angle relative to the field in some direction determined by speed(i think)
        // used in auto
        // KINDA OBSOLERE
        double threshold = .5;
        double startDegrees = getAngles().firstAngle;
        while(Math.abs(getAngles().firstAngle - AngleUnit.normalizeDegrees(degrees)) > threshold){ // make sure that the difference between the normalized degree to turn to and current degrees is withing a threshold
            if (getAngles().firstAngle - AngleUnit.normalizeDegrees(degrees) >= 0){ // turn the correct way
                motorSpinRight(speed);
            } else {
                motorSpinLeft(speed);
            }
        }
        motorStop();

    }
    public void motorSpinLeftEncoded(double speed, int distance){
        //OBSOLETE
        motorDriveEncoded(-speed, -speed, speed, speed, distance);
    }
    public void motorSpinRightEncoded(double speed, int distance){
        //OBSOLETE
        motorDriveEncoded(speed, speed, -speed, -speed, distance);
    }

    public void motorSpinLeftTime(double speed, double time){
        //OBSOLETE
        motorDriveTime(-speed, -speed, speed, speed, time);
    }
    public void motorSpinRightTime(double speed, double time){
        //OBSOLETE
        motorDriveTime(speed, speed, -speed, -speed, time);
    }
}
