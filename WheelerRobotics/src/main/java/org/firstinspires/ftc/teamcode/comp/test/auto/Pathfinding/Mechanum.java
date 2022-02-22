package org.firstinspires.ftc.teamcode.comp.test.auto.Pathfinding;


import static java.lang.Math.abs;
import static java.lang.Math.cos;
import static java.lang.Math.min;
import static java.lang.Math.pow;
import static java.lang.Math.sin;

import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.HashMap;

// robot driving and motion class

public class Mechanum extends RobotSystem {
    // NOT MECCANUM
    /*
        make functions that weigh sensor and encoder measurements


     */


    public void updateSystems(){

    }

    public double cameraSwivel(){
        double completedness = 0.0;
        Servo camServo = null;
        double camangle = camServo.getPosition();
        return camangle;
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

    public void driveRobotSVector(SlopeVector2d vector2d){

    }
    public void driveSto(){

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
    public void motorDriveTime(double motorFrontLeftPower, double motorBackLeftPower, double motorFrontRightPower, double motorBackRightPower, double time){
        motorDrive(motorFrontLeftPower, motorBackLeftPower, motorFrontRightPower, motorBackRightPower);
        motorStop();
    }

    public void motorSpinLeft(double speed){
        motorDrive(-speed, -speed, speed, speed);
    }
    public void motorSpinRight(double speed){
        motorDrive(speed, speed, -speed, -speed);
    }

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

        motorBackLeft.setPower(MOTOR_STOP);
        motorFrontLeft.setPower(MOTOR_STOP);
        motorBackRight.setPower(MOTOR_STOP);
        motorFrontRight.setPower(MOTOR_STOP);
    }
    // both
    public void setServo(double angle){
        servo0.setPosition(angle);
    }

    // ONLY PID
    public void turnDeg(double degrees, double speed, Telemetry telemetry) {
        // self correcting method to turn to an angle relative to current angle
        // used in auto (SUPER COOL)
        double threshold = 0.1;
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


    public void turnDegPID(double degrees, Telemetry telemetry) {
        // self correcting method to turn to an angle relative to current angle
        // used in auto (SUPER COOL)
        double kd = 0.6;
        double kp = 0.2;
        double threshold = 0.1;
        double PROP_CONST = 0.02;
        double startPos = spinner.getCurrentPosition();
        ElapsedTime time = new ElapsedTime();
        time.reset();
        double integration = 0.0;
        double derivative = 0.0;
        double proportion = 0.0;

        HashMap<Integer, Double> et = new HashMap<Integer, Double>(); // error: rt-yt
        HashMap<Integer, Double> yt = new HashMap<Integer, Double>(); // measured output
        HashMap<Integer, Double> rt = new HashMap<Integer, Double>(); // setpoint/target NOTE: this will not change, but using a hashmap instead of static var makes it future proof
        HashMap<Integer, Double> dt = new HashMap<Integer, Double>(); // change in time over previous interval in millis

        ElapsedTime changeTime = new ElapsedTime();

        double crt = 0.0;
        double cyt = 0.0;
        double cet = 0.0;

        while(true){ // start degrees + degrees is the actual target postition

            changeTime.reset();

            double writeVal = 0;
            //current vals
            crt = startPos + degrees;
            cyt = startPos - spinner.getCurrentPosition();
            cet = cyt - crt;


            telemetry.addData("siz", dt.size());
            telemetry.update();

            int ct = dt.size();
            rt.put(rt.size(), crt);
            yt.put(yt.size(), cyt);
            et.put(et.size(), cet);
            dt.put(dt.size(), changeTime.milliseconds());



            //HashMap<Integer, Double> ut = new HashMap<>(); // input
            // maybe change from traditional pid to a pid with an integral that only reacts to change if there is notable difference between yt and rt
            //integration += et.get(ct) * dt.get(ct); // integrates over time
            if(et.containsKey(ct-1)) derivative = ( cet - et.get(ct-1) ) / dt.get(ct); // slope between now and last time step
            else derivative = 0;
            proportion = cet; // const

            writeVal = proportion * kp + derivative * kd;

            spinner.setPower(writeVal);

            telemetry.addData("derivative", derivative);
            telemetry.addData("proportion", proportion);

            telemetry.addData("yt", cyt);
            telemetry.addData("rt", crt);
            telemetry.addData("et", cet);

            telemetry.update();



        }
        //motorStop();

    }
    // claw
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

}
