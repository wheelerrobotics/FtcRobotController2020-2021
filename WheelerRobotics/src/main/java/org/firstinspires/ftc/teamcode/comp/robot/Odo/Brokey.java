package org.firstinspires.ftc.teamcode.comp.robot.Odo;

import static java.lang.Math.abs;

import android.content.Context;

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

import org.firstinspires.ftc.teamcode.comp.chassis.Meccanum.Meccanum;
import org.firstinspires.ftc.teamcode.comp.robot.Robot;

public class Brokey extends Meccanum implements Robot {

    protected DcMotor auxMotor1 = null;
    protected DcMotor auxMotor2 = null;

    protected Servo servo = null;
    protected CRServo crServo = null;

    protected LED led1 = null;
    protected LED led2 = null;

    protected HardwareMap hw = null;

    @Override
    public void init(HardwareMap hardwareMap) {
        super.init(hardwareMap);
        // init the class, declare all the sensors and motors and stuff
        // should be called before using class ALWAYS

        // internal IMU setup (copied and pasted, idk what it really does, but it works)
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        // angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);

        //distace sensors (unused for now)

        distanceBack = hardwareMap.get(DistanceSensor.class, "distanceBack");
        distanceRight = hardwareMap.get(DistanceSensor.class, "distanceRight");
        distanceLeft = hardwareMap.get(DistanceSensor.class, "distanceLeft");
        distanceFront = null;

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
        servo = hardwareMap.get(Servo.class, "servo");
        // crServo = hardwareMap.get(CRServo.class, "cr_servo");
        auxMotor2 = hardwareMap.get(DcMotor.class, "arm");
        auxMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        auxMotor1 = hardwareMap.get(DcMotor.class, "spinner");
        auxMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

  //      led1 = hardwareMap.get(LED.class, "led_1");
//        led2 = hardwareMap.get(LED.class, "led_2");

        //set prefs for arm and servo
        servo.setDirection(Servo.Direction.FORWARD);

        // define hw as the hardware map for possible access later in this class
        hw = hardwareMap;


        runtime.reset();
    }

    public void alignCamera(){
        servo.setPosition(0.5);
    }

    public void setServo(double angle){
        // sets claw  to an angle
        // used to change claw openess

        servo.setPosition(angle);
    }

    public void move(double power){
        // sets arm to a power
        // used to change arm position

        auxMotor2.setPower(power);
    }
    public void spinnySpin(double speed){
        auxMotor1.setPower(speed);
    }

    public void spinnyStop() {
        // stops spinner spinning
        // used in auto and tele
        auxMotor1.setPower(0);
    }
    // angles
    public void playSound(String filename){
        // play a sound
        // doesnt work but would be really fun :(

        int startupID = hw.appContext.getResources().getIdentifier(filename, "raw", hw.appContext.getPackageName());
        Context appContext = hw.appContext;
        SoundPlayer.getInstance().startPlaying(appContext, startupID);
    }
}
