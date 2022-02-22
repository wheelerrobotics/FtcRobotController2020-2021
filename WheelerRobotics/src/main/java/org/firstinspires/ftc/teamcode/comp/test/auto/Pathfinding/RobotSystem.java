package org.firstinspires.ftc.teamcode.comp.test.auto.Pathfinding;

import android.content.Context;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorREV2mDistance;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class RobotSystem {

    protected final ElapsedTime runtime = new ElapsedTime(); // getting a warning to make it final

    protected Servo servo0;
    protected DcMotor arm;

    protected double rx;


    protected final String SINGLEPLAYER_CONTROL = "SINGLEPLAYER";
    protected final String MULTIPLAYER_CONTROL = "MULTIPLAYER";

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

    protected boolean opModeActive = false;

    protected DcMotor spinner;

    protected BNO055IMU imu;

    protected DcMotor motorFrontRight;
    protected DcMotor motorBackRight;
    protected DcMotor motorFrontLeft;
    protected DcMotor motorBackLeft;

    protected Orientation angles;

    protected HardwareMap hw;

    protected float INITIAL_ANGLE;

    protected FtcDashboard dash = FtcDashboard.getInstance();

    protected SensorREV2mDistance distanceBack;
    protected SensorREV2mDistance distanceRight;
    protected SensorREV2mDistance distanceLeft;



    protected int startupID;
    protected Context appContext;

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
        spinner.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //set prefs for arm and servo
        servo0.setDirection(Servo.Direction.FORWARD);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // define hw as the hardware map for possible access later in this class
        hw = hardwareMap;


        runtime.reset();
    }
    public void init2(@NonNull HardwareMap hardwareMap){
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

        //distace sensors (unused for now)
        /*
        distanceBack = hw.get(SensorREV2mDistance.class, "distanceBack");
        distanceRight = hw.get(SensorREV2mDistance.class, "distanceRight");
        distanceLeft = hw.get(SensorREV2mDistance.class, "distanceLeft");
        */

        // Meccanum Motors Definition and setting prefs

        motorFrontLeft = hardwareMap.dcMotor.get("motorfrontleft");
        motorBackLeft = hardwareMap.dcMotor.get("motorbackleft");
        motorFrontRight = hardwareMap.dcMotor.get("motorfrontright");
        motorBackRight = hardwareMap.dcMotor.get("motorbackright");

        // Reverse the left side motors and set behaviors to stop instead of coast
        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //define arm and servo objects and also spinner
        servo0 = hardwareMap.get(Servo.class, "bucket");
        arm = hardwareMap.get(DcMotor.class, "motorlinearslide");
        spinner = hardwareMap.get(DcMotor.class, "motornom");
        spinner.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //set prefs for arm and servo
        servo0.setDirection(Servo.Direction.FORWARD);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // define hw as the hardware map for possible access later in this class
        hw = hardwareMap;

        runtime.reset();
    }


    public Point getPosition(){
        Point pos = new Point(0, 0);
        return pos;
    }

}
