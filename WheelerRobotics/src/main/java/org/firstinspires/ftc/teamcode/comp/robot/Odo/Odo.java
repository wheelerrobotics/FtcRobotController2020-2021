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

public class Odo extends Meccanum implements Robot {

    protected DcMotor aux_motor_1 = null;
    protected DcMotor aux_motor_2 = null;

    protected Servo servo = null;
    protected CRServo cr_servo = null;

    protected LED led_1 = null;
    protected LED led_2 = null;

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

        distance_back = hardwareMap.get(DistanceSensor.class, "distanceBack");
        distance_right = hardwareMap.get(DistanceSensor.class, "distanceRight");
        distance_left = hardwareMap.get(DistanceSensor.class, "distanceLeft");
        distance_front = null;

        // Meccanum Motors Definition and setting prefs

        motor_front_left = hardwareMap.dcMotor.get("motorFrontLeft");
        motor_back_left = hardwareMap.dcMotor.get("motorBackLeft");
        motor_front_right = hardwareMap.dcMotor.get("motorFrontRight");
        motor_back_right = hardwareMap.dcMotor.get("motorBackRight");

        // Reverse the left side motors and set behaviors to stop instead of coast
        motor_front_left.setDirection(DcMotorSimple.Direction.REVERSE);
        motor_back_left.setDirection(DcMotorSimple.Direction.REVERSE);

        motor_front_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor_back_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor_front_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor_back_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //define arm and servo objects and also spinner
        servo = hardwareMap.get(Servo.class, "servo");
        cr_servo = hardwareMap.get(CRServo.class, "cr_servo");
        aux_motor_1 = hardwareMap.get(DcMotor.class, "arm");
        aux_motor_1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        aux_motor_1 = hardwareMap.get(DcMotor.class, "spinner");
        aux_motor_1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        led_1 = hardwareMap.get(LED.class, "led_1");
        led_2 = hardwareMap.get(LED.class, "led_2");

        //set prefs for arm and servo
        servo.setDirection(Servo.Direction.FORWARD);

        // define hw as the hardware map for possible access later in this class
        hw = hardwareMap;


        runtime.reset();
    }

    public void alignCamera(){
        servo.setPosition(0.5);
    }

    public void spinnySpinEncoded(double speed, double target, int start){
        // spins spinner for target ticks
        // used in auto
        while (abs(aux_motor_1.getCurrentPosition()-start) < target){
            spinnySpin(speed);
        }
        spinnyStop();
    }
    public void setServo(double angle){
        // sets claw  to an angle
        // used to change claw openess

        servo.setPosition(angle);
    }

    public void spinnySpin(double speed){
        aux_motor_1.setPower(speed);
    }

    public void spinnyStop() {
        // stops spinner spinning
        // used in auto and tele
        aux_motor_1.setPower(0);
    }
    public void spinnySpinTime(double speed, double time){
        // spins spinner for time millis
        // used in auto
        spinnySpin(speed);
        delay(time);
        spinnyStop();
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
