package org.firstinspires.ftc.teamcode.comp.demo;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp
@Disabled
public class DemoOpMode extends LinearOpMode {

    // declaring motors so they are able to be accessed in other functions
    public DcMotor motorFrontLeft = null;
    public DcMotor motorFrontRight = null;
    public DcMotor motorBackLeft = null;
    public DcMotor motorBackRight = null;

    @Override
    public void runOpMode() throws InterruptedException {
        // setting motors to their respective devices in the hardwareMap
        motorFrontLeft = hardwareMap.get(DcMotor.class, "motorFrontLeft");
        motorFrontRight = hardwareMap.get(DcMotor.class, "motorFrontRight");
        motorBackLeft = hardwareMap.get(DcMotor.class, "motorBackLeft");
        motorBackRight = hardwareMap.get(DcMotor.class, "motorBackRight");
        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();
        // only run while the opmode is running
        while ( opModeIsActive() ){
            // power left and right motors according to the values of their gamepad sticks
            driveLeftMotors(gamepad1.right_stick_y);
            driveRightMotors(gamepad1.left_stick_y);
        }


    }

    public void driveLeftMotors(double power){
        motorBackLeft.setPower(power);
        motorFrontLeft.setPower(power);
    }

    public void driveRightMotors(double power){
        motorBackRight.setPower(power);
        motorFrontRight.setPower(power);
    }
}
