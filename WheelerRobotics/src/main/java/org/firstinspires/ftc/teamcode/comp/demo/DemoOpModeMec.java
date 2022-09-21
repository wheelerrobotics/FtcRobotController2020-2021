package org.firstinspires.ftc.teamcode.comp.demo;

import static java.lang.Math.abs;
import static java.lang.Math.pow;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


@TeleOp
public class DemoOpModeMec extends LinearOpMode {

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
        motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);

        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();
        // only run while the opmode is running
        while ( opModeIsActive() ){
            // power left and right motors according to the values of their gamepad sticks
            driveLeftMotors(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        }


    }

    public void driveLeftMotors(double yvec,double  xvec,double  spinvec){

        double y = -pow(yvec,1); // Remember, this is reversed!
        double x = pow(xvec * 1.1,1); // Counteract imperfect strafing
        double rx = pow(spinvec,1);

        double denominator = Math.max(abs(y) + abs(x) + abs(rx), 1);
        motorFrontLeft.setPower((y + x + rx) / denominator);
        motorBackLeft.setPower((y - x + rx) / denominator);
        motorFrontRight.setPower((y - x - rx) / denominator);
        motorBackRight.setPower((y + x - rx) / denominator);
    }
}
