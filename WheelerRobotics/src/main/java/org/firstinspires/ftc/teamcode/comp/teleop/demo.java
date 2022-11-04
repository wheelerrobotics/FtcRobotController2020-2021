package org.firstinspires.ftc.teamcode.comp.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class demo extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
            DcMotor fl = hardwareMap.dcMotor.get("motorFrontLeft");
            DcMotor fr = hardwareMap.dcMotor.get("motorFrontRight");
            DcMotor bl = hardwareMap.dcMotor.get("motorBackLeft");
            DcMotor br = hardwareMap.dcMotor.get("motorBackRight");

            fl.setDirection(DcMotor.Direction.REVERSE);
            bl.setDirection(DcMotor.Direction.REVERSE);

            fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        waitForStart();


            while (opModeIsActive()) {
                double flp = gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x;
                double frp = gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x;
                double blp = gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x;
                double brp = gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x;

                fl.setPower(flp);
                fr.setPower(frp);
                bl.setPower(blp);
                br.setPower(brp);
            }
    }
}
