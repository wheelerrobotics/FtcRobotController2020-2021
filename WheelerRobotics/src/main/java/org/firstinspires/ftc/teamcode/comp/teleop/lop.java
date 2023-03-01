package org.firstinspires.ftc.teamcode.comp.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.comp.controller.Odo.ControllerMapLenny;
import org.firstinspires.ftc.teamcode.comp.robot.Odo.Lenny;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp(name="lop")
public class lop extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        // # Conservation of mass, The law of definite proportions, The law of multiple proportions, the law of
        //SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        SampleMecanumDrive drive = null;

        Lenny l = new Lenny();
        l.init(hardwareMap);
        l.slideinit();
        l.cawtinit();


        ControllerMapLenny cml = new ControllerMapLenny();
        cml.init(l, drive, gamepad1, gamepad2);
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        while (opModeIsActive()) {
            cml.checkControls();
        }
    }
}
