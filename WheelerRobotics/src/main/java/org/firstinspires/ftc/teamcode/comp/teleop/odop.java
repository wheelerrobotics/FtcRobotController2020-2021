package org.firstinspires.ftc.teamcode.comp.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.comp.controller.Odo.ControllerMapOdo;
import org.firstinspires.ftc.teamcode.comp.robot.Odo.Odo;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp(name="roo coo coo")
public class odop extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        // # Conservation of mass, The law of definite proportions, The law of multiple proportions, the law of
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Odo o = new Odo();
        o.init(hardwareMap);
        ControllerMapOdo cmo = new ControllerMapOdo();
        cmo.init(o, drive, gamepad1, gamepad2);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        while (opModeIsActive()) {
            cmo.checkControls();
        }
    }
}
