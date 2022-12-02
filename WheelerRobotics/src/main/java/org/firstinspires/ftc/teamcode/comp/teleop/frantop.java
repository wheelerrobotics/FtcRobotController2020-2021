package org.firstinspires.ftc.teamcode.comp.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.comp.controller.Odo.ControllerMapFrant;
import org.firstinspires.ftc.teamcode.comp.robot.Odo.Frant;

@TeleOp(name="Griffin's Wonderful Coding Adventure")
public class frantop extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        // # Conservation of mass, The law of definite proportions, The law of multiple proportions, the law of
        Frant f = new Frant();
        f.init(hardwareMap);
        ControllerMapFrant cmf = new ControllerMapFrant();
        cmf.init(f, gamepad1, gamepad2);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        while (opModeIsActive()) {
            cmf.checkControls();
        }
    }
}
