package org.firstinspires.ftc.teamcode.comp.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.comp.controller.Odo.ControllerMapFrantFC;
import org.firstinspires.ftc.teamcode.comp.robot.Odo.Frant;

@TeleOp(name="Myles's Mighty Menacing Machine")
public class frantopfc extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        // # Conservation of mass, The law of definite proportions, The law of multiple proportions, the law of
        Frant f = new Frant();
        f.init(hardwareMap);
        ControllerMapFrantFC cmf = new ControllerMapFrantFC();
        cmf.init(f, gamepad1, gamepad2);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        while (opModeIsActive()) {
            cmf.checkControls();
        }
    }
}
