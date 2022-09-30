package org.firstinspires.ftc.teamcode.comp.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.comp.robot.Odo.Odo;

public class PIDImplementationYeah extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Odo o = new Odo();
        o.init(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {

        }
    }
}
